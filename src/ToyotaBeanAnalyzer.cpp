#include "ToyotaBeanAnalyzer.h"

#include "ToyotaBeanAnalyzerSettings.h"

#include <AnalyzerChannelData.h>
#include <queue>

ToyotaBeanAnalyzer::ToyotaBeanAnalyzer()
    : Analyzer2(), mSettings(new ToyotaBeanAnalyzerSettings()), mSimulationInitilized(false)
{
    SetAnalyzerSettings(mSettings.get());
}

ToyotaBeanAnalyzer::~ToyotaBeanAnalyzer()
{
    KillThread();
}

void ToyotaBeanAnalyzer::SetupResults()
{
    mResults.reset(new ToyotaBeanAnalyzerResults(this, mSettings.get()));
    SetAnalyzerResults(mResults.get());
    mResults->AddChannelBubblesWillAppearOn(mSettings->mInputChannel);
}

enum class BitResult
{
    Zero,
    One,
    StuffErrorOnes,
    StuffErrorZeroes,
};

class BitUnstuffer
{
public:
    BitUnstuffer(
        AnalyzerChannelData* data, std::unique_ptr<ToyotaBeanAnalyzerResults>& results, Channel channel, U32 bitTime)
        : mData(data), mResults(results), mBitTime(bitTime), mChannel(channel)
    {
    }

    bool ReadBitNostuff()
    {
        // If there's a transition in the next bit time, resync on it
        if (mData->WouldAdvancingCauseTransition(mBitTime))
        {
            mData->AdvanceToNextEdge();
            mData->Advance(mBitTime / 2);
        }
        else
        {
            mData->Advance(mBitTime);
        }

        return mData->GetBitState() == BIT_HIGH;
    }

    BitResult ReadBit()
    {
        bool bit = ReadBitNostuff();
        // This is a data bit, mark it with the dot
        mResults->AddMarker(mData->GetSampleNumber(), AnalyzerResults::Dot, mChannel);

        bool expectStuff = false;

        // This is a natural transition, reset stuffing
        if (bit != mLastBit)
        {
            mLastBit = bit;
            mBitsInSequence = 1;
        }
        else
        {
            mBitsInSequence++;

            if (mBitsInSequence == 5)
            {
                // This is 5 in a row, next bit should be a stuff!
                expectStuff = true;
            }
        }

        if (expectStuff)
        {
            bool stuffBit = ReadBitNostuff();

            // Stuff bit should be opposite the last data bit
            // If not, it's a bit stuffing error
            if (stuffBit == bit)
            {
                mResults->AddMarker(mData->GetSampleNumber(), AnalyzerResults::Dot, mChannel);

                return bit ? BitResult::StuffErrorOnes : BitResult::StuffErrorZeroes;
            }
            else
            {
                // mark the stuffed bit with the square
                mResults->AddMarker(mData->GetSampleNumber(), AnalyzerResults::X, mChannel);

                // The last bit was now the stuffed bit, reset state to that
                mLastBit = stuffBit;
                mBitsInSequence = 1;
            }
        }

        return bit ? BitResult::One : BitResult::Zero;
    }

    void Reset(bool state)
    {
        mLastBit = state;
        mBitsInSequence = 1;
    }

private:
    AnalyzerChannelData* mData;
    std::unique_ptr<ToyotaBeanAnalyzerResults>& mResults;
    Channel mChannel;
    U32 mBitTime;

    // We start at the start bit, which is a single high bit
    bool mLastBit = true;
    size_t mBitsInSequence = 1;
};

struct StampedBit
{
    bool Bit;
    U64 Time;
};

struct StampedByte
{
    uint8_t Data = 0;
    U64 Start;
    U64 End;
};

struct BitQueue
{
public:
    void push(bool bit, U64 time)
    {
        mBits.push({bit, time});
    }

    bool empty() const
    {
        return mBits.empty();
    }

    size_t size() const
    {
        return mBits.size();
    }

    StampedBit ReadBit()
    {
        auto x = mBits.front();
        mBits.pop();
        return x;
    }

    StampedByte ReadBits(size_t bits)
    {
        StampedByte result;

        {
            auto b = ReadBit();
            result.Start = b.Time;
            result.Data = b.Bit ? 1 : 0;
        }

        // MSB first
        for (size_t i = 0; i < bits - 2; i++)
        {
            result.Data = result.Data << 1;
            result.Data |= (ReadBit().Bit ? 1 : 0);
        }

        {
            auto b = ReadBit();
            result.Data = result.Data << 1;
            result.Data |= (b.Bit ? 1 : 0);
            result.End = b.Time;
        }

        return result;
    }

    StampedByte ReadByte()
    {
        return ReadBits(8);
    }

private:
    std::queue<StampedBit> mBits;
};

void ToyotaBeanAnalyzer::WaitFor6LowBits(U32 bitTime)
{
    if (mSerial->GetBitState() == BIT_HIGH)
    {
        mSerial->AdvanceToNextEdge();
    }

    while (true)
    {
        if (!mSerial->WouldAdvancingCauseTransition(6 * bitTime))
        {
            // No rising edge in the next 6 bit times, this is indeed between frames!
            return;
        }

        // Advance to the next falling edge
        mSerial->AdvanceToNextEdge();
        mSerial->AdvanceToNextEdge();
    }
}

static void MakeFrameFromBits(StampedByte data, std::unique_ptr<ToyotaBeanAnalyzerResults>& results, uint8_t type, U32 samples_per_bit)
{
    U32 offset = 0.4f * samples_per_bit;

    Frame f;
    f.mStartingSampleInclusive = data.Start - offset;
    f.mEndingSampleInclusive = data.End + offset;
    f.mData1 = data.Data;
    f.mType = type;
    results->AddFrame(f);
}

static const uint8_t crcTable[] = {
    0x00, 0x13, 0x26, 0x35, 0x4C, 0x5F, 0x6A, 0x79, 0x98, 0x8B, 0xBE, 0xAD, 0xD4, 0xC7, 0xF2, 0xE1, 0x23, 0x30, 0x05,
    0x16, 0x6F, 0x7C, 0x49, 0x5A, 0xBB, 0xA8, 0x9D, 0x8E, 0xF7, 0xE4, 0xD1, 0xC2, 0x46, 0x55, 0x60, 0x73, 0x0A, 0x19,
    0x2C, 0x3F, 0xDE, 0xCD, 0xF8, 0xEB, 0x92, 0x81, 0xB4, 0xA7, 0x65, 0x76, 0x43, 0x50, 0x29, 0x3A, 0x0F, 0x1C, 0xFD,
    0xEE, 0xDB, 0xC8, 0xB1, 0xA2, 0x97, 0x84, 0x8C, 0x9F, 0xAA, 0xB9, 0xC0, 0xD3, 0xE6, 0xF5, 0x14, 0x07, 0x32, 0x21,
    0x58, 0x4B, 0x7E, 0x6D, 0xAF, 0xBC, 0x89, 0x9A, 0xE3, 0xF0, 0xC5, 0xD6, 0x37, 0x24, 0x11, 0x02, 0x7B, 0x68, 0x5D,
    0x4E, 0xCA, 0xD9, 0xEC, 0xFF, 0x86, 0x95, 0xA0, 0xB3, 0x52, 0x41, 0x74, 0x67, 0x1E, 0x0D, 0x38, 0x2B, 0xE9, 0xFA,
    0xCF, 0xDC, 0xA5, 0xB6, 0x83, 0x90, 0x71, 0x62, 0x57, 0x44, 0x3D, 0x2E, 0x1B, 0x08, 0x0B, 0x18, 0x2D, 0x3E, 0x47,
    0x54, 0x61, 0x72, 0x93, 0x80, 0xB5, 0xA6, 0xDF, 0xCC, 0xF9, 0xEA, 0x28, 0x3B, 0x0E, 0x1D, 0x64, 0x77, 0x42, 0x51,
    0xB0, 0xA3, 0x96, 0x85, 0xFC, 0xEF, 0xDA, 0xC9, 0x4D, 0x5E, 0x6B, 0x78, 0x01, 0x12, 0x27, 0x34, 0xD5, 0xC6, 0xF3,
    0xE0, 0x99, 0x8A, 0xBF, 0xAC, 0x6E, 0x7D, 0x48, 0x5B, 0x22, 0x31, 0x04, 0x17, 0xF6, 0xE5, 0xD0, 0xC3, 0xBA, 0xA9,
    0x9C, 0x8F, 0x87, 0x94, 0xA1, 0xB2, 0xCB, 0xD8, 0xED, 0xFE, 0x1F, 0x0C, 0x39, 0x2A, 0x53, 0x40, 0x75, 0x66, 0xA4,
    0xB7, 0x82, 0x91, 0xE8, 0xFB, 0xCE, 0xDD, 0x3C, 0x2F, 0x1A, 0x09, 0x70, 0x63, 0x56, 0x45, 0xC1, 0xD2, 0xE7, 0xF4,
    0x8D, 0x9E, 0xAB, 0xB8, 0x59, 0x4A, 0x7F, 0x6C, 0x15, 0x06, 0x33, 0x20, 0xE2, 0xF1, 0xC4, 0xD7, 0xAE, 0xBD, 0x88,
    0x9B, 0x7A, 0x69, 0x5C, 0x4F, 0x36, 0x25, 0x10, 0x03};

static uint8_t crc8(uint8_t* data, size_t len)
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t b = data[i];
        /* XOR-in next input byte */
        uint8_t data = (b ^ crc);
        /* get current CRC value = remainder */
        crc = crcTable[data];
    }

    return crc;
}

void ToyotaBeanAnalyzer::WorkerThread()
{
    mSampleRateHz = GetSampleRate();

    mSerial = GetAnalyzerChannelData(mSettings->mInputChannel);

    U32 samples_per_bit = mSampleRateHz / mSettings->mBitRate;
    // MSB first
    // 1 - High start bit
    // 4 - priority bits
    // 4 - message length bits (ID + DATA)
    // 8 - DST-ID destination ID
    // 8 - MES-ID message ID
    // 8*1-11 - 1-11 data bytes
    // 8 - CRC error check
    // 8 - EOM end of frame - 01111110 (no stuffing)
    // 2 - RSP response. 10 = ACK, 01 = NAK
    // 6 - EOF

    // Make sure we're between frames, so that we start decoding the first frame in the right spot
    WaitFor6LowBits(samples_per_bit);
    
    while (true)
    {
        // Find the next rising edge
        while (mSerial->GetBitState() != BIT_HIGH)
            mSerial->AdvanceToNextEdge();

        auto frameStartSample = mSerial->GetSampleNumber();
        
        // Advance to the center of the start bit
        mSerial->Advance(samples_per_bit / 2);
        mResults->AddMarker(mSerial->GetSampleNumber(), AnalyzerResults::Start, mSettings->mInputChannel);

        BitUnstuffer bits(mSerial, mResults, mSettings->mInputChannel, samples_per_bit);

        BitQueue bq;

        bool err = false;

        while (true)
        {
            auto bit = bits.ReadBit();

            if (bit == BitResult::StuffErrorZeroes)
            {
                // We shouldn't get 6 consecutive zeroes until after EOM
                err = true;
                break;
            }
            else if (bit == BitResult::StuffErrorOnes)
            {
                // End of message - 6 consecutive ones
                break;
            }
            else
            {
                // Normal bit - enqueue it
                bq.push(bit == BitResult::One, mSerial->GetSampleNumber());
            }
        }

        if (err)
        {
            WaitFor6LowBits(samples_per_bit);
            continue;
        }

        // We should now be at the 6th bit of the EOM - next bit is a zero, then two ack bits, then 6 low bits
        if (BitResult::Zero != bits.ReadBit())
        {
            // TODO;
            continue;
        }

        // ack
        auto ack1 = bits.ReadBitNostuff();
        mResults->AddMarker(mSerial->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mInputChannel);
        auto ack2 = bits.ReadBitNostuff();
        mResults->AddMarker(mSerial->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mInputChannel);

        // EOF
        // TODO: check that these are all zeroes
        for (int i = 0; i < 6; i++)
        {
            bits.ReadBitNostuff();
        }

        mResults->AddMarker(mSerial->GetSampleNumber(), AnalyzerResults::Stop, mSettings->mInputChannel);

        // Now, decode that sequence of bits in to a frame

        if (bq.size() < 8)
        {
            // TODO: not enough bits to read size!
            WaitFor6LowBits(samples_per_bit);
            continue;
        }

        // Priority
        auto pri = bq.ReadBits(4);
        MakeFrameFromBits(pri, mResults, 1, samples_per_bit);

        auto ml = bq.ReadBits(4);
        MakeFrameFromBits(ml, mResults, 2, samples_per_bit);

        // Now we know exactly how many bits we should have read in
        // TODO: what is correct value here?
        size_t expectedBitsRemaining = 8 * ml.Data + 13;

        if (bq.size() < expectedBitsRemaining)
        {
            // TODO: error, wrong number of bits
            continue;
        }

        auto dstId = bq.ReadByte();
        MakeFrameFromBits(dstId, mResults, 3, samples_per_bit);
        auto mesId = bq.ReadByte();
        MakeFrameFromBits(mesId, mResults, 4, samples_per_bit);
        uint8_t data[11];

        // Actual number of data bytes is (ml - 2 for dstId, mesId)
        size_t dataBytes = ml.Data - 2;
        for (size_t i = 0; i < dataBytes; i++)
        {
            auto b = bq.ReadByte();
            MakeFrameFromBits(b, mResults, 5, samples_per_bit);
            data[i] = b.Data;
        }

        uint8_t crcbuf[32];
        crcbuf[0] = pri.Data << 4 | ml.Data;
        crcbuf[1] = dstId.Data;
        crcbuf[2] = mesId.Data;
        memcpy(crcbuf + 3, data, dataBytes);

        uint8_t crcActual = crc8(crcbuf, dataBytes + 3);

        auto crc = bq.ReadByte();
        MakeFrameFromBits(crc, mResults, crcActual == crc.Data ? 6 : 9, samples_per_bit);
        auto eom = bq.ReadBits(5);
        // fudge the end time of EOM
        eom.End += 3 * samples_per_bit;
        U64 eomEnd = eom.End;
        MakeFrameFromBits(eom, mResults, 7, samples_per_bit);

        // EOM should equal exactly 0b01111110, but the last 3 bits aren't captured in this layer
        if (eom.Data != 0b01111)
        {
            continue;
        }

        // Fake the RSP bits/timing
        StampedByte rsp;
        rsp.Data = (ack1 ? 2 : 0) + (ack2 ? 1 : 0);
        rsp.Start = eomEnd + samples_per_bit;
        rsp.End = rsp.Start + samples_per_bit;
        MakeFrameFromBits(rsp, mResults, 8, samples_per_bit);

        // We have a frame to save
        FrameV2 frame;
        frame.AddInteger("PRI", pri.Data);
        frame.AddInteger("ML", ml.Data);
        frame.AddInteger("DST-ID", dstId.Data);
        frame.AddInteger("MES-ID", mesId.Data);
        frame.AddByteArray("Data", data, dataBytes);
        frame.AddInteger("CRC8", crc.Data);
        frame.AddInteger("RSP", rsp.Data);
        mResults->AddFrameV2(frame, "bean", frameStartSample, mSerial->GetSampleNumber());
        mResults->CommitResults();
        ReportProgress(mSerial->GetSampleNumber());

        CheckIfThreadShouldExit();
    }
}

bool ToyotaBeanAnalyzer::NeedsRerun()
{
    return false;
}

U32 ToyotaBeanAnalyzer::GenerateSimulationData(
    U64 minimum_sample_index, U32 device_sample_rate, SimulationChannelDescriptor** simulation_channels)
{
    if (mSimulationInitilized == false)
    {
        mSimulationDataGenerator.Initialize(GetSimulationSampleRate(), mSettings.get());
        mSimulationInitilized = true;
    }

    return mSimulationDataGenerator.GenerateSimulationData(
        minimum_sample_index, device_sample_rate, simulation_channels);
}

U32 ToyotaBeanAnalyzer::GetMinimumSampleRateHz()
{
    return mSettings->mBitRate * 4;
}

const char* ToyotaBeanAnalyzer::GetAnalyzerName() const
{
    return "ToyotaBean";
}

const char* GetAnalyzerName()
{
    return "ToyotaBean";
}

Analyzer* CreateAnalyzer()
{
    return new ToyotaBeanAnalyzer();
}

void DestroyAnalyzer(Analyzer* analyzer)
{
    delete analyzer;
}