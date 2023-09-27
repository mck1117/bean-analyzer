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
        mData->Advance(mBitTime);
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
            continue;
        }

        // Priority
        auto pri = bq.ReadBits(4);
        MakeFrameFromBits(pri, mResults, 1, samples_per_bit);

        auto ml = bq.ReadBits(4);
        MakeFrameFromBits(ml, mResults, 2, samples_per_bit);

        // Now we know exactly how many bits we should have read in
        // TODO: what is correct value here?
        size_t expectedBitsRemaining = 8 * ml.Data + 5;

        if (bq.size() != expectedBitsRemaining)
        {
            // TODO: error, wrong number of bits
            continue;
        }

        auto dstId = bq.ReadByte();
        MakeFrameFromBits(dstId, mResults, 3, samples_per_bit);
        auto mesId = bq.ReadByte();
        MakeFrameFromBits(mesId, mResults, 4, samples_per_bit);
        uint8_t data[11];

        // Actual number of data bytes is (ml - 3 for dstId, mesId, crc)
        size_t dataBytes = ml.Data - 3;
        for (size_t i = 0; i < dataBytes; i++)
        {
            auto b = bq.ReadByte();
            MakeFrameFromBits(b, mResults, 5, samples_per_bit);
            data[i] = b.Data;
        }

        auto crc = bq.ReadByte();
        MakeFrameFromBits(crc, mResults, 6, samples_per_bit);
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