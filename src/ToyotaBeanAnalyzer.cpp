#include "ToyotaBeanAnalyzer.h"

#include "ToyotaBeanAnalyzerSettings.h"

#include <AnalyzerChannelData.h>
#include <queue>

ToyotaBeanAnalyzer::ToyotaBeanAnalyzer()
    : Analyzer2(), mSettings(new ToyotaBeanAnalyzerSettings())
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

    BitResult ReadBit()
    {
        mData->Advance(mBitTime);
        // This is a data bit, mark it with the dot
        mResults->AddMarker(mData->GetSampleNumber(), AnalyzerResults::Dot, mChannel);
        bool bit = mData->GetBitState() == BIT_HIGH;

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
            mData->Advance(mBitTime);
            bool stuffBit = mData->GetBitState() == BIT_HIGH;

            // Stuff bit should be opposite the last data bit
            // If not, it's a bit stuffing error
            if (stuffBit == bit)
            {
                // This is a mis-stuff, mark it as an error
                mResults->AddMarker(mData->GetSampleNumber(), AnalyzerResults::ErrorX, mChannel);

                return bit ? BitResult::StuffErrorOnes : BitResult::StuffErrorZeroes;
            }
            else
            {
                // mark the stuffed bit with the square
                mResults->AddMarker(mData->GetSampleNumber(), AnalyzerResults::Square, mChannel);

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

struct BitQueue
{
public:
    void push(bool bit)
    {
        mBits.push(bit);
    }

    bool empty() const
    {
        return mBits.empty();
    }

    size_t size() const
    {
        return mBits.size();
    }

    bool ReadBit()
    {
        bool x = mBits.front();
        mBits.pop();
        return x;
    }

    uint8_t ReadBits(size_t bits)
    {
        uint8_t result = 0;

        // TODO: is it MSB first or LSB first?
        for (size_t i = 0; i < bits; i++)
        {
            result = result << 1;
            result |= (ReadBit() ? 1 : 0);
        }

        return result;
    }

    uint8_t ReadByte()
    {
        return ReadBits(8);
    }

private:
    std::queue<bool> mBits;
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

void ToyotaBeanAnalyzer::WorkerThread()
{
    mSampleRateHz = GetSampleRate();

    mSerial = GetAnalyzerChannelData(mSettings->mInputChannel);

    U32 samples_per_bit = mSampleRateHz / mSettings->mBitRate;
    // 1 - High start bit
    // 4 - priority bits
    // 4 - message length bits (ID + DATA)
    // 8 - DST-ID destination ID
    // 8 - MES-ID message ID
    // 8*1-11 - 1-11 data bytes
    // 8 - CRC error check
    // 8 - EOM end of frame
    // 2 - RSP response (?)
    // 6 - EOF

    // Make sure we're between frames, so that we start decoding the first frame in the right spot
    WaitFor6LowBits(samples_per_bit);
    
    while (true)
    {
        // Find the next rising edge
        while (mSerial->GetBitState() != BIT_HIGH)
            mSerial->AdvanceToNextEdge();

        auto frameStartSample = mSerial->GetSampleNumber();
        mResults->AddMarker(mSerial->GetSampleNumber(), AnalyzerResults::Start, mSettings->mInputChannel);

        // Advance to the center of the first bit after the start bit
        mSerial->Advance(samples_per_bit / 2);

        BitUnstuffer bits(mSerial, mResults, mSettings->mInputChannel, samples_per_bit);

        BitQueue bq;

        bool err = false;

        while (true)
        {
            auto bit = bits.ReadBit();

            if (bit == BitResult::StuffErrorZeroes)
            {
                // End of frame - 6 consecutive low bits
                break;
            }
            else if (bit == BitResult::StuffErrorOnes)
            {
                // TODO: what do we do about a 6-ones-error?
                err = true;
                break;
            }
            else
            {
                // Normal bit - enqueue it
                bq.push(bit == BitResult::One);
            }
        }

        mResults->AddMarker(mSerial->GetSampleNumber(), AnalyzerResults::Stop, mSettings->mInputChannel);

        if (err)
        {
            WaitFor6LowBits(samples_per_bit);
            continue;
        }

        // Now, decode that sequence of bits in to a frame

        if (bq.size() < 8)
        {
            // TODO: not enough bits to read size!
            continue;
        }

        // Priority
        uint8_t pri = bq.ReadBits(4);
        uint8_t ml = bq.ReadBits(4);

        // Now we know exactly how many bits we should have read in
        // TODO: what is correct value here?
        size_t expectedBitsRemaining = 8 * ml + 40 - 8;

        if (bq.size() != expectedBitsRemaining)
        {
            // TODO: error, wrong number of bits
            continue;
        }

        uint8_t dstId = bq.ReadByte();
        uint8_t mesId = bq.ReadByte();
        uint8_t data[11];

        // Actual number of data bytes is (ml - 3 for dstId, mesId, crc)
        size_t dataBytes = ml - 3;
        for (size_t i = 0; i < dataBytes; i++)
        {
           data[i] = bq.ReadByte();
        }

        uint8_t crc = bq.ReadByte();
        uint8_t eom = bq.ReadBits(8);

        // EOM should equal exactly 0b01111110
        if (eom != 0b0111'1110)
        {
            continue;
        }

        uint8_t rsp = bq.ReadBits(2);

        // We have a frame to save
        FrameV2 frame;
        frame.AddInteger("PRI", pri);
        frame.AddInteger("ML", ml);
        frame.AddInteger("DST-ID", dstId);
        frame.AddInteger("MES-ID", mesId);
        frame.AddByteArray("Data", data, dataBytes);
        frame.AddInteger("CRC8", crc);
        frame.AddInteger("RSP", rsp);
        frame.AddInteger("EOM", eom);

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