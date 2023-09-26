#include "ToyotaBeanAnalyzer.h"

#include "ToyotaBeanAnalyzerSettings.h"

#include <AnalyzerChannelData.h>

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

class BitUnstuffer
{
public:
    BitUnstuffer(
        AnalyzerChannelData* data, std::auto_ptr<ToyotaBeanAnalyzerResults> results, Channel channel, U32 bitTime)
        : mData(data), mResults(results), mBitTime(bitTime), mChannel(channel)
    {
    }

    bool ReadBit()
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
            // mark the stuffed bit with the square
            mResults->AddMarker(mData->GetSampleNumber(), AnalyzerResults::Square, mChannel);
            bool stuffBit = mData->GetBitState() == BIT_HIGH;

            // Stuff bit should be opposite the last data bit
            // If not, it's a bit stuffing error
            if (stuffBit == bit)
            {
                throw "bit stuff err";
            }
            else
            {
                // The last bit was now the stuffed bit, reset state to that
                mLastBit = stuffBit;
                mBitsInSequence = 1;
            }
        }

        return bit;
    }

    uint8_t ReadBits(size_t bits)
    {
        uint8_t result = 0;

        // TODO: is it MSB first or LSB first?
        for (size_t i = 0; i < bits; i++)
        {
            result = result << 1;
            result |= ReadBit();
        }

        return result;
    }

    uint8_t ReadByte()
    {
        return ReadBits(8);
    }

private:
    AnalyzerChannelData* mData;
    std::auto_ptr<ToyotaBeanAnalyzerResults> mResults;
    Channel mChannel;
    U32 mBitTime;

    // We start at the start bit, which is a single high bit
    bool mLastBit = true;
    size_t mBitsInSequence = 1;
};

void ToyotaBeanAnalyzer::WorkerThread()
{
    mSampleRateHz = GetSampleRate();

    mSerial = GetAnalyzerChannelData(mSettings->mInputChannel);

    U32 samples_per_bit = mSampleRateHz / mSettings->mBitRate;
    U32 samples_to_center_of_first_data_bit = U32(1.5 * double(mSampleRateHz) / double(mSettings->mBitRate));
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

    while (true)
    {
        // Find the first rising edge
        while (mSerial->GetBitState() != BIT_HIGH)
            mSerial->AdvanceToNextEdge();

        auto frameStartSample = mSerial->GetSampleNumber();

        // Advance to the center of the first bit after the start bit
        mSerial->Advance(samples_to_center_of_first_data_bit);

        BitUnstuffer bits(mSerial, mResults, mSettings->mInputChannel, samples_per_bit);

        // Priority
        uint8_t pri = bits.ReadBits(4);
        uint8_t ml = bits.ReadBits(4);
        uint8_t dstId = bits.ReadByte();
        uint8_t mesId = bits.ReadByte();
        uint8_t data[11];

        // Actual number of data bytes is (ml - 3 for dstId, mesId, crc)
        _ASSERT(ml > 3 && ml <= 14);
        size_t dataBytes = ml - 3;
        for (size_t i = 0; i < dataBytes; i++)
        {
            data[i] = bits.ReadByte();
        }

        uint8_t crc = bits.ReadByte();

        // TODO: is there an always-zero bit here?

        uint8_t rsp = bits.ReadBits(2);
        uint8_t eom = bits.ReadBits(8);

        // We have a frame to save
        FrameV2 frame;
        frame.AddInteger("PRI", pri);
        frame.AddInteger("ML", ml);
        frame.AddInteger("DST-ID", dstId);
        frame.AddInteger("MES-ID", mesId);

        mResults->AddFrameV2(frame, "bean", frameStartSample, mSerial->GetSampleNumber());
        mResults->CommitResults();
        ReportProgress(mSerial->GetSampleNumber());
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