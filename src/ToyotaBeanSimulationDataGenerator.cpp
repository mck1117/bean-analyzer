#include "ToyotaBeanSimulationDataGenerator.h"

#include "ToyotaBeanAnalyzerSettings.h"

#include <AnalyzerHelpers.h>

ToyotaBeanSimulationDataGenerator::ToyotaBeanSimulationDataGenerator()
    : mSerialText("My first analyzer, woo hoo!"), mStringIndex(0)
{
}

ToyotaBeanSimulationDataGenerator::~ToyotaBeanSimulationDataGenerator() {}

void ToyotaBeanSimulationDataGenerator::Initialize(U32 simulation_sample_rate, ToyotaBeanAnalyzerSettings* settings)
{
    mSimulationSampleRateHz = simulation_sample_rate;
    mSettings = settings;

    mSerialSimulationData.SetChannel(mSettings->mInputChannel);
    mSerialSimulationData.SetSampleRate(simulation_sample_rate);
    mSerialSimulationData.SetInitialBitState(BIT_LOW);
}

U32 ToyotaBeanSimulationDataGenerator::GenerateSimulationData(
    U64 largest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channel)
{
    U64 adjusted_largest_sample_requested =
        AnalyzerHelpers::AdjustSimulationTargetSample(largest_sample_requested, sample_rate, mSimulationSampleRateHz);

    while (mSerialSimulationData.GetCurrentSampleNumber() < adjusted_largest_sample_requested)
    {
        CreateSerialByte();
    }

    *simulation_channel = &mSerialSimulationData;
    return 1;
}

struct BitStuffer
{
    BitStuffer(SimulationChannelDescriptor& data) : mData(data) { }

    void Write(bool b, U32 samples_per_bit)
    {
        mData.TransitionIfNeeded(b ? BIT_HIGH : BIT_LOW);
        mData.Advance(samples_per_bit);
        
        if (b != mLastBit)
        {
            mLastBit = b;
            mBitCount = 1;
        }
        else
        {
            mBitCount++;
        }
        
        if (mBitCount == 5)
        {
            mLastBit = !b;
            mBitCount = 1;

            mData.TransitionIfNeeded(b ? BIT_LOW : BIT_HIGH);
            mData.Advance(samples_per_bit);
        }
    }

private:
    SimulationChannelDescriptor& mData;

    bool mLastBit = true;
    size_t mBitCount = 1;
};

void ToyotaBeanSimulationDataGenerator::CreateSerialByte()
{
    U32 samples_per_bit = mSimulationSampleRateHz / mSettings->mBitRate;

    mSerialSimulationData.Advance(10 * samples_per_bit);

    // start bit
    mSerialSimulationData.TransitionIfNeeded(BIT_LOW);
    mSerialSimulationData.Advance(samples_per_bit);

    BitStuffer bs(mSerialSimulationData);

    // 64 random data bits
    for (size_t i = 0; i < 64; i++)
    {
        bs.Write((rand() & 1) == 0, samples_per_bit);
        // bs.Write(false, samples_per_bit);
        // bs.Write(true, samples_per_bit);
    }

    mSerialSimulationData.TransitionIfNeeded(BIT_LOW);
    mSerialSimulationData.Advance(10 * samples_per_bit);
}
