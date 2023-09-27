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

    void WriteByte(uint8_t data, U32 samples_per_bit)
    {
        for (size_t i = 0; i < 8; i++)
        {
            Write((data & 0x80) ? true : false, samples_per_bit);
            data = data << 1;
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
    mSerialSimulationData.TransitionIfNeeded(BIT_HIGH);
    mSerialSimulationData.Advance(samples_per_bit);

    BitStuffer bs(mSerialSimulationData);

    uint8_t pri = rand() % 0xF;
    uint8_t dataLength = rand() % 10 + 1;

    uint8_t firstByte = pri << 4 | (dataLength + 3);

    bs.WriteByte(firstByte, samples_per_bit);

    // DST-ID
    bs.WriteByte(rand() & 0xFF, samples_per_bit);
    // MES-ID
    bs.WriteByte(rand() & 0xFF, samples_per_bit);

    // 1-11 data bytes
    for (size_t i = 0; i < dataLength; i++)
    {
        bs.WriteByte(rand() & 0xFF, samples_per_bit);
    }

    // CRC byte (todo: real CRC!)
    bs.WriteByte(rand() & 0xFF, samples_per_bit);

    // End of frame (this doesn't get stuffed!)
    mSerialSimulationData.TransitionIfNeeded(BIT_LOW);
    mSerialSimulationData.Advance(samples_per_bit);
    mSerialSimulationData.TransitionIfNeeded(BIT_HIGH);
    mSerialSimulationData.Advance(6 * samples_per_bit);
    mSerialSimulationData.TransitionIfNeeded(BIT_LOW);
    mSerialSimulationData.Advance(samples_per_bit);

    // ACK bits: 10 = ACK, 01 = NAK
    mSerialSimulationData.TransitionIfNeeded((rand() & 1) == 0 ? BIT_HIGH : BIT_LOW);
    mSerialSimulationData.Advance(samples_per_bit);
    mSerialSimulationData.TransitionIfNeeded((rand() & 1) == 0 ? BIT_HIGH : BIT_LOW);
    mSerialSimulationData.Advance(samples_per_bit);

    mSerialSimulationData.TransitionIfNeeded(BIT_LOW);
    mSerialSimulationData.Advance(10 * samples_per_bit);
}
