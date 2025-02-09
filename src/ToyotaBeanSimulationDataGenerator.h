#ifndef TOYOTABEAN_SIMULATION_DATA_GENERATOR
#define TOYOTABEAN_SIMULATION_DATA_GENERATOR

#include <SimulationChannelDescriptor.h>
#include <string>
class ToyotaBeanAnalyzerSettings;

class ToyotaBeanSimulationDataGenerator
{
public:
    ToyotaBeanSimulationDataGenerator();
    ~ToyotaBeanSimulationDataGenerator();

    void Initialize(U32 simulation_sample_rate, ToyotaBeanAnalyzerSettings* settings);
    U32 GenerateSimulationData(
        U64 newest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channel);

protected:
    ToyotaBeanAnalyzerSettings* mSettings;
    U32 mSimulationSampleRateHz;

protected:
    void CreateSerialByte();
    std::string mSerialText;
    U32 mStringIndex;

    SimulationChannelDescriptor mSerialSimulationData;
};
#endif // TOYOTABEAN_SIMULATION_DATA_GENERATOR