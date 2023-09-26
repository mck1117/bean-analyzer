#ifndef TOYOTABEAN_ANALYZER_H
#define TOYOTABEAN_ANALYZER_H

#include "ToyotaBeanAnalyzerResults.h"
#include "ToyotaBeanSimulationDataGenerator.h"

#include <Analyzer.h>

class ToyotaBeanAnalyzerSettings;
class ANALYZER_EXPORT ToyotaBeanAnalyzer : public Analyzer2
{
public:
    ToyotaBeanAnalyzer();
    virtual ~ToyotaBeanAnalyzer();

    virtual void SetupResults();
    virtual void WorkerThread();

    virtual U32 GenerateSimulationData(
        U64 newest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channels);
    virtual U32 GetMinimumSampleRateHz();

    virtual const char* GetAnalyzerName() const;
    virtual bool NeedsRerun();

protected: // vars
    std::unique_ptr<ToyotaBeanAnalyzerSettings> mSettings;
    std::unique_ptr<ToyotaBeanAnalyzerResults> mResults;
    AnalyzerChannelData* mSerial;

    ToyotaBeanSimulationDataGenerator mSimulationDataGenerator;
    bool mSimulationInitilized;

    // Serial analysis vars:
    U32 mSampleRateHz;
    U32 mStartOfStopBitOffset;
    U32 mEndOfStopBitOffset;

    void WaitFor6LowBits(U32 bitTime);
};

extern "C" ANALYZER_EXPORT const char* __cdecl GetAnalyzerName();
extern "C" ANALYZER_EXPORT Analyzer* __cdecl CreateAnalyzer();
extern "C" ANALYZER_EXPORT void __cdecl DestroyAnalyzer(Analyzer* analyzer);

#endif // TOYOTABEAN_ANALYZER_H
