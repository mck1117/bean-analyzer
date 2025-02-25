#include "ToyotaBeanAnalyzerResults.h"

#include "ToyotaBeanAnalyzer.h"
#include "ToyotaBeanAnalyzerSettings.h"

#include <AnalyzerHelpers.h>
#include <fstream>
#include <iostream>
#include <sstream>

ToyotaBeanAnalyzerResults::ToyotaBeanAnalyzerResults(ToyotaBeanAnalyzer* analyzer, ToyotaBeanAnalyzerSettings* settings)
    : AnalyzerResults(), mSettings(settings), mAnalyzer(analyzer)
{
}

ToyotaBeanAnalyzerResults::~ToyotaBeanAnalyzerResults() {}

void ToyotaBeanAnalyzerResults::GenerateBubbleText(U64 frame_index, Channel& channel, DisplayBase display_base)
{
    ClearResultStrings();
    Frame frame = GetFrame(frame_index);

    std::stringstream ss;

    switch (frame.mType)
    {
    case 1: ss << "PRI: "; break;
    case 2: ss << "ML: "; break;
    case 3: ss << "DST-ID: "; break;
    case 4: ss << "MES-ID: "; break;
    case 5: break; // no prefix for data
    case 6: ss << "CRC OK"; break;
    case 7: ss << "EOM"; break;
    case 8: ss << "RSP: "; break;
    case 9: ss << "CRC BAD: "; break;
    }

    if (frame.mType != 7 && frame.mType != 6)
    {
        char number_str[128];
        AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 8, number_str, 128);
        ss << number_str;
    }

    AddResultString(ss.str().c_str());
}

void ToyotaBeanAnalyzerResults::GenerateExportFile(const char* file, DisplayBase display_base, U32 export_type_user_id)
{
    std::ofstream file_stream(file, std::ios::out);

    U64 trigger_sample = mAnalyzer->GetTriggerSample();
    U32 sample_rate = mAnalyzer->GetSampleRate();

    file_stream << "Time [s],Value" << std::endl;

    U64 num_frames = GetNumFrames();
    for (U32 i = 0; i < num_frames; i++)
    {
        Frame frame = GetFrame(i);

        char time_str[128];
        AnalyzerHelpers::GetTimeString(frame.mStartingSampleInclusive, trigger_sample, sample_rate, time_str, 128);

        char number_str[128];
        AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 8, number_str, 128);

        file_stream << time_str << "," << number_str << std::endl;

        if (UpdateExportProgressAndCheckForCancel(i, num_frames) == true)
        {
            file_stream.close();
            return;
        }
    }

    file_stream.close();
}

void ToyotaBeanAnalyzerResults::GenerateFrameTabularText(U64 frame_index, DisplayBase display_base)
{
#ifdef SUPPORTS_PROTOCOL_SEARCH
    Frame frame = GetFrame(frame_index);
    ClearTabularText();

    char number_str[128];
    AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 8, number_str, 128);
    AddTabularText(number_str);
#endif
}

void ToyotaBeanAnalyzerResults::GeneratePacketTabularText(U64 packet_id, DisplayBase display_base)
{
    // not supported
}

void ToyotaBeanAnalyzerResults::GenerateTransactionTabularText(U64 transaction_id, DisplayBase display_base)
{
    // not supported
}