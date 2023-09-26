#include "ToyotaBeanAnalyzerSettings.h"

#include <AnalyzerHelpers.h>

ToyotaBeanAnalyzerSettings::ToyotaBeanAnalyzerSettings() : mInputChannel(UNDEFINED_CHANNEL), mBitRate(9600)
{
    mInputChannelInterface.reset(new AnalyzerSettingInterfaceChannel());
    mInputChannelInterface->SetTitleAndTooltip("Serial", "Standard ToyotaBean");
    mInputChannelInterface->SetChannel(mInputChannel);

    mBitRateInterface.reset(new AnalyzerSettingInterfaceInteger());
    mBitRateInterface->SetTitleAndTooltip("Bit Rate (Bits/S)", "Specify the bit rate in bits per second.");
    mBitRateInterface->SetMax(6000000);
    mBitRateInterface->SetMin(1);
    mBitRateInterface->SetInteger(mBitRate);

    AddInterface(mInputChannelInterface.get());
    AddInterface(mBitRateInterface.get());

    AddExportOption(0, "Export as text/csv file");
    AddExportExtension(0, "text", "txt");
    AddExportExtension(0, "csv", "csv");

    ClearChannels();
    AddChannel(mInputChannel, "Serial", false);
}

ToyotaBeanAnalyzerSettings::~ToyotaBeanAnalyzerSettings() {}

bool ToyotaBeanAnalyzerSettings::SetSettingsFromInterfaces()
{
    mInputChannel = mInputChannelInterface->GetChannel();
    mBitRate = mBitRateInterface->GetInteger();

    ClearChannels();
    AddChannel(mInputChannel, "ToyotaBean", true);

    return true;
}

void ToyotaBeanAnalyzerSettings::UpdateInterfacesFromSettings()
{
    mInputChannelInterface->SetChannel(mInputChannel);
    mBitRateInterface->SetInteger(mBitRate);
}

void ToyotaBeanAnalyzerSettings::LoadSettings(const char* settings)
{
    SimpleArchive text_archive;
    text_archive.SetString(settings);

    text_archive >> mInputChannel;
    text_archive >> mBitRate;

    ClearChannels();
    AddChannel(mInputChannel, "ToyotaBean", true);

    UpdateInterfacesFromSettings();
}

const char* ToyotaBeanAnalyzerSettings::SaveSettings()
{
    SimpleArchive text_archive;

    text_archive << mInputChannel;
    text_archive << mBitRate;

    return SetReturnString(text_archive.GetString());
}
