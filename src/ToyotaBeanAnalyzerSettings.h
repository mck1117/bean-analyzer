#ifndef TOYOTABEAN_ANALYZER_SETTINGS
#define TOYOTABEAN_ANALYZER_SETTINGS

#include <AnalyzerSettings.h>
#include <AnalyzerTypes.h>

class ToyotaBeanAnalyzerSettings : public AnalyzerSettings
{
public:
	ToyotaBeanAnalyzerSettings();
	virtual ~ToyotaBeanAnalyzerSettings();

	virtual bool SetSettingsFromInterfaces();
	void UpdateInterfacesFromSettings();
	virtual void LoadSettings( const char* settings );
	virtual const char* SaveSettings();

	
	Channel mInputChannel;
	U32 mBitRate;

protected:
	std::auto_ptr< AnalyzerSettingInterfaceChannel >	mInputChannelInterface;
	std::auto_ptr< AnalyzerSettingInterfaceInteger >	mBitRateInterface;
};

#endif //TOYOTABEAN_ANALYZER_SETTINGS
