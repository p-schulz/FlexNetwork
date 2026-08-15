#include "FlexNetworkSettings.h"

UFlexNetworkSettings::UFlexNetworkSettings()
{
	CategoryName = TEXT("Plugins");
}

#if WITH_EDITOR
FText UFlexNetworkSettings::GetSectionText() const
{
	return NSLOCTEXT("FlexNetwork", "SettingsSectionText", "Flex Network");
}
#endif
