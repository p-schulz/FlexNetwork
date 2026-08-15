#include "FlexNetworkEditorModule.h"
#include "FlexNetworkEdMode.h"
#include "EditorModeRegistry.h"
#include "Textures/SlateIcon.h"

void FFlexNetworkEditorModule::StartupModule()
{
	FEditorModeRegistry::Get().RegisterMode<FFlexNetworkEdMode>(
		FlexNetworkEdModeId,
		NSLOCTEXT("FlexNetwork", "FlexNetworkEdModeName", "Flex Network"),
		FSlateIcon(),
		/*bVisible=*/ true);
}

void FFlexNetworkEditorModule::ShutdownModule()
{
	FEditorModeRegistry::Get().UnregisterMode(FlexNetworkEdModeId);
}

IMPLEMENT_MODULE(FFlexNetworkEditorModule, FlexNetworkEditor)
