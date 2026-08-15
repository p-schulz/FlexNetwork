#include "FlexNetworkEdModeToolkit.h"
#include "FlexNetworkEdMode.h"
#include "FlexNetworkEdModeSettings.h"
#include "PropertyEditorModule.h"
#include "Modules/ModuleManager.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"

FFlexNetworkEdModeToolkit::FFlexNetworkEdModeToolkit()
{
}

void FFlexNetworkEdModeToolkit::Init(const TSharedPtr<IToolkitHost>& InitToolkitHost)
{
	FModeToolkit::Init(InitToolkitHost);

	FPropertyEditorModule& PropertyEditorModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
	FDetailsViewArgs DetailsArgs;
	DetailsArgs.bAllowSearch = false;
	DetailsArgs.NameAreaSettings = FDetailsViewArgs::HideNameArea;
	const TSharedRef<IDetailsView> RoadSettingsView = PropertyEditorModule.CreateDetailView(DetailsArgs);

	if (FFlexNetworkEdMode* Mode = static_cast<FFlexNetworkEdMode*>(GetEditorMode()))
	{
		RoadSettingsView->SetObject(Mode->GetOrCreateModeSettings());
	}

	ToolkitWidget = SNew(SVerticalBox)
		+ SVerticalBox::Slot()
		.AutoHeight()
		.Padding(4.f)
		[
			SNew(STextBlock)
			.AutoWrapText(true)
			.Text(NSLOCTEXT("FlexNetwork", "Instructions",
				"Click-drag in the viewport to draw a road on the ground plane. "
				"Dragging from or onto an existing node/road snaps and connects to it; "
				"dragging across an existing road splits it automatically. "
				"Green preview = valid, red = invalid (too short, too sharp, or self-intersecting)."))
		]
		+ SVerticalBox::Slot()
		.FillHeight(1.f)
		[
			RoadSettingsView
		];
}

FText FFlexNetworkEdModeToolkit::GetBaseToolkitName() const
{
	return NSLOCTEXT("FlexNetwork", "ToolkitName", "Flex Network");
}

FEdMode* FFlexNetworkEdModeToolkit::GetEditorMode() const
{
	return FModeToolkit::GetEditorMode();
}
