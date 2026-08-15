#include "FlexNetworkEdModeToolkit.h"
#include "FlexNetworkEdMode.h"
#include "FlexNetworkEdModeSettings.h"
#include "PropertyEditorModule.h"
#include "Modules/ModuleManager.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"
#include "EditorModeManager.h"

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
				"Draw Mode (default): click once to start a road, move the mouse to preview it, "
				"click again to commit -- like the Landscape Splines \"Add Control Point\" tool. "
				"Clicking on/near an existing node or road snaps and connects to it; drawing across "
				"an existing road splits it automatically. Green preview = valid, red = invalid "
				"(too short, too sharp, or self-intersecting). Right-click/Escape cancels.\n\n"
				"Select/Move Mode (toggle off Draw Mode below): click a node to select it (yellow), "
				"then drag the viewport gizmo to move it."))
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
	// FModeToolkit's own default just returns nullptr -- every concrete mode toolkit (Foliage,
	// Landscape, ...) overrides this by looking itself up from the mode manager. Skipping this
	// override is exactly what silently prevented the toolkit from ever registering/showing:
	// GetEditorModeInfo() (and therefore FModeToolkit::OnModeIDChanged's
	// FToolkitManager::RegisterNewToolkit call, which is what makes the panel tab appear) both
	// depend on this resolving to a real mode instead of null.
	//
	// GetEditorModeManager() itself asserts IsHosted() -- this can get called (e.g. f