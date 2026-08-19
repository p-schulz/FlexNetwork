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
				"Node Edit Mode (toggle off Draw Mode below): choose Move Node, Rotate Node + "
				"Connected Tangents, or Adjust Tangent Handles. Click a node to select it (yellow). "
				"Move uses the translate gizmo; Rotate changes the node up vector and rotates every "
				"connected endpoint tangent together; Tangent mode shows magenta handles--click one, "
				"then drag its translate gizmo."))
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
	// GetEditorModeManager() itself asserts IsHosted() -- this can get called (e.g. from
	// FModeToolkit's own bookkeeping) after the toolkit host has gone away during editor
	// shutdown, so guard it explicitly rather than crashing on exit.
	if (!IsHosted())
	{
		return nullptr;
	}
	return GetEditorModeManager().GetActiveMode(FlexNetworkEdModeId);
}
