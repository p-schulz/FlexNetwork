using UnrealBuildTool;

// The interactive drag-to-draw road editor mode (legacy FEdMode -- still fully supported in
// UE5.8 and a much better fit here than the modern UEdMode/Interactive-Tools-Framework stack,
// which routes input through UInteractiveTool + Behaviors/Builders rather than exposing direct
// InputKey/Render overrides; FEdMode gives us MouseMove/InputKey/StartTracking/EndTracking/
// Render directly, which is exactly the shape a click-drag-drop authoring tool needs), ghost
// preview rendering, undo/redo, and a minimal details toolkit. Contains no algorithmic logic --
// everything here calls into FlexNetworkRuntime's UFlexNetworkSubsystem.
public class FlexNetworkEditor : ModuleRules
{
	public FlexNetworkEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"UnrealEd",
			"FlexNetworkRuntime"
		});

		PrivateDependencyModuleNames.AddRange(new[]
		{
			"Slate",
			"SlateCore",
			"InputCore",
			"ToolMenus",
			"PropertyEditor",
			// FEditorModeInfo/FToolkitManager live here, not in UnrealEd itself.
			"EditorFramework",
			// FlexNetworkCreateDefaultProfilesCommand.cpp saves the generated sample profile assets.
			"AssetRegistry"
		});
	}
}
