using UnrealBuildTool;

public class FlexNetworkPCG : ModuleRules
{
	public FlexNetworkPCG(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		PublicDependencyModuleNames.AddRange(new[] { "Core", "CoreUObject", "Engine", "PCG", "GeometryCore", "FlexNetworkRuntime" });
	}
}
