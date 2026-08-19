using UnrealBuildTool;

// Converts FlexNetwork's authoritative segment/lane/connector graph into ZoneGraph shapes and
// immediately-baked AZoneGraphData, then refreshes the MassTraffic/MassCrowd lane caches.
public class FlexNetworkZoneGraph : ModuleRules
{
	public FlexNetworkZoneGraph(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"FlexNetworkRuntime",
			"ZoneGraph",
			"ZoneGraphAnnotations",
			"MassCrowd",
			"MassTraffic"
		});

		if (Target.bBuildEditor)
		{
			PrivateDependencyModuleNames.AddRange(new[] { "AssetRegistry", "UnrealEd" });
		}
	}
}
