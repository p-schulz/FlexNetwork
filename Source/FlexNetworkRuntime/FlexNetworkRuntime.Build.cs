using UnrealBuildTool;

// Graph data model, curve/geometry math, mesh generation, intersection + lane-connector
// construction, terrain conforming, and the query/mutation API the external traffic
// simulation consumes. Deliberately free of UnrealEd/Slate outside WITH_EDITOR guards so it
// stays packageable into shipping/server builds; FlexNetworkEditor depends on this module,
// never the other way around.
public class FlexNetworkRuntime : ModuleRules
{
	public FlexNetworkRuntime(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"DeveloperSettings",
			"ProceduralMeshComponent",
			"PCG",
			// FOsmXmlParser (Osm/OsmXmlParser.cpp) -- a plain Runtime module, not editor-only, so
			// OSM data can be parsed outside the Content Browser import flow too.
			"XmlParser"
		});

		PrivateDependencyModuleNames.AddRange(new[]
		{
			"RenderCore",
			"Landscape",
			"GeometryCore",
			"GeometryAlgorithms"
		});

		if (Target.bBuildEditor)
		{
			// LandscapeEdit.h (FLandscapeEditDataInterface, used by FlexLandscapeConformer's
			// WITH_EDITOR-gated heightmap painting) transitively includes InstancedFoliageActor.h.
			PrivateDependencyModuleNames.Add("Foliage");

			// FlexNetworkPipelineTests.cpp (WITH_DEV_AUTOMATION_TESTS-gated, never compiled into
			// shipping) needs GEditor/UEditorEngine to get a live UWorld to exercise the subsystem
			// against -- editor-only, but tests aren't part of what "packages into shipping" means.
			PrivateDependencyModuleNames.Add("UnrealEd");
		}
	}
}
