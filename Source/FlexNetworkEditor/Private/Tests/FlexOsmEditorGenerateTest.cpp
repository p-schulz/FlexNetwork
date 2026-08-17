#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Osm/OsmDataAsset.h"
#include "Osm/OsmXmlParser.h"
#include "Osm/FlexOsmGraphBuilder.h"
#include "FlexNetworkSubsystem.h"
#include "FlexNetworkMeshActor.h"
#include "FlexNetworkAssetUtils.h"
#include "RoadTypeProfile.h"
#include "ProceduralMeshComponent.h"
#include "Editor.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Misc/Paths.h"

// Reproduces UFlexNetworkEdModeSettings::GenerateRoadsFromOsm's exact code path -- including the
// REAL asset-saving profile resolver (FlexNetworkAssetUtils::CreateRoadTypeProfileAsset), not the
// transient in-memory one FlexNetwork.Osm.BuildGraph/RealFileSmokeTest use -- and additionally
// verifies actual mesh geometry gets applied to the world, not just graph data. Exists to isolate
// whether a report of "OSM import produces no visible roads" is a graph-building problem (which
// the other Osm tests would already have caught) or specific to the editor's asset-saving/mesh
// pipeline downstream of it.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexOsmEditorGenerateTest, "FlexNetwork.Osm.EditorGenerateProducesVisibleMesh", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FFlexOsmEditorGenerateTest::RunTest(const FString& Parameters)
{
	const FString FilePath = FPaths::Combine(FPaths::ProjectDir(), TEXT("Small.osm"));
	if (!FPaths::FileExists(FilePath))
	{
		AddInfo(FString::Printf(TEXT("Skipping: '%s' not present."), *FilePath));
		return true;
	}

	if (!GEditor)
	{
		AddError(TEXT("Test requires a running editor context (GEditor)."));
		return false;
	}
	UWorld* World = GEditor->GetEditorWorldContext().World();
	if (!TestNotNull(TEXT("Editor world available"), World))
	{
		return false;
	}
	UFlexNetworkSubsystem* Subsystem = World->GetSubsystem<UFlexNetworkSubsystem>();
	if (!TestNotNull(TEXT("Subsystem available"), Subsystem))
	{
		return false;
	}

	UOsmDataAsset* Asset = NewObject<UOsmDataAsset>(GetTransientPackage());
	TArray<FString> ParseWarnings;
	if (!TestTrue(TEXT("Real .osm file parses"), FOsmXmlParser::ParseFile(FilePath, *Asset, &ParseWarnings)))
	{
		return false;
	}

	FFlexOsmImportSettings Settings;

	// The real resolver: creates+saves an actual URoadTypeProfile asset, exactly like the editor
	// button does -- if profile *asset saving* is what's silently failing, this is what surfaces it.
	TMap<FString, URoadTypeProfile*> ProfileCache;
	int32 NumProfileCreateFailures = 0;
	const FFlexOsmGraphBuilder::FImportResult Result = FFlexOsmGraphBuilder::BuildFromOsm(*Subsystem, *Asset, Settings,
		[&ProfileCache, &NumProfileCreateFailures](const FFlexOsmGraphBuilder::FLaneSignature& Signature) -> URoadTypeProfile*
		{
			const FString Key = Signature.ToKey();
			if (URoadTypeProfile** Existing = ProfileCache.Find(Key))
			{
				return *Existing;
			}
			URoadTypeProfile* NewProfile = FlexNetworkAssetUtils::CreateRoadTypeProfileAsset(
				TEXT("/FlexNetwork/Profiles/OSMTest"),
				TEXT("DA_OSMTest_") + Key,
				[&Signature](URoadTypeProfile& Profile) { FFlexOsmGraphBuilder::ConfigureProfileFromLaneSignature(Profile, Signature); });
			if (!NewProfile)
			{
				++NumProfileCreateFailures;
			}
			else
			{
				ProfileCache.Add(Key, NewProfile);
			}
			return NewProfile;
		});

	for (const FString& Warning : Result.Warnings)
	{
		AddWarning(Warning);
	}

	AddInfo(FString::Printf(TEXT("Import result: %d way(s), %d node(s), %d segment(s), %d profile(s), %d profile-creation failure(s)."),
		Result.NumWaysImported, Result.NumNodesCreated, Result.NumSegmentsCreated, Result.NumDistinctLaneSignatures, NumProfileCreateFailures));

	TestEqual(TEXT("No profile asset creation failures"), NumProfileCreateFailures, 0);
	TestTrue(TEXT("At least one way imported"), Result.NumWaysImported > 0);
	TestTrue(TEXT("At least one segment created"), Result.NumSegmentsCreated > 0);

	// Now check the actual visible-mesh side: did AFlexNetworkMeshActor get spawned, and does at
	// least one of its UProceduralMeshComponent children have real section 0 (roadway) geometry?
	AFlexNetworkMeshActor* MeshActor = Subsystem->GetMeshActor();
	if (!TestNotNull(TEXT("AFlexNetworkMeshActor was spawned"), MeshActor))
	{
		return false;
	}

	int32 NumProcMeshComponents = 0;
	int32 NumComponentsWithGeometry = 0;
	TArray<UProceduralMeshComponent*> ProcMeshComponents;
	MeshActor->GetComponents<UProceduralMeshComponent>(ProcMeshComponents);
	for (UProceduralMeshComponent* Comp : ProcMeshComponents)
	{
		++NumProcMeshComponents;
		if (FProcMeshSection* Section = Comp->GetProcMeshSection(0))
		{
			if (Section->ProcVertexBuffer.Num() > 0)
			{
				++NumComponentsWithGeometry;
			}
		}
	}

	AddInfo(FString::Printf(TEXT("Mesh actor has %d UProceduralMeshComponent child(ren), %d with non-empty section 0 geometry."), NumProcMeshComponents, NumComponentsWithGeometry));
	TestTrue(TEXT("Mesh actor has at least one procedural mesh component"), NumProcMeshComponents > 0);
	TestTrue(TEXT("At least one component actually has roadway geometry applied"), NumComponentsWithGeometry > 0);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
