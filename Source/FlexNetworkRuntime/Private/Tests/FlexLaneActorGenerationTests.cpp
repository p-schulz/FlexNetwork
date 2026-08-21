#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "FlexNetworkSubsystem.h"
#include "RoadTypeProfile.h"
#include "Editor.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"

// UFlexNetworkSubsystem::GenerateLaneActors spawns real AActor instances into the world, so
// (like FlexNetworkPipelineTests.cpp) this needs a live editor world rather than the
// no-world/no-compiler-feedback style the pure geometry builder tests use. Uses AActor::StaticClass()
// itself as the spawned class -- AActor is concrete (not Abstract), so it's a valid, minimal,
// dependency-free stand-in for "some Blueprint" here.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexLaneActorGenerationTest, "FlexNetwork.LaneActors.SpawnsAlongLaneAndReplacesOnRegenerate", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FFlexLaneActorGenerationTest::RunTest(const FString& Parameters)
{
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
	if (!TestNotNull(TEXT("UFlexNetworkSubsystem present on the editor world"), Subsystem))
	{
		return false;
	}

	URoadTypeProfile* Profile = NewObject<URoadTypeProfile>(GetTransientPackage());
	FRoadLaneDescriptor Lane;
	Lane.LateralOffset = 0.f;
	Lane.Width = 350.f;
	Lane.Type = EFlexLaneType::Vehicle;
	Lane.Direction = EFlexLaneDirection::Forward;

	FFlexLaneActorSpawnEntry SpawnEntry;
	SpawnEntry.ActorClass = AActor::StaticClass();
	SpawnEntry.SpacingDistance = 500.f;
	SpawnEntry.LaneEdgeOffset = 0.f;
	Lane.LaneActors.Add(SpawnEntry);
	Profile->Lanes.Add(Lane);
	Profile->SidewalkWidth = 0.f;
	Profile->MinTurnRadius = 500.f;
	Profile->MinSegmentLengthOverride = 100.f;

	const FFlexNodeId NodeA = Subsystem->AddNode(FVector(0, 0, 0));
	const FFlexNodeId NodeB = Subsystem->AddNode(FVector(3000, 0, 0));
	const FFlexSegmentId SegmentId = Subsystem->AddSegment(NodeA, NodeB, FVector(1000, 0, 0), FVector(2000, 0, 0), Profile);
	if (!TestTrue(TEXT("Segment created"), SegmentId.IsValid()))
	{
		return false;
	}

	const int32 FirstSpawnCount = Subsystem->GenerateLaneActors();
	TestTrue(TEXT("At least one lane actor was spawned along the segment"), FirstSpawnCount > 0);

	TArray<TWeakObjectPtr<AActor>> FirstBatch;
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		if (It->GetClass() == AActor::StaticClass())
		{
			FirstBatch.Add(*It);
		}
	}
	TestEqual(TEXT("World actor count of the spawned class matches the reported spawn count"), FirstBatch.Num(), FirstSpawnCount);

	const int32 SecondSpawnCount = Subsystem->GenerateLaneActors();
	TestEqual(TEXT("Regenerating with an unchanged graph spawns the same count again"), SecondSpawnCount, FirstSpawnCount);

	int32 StillValidFromFirstBatch = 0;
	for (const TWeakObjectPtr<AActor>& WeakActor : FirstBatch)
	{
		if (WeakActor.IsValid())
		{
			++StillValidFromFirstBatch;
		}
	}
	TestEqual(TEXT("Every actor from the first batch was destroyed before the second batch was spawned"), StillValidFromFirstBatch, 0);

	// Clean up so this test doesn't leak actors into the shared editor world for other tests.
	Subsystem->RemoveSegment(SegmentId);
	Subsystem->RemoveNode(NodeA);
	Subsystem->RemoveNode(NodeB);
	const int32 FinalSpawnCount = Subsystem->GenerateLaneActors();
	TestEqual(TEXT("No lane actors remain once the segment providing them is removed"), FinalSpawnCount, 0);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
