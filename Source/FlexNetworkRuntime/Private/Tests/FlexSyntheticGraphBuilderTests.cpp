#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Synthetic/FlexSyntheticGraphBuilder.h"
#include "FlexNetworkSubsystem.h"
#include "RoadTypeProfile.h"
#include "Editor.h"
#include "Engine/World.h"

// SampleFieldDirection's doubled-angle blend should reduce to the single region's own direction
// when only one region is in play -- a basic sanity check on the tensor math before trusting it
// inside the tracer. Pure C++, no world needed.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexSyntheticGraphBuilderSingleRegionDirectionTest, "FlexNetwork.Synthetic.SingleRegionDirectionMatchesGridAngle", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexSyntheticGraphBuilderSingleRegionDirectionTest::RunTest(const FString& Parameters)
{
	FFlexSyntheticFieldRegion GridRegion;
	GridRegion.Kind = EFlexSyntheticFieldKind::Grid;
	GridRegion.Center = FVector2d(0.0, 0.0);
	GridRegion.GridAngleDegrees = 45.0;
	GridRegion.DecayRadius = 10000.0;

	const TArray<FFlexSyntheticFieldRegion> Regions = { GridRegion };
	const FVector2d Direction = FlexSyntheticGraphBuilder::SampleFieldDirection(Regions, FVector2d(100.0, 100.0));

	const double ExpectedAngleRadians = FMath::DegreesToRadians(45.0);
	const FVector2d Expected(FMath::Cos(ExpectedAngleRadians), FMath::Sin(ExpectedAngleRadians));

	// The doubled-angle recovery is only defined up to a 180-degree sign flip (Theta and
	// Theta+180 encode the same undirected line), so either Expected or -Expected is a pass.
	const bool bMatchesEitherSign = Direction.Equals(Expected, 1e-6) || Direction.Equals(-Expected, 1e-6);
	TestTrue(TEXT("A single Grid region's sampled direction matches its own GridAngleDegrees (up to a 180-degree sign flip)"), bMatchesEitherSign);

	return true;
}

namespace FlexSyntheticGraphBuilderTestHelpers
{
	URoadTypeProfile* MakeTestProfile()
	{
		URoadTypeProfile* Profile = NewObject<URoadTypeProfile>(GetTransientPackage());
		FRoadLaneDescriptor Forward;
		Forward.LateralOffset = 175.f;
		Forward.Width = 350.f;
		Forward.Type = EFlexLaneType::Vehicle;
		Forward.Direction = EFlexLaneDirection::Forward;
		Profile->Lanes.Add(Forward);
		FRoadLaneDescriptor Backward;
		Backward.LateralOffset = -175.f;
		Backward.Width = 350.f;
		Backward.Type = EFlexLaneType::Vehicle;
		Backward.Direction = EFlexLaneDirection::Backward;
		Profile->Lanes.Add(Backward);
		Profile->SidewalkWidth = 200.f;
		Profile->MinTurnRadius = 200.f; // generous -- this test isn't about turn-radius rejection
		Profile->MinSegmentLengthOverride = 50.f;
		return Profile;
	}
}

// End-to-end: generate a small synthetic network and confirm it actually lands in
// UFlexNetworkSubsystem's real graph -- the part FlexSyntheticSpike (Phase 0) deliberately never
// touched. Needs a live editor world (like FlexNetworkPipelineTests.cpp), so this can't be a pure
// unit test the way the direction-sampling one above is.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexSyntheticGraphBuilderEndToEndTest, "FlexNetwork.Synthetic.GeneratesIntoFlexNetworkGraph", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FFlexSyntheticGraphBuilderEndToEndTest::RunTest(const FString& Parameters)
{
	using namespace FlexSyntheticGraphBuilderTestHelpers;

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

	// Record what's already in the graph so this test only cleans up what it adds -- doesn't
	// assume it's running against an empty world.
	TSet<FFlexNodeId> PreExistingNodes;
	for (const TPair<FFlexNodeId, FFlexRoadNode>& Pair : Subsystem->GetAllNodes())
	{
		PreExistingNodes.Add(Pair.Key);
	}
	TSet<FFlexSegmentId> PreExistingSegments;
	for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Subsystem->GetAllSegments())
	{
		PreExistingSegments.Add(Pair.Key);
	}

	URoadTypeProfile* ArterialProfile = MakeTestProfile();
	URoadTypeProfile* LocalProfile = MakeTestProfile();

	FFlexSyntheticGenerationSettings Settings;
	Settings.DomainMin = FVector2d(-5000.0, -5000.0);
	Settings.DomainMax = FVector2d(5000.0, 5000.0);
	Settings.StepSize = 150.0;
	Settings.MaxStepsPerStreamline = 40;
	Settings.NumMajorSeeds = 2;
	Settings.MinorSeedSpacing = 2000.0;
	Settings.MaxRawSegmentsToInsert = 2000;

	FFlexSyntheticFieldRegion RadialRegion;
	RadialRegion.Kind = EFlexSyntheticFieldKind::Radial;
	RadialRegion.Center = FVector2d(0.0, 0.0);
	RadialRegion.DecayRadius = 4000.0;
	Settings.FieldRegions.Add(RadialRegion);

	FFlexSyntheticFieldRegion GridRegion;
	GridRegion.Kind = EFlexSyntheticFieldKind::Grid;
	GridRegion.Center = FVector2d(2500.0, 2500.0);
	GridRegion.GridAngleDegrees = 20.0;
	GridRegion.DecayRadius = 4000.0;
	Settings.FieldRegions.Add(GridRegion);

	const FFlexSyntheticGenerationResult Result = FlexSyntheticGraphBuilder::GenerateSyntheticNetwork(
		*Subsystem, Settings,
		[ArterialProfile, LocalProfile](EFlexSyntheticRoadTier Tier) -> URoadTypeProfile*
		{
			return Tier == EFlexSyntheticRoadTier::Arterial ? ArterialProfile : LocalProfile;
		});

	TestFalse(TEXT("Planarization did not report self-intersections"), Result.bHasSelfIntersectionsAfterPlanarization);
	TestTrue(TEXT("At least one node was created"), Result.NumNodesCreated > 0);
	TestTrue(TEXT("At least one segment was created"), Result.NumSegmentsCreated > 0);

	// Confirm the nodes/segments are actually reachable through the subsystem's own public API --
	// the point of Phase 1 over Phase 0's spike is that this lands in the real graph, not just a
	// debug-only result struct.
	int32 NewNodeCount = 0;
	TArray<FFlexNodeId> NewNodeIds;
	for (const TPair<FFlexNodeId, FFlexRoadNode>& Pair : Subsystem->GetAllNodes())
	{
		if (!PreExistingNodes.Contains(Pair.Key))
		{
			++NewNodeCount;
			NewNodeIds.Add(Pair.Key);
		}
	}
	TestEqual(TEXT("New node count in the subsystem matches the reported count"), NewNodeCount, Result.NumNodesCreated);

	int32 NewSegmentCount = 0;
	TArray<FFlexSegmentId> NewSegmentIds;
	for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Subsystem->GetAllSegments())
	{
		if (!PreExistingSegments.Contains(Pair.Key))
		{
			++NewSegmentCount;
			NewSegmentIds.Add(Pair.Key);
		}
	}
	TestEqual(TEXT("New segment count in the subsystem matches the reported count"), NewSegmentCount, Result.NumSegmentsCreated);

	AddInfo(FString::Printf(TEXT("%d streamlines -> %d nodes / %d segments in %.4f s"),
		Result.NumStreamlinesTraced, Result.NumNodesCreated, Result.NumSegmentsCreated, Result.GenerationSeconds));

	// Clean up so this test doesn't leave a permanent network behind in the shared editor world.
	for (const FFlexSegmentId& SegmentId : NewSegmentIds)
	{
		Subsystem->RemoveSegment(SegmentId);
	}
	for (const FFlexNodeId& NodeId : NewNodeIds)
	{
		Subsystem->RemoveNode(NodeId);
	}

	return true;
}

// A meaningfully larger network than any of the manual toolkit testing has used so far -- answers
// the two remaining Phase 1 unknowns from the plan document: does FFlexIntersectionBuilder survive
// the actual intersection-angle/degree distribution this generator produces (exercised as a side
// effect of AddSegment inside BeginBatchUpdate/EndBatchUpdate during generation itself), and what
// does real mesh-generation performance look like at a bigger segment count (exercised explicitly
// via RebuildAllNetworkGeometry() afterward, which is the same full-rebuild path the toolkit's
// "Rebuild All Network Geometry" button calls). Not asserted against a hard time budget -- logs
// both timings via AddInfo so a human reading the automation report can judge "acceptable."
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexSyntheticGraphBuilderStressTest, "FlexNetwork.Synthetic.StressTestLargerNetwork", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FFlexSyntheticGraphBuilderStressTest::RunTest(const FString& Parameters)
{
	using namespace FlexSyntheticGraphBuilderTestHelpers;

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

	TSet<FFlexNodeId> PreExistingNodes;
	for (const TPair<FFlexNodeId, FFlexRoadNode>& Pair : Subsystem->GetAllNodes())
	{
		PreExistingNodes.Add(Pair.Key);
	}
	TSet<FFlexSegmentId> PreExistingSegments;
	for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Subsystem->GetAllSegments())
	{
		PreExistingSegments.Add(Pair.Key);
	}

	URoadTypeProfile* ArterialProfile = MakeTestProfile();
	URoadTypeProfile* LocalProfile = MakeTestProfile();

	FFlexSyntheticGenerationSettings Settings;
	Settings.DomainMin = FVector2d(-20000.0, -20000.0);
	Settings.DomainMax = FVector2d(20000.0, 20000.0);
	Settings.StepSize = 150.0;
	Settings.MaxStepsPerStreamline = 150;
	Settings.NumMajorSeeds = 8;
	Settings.MinorSeedSpacing = 1200.0;
	Settings.MinStreamlineSeparation = 600.0;
	Settings.MaxRawSegmentsToInsert = 15000;

	FFlexSyntheticFieldRegion RadialRegion;
	RadialRegion.Kind = EFlexSyntheticFieldKind::Radial;
	RadialRegion.Center = FVector2d(0.0, 0.0);
	RadialRegion.DecayRadius = 15000.0;
	Settings.FieldRegions.Add(RadialRegion);

	FFlexSyntheticFieldRegion GridRegion;
	GridRegion.Kind = EFlexSyntheticFieldKind::Grid;
	GridRegion.Center = FVector2d(10000.0, 10000.0);
	GridRegion.GridAngleDegrees = 20.0;
	GridRegion.DecayRadius = 15000.0;
	Settings.FieldRegions.Add(GridRegion);

	const FFlexSyntheticGenerationResult Result = FlexSyntheticGraphBuilder::GenerateSyntheticNetwork(
		*Subsystem, Settings,
		[ArterialProfile, LocalProfile](EFlexSyntheticRoadTier Tier) -> URoadTypeProfile*
		{
			return Tier == EFlexSyntheticRoadTier::Arterial ? ArterialProfile : LocalProfile;
		});

	TestFalse(TEXT("Planarization did not report self-intersections at this larger scale"), Result.bHasSelfIntersectionsAfterPlanarization);
	TestTrue(TEXT("A meaningfully larger network was actually created"), Result.NumSegmentsCreated > 50);

	const double RebuildStartSeconds = FPlatformTime::Seconds();
	Subsystem->RebuildAllNetworkGeometry();
	const double RebuildSeconds = FPlatformTime::Seconds() - RebuildStartSeconds;

	AddInfo(FString::Printf(TEXT("Stress test: %d streamlines -> %d nodes / %d segments, generation %.3f s, full mesh rebuild %.3f s (budget exceeded: %s)"),
		Result.NumStreamlinesTraced, Result.NumNodesCreated, Result.NumSegmentsCreated, Result.GenerationSeconds, RebuildSeconds,
		Result.bSegmentBudgetExceeded ? TEXT("yes") : TEXT("no")));

	// Clean up.
	TArray<FFlexSegmentId> NewSegmentIds;
	for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Subsystem->GetAllSegments())
	{
		if (!PreExistingSegments.Contains(Pair.Key))
		{
			NewSegmentIds.Add(Pair.Key);
		}
	}
	TArray<FFlexNodeId> NewNodeIds;
	for (const TPair<FFlexNodeId, FFlexRoadNode>& Pair : Subsystem->GetAllNodes())
	{
		if (!PreExistingNodes.Contains(Pair.Key))
		{
			NewNodeIds.Add(Pair.Key);
		}
	}
	for (const FFlexSegmentId& SegmentId : NewSegmentIds)
	{
		Subsystem->RemoveSegment(SegmentId);
	}
	for (const FFlexNodeId& NodeId : NewNodeIds)
	{
		Subsystem->RemoveNode(NodeId);
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
