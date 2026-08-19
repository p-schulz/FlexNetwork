#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "FlexNetworkSubsystem.h"
#include "FlexNetworkSettings.h"
#include "RoadTypeProfile.h"
#include "Rail/FlexTrackGraphBuilder.h"
#include "Rail/FlexTrackJunctionSolver.h"
#include "Rail/FlexRailGraphBuilder.h"
#include "Editor.h"
#include "Engine/World.h"

namespace FlexRailGraphBuilderTestHelpers
{
	URoadTypeProfile* MakeTramProfile()
	{
		URoadTypeProfile* Profile = NewObject<URoadTypeProfile>(GetTransientPackage());
		Profile->bIsRailProfile = true;
		Profile->RailGauge = 143.5f;
		Profile->RailWidth = 15.6f;
		Profile->MinTurnRadius = 1800.f;
		Profile->SidewalkWidth = 0.f;
		Profile->CurbHeight = 0.f;

		FRoadLaneDescriptor Track;
		Track.Type = EFlexLaneType::Rail;
		Track.Direction = EFlexLaneDirection::Bidirectional;
		Track.Width = Profile->RailGauge + Profile->RailWidth;
		Profile->Lanes.Add(Track);
		return Profile;
	}
}

// A gentle three-way turnout's common leg should fold into at least one Shared rail edge (the
// two branches' left/right rails coincide near the shared port before diverging), and none of
// the resulting edges should be classified as a Crossing (nothing here actually crosses).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexRailGraphBuilderTurnoutTest, "FlexNetwork.Rail.RailGraphBuilderTurnoutMerge", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FFlexRailGraphBuilderTurnoutTest::RunTest(const FString& Parameters)
{
	using namespace FlexRailGraphBuilderTestHelpers;

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
	if (!TestNotNull(TEXT("UFlexNetworkSubsystem present"), Subsystem))
	{
		return false;
	}

	URoadTypeProfile* TramProfile = MakeTramProfile();

	const FFlexNodeId Common = Subsystem->AddNode(FVector(0.f, 40000.f, 0.f));
	const FFlexNodeId Hub = Subsystem->AddNode(FVector(4000.f, 40000.f, 0.f));
	const FFlexNodeId BranchA = Subsystem->AddNode(FVector(9000.f, 42500.f, 0.f));
	const FFlexNodeId BranchB = Subsystem->AddNode(FVector(9000.f, 37500.f, 0.f));
	Subsystem->AddSegment(Common, Hub, FVector(1300.f, 40000.f, 0.f), FVector(2700.f, 40000.f, 0.f), TramProfile);
	Subsystem->AddSegment(Hub, BranchA, FVector(5700.f, 40400.f, 0.f), FVector(7300.f, 41600.f, 0.f), TramProfile);
	Subsystem->AddSegment(Hub, BranchB, FVector(5700.f, 39600.f, 0.f), FVector(7300.f, 38400.f, 0.f), TramProfile);

	const FFlexTrackGraph TrackGraph = FFlexTrackGraphBuilder::Build(*Subsystem, TramProfile);

	TArray<FFlexTrackJunction> Junctions;
	for (FFlexNodeId JunctionNodeId : TrackGraph.JunctionNodeIds)
	{
		Junctions.Add(FFlexTrackJunctionSolver::Solve(JunctionNodeId, *Subsystem, TramProfile, 1000.f, 2000.f, 300.f, 4));
	}
	if (!TestEqual(TEXT("Exactly one junction node (Hub)"), Junctions.Num(), 1))
	{
		return false;
	}

	const FFlexRailGraph RailGraph = FFlexRailGraphBuilder::Build(TrackGraph, Junctions, *Subsystem, 100.f, 3.f, 10.f, 15.f);

	TestTrue(TEXT("RailGraph produces at least one edge"), RailGraph.Edges.Num() > 0);

	int32 NumShared = 0;
	int32 NumCrossing = 0;
	for (const FFlexRailEdge& Edge : RailGraph.Edges)
	{
		TestTrue(TEXT("Every edge has at least two frames"), Edge.Frames.Num() >= 2);
		if (Edge.Type == ERailEdgeType::Shared)
		{
			++NumShared;
		}
		else if (Edge.Type == ERailEdgeType::Crossing)
		{
			++NumCrossing;
		}
	}
	TestTrue(TEXT("The turnout's common leg produces at least one Shared edge"), NumShared > 0);
	TestEqual(TEXT("A non-crossing turnout produces no Crossing edges"), NumCrossing, 0);

	return true;
}

// Two independent straight rail lines that don't share any node should never merge or cross --
// every resulting edge should be Normal.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexRailGraphBuilderStraightTest, "FlexNetwork.Rail.RailGraphBuilderStraightTrackIsNormal", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FFlexRailGraphBuilderStraightTest::RunTest(const FString& Parameters)
{
	using namespace FlexRailGraphBuilderTestHelpers;

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
	if (!TestNotNull(TEXT("UFlexNetworkSubsystem present"), Subsystem))
	{
		return false;
	}

	URoadTypeProfile* TramProfile = MakeTramProfile();

	const FFlexNodeId A = Subsystem->AddNode(FVector(0.f, 60000.f, 0.f));
	const FFlexNodeId B = Subsystem->AddNode(FVector(5000.f, 60000.f, 0.f));
	Subsystem->AddSegment(A, B, FVector(1600.f, 60000.f, 0.f), FVector(3400.f, 60000.f, 0.f), TramProfile);

	const FFlexTrackGraph TrackGraph = FFlexTrackGraphBuilder::Build(*Subsystem, TramProfile);
	TestEqual(TEXT("No junction nodes on an isolated straight line"), TrackGraph.JunctionNodeIds.Num(), 0);

	TArray<FFlexTrackJunction> EmptyJunctions;
	const FFlexRailGraph RailGraph = FFlexRailGraphBuilder::Build(TrackGraph, EmptyJunctions, *Subsystem, 100.f, 3.f, 10.f, 15.f);

	TestEqual(TEXT("A single straight segment produces exactly two rail edges (left/right)"), RailGraph.Edges.Num(), 2);
	for (const FFlexRailEdge& Edge : RailGraph.Edges)
	{
		TestTrue(TEXT("Straight, non-junction track is Normal"), Edge.Type == ERailEdgeType::Normal);
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
