#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "FlexNetworkSubsystem.h"
#include "RoadTypeProfile.h"
#include "Rail/FlexTrackGraphBuilder.h"
#include "Editor.h"
#include "Engine/World.h"

namespace FlexTrackGraphBuilderTestHelpers
{
	URoadTypeProfile* MakeTramProfile()
	{
		URoadTypeProfile* Profile = NewObject<URoadTypeProfile>(GetTransientPackage());
		Profile->bIsRailProfile = true;
		Profile->RailGauge = 143.5f;
		Profile->RailWidth = 15.6f;
		Profile->RailTopWidth = 11.5f;
		Profile->RailHeight = 7.2f;
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

// A straight-through pair of rail segments meeting at a bend should NOT be classified as a
// junction node (ordinary track continues through it), while a genuine three-way turnout node
// should be.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexTrackGraphBuilderTest, "FlexNetwork.Rail.TrackGraphBuilderJunctionDetection", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FFlexTrackGraphBuilderTest::RunTest(const FString& Parameters)
{
	using namespace FlexTrackGraphBuilderTestHelpers;

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

	// --- Straight-through bend: A -> Bend -> C, both segments nearly collinear. ---
	const FFlexNodeId BendA = Subsystem->AddNode(FVector(0.f, 0.f, 0.f));
	const FFlexNodeId BendMid = Subsystem->AddNode(FVector(3000.f, 0.f, 0.f));
	const FFlexNodeId BendC = Subsystem->AddNode(FVector(6000.f, 0.f, 0.f));
	Subsystem->AddSegment(BendA, BendMid, FVector(1000.f, 0.f, 0.f), FVector(2000.f, 0.f, 0.f), TramProfile);
	Subsystem->AddSegment(BendMid, BendC, FVector(4000.f, 0.f, 0.f), FVector(5000.f, 0.f, 0.f), TramProfile);

	// --- Three-way turnout: Common -> Hub, plus two diverging branches from Hub. ---
	const FFlexNodeId Common = Subsystem->AddNode(FVector(0.f, 20000.f, 0.f));
	const FFlexNodeId Hub = Subsystem->AddNode(FVector(3000.f, 20000.f, 0.f));
	const FFlexNodeId BranchA = Subsystem->AddNode(FVector(6000.f, 21500.f, 0.f));
	const FFlexNodeId BranchB = Subsystem->AddNode(FVector(6000.f, 18500.f, 0.f));
	Subsystem->AddSegment(Common, Hub, FVector(1000.f, 20000.f, 0.f), FVector(2000.f, 20000.f, 0.f), TramProfile);
	Subsystem->AddSegment(Hub, BranchA, FVector(4000.f, 20200.f, 0.f), FVector(5000.f, 20900.f, 0.f), TramProfile);
	Subsystem->AddSegment(Hub, BranchB, FVector(4000.f, 19800.f, 0.f), FVector(5000.f, 19100.f, 0.f), TramProfile);

	const FFlexTrackGraph TrackGraph = FFlexTrackGraphBuilder::Build(*Subsystem);

	TestEqual(TEXT("Every rail segment becomes a track"), TrackGraph.Tracks.Num(), Subsystem->GetAllSegments().Num());
	TestFalse(TEXT("A smooth straight-through bend is not a junction node"), TrackGraph.JunctionNodeIds.Contains(BendMid));
	TestTrue(TEXT("A genuine three-way turnout is a junction node"), TrackGraph.JunctionNodeIds.Contains(Hub));
	TestFalse(TEXT("A dead end is not a junction node"), TrackGraph.JunctionNodeIds.Contains(BendA));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
