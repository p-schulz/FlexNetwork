#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "FlexNetworkSubsystem.h"
#include "RoadTypeProfile.h"
#include "Editor.h"
#include "Engine/World.h"

namespace FlexNetworkPipelineTestHelpers
{
	URoadTypeProfile* MakeTestProfile(float LaneWidth)
	{
		URoadTypeProfile* Profile = NewObject<URoadTypeProfile>(GetTransientPackage());

		FRoadLaneDescriptor Forward;
		Forward.LateralOffset = LaneWidth * 0.5f;
		Forward.Width = LaneWidth;
		Forward.Type = EFlexLaneType::Vehicle;
		Forward.Direction = EFlexLaneDirection::Forward;
		Profile->Lanes.Add(Forward);

		FRoadLaneDescriptor Backward;
		Backward.LateralOffset = -LaneWidth * 0.5f;
		Backward.Width = LaneWidth;
		Backward.Type = EFlexLaneType::Vehicle;
		Backward.Direction = EFlexLaneDirection::Backward;
		Profile->Lanes.Add(Backward);

		Profile->SidewalkWidth = 200.f;
		Profile->MinTurnRadius = 500.f;
		Profile->MinSegmentLengthOverride = 100.f;
		return Profile;
	}
}

// Drives UFlexNetworkSubsystem's public API through the same scenario the spec's acceptance
// criteria ask for (straight road, curved road, 3-way junction, 4-way junction with mismatched
// widths, elevated ramp, and a drag-across-existing-road auto-split) programmatically. This
// project has no way to drive the Unreal Editor's viewport/mouse input from an automated test,
// so this is the closest available substitute for "generated purely through the editor tool" --
// it exercises the exact same UFlexNetworkSubsystem calls the editor mode makes, just called
// directly instead of via mouse drags. See the plugin README for the manual-QA steps that cover
// the actual click-drag UX this can't reach.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexNetworkPipelineTest, "FlexNetwork.Pipeline.EndToEndScenario", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FFlexNetworkPipelineTest::RunTest(const FString& Parameters)
{
	using namespace FlexNetworkPipelineTestHelpers;

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

	URoadTypeProfile* NarrowProfile = MakeTestProfile(300.f);
	URoadTypeProfile* WideProfile = MakeTestProfile(350.f);
	WideProfile->Lanes[0].LateralOffset = 700.f;
	WideProfile->Lanes[1].LateralOffset = -700.f;

	// --- Straight two-lane road ---
	const FFlexNodeId StraightA = Subsystem->AddNode(FVector(0, 0, 0));
	const FFlexNodeId StraightB = Subsystem->AddNode(FVector(2000, 0, 0));
	const FFlexSegmentId StraightSeg = Subsystem->AddSegment(StraightA, StraightB, FVector(700, 0, 0), FVector(1300, 0, 0), NarrowProfile);
	TestTrue(TEXT("Straight segment created"), StraightSeg.IsValid());
	if (const FFlexRoadSegment* StraightSegData = Subsystem->GetSegment(StraightSeg))
	{
		TestTrue(TEXT("Straight segment has a built arc-length table"), StraightSegData->ArcLengthTable.IsValid());
	}

	// --- Curved road ---
	const FFlexNodeId CurveA = Subsystem->AddNode(FVector(0, 3000, 0));
	const FFlexNodeId CurveB = Subsystem->AddNode(FVector(2000, 5000, 0));
	const FFlexSegmentId CurveSeg = Subsystem->AddSegment(CurveA, CurveB, FVector(1500, 3000, 0), FVector(0, 5000, 0), NarrowProfile);
	TestTrue(TEXT("Curved segment created"), CurveSeg.IsValid());

	// --- 3-way junction ---
	const FFlexNodeId Hub3 = Subsystem->AddNode(FVector(0, 10000, 0));
	const FFlexNodeId Arm3A = Subsystem->AddNode(FVector(-2000, 10000, 0));
	const FFlexNodeId Arm3B = Subsystem->AddNode(FVector(2000, 10000, 0));
	const FFlexNodeId Arm3C = Subsystem->AddNode(FVector(0, 12000, 0));
	Subsystem->AddSegment(Arm3A, Hub3, FVector(-1300, 10000, 0), FVector(-700, 10000, 0), NarrowProfile);
	Subsystem->AddSegment(Hub3, Arm3B, FVector(700, 10000, 0), FVector(1300, 10000, 0), NarrowProfile);
	Subsystem->AddSegment(Hub3, Arm3C, FVector(0, 10700, 0), FVector(0, 11300, 0), NarrowProfile);

	if (const FFlexRoadNode* Hub3Node = Subsystem->GetNode(Hub3))
	{
		TestTrue(TEXT("3-way hub is flagged as a junction"), Hub3Node->HasRole(EFlexNodeRole::Junction));
	}
	const FFlexJunctionData* Hub3Junction = Subsystem->GetJunctionData(Hub3);
	if (TestNotNull(TEXT("3-way junction data was built"), Hub3Junction))
	{
		TestTrue(TEXT("3-way junction has a polygon"), Hub3Junction->PolygonBoundary.Num() >= 3);
		TestTrue(TEXT("3-way junction has lane connectors"), Hub3Junction->LaneConnectors.Num() > 0);
	}

	// --- 4-way junction with mismatched road widths ---
	const FFlexNodeId Hub4 = Subsystem->AddNode(FVector(0, 20000, 0));
	const FFlexNodeId Arm4N = Subsystem->AddNode(FVector(0, 22000, 0));
	const FFlexNodeId Arm4S = Subsystem->AddNode(FVector(0, 18000, 0));
	const FFlexNodeId Arm4E = Subsystem->AddNode(FVector(2000, 20000, 0));
	const FFlexNodeId Arm4W = Subsystem->AddNode(FVector(-2000, 20000, 0));
	Subsystem->AddSegment(Hub4, Arm4N, FVector(0, 20700, 0), FVector(0, 21300, 0), WideProfile);
	Subsystem->AddSegment(Arm4S, Hub4, FVector(0, 18700, 0), FVector(0, 19300, 0), WideProfile);
	Subsystem->AddSegment(Hub4, Arm4E, FVector(700, 20000, 0), FVector(1300, 20000, 0), NarrowProfile);
	Subsystem->AddSegment(Arm4W, Hub4, FVector(-1300, 20000, 0), FVector(-700, 20000, 0), NarrowProfile);

	if (const FFlexRoadNode* Hub4Node = Subsystem->GetNode(Hub4))
	{
		TestTrue(TEXT("4-way hub is flagged as a junction"), Hub4Node->HasRole(EFlexNodeRole::Junction));
	}
	const FFlexJunctionData* Hub4Junction = Subsystem->GetJunctionData(Hub4);
	if (TestNotNull(TEXT("4-way (mismatched width) junction data was built"), Hub4Junction))
	{
		TestTrue(TEXT("4-way junction polygon has at least 4 corners"), Hub4Junction->PolygonBoundary.Num() >= 4);
		TestTrue(TEXT("4-way junction triangulated into at least 2 triangles"), Hub4Junction->PolygonTriangleIndices.Num() >= 6);
	}

	// --- Elevated ramp transitioning to ground ---
	const FFlexNodeId RampGround = Subsystem->AddNode(FVector(0, 30000, 0), EFlexRoadElevationType::Ground);
	const FFlexNodeId RampTop = Subsystem->AddNode(FVector(2000, 30000, 800), EFlexRoadElevationType::Elevated);
	const FFlexSegmentId RampSeg = Subsystem->AddSegment(RampGround, RampTop, FVector(700, 30000, 0), FVector(1300, 30000, 800), NarrowProfile, EFlexRoadElevationType::Ramp);
	if (const FFlexRoadSegment* RampSegData = Subsystem->GetSegment(RampSeg))
	{
		// EaseInOut (the default) levels each tangent handle with its own endpoint's height, so
		// the ramp's own curve derivative is flat entering/leaving each end -- see
		// FlexNetworkSubsystem.cpp's ApplyElevationEase for why this alone gives an eased profile.
		TestTrue(TEXT("Ramp start handle levels with the ground endpoint (eased, not linear)"), FMath::IsNearlyEqual(RampSegData->Curve.P1.Z, RampSegData->Curve.P0.Z, 0.01));
		TestTrue(TEXT("Ramp end handle levels with the elevated endpoint (eased, not linear)"), FMath::IsNearlyEqual(RampSegData->Curve.P2.Z, RampSegData->Curve.P3.Z, 0.01));
	}

	// --- Dragging a new road across an existing one auto-splits the existing one ---
	const FFlexNodeId CrossA = Subsystem->AddNode(FVector(-2000, 40000, 0));
	const FFlexNodeId CrossB = Subsystem->AddNode(FVector(2000, 40000, 0));
	Subsystem->AddSegment(CrossA, CrossB, FVector(-1300, 40000, 0), FVector(1300, 40000, 0), NarrowProfile);

	FFlexBezierCurve ProposedCrossing;
	ProposedCrossing.P0 = FVector(0, 38000, 0);
	ProposedCrossing.P1 = FVector(0, 39000, 0);
	ProposedCrossing.P2 = FVector(0, 41000, 0);
	ProposedCrossing.P3 = FVector(0, 42000, 0);

	const TArray<FFlexSegmentCrossing> Crossings = Subsystem->FindCrossings(ProposedCrossing);
	TestEqual(TEXT("Exactly one crossing detected against the perpendicular road"), Crossings.Num(), 1);

	if (Crossings.Num() > 0)
	{
		const int32 SegmentCountBeforeSplit = Subsystem->GetAllSegments().Num();
		const FFlexNodeId JunctionFromSplit = Subsystem->SplitSegment(Crossings[0].ExistingSegmentId, Crossings[0].ArcLengthOnExistingSegment);
		TestTrue(TEXT("Split produced a new junction node"), JunctionFromSplit.IsValid());
		TestEqual(TEXT("Splitting one segment adds exactly one more segment"), Subsystem->GetAllSegments().Num(), SegmentCountBeforeSplit + 1);

		if (const FFlexRoadNode* SplitNode = Subsystem->GetNode(JunctionFromSplit))
		{
			TestTrue(TEXT("Node created by the split sits at the crossing point"), FVector::Dist2D(SplitNode->Position, Crossings[0].WorldPoint) < 50.f);
		}
	}

	// --- Incremental rebuild: editing one segment's profile touches far fewer segments than the whole network. ---
	const int32 TotalSegmentsBeforeProfileEdit = Subsystem->GetAllSegments().Num();
	TArray<FFlexSegmentId> LastChangedSegments;
	const FDelegateHandle Handle = Subsystem->OnRoadNetworkChanged.AddLambda([&LastChangedSegments](const TArray<FFlexNodeId>&, const TArray<FFlexSegmentId>& ChangedSegments)
	{
		LastChangedSegments = ChangedSegments;
	});
	Subsystem->SetSegmentProfile(StraightSeg, WideProfile);
	Subsystem->OnRoadNetworkChanged.Remove(Handle);

	TestTrue(TEXT("Incremental rebuild after one segment-profile edit touches only a handful of segments, not the whole network"),
		LastChangedSegments.Num() > 0 && LastChangedSegments.Num() < TotalSegmentsBeforeProfileEdit);

	// --- Lane connectors are drivable curves between distinct approaches. ---
	if (Hub3Junction)
	{
		for (const FFlexLaneConnector& Connector : Hub3Junction->LaneConnectors)
		{
			TestTrue(TEXT("Lane connector goes between two different segments"), Connector.FromSegment != Connector.ToSegment);
			TestTrue(TEXT("Lane connector curve has non-zero length"), FVector::Dist(Connector.ConnectorCurve.P0, Connector.ConnectorCurve.P3) > 1.f);
		}
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
