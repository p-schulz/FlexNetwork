#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "RoadTypeProfile.h"
#include "Intersection/FlexLaneConnectorGraph.h"
#include "Math/FlexBezierMath.h"
#include "Mesh/FlexRoadMarkingBuilder.h"

namespace FlexRoadMarkingBuilderTestHelpers
{
	FFlexRoadMarkingParams MakeParams()
	{
		FFlexRoadMarkingParams Params;
		Params.SolidLineWidth = 12.f;
		Params.LaneDashWidth = 12.f;
		Params.LaneDashLength = 300.f;
		Params.LaneDashGap = 200.f;
		Params.IntersectionDashWidth = 10.f;
		Params.IntersectionDashLength = 100.f;
		Params.IntersectionDashGap = 100.f;
		Params.CrosswalkDashWidth = 15.f;
		Params.CrosswalkDashLength = 40.f;
		Params.CrosswalkDashGap = 40.f;
		Params.SolidToDashedTransitionDistance = 500.f;
		Params.StopLineThickness = 30.f;
		Params.StopLineSetback = 50.f;
		Params.VerticalOffset = 0.5f;
		return Params;
	}

	FRoadLaneDescriptor MakeLane(float LateralOffset, float Width, EFlexLaneDirection Direction, EFlexLaneType Type = EFlexLaneType::Vehicle)
	{
		FRoadLaneDescriptor Lane;
		Lane.LateralOffset = LateralOffset;
		Lane.Width = Width;
		Lane.Direction = Direction;
		Lane.Type = Type;
		return Lane;
	}

	URoadTypeProfile* MakeProfile(TArray<FRoadLaneDescriptor> Lanes)
	{
		URoadTypeProfile* Profile = NewObject<URoadTypeProfile>(GetTransientPackage());
		Profile->Lanes = MoveTemp(Lanes);
		return Profile;
	}
}

// A two-lane road (one lane each direction) gets exactly one dashed boundary, even though the two
// lanes travel in opposite directions -- the explicit exception to the direction-based rule.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexRoadMarkingTwoLaneTest, "FlexNetwork.Markings.TwoLaneRoadIsAlwaysDashed", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexRoadMarkingTwoLaneTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMarkingBuilderTestHelpers;

	FFlexBezierCurve Curve;
	Curve.P0 = FVector(0, 0, 0);
	Curve.P1 = FVector(1000, 0, 0);
	Curve.P2 = FVector(2000, 0, 0);
	Curve.P3 = FVector(3000, 0, 0);
	const FFlexArcLengthTable Table = FFlexBezierMath::BuildArcLengthTable(Curve);

	URoadTypeProfile* Profile = MakeProfile({
		MakeLane(-175.f, 350.f, EFlexLaneDirection::Backward),
		MakeLane(175.f, 350.f, EFlexLaneDirection::Forward)
	});

	FFlexMeshSectionData Solid, LaneDash;
	FFlexRoadMarkingBuilder::BuildSegmentLaneMarkings(Curve, Table, Profile, FVector::UpVector, 100.f, 0.f,
		Table.GetTotalLength(), false, false, TArray<FVector2D>(), MakeParams(), &Solid, &LaneDash);

	TestTrue(TEXT("A two-lane road produces dashed markings"), !LaneDash.IsEmpty());
	TestTrue(TEXT("A two-lane road produces no solid markings"), Solid.IsEmpty());

	return true;
}

// Four lanes -- two same-direction pairs separated by an opposite-direction pair in the middle --
// produce dashed / solid / dashed in that order.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexRoadMarkingFourLaneTest, "FlexNetwork.Markings.FourLaneRoadMixesDashedAndSolid", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexRoadMarkingFourLaneTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMarkingBuilderTestHelpers;

	FFlexBezierCurve Curve;
	Curve.P0 = FVector(0, 0, 0);
	Curve.P1 = FVector(1000, 0, 0);
	Curve.P2 = FVector(2000, 0, 0);
	Curve.P3 = FVector(3000, 0, 0);
	const FFlexArcLengthTable Table = FFlexBezierMath::BuildArcLengthTable(Curve);

	// Left to right: Backward, Backward | Forward, Forward -- one opposite-direction boundary in
	// the middle (Backward/Forward), same-direction boundaries on either side.
	URoadTypeProfile* Profile = MakeProfile({
		MakeLane(-525.f, 350.f, EFlexLaneDirection::Backward),
		MakeLane(-175.f, 350.f, EFlexLaneDirection::Backward),
		MakeLane(175.f, 350.f, EFlexLaneDirection::Forward),
		MakeLane(525.f, 350.f, EFlexLaneDirection::Forward)
	});

	FFlexMeshSectionData Solid, LaneDash;
	FFlexRoadMarkingBuilder::BuildSegmentLaneMarkings(Curve, Table, Profile, FVector::UpVector, 100.f, 0.f,
		Table.GetTotalLength(), false, false, TArray<FVector2D>(), MakeParams(), &Solid, &LaneDash);

	TestTrue(TEXT("Same-direction boundaries produce dashed markings"), !LaneDash.IsEmpty());
	TestTrue(TEXT("The opposite-direction boundary produces solid markings"), !Solid.IsEmpty());

	return true;
}

// The same opposite-direction boundary, but now both ends border a junction and the transition
// distance covers the whole segment -- the solid line must taper away entirely into dashed.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexRoadMarkingSolidTapersNearJunctionTest, "FlexNetwork.Markings.SolidLineTapersToDashedNearJunction", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexRoadMarkingSolidTapersNearJunctionTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMarkingBuilderTestHelpers;

	FFlexBezierCurve Curve;
	Curve.P0 = FVector(0, 0, 0);
	Curve.P1 = FVector(1000, 0, 0);
	Curve.P2 = FVector(2000, 0, 0);
	Curve.P3 = FVector(3000, 0, 0);
	const FFlexArcLengthTable Table = FFlexBezierMath::BuildArcLengthTable(Curve);

	URoadTypeProfile* Profile = MakeProfile({
		MakeLane(-175.f, 350.f, EFlexLaneDirection::Backward),
		MakeLane(175.f, 350.f, EFlexLaneDirection::Forward),
		MakeLane(525.f, 350.f, EFlexLaneDirection::Forward)
	});
	// -175/175 differ in direction (solid) and 175/525 share direction (dashed); with the whole
	// segment inside the transition zone, the 175 boundary should end up entirely dashed.

	FFlexRoadMarkingParams Params = MakeParams();
	Params.SolidToDashedTransitionDistance = Table.GetTotalLength(); // Covers the whole segment.

	FFlexMeshSectionData SolidBothJunctions, DashBothJunctions;
	FFlexRoadMarkingBuilder::BuildSegmentLaneMarkings(Curve, Table, Profile, FVector::UpVector, 100.f, 0.f,
		Table.GetTotalLength(), /*bStartAtJunction*/ true, /*bEndAtJunction*/ true, TArray<FVector2D>(), Params, &SolidBothJunctions, &DashBothJunctions);
	TestTrue(TEXT("A fully-tapered boundary produces no solid geometry"), SolidBothJunctions.IsEmpty());
	TestTrue(TEXT("A fully-tapered boundary still produces dashed geometry"), !DashBothJunctions.IsEmpty());

	FFlexMeshSectionData SolidNoJunctions, DashNoJunctions;
	FFlexRoadMarkingBuilder::BuildSegmentLaneMarkings(Curve, Table, Profile, FVector::UpVector, 100.f, 0.f,
		Table.GetTotalLength(), /*bStartAtJunction*/ false, /*bEndAtJunction*/ false, TArray<FVector2D>(), Params, &SolidNoJunctions, &DashNoJunctions);
	TestTrue(TEXT("The same boundary away from any junction stays solid"), !SolidNoJunctions.IsEmpty());

	return true;
}

// A crosswalk exclusion range punched through the middle of a dashed boundary's span leaves no
// dash geometry inside that range.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexRoadMarkingCrosswalkExclusionTest, "FlexNetwork.Markings.LaneDashesAvoidCrosswalkExclusionRange", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexRoadMarkingCrosswalkExclusionTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMarkingBuilderTestHelpers;

	FFlexBezierCurve Curve;
	Curve.P0 = FVector(0, 0, 0);
	Curve.P1 = FVector(1000, 0, 0);
	Curve.P2 = FVector(2000, 0, 0);
	Curve.P3 = FVector(3000, 0, 0);
	const FFlexArcLengthTable Table = FFlexBezierMath::BuildArcLengthTable(Curve);

	URoadTypeProfile* Profile = MakeProfile({
		MakeLane(-175.f, 350.f, EFlexLaneDirection::Backward),
		MakeLane(175.f, 350.f, EFlexLaneDirection::Forward)
	});

	const TArray<FVector2D> Exclusions = { FVector2D(1200.f, 1800.f) };
	FFlexMeshSectionData Solid, LaneDash;
	FFlexRoadMarkingBuilder::BuildSegmentLaneMarkings(Curve, Table, Profile, FVector::UpVector, 100.f, 0.f,
		Table.GetTotalLength(), false, false, Exclusions, MakeParams(), &Solid, &LaneDash);

	TestTrue(TEXT("Dashes are still produced outside the exclusion range"), !LaneDash.IsEmpty());
	for (const FVector& Vertex : LaneDash.Vertices)
	{
		const bool bOutsideExclusion = Vertex.X <= Exclusions[0].X || Vertex.X >= Exclusions[0].Y;
		if (!TestTrue(TEXT("No dash vertex falls inside the crosswalk exclusion range"), bOutsideExclusion))
		{
			break;
		}
	}

	return true;
}

// FFlexRoadMarkingBuilder::SubtractExcludedRanges in isolation: punching a hole in the middle
// splits one range into two; a hole covering the whole range empties it; a non-overlapping hole
// leaves the range untouched.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexRoadMarkingSubtractRangesTest, "FlexNetwork.Markings.SubtractExcludedRanges", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexRoadMarkingSubtractRangesTest::RunTest(const FString& Parameters)
{
	{
		const TArray<FVector2D> Result = FFlexRoadMarkingBuilder::SubtractExcludedRanges(0.f, 1000.f, TArray<FVector2D>{ FVector2D(400.f, 600.f) });
		TestEqual(TEXT("A hole in the middle splits the range in two"), Result.Num(), 2);
		if (Result.Num() == 2)
		{
			TestTrue(TEXT("First remaining range is [0,400]"), Result[0].Equals(FVector2D(0.f, 400.f)));
			TestTrue(TEXT("Second remaining range is [600,1000]"), Result[1].Equals(FVector2D(600.f, 1000.f)));
		}
	}
	{
		const TArray<FVector2D> Result = FFlexRoadMarkingBuilder::SubtractExcludedRanges(0.f, 1000.f, TArray<FVector2D>{ FVector2D(-100.f, 1100.f) });
		TestEqual(TEXT("A hole covering the whole range empties it"), Result.Num(), 0);
	}
	{
		const TArray<FVector2D> Result = FFlexRoadMarkingBuilder::SubtractExcludedRanges(0.f, 1000.f, TArray<FVector2D>{ FVector2D(2000.f, 3000.f) });
		TestEqual(TEXT("A non-overlapping hole leaves exactly one range"), Result.Num(), 1);
		if (Result.Num() == 1)
		{
			TestTrue(TEXT("The untouched range is unchanged"), Result[0].Equals(FVector2D(0.f, 1000.f)));
		}
	}
	return true;
}

// A stop line is one solid quad centered at the given frame and spanning exactly the given lateral
// range -- placing it StopLineSetback before the crosswalk is the caller's job (it samples the
// frame at an already-offset arc length), not this function's.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexRoadMarkingStopLineTest, "FlexNetwork.Markings.StopLineSpansGivenLanes", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexRoadMarkingStopLineTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMarkingBuilderTestHelpers;

	FFlexCurveFrame Frame;
	Frame.Position = FVector(1000.f, 0.f, 0.f);
	Frame.Tangent = FVector::ForwardVector;
	Frame.Right = FVector::RightVector;
	Frame.Up = FVector::UpVector;

	FFlexMeshSectionData StopLine;
	FFlexRoadMarkingBuilder::BuildStopLineMarking(Frame, -175.f, 175.f, MakeParams(), &StopLine);

	TestTrue(TEXT("Stop line produces geometry"), !StopLine.IsEmpty());
	for (const FVector& Vertex : StopLine.Vertices)
	{
		const float LateralOffset = FVector::DotProduct(Vertex - Frame.Position, Frame.Right);
		if (!TestTrue(TEXT("Every stop line vertex stays within the given lateral span"), LateralOffset >= -175.f - KINDA_SMALL_NUMBER && LateralOffset <= 175.f + KINDA_SMALL_NUMBER))
		{
			break;
		}
	}

	return true;
}

// A synthetic crosswalk produces dash geometry only near its two long edges (Center +/- Across*Width/2,
// where Across is parallel to the road), never near its short near/far curb-line edges.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexRoadMarkingCrosswalkTest, "FlexNetwork.Markings.CrosswalkDashesSitOnLongEdges", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexRoadMarkingCrosswalkTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMarkingBuilderTestHelpers;

	FFlexCrosswalkPlacement Crosswalk;
	Crosswalk.Center = FVector(500.f, 500.f, 0.f);
	Crosswalk.CrossingDirection = FVector(0.f, 1.f, 0.f);
	Crosswalk.Width = 300.f;
	Crosswalk.Length = 600.f;

	FFlexMeshSectionData CrosswalkDash;
	FFlexRoadMarkingBuilder::BuildCrosswalkMarkings(Crosswalk, FVector::UpVector, MakeParams(), &CrosswalkDash);

	TestTrue(TEXT("Crosswalk produces dash geometry"), !CrosswalkDash.IsEmpty());

	const FVector Along = Crosswalk.CrossingDirection.GetSafeNormal();
	const FVector Across = FVector::CrossProduct(FVector::UpVector, Along).GetSafeNormal();
	const float ExpectedEdgeDistance = Crosswalk.Width * 0.5f;
	for (const FVector& Vertex : CrosswalkDash.Vertices)
	{
		const float DistanceAcrossRoad = FVector::DotProduct(Vertex - Crosswalk.Center, Across);
		const float DistanceAlongCrossing = FVector::DotProduct(Vertex - Crosswalk.Center, Along);
		const bool bNearLongEdge = FMath::Abs(FMath::Abs(DistanceAcrossRoad) - ExpectedEdgeDistance) < 25.f;
		const bool bWithinLength = FMath::Abs(DistanceAlongCrossing) <= Crosswalk.Length * 0.5f + KINDA_SMALL_NUMBER;
		if (!TestTrue(TEXT("Every dash vertex sits near one of the two crosswalk long edges"), bNearLongEdge)
			|| !TestTrue(TEXT("Every dash vertex stays within the crosswalk's length"), bWithinLength))
		{
			break;
		}
	}

	return true;
}

// A lane connector's dashed guide line sits on its left border (negative Right offset), not its right.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexRoadMarkingIntersectionTest, "FlexNetwork.Markings.IntersectionDashSitsOnLeftBorder", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexRoadMarkingIntersectionTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMarkingBuilderTestHelpers;

	FFlexLaneConnector Connector;
	Connector.ConnectorCurve.P0 = FVector(0, 0, 0);
	Connector.ConnectorCurve.P1 = FVector(300, 0, 0);
	Connector.ConnectorCurve.P2 = FVector(600, 0, 0);
	Connector.ConnectorCurve.P3 = FVector(900, 0, 0);

	const float LaneWidth = 350.f;
	FFlexMeshSectionData IntersectionDash;
	FFlexRoadMarkingBuilder::BuildIntersectionLaneMarking(Connector, LaneWidth, FVector::UpVector, MakeParams(), &IntersectionDash);

	TestTrue(TEXT("Intersection connector produces dash geometry"), !IntersectionDash.IsEmpty());

	// Straight curve along +X with world-up reference: Right = CrossProduct(Up, Tangent) = +Y (UE's
	// own X-forward/Y-right/Z-up convention), so "left" is -Y. Every dash vertex should sit there.
	for (const FVector& Vertex : IntersectionDash.Vertices)
	{
		if (!TestTrue(TEXT("Every dash vertex sits on the connector's left (-Y) side"), Vertex.Y < 0.f))
		{
			break;
		}
	}

	return true;
}

// One divider line is generated at every interior ParkingSpotSpacing boundary strictly inside
// [TrimStart, TrimEnd] -- not at the span's own open ends.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexParkingSpotSpacingTest, "FlexNetwork.Markings.ParkingSpotDividersMatchSpacing", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexParkingSpotSpacingTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMarkingBuilderTestHelpers;

	FFlexBezierCurve Curve;
	Curve.P0 = FVector(0, 0, 0);
	Curve.P1 = FVector(1000, 0, 0);
	Curve.P2 = FVector(2000, 0, 0);
	Curve.P3 = FVector(3000, 0, 0);
	const FFlexArcLengthTable Table = FFlexBezierMath::BuildArcLengthTable(Curve);

	FRoadLaneDescriptor Lane;
	Lane.Type = EFlexLaneType::Parking;
	Lane.Width = 250.f;
	Lane.ParkingAngleDegrees = 0.f;

	FFlexRoadMarkingParams Params = MakeParams();
	Params.ParkingSpotSpacing = 550.f;

	FFlexMeshSectionData ParkingMarking;
	FFlexRoadMarkingBuilder::BuildParkingSpotMarkings(Curve, Table, Lane, 0.f, FVector::UpVector, 0.f, 3000.f, Params, &ParkingMarking);

	// Boundaries at 550, 1100, 1650, 2200, 2750 -- five interior multiples of 550 below 3000, one
	// quad (two triangles) each.
	TestEqual(TEXT("Five interior spacing boundaries produce five divider quads"), ParkingMarking.Triangles.Num() / 3, 10);

	return true;
}

// A divider's own thickness axis follows Frame.Tangent at ParkingAngleDegrees = 0 (parallel
// parking -- like a mini stop line, thickness measured along the road) and swaps to Frame.Right at
// ParkingAngleDegrees = 90 (perpendicular parking -- thickness measured laterally instead), while
// the divider's long axis does the opposite in each case.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexParkingSpotAngleOrientationTest, "FlexNetwork.Markings.ParkingSpotAngleControlsOrientation", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexParkingSpotAngleOrientationTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMarkingBuilderTestHelpers;

	FFlexBezierCurve Curve;
	Curve.P0 = FVector(0, 0, 0);
	Curve.P1 = FVector(1000, 0, 0);
	Curve.P2 = FVector(2000, 0, 0);
	Curve.P3 = FVector(3000, 0, 0);
	const FFlexArcLengthTable Table = FFlexBezierMath::BuildArcLengthTable(Curve);

	FFlexRoadMarkingParams Params = MakeParams();
	Params.ParkingSpotSpacing = 1000.f;
	Params.ParkingLineWidth = 10.f;

	auto MeasureExtents = [](const FFlexMeshSectionData& Section, float& OutExtentX, float& OutExtentY)
	{
		FVector Min(MAX_flt, MAX_flt, MAX_flt), Max(-MAX_flt, -MAX_flt, -MAX_flt);
		for (const FVector& Vertex : Section.Vertices)
		{
			Min = FVector::Min(Min, Vertex);
			Max = FVector::Max(Max, Vertex);
		}
		OutExtentX = Max.X - Min.X;
		OutExtentY = Max.Y - Min.Y;
	};

	// Straight curve along +X with world-up reference: Tangent = +X, Right = CrossProduct(Up, Tangent) = +Y.
	FRoadLaneDescriptor ParallelLane;
	ParallelLane.Type = EFlexLaneType::Parking;
	ParallelLane.Width = 250.f;
	ParallelLane.ParkingAngleDegrees = 0.f;
	FFlexMeshSectionData ParallelMarking;
	FFlexRoadMarkingBuilder::BuildParkingSpotMarkings(Curve, Table, ParallelLane, 0.f, FVector::UpVector, 0.f, 1999.f, Params, &ParallelMarking);
	float ParallelExtentX = 0.f, ParallelExtentY = 0.f;
	MeasureExtents(ParallelMarking, ParallelExtentX, ParallelExtentY);
	TestTrue(TEXT("At 0 deg, the divider's long axis (Lane.Width) runs laterally (+Y), dwarfing its own thickness along the road (+X)"), ParallelExtentY > ParallelExtentX);

	FRoadLaneDescriptor OrthogonalLane = ParallelLane;
	OrthogonalLane.ParkingAngleDegrees = 90.f;
	FFlexMeshSectionData OrthogonalMarking;
	FFlexRoadMarkingBuilder::BuildParkingSpotMarkings(Curve, Table, OrthogonalLane, 0.f, FVector::UpVector, 0.f, 1999.f, Params, &OrthogonalMarking);
	float OrthogonalExtentX = 0.f, OrthogonalExtentY = 0.f;
	MeasureExtents(OrthogonalMarking, OrthogonalExtentX, OrthogonalExtentY);
	TestTrue(TEXT("At 90 deg, the divider's long axis runs along the road (+X), dwarfing its own lateral thickness (+Y)"), OrthogonalExtentX > OrthogonalExtentY);

	return true;
}

// A non-Parking lane produces no parking-spot markings at all.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexParkingSpotNonParkingLaneTest, "FlexNetwork.Markings.ParkingSpotNoOpForNonParkingLane", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexParkingSpotNonParkingLaneTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMarkingBuilderTestHelpers;

	FFlexBezierCurve Curve;
	Curve.P0 = FVector(0, 0, 0);
	Curve.P1 = FVector(1000, 0, 0);
	Curve.P2 = FVector(2000, 0, 0);
	Curve.P3 = FVector(3000, 0, 0);
	const FFlexArcLengthTable Table = FFlexBezierMath::BuildArcLengthTable(Curve);

	FRoadLaneDescriptor Lane;
	Lane.Type = EFlexLaneType::Vehicle;
	Lane.Width = 350.f;

	FFlexMeshSectionData ParkingMarking;
	FFlexRoadMarkingBuilder::BuildParkingSpotMarkings(Curve, Table, Lane, 0.f, FVector::UpVector, 0.f, 3000.f, MakeParams(), &ParkingMarking);

	TestTrue(TEXT("A Vehicle lane produces no parking-spot markings"), ParkingMarking.IsEmpty());

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
