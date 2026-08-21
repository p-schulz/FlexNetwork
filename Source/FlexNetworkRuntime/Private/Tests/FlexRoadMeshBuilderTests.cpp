#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "RoadTypeProfile.h"
#include "Math/FlexBezierMath.h"
#include "Mesh/FlexRoadMeshBuilder.h"

namespace FlexRoadMeshBuilderTestHelpers
{
	FRoadLaneDescriptor MakeLane(float LateralOffset, float Width, EFlexLaneType Type)
	{
		FRoadLaneDescriptor Lane;
		Lane.LateralOffset = LateralOffset;
		Lane.Width = Width;
		Lane.Direction = EFlexLaneDirection::Forward;
		Lane.Type = Type;
		return Lane;
	}

	URoadTypeProfile* MakeProfile(TArray<FRoadLaneDescriptor> Lanes)
	{
		URoadTypeProfile* Profile = NewObject<URoadTypeProfile>(GetTransientPackage());
		Profile->Lanes = MoveTemp(Lanes);
		return Profile;
	}

	FFlexBezierCurve MakeStraightCurve()
	{
		FFlexBezierCurve Curve;
		Curve.P0 = FVector(0, 0, 0);
		Curve.P1 = FVector(1000, 0, 0);
		Curve.P2 = FVector(2000, 0, 0);
		Curve.P3 = FVector(3000, 0, 0);
		return Curve;
	}

	TArray<FFlexCurveFrame> MakeStraightFrames()
	{
		const FFlexBezierCurve Curve = MakeStraightCurve();
		const FFlexArcLengthTable Table = FFlexBezierMath::BuildArcLengthTable(Curve);
		return FFlexRoadMeshBuilder::BuildFramesForRange(Curve, Table, FVector::UpVector, 100.f, 0.f, Table.GetTotalLength());
	}
}

// Two Bike lanes with no other lane between them (a bidirectional cycle track split down the
// middle, say) share one contiguous overlay strip rather than producing two separate strips.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexBikeLaneOverlayMergedRunTest, "FlexNetwork.Mesh.BikeLaneOverlayMergesAdjacentRun", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexBikeLaneOverlayMergedRunTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMeshBuilderTestHelpers;

	URoadTypeProfile* Profile = MakeProfile({
		MakeLane(-100.f, 200.f, EFlexLaneType::Bike),
		MakeLane(100.f, 200.f, EFlexLaneType::Bike)
	});

	const TArray<FFlexCurveFrame> Frames = MakeStraightFrames();
	FFlexMeshSectionData Section;
	FFlexRoadMeshBuilder::AppendBikeLaneOverlay(Section, Frames, *Profile, 0.3f);

	TestFalse(TEXT("Adjacent Bike lanes produce overlay geometry"), Section.IsEmpty());
	// One merged strip spanning both lanes = one quad per consecutive frame pair.
	const int32 ExpectedQuads = Frames.Num() - 1;
	TestEqual(TEXT("Adjacent Bike lanes merge into a single strip (one quad per frame pair)"), Section.Triangles.Num() / 3, ExpectedQuads * 2);

	return true;
}

// A Vehicle lane sitting between two Bike lanes breaks them into two separate overlay strips
// instead of one strip spanning across the vehicle lane.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexBikeLaneOverlaySplitRunTest, "FlexNetwork.Mesh.BikeLaneOverlaySplitsAcrossNonBikeLane", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexBikeLaneOverlaySplitRunTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMeshBuilderTestHelpers;

	URoadTypeProfile* Profile = MakeProfile({
		MakeLane(-350.f, 200.f, EFlexLaneType::Bike),
		MakeLane(0.f, 350.f, EFlexLaneType::Vehicle),
		MakeLane(350.f, 200.f, EFlexLaneType::Bike)
	});

	const TArray<FFlexCurveFrame> Frames = MakeStraightFrames();
	FFlexMeshSectionData Section;
	FFlexRoadMeshBuilder::AppendBikeLaneOverlay(Section, Frames, *Profile, 0.3f);

	TestFalse(TEXT("Two separated Bike lanes still produce overlay geometry"), Section.IsEmpty());
	// Two separate strips = two quads per consecutive frame pair.
	const int32 ExpectedQuadsPerStrip = Frames.Num() - 1;
	TestEqual(TEXT("A Vehicle lane between two Bike lanes forces two separate strips"), Section.Triangles.Num() / 3, ExpectedQuadsPerStrip * 2 * 2);

	return true;
}

// The overlay's vertices sit exactly VerticalOffset above the roadway surface the input frames
// describe, along each frame's own Up axis -- the same technique road markings use to avoid
// z-fighting with the roadway beneath.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexBikeLaneOverlayVerticalOffsetTest, "FlexNetwork.Mesh.BikeLaneOverlayAppliesVerticalOffset", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexBikeLaneOverlayVerticalOffsetTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMeshBuilderTestHelpers;

	URoadTypeProfile* Profile = MakeProfile({
		MakeLane(0.f, 200.f, EFlexLaneType::Bike)
	});

	const TArray<FFlexCurveFrame> Frames = MakeStraightFrames();
	FFlexMeshSectionData Section;
	FFlexRoadMeshBuilder::AppendBikeLaneOverlay(Section, Frames, *Profile, 0.75f);

	TestFalse(TEXT("A single Bike lane produces overlay geometry"), Section.IsEmpty());
	for (const FVector& Vertex : Section.Vertices)
	{
		TestEqual(TEXT("Every overlay vertex sits VerticalOffset above the flat roadway plane"), static_cast<float>(Vertex.Z), 0.75f, KINDA_SMALL_NUMBER);
	}

	return true;
}

// A profile with no Bike-type lanes produces no overlay geometry at all.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexBikeLaneOverlayNoBikeLanesTest, "FlexNetwork.Mesh.BikeLaneOverlayNoOpWithoutBikeLanes", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexBikeLaneOverlayNoBikeLanesTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMeshBuilderTestHelpers;

	URoadTypeProfile* Profile = MakeProfile({
		MakeLane(-175.f, 350.f, EFlexLaneType::Vehicle),
		MakeLane(175.f, 350.f, EFlexLaneType::Vehicle)
	});

	const TArray<FFlexCurveFrame> Frames = MakeStraightFrames();
	FFlexMeshSectionData Section;
	FFlexRoadMeshBuilder::AppendBikeLaneOverlay(Section, Frames, *Profile, 0.3f);

	TestTrue(TEXT("A profile with no Bike lanes produces no overlay geometry"), Section.IsEmpty());

	return true;
}

// A single Median-type lane produces one raised top strip and two curb walls (one per long edge).
// Each of the three pieces gets one quad per consecutive frame pair, so their triangle counts must
// be exactly proportional (1:2 top-to-walls).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexMedianOverlayGeometryTest, "FlexNetwork.Mesh.MedianOverlayGeneratesTopAndWalls", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexMedianOverlayGeometryTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMeshBuilderTestHelpers;

	URoadTypeProfile* Profile = MakeProfile({
		MakeLane(0.f, 200.f, EFlexLaneType::Median)
	});

	const TArray<FFlexCurveFrame> Frames = MakeStraightFrames();
	FFlexMeshSectionData Top, Walls;
	FFlexRoadMeshBuilder::AppendMedianOverlay(Top, Walls, Frames, *Profile, 20.f);

	TestFalse(TEXT("A Median lane produces top geometry"), Top.IsEmpty());
	TestFalse(TEXT("A Median lane produces curb wall geometry"), Walls.IsEmpty());

	const int32 ExpectedQuadsPerStrip = Frames.Num() - 1;
	TestEqual(TEXT("Top is one strip"), Top.Triangles.Num() / 3, ExpectedQuadsPerStrip * 2);
	TestEqual(TEXT("Walls are two strips (one per long edge)"), Walls.Triangles.Num() / 3, ExpectedQuadsPerStrip * 2 * 2);

	// Every top vertex sits exactly MedianHeight above the flat roadway plane (straight, uncurved
	// input curve, same reasoning as the bike-lane vertical-offset test).
	for (const FVector& Vertex : Top.Vertices)
	{
		TestEqual(TEXT("Every top vertex sits MedianHeight above the roadway"), static_cast<float>(Vertex.Z), 20.f, KINDA_SMALL_NUMBER);
	}

	// Each wall quad contributes two "bottom" (Z=0) and two "top" (Z=MedianHeight) vertices,
	// regardless of which winding branch AppendVerticalCurbWall's auto-detection picked -- so across
	// all wall vertices, exactly half should sit at each height.
	int32 NumBottom = 0, NumTop = 0;
	for (const FVector& Vertex : Walls.Vertices)
	{
		if (FMath::IsNearlyEqual(static_cast<float>(Vertex.Z), 0.f, KINDA_SMALL_NUMBER)) ++NumBottom;
		else if (FMath::IsNearlyEqual(static_cast<float>(Vertex.Z), 20.f, KINDA_SMALL_NUMBER)) ++NumTop;
	}
	TestEqual(TEXT("Half the wall vertices sit at the roadway surface"), NumBottom, Walls.Vertices.Num() / 2);
	TestEqual(TEXT("Half the wall vertices sit at MedianHeight"), NumTop, Walls.Vertices.Num() / 2);

	return true;
}

// A Vehicle lane between two Median lanes splits them into two separate top strips / four separate
// wall strips, the same run-splitting behavior AppendBikeLaneOverlay already has.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexMedianOverlaySplitRunTest, "FlexNetwork.Mesh.MedianOverlaySplitsAcrossNonMedianLane", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexMedianOverlaySplitRunTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMeshBuilderTestHelpers;

	URoadTypeProfile* Profile = MakeProfile({
		MakeLane(-350.f, 200.f, EFlexLaneType::Median),
		MakeLane(0.f, 350.f, EFlexLaneType::Vehicle),
		MakeLane(350.f, 200.f, EFlexLaneType::Median)
	});

	const TArray<FFlexCurveFrame> Frames = MakeStraightFrames();
	FFlexMeshSectionData Top, Walls;
	FFlexRoadMeshBuilder::AppendMedianOverlay(Top, Walls, Frames, *Profile, 20.f);

	const int32 ExpectedQuadsPerStrip = Frames.Num() - 1;
	TestEqual(TEXT("Two separated Median lanes produce two top strips"), Top.Triangles.Num() / 3, ExpectedQuadsPerStrip * 2 * 2);
	TestEqual(TEXT("Two separated Median lanes produce four wall strips"), Walls.Triangles.Num() / 3, ExpectedQuadsPerStrip * 2 * 4);

	return true;
}

// A MedianHeight of 0 produces a flush top strip (no z-fighting risk since nothing else occupies
// that exact plane in this synthetic test) but no curb walls at all.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexMedianOverlayZeroHeightTest, "FlexNetwork.Mesh.MedianOverlayZeroHeightHasNoWalls", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexMedianOverlayZeroHeightTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMeshBuilderTestHelpers;

	URoadTypeProfile* Profile = MakeProfile({
		MakeLane(0.f, 200.f, EFlexLaneType::Median)
	});

	const TArray<FFlexCurveFrame> Frames = MakeStraightFrames();
	FFlexMeshSectionData Top, Walls;
	FFlexRoadMeshBuilder::AppendMedianOverlay(Top, Walls, Frames, *Profile, 0.f);

	TestFalse(TEXT("A flush (MedianHeight=0) Median lane still produces top geometry"), Top.IsEmpty());
	TestTrue(TEXT("A flush Median lane produces no curb walls"), Walls.IsEmpty());

	return true;
}

// AppendParkingLaneOverlay and AppendBikeLaneOverlay share the same underlying run-merging
// implementation, parameterized only by which EFlexLaneType they target -- this checks that
// targeting actually happens (a Parking lane is picked up, an adjacent Bike lane is not merged
// into the same run or picked up at all).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexParkingLaneOverlaySelectsParkingTypeTest, "FlexNetwork.Mesh.ParkingLaneOverlaySelectsParkingType", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexParkingLaneOverlaySelectsParkingTypeTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMeshBuilderTestHelpers;

	URoadTypeProfile* Profile = MakeProfile({
		MakeLane(-175.f, 200.f, EFlexLaneType::Parking),
		MakeLane(150.f, 200.f, EFlexLaneType::Bike)
	});

	const TArray<FFlexCurveFrame> Frames = MakeStraightFrames();

	FFlexMeshSectionData ParkingSection;
	FFlexRoadMeshBuilder::AppendParkingLaneOverlay(ParkingSection, Frames, *Profile, 0.3f);
	TestFalse(TEXT("A Parking lane produces parking-overlay geometry"), ParkingSection.IsEmpty());
	const int32 ExpectedQuads = Frames.Num() - 1;
	TestEqual(TEXT("The Parking lane alone forms a single strip (the adjacent Bike lane is not merged in)"), ParkingSection.Triangles.Num() / 3, ExpectedQuads * 2);

	FFlexMeshSectionData BikeSection;
	FFlexRoadMeshBuilder::AppendBikeLaneOverlay(BikeSection, Frames, *Profile, 0.3f);
	TestFalse(TEXT("The same profile's Bike lane still produces bike-overlay geometry"), BikeSection.IsEmpty());

	FFlexMeshSectionData VehicleOnlySection;
	URoadTypeProfile* VehicleOnlyProfile = MakeProfile({ MakeLane(0.f, 350.f, EFlexLaneType::Vehicle) });
	FFlexRoadMeshBuilder::AppendParkingLaneOverlay(VehicleOnlySection, Frames, *VehicleOnlyProfile, 0.3f);
	TestTrue(TEXT("A profile with no Parking lanes produces no parking-overlay geometry"), VehicleOnlySection.IsEmpty());

	return true;
}

// AppendParkingLaneCurbs produces two curb wall strips (one per long edge) and no top surface --
// the parking lane's own drivable surface is already part of the ordinary roadway.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexParkingLaneCurbsGeometryTest, "FlexNetwork.Mesh.ParkingLaneCurbsGenerateWallsOnly", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexParkingLaneCurbsGeometryTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMeshBuilderTestHelpers;

	URoadTypeProfile* Profile = MakeProfile({
		MakeLane(0.f, 200.f, EFlexLaneType::Parking)
	});

	const TArray<FFlexCurveFrame> Frames = MakeStraightFrames();
	FFlexMeshSectionData Walls;
	FFlexRoadMeshBuilder::AppendParkingLaneCurbs(Walls, Frames, *Profile, 15.f);

	TestFalse(TEXT("A Parking lane run produces curb wall geometry"), Walls.IsEmpty());
	const int32 ExpectedQuadsPerStrip = Frames.Num() - 1;
	TestEqual(TEXT("Two walls (one per long edge)"), Walls.Triangles.Num() / 3, ExpectedQuadsPerStrip * 2 * 2);

	// BaseVerticalOffset is 0 (roadway surface) for parking lane curbs, so every wall vertex sits at
	// either the roadway surface or WallHeight above it.
	int32 NumBottom = 0, NumTop = 0;
	for (const FVector& Vertex : Walls.Vertices)
	{
		if (FMath::IsNearlyEqual(static_cast<float>(Vertex.Z), 0.f, KINDA_SMALL_NUMBER)) ++NumBottom;
		else if (FMath::IsNearlyEqual(static_cast<float>(Vertex.Z), 15.f, KINDA_SMALL_NUMBER)) ++NumTop;
	}
	TestEqual(TEXT("Half the wall vertices sit at the roadway surface"), NumBottom, Walls.Vertices.Num() / 2);
	TestEqual(TEXT("Half the wall vertices sit at WallHeight"), NumTop, Walls.Vertices.Num() / 2);

	return true;
}

// A profile with no Parking lanes, or with bGenerateParkingLaneCurbs left off (checked at the
// caller, not inside AppendParkingLaneCurbs itself -- this test covers the "no Parking lanes" half
// of that gating directly), produces no curb geometry.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexParkingLaneCurbsNoOpTest, "FlexNetwork.Mesh.ParkingLaneCurbsNoOpWithoutParkingLanes", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexParkingLaneCurbsNoOpTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMeshBuilderTestHelpers;

	URoadTypeProfile* Profile = MakeProfile({
		MakeLane(-175.f, 350.f, EFlexLaneType::Vehicle),
		MakeLane(175.f, 350.f, EFlexLaneType::Vehicle)
	});

	const TArray<FFlexCurveFrame> Frames = MakeStraightFrames();
	FFlexMeshSectionData Walls;
	FFlexRoadMeshBuilder::AppendParkingLaneCurbs(Walls, Frames, *Profile, 15.f);

	TestTrue(TEXT("A profile with no Parking lanes produces no parking curb geometry"), Walls.IsEmpty());

	return true;
}

// Sidewalk tree patches: one small raised+curbed island every PatchSpacing interval, each with a
// top cap (1 quad) and four curb walls (2 long edges + 2 end caps, 1 quad each), sitting
// BaseVerticalOffset above the roadway and PatchHeight above that.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexSidewalkTreePatchesGeometryTest, "FlexNetwork.Mesh.SidewalkTreePatchesGenerateSpacedIslands", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexSidewalkTreePatchesGeometryTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMeshBuilderTestHelpers;

	const FFlexBezierCurve Curve = MakeStraightCurve();
	const FFlexArcLengthTable Table = FFlexBezierMath::BuildArcLengthTable(Curve);

	FFlexMeshSectionData Top, Walls;
	FFlexRoadMeshBuilder::AppendSidewalkTreePatches(Top, Walls, Curve, Table, FVector::UpVector,
		/*PatchLateralOffsetA*/ -300.f, /*PatchLateralOffsetB*/ -200.f,
		/*BaseVerticalOffset*/ 20.f, /*PatchHeight*/ 10.f, /*PatchLength*/ 150.f, /*PatchSpacing*/ 800.f,
		/*TrimStart*/ 0.f, /*TrimEnd*/ 3000.f);

	TestFalse(TEXT("Tree patches produce top geometry"), Top.IsEmpty());
	TestFalse(TEXT("Tree patches produce curb wall geometry"), Walls.IsEmpty());

	// Patch centers at 400, 1200, 2000, 2800 (TrimStart + Spacing/2, then every 800, staying below
	// TrimEnd=3000) -- four patches, none clipped by the trim range since each sits well inside it.
	constexpr int32 ExpectedPatchCount = 4;
	TestEqual(TEXT("One top quad (2 triangles) per patch"), Top.Triangles.Num() / 3, ExpectedPatchCount * 2);
	// Per patch: 2 long-edge walls + 2 end caps, one quad (2 triangles) each = 8 triangles/patch.
	TestEqual(TEXT("Four curb-wall quads per patch (2 long edges + 2 end caps)"), Walls.Triangles.Num() / 3, ExpectedPatchCount * 8);

	for (const FVector& Vertex : Top.Vertices)
	{
		TestEqual(TEXT("Every top vertex sits BaseVerticalOffset + PatchHeight above the roadway"), static_cast<float>(Vertex.Z), 30.f, KINDA_SMALL_NUMBER);
	}

	int32 NumBottom = 0, NumTop = 0;
	for (const FVector& Vertex : Walls.Vertices)
	{
		if (FMath::IsNearlyEqual(static_cast<float>(Vertex.Z), 20.f, KINDA_SMALL_NUMBER)) ++NumBottom;
		else if (FMath::IsNearlyEqual(static_cast<float>(Vertex.Z), 30.f, KINDA_SMALL_NUMBER)) ++NumTop;
	}
	TestEqual(TEXT("Half the wall vertices sit at BaseVerticalOffset (the sidewalk surface)"), NumBottom, Walls.Vertices.Num() / 2);
	TestEqual(TEXT("Half the wall vertices sit at BaseVerticalOffset + PatchHeight"), NumTop, Walls.Vertices.Num() / 2);

	return true;
}

// A degenerate patch width (both lateral offsets equal) produces no geometry at all.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexSidewalkTreePatchesZeroWidthTest, "FlexNetwork.Mesh.SidewalkTreePatchesNoOpForZeroWidth", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexSidewalkTreePatchesZeroWidthTest::RunTest(const FString& Parameters)
{
	using namespace FlexRoadMeshBuilderTestHelpers;

	const FFlexBezierCurve Curve = MakeStraightCurve();
	const FFlexArcLengthTable Table = FFlexBezierMath::BuildArcLengthTable(Curve);

	FFlexMeshSectionData Top, Walls;
	FFlexRoadMeshBuilder::AppendSidewalkTreePatches(Top, Walls, Curve, Table, FVector::UpVector,
		-300.f, -300.f, 20.f, 10.f, 150.f, 800.f, 0.f, 3000.f);

	TestTrue(TEXT("A zero-width patch span produces no top geometry"), Top.IsEmpty());
	TestTrue(TEXT("A zero-width patch span produces no wall geometry"), Walls.IsEmpty());

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
