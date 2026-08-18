#include "Misc/AutomationTest.h"
#include "Math/FlexGeometry2D.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "FlexNetworkSubsystem.h"
#include "RoadTypeProfile.h"
#include "Editor.h"
#include "Engine/World.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexIsSimplePolygonTest, "FlexNetwork.Math.IsSimplePolygon", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexIsSimplePolygonTest::RunTest(const FString& Parameters)
{
	// A plain, convex square: no crossing edges.
	const TArray<FVector2D> Square = { FVector2D(0, 0), FVector2D(4, 0), FVector2D(4, 4), FVector2D(0, 4) };
	TestTrue(TEXT("A convex square is a simple polygon"), FlexGeometry2D::IsSimplePolygon(Square));

	// A bowtie: vertices ordered so edge (0,1) and edge (2,3) cross each other in the middle --
	// exactly the shape of the reported bug (two junction corners' arcs overlapping).
	const TArray<FVector2D> Bowtie = { FVector2D(0, 0), FVector2D(4, 4), FVector2D(4, 0), FVector2D(0, 4) };
	TestFalse(TEXT("A self-intersecting bowtie is not a simple polygon"), FlexGeometry2D::IsSimplePolygon(Bowtie));

	// A concave (but still simple) pentagon must still pass -- self-intersection and concavity
	// are different things.
	const TArray<FVector2D> ConcavePentagon = { FVector2D(0, 0), FVector2D(4, 0), FVector2D(2, 2), FVector2D(4, 4), FVector2D(0, 4) };
	TestTrue(TEXT("A concave-but-simple pentagon is still a simple polygon"), FlexGeometry2D::IsSimplePolygon(ConcavePentagon));

	// Fewer than 3 points can't form a polygon at all.
	const TArray<FVector2D> TwoPoints = { FVector2D(0, 0), FVector2D(1, 0) };
	TestFalse(TEXT("Fewer than 3 points is not a simple polygon"), FlexGeometry2D::IsSimplePolygon(TwoPoints));

	return true;
}

namespace FlexSafeIntersectionTestHelpers
{
	URoadTypeProfile* MakeSidewalkProfile(float LaneWidth, float SidewalkWidth)
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

		Profile->SidewalkWidth = SidewalkWidth;
		Profile->MinTurnRadius = 500.f;
		Profile->MinSegmentLengthOverride = 100.f;
		return Profile;
	}
}

// Reproduces the exact reported bug: an ordinary, perfectly symmetric, perpendicular 4-way
// intersection at default settings (RoadwayHalfWidth ~350cm, CurbReturnRadius 450cm by default)
// used to self-intersect at the node's center -- no acute angle or narrow road required, so no
// per-pair heuristic cap could ever have caught it. This asserts the junction polygon that comes
// out is always simple (no crossing edges), regardless of what CurbReturnRadius is configured to.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexSafeSymmetricFourWayTest, "FlexNetwork.Intersection.SafeSymmetricFourWay", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FFlexSafeSymmetricFourWayTest::RunTest(const FString& Parameters)
{
	using namespace FlexSafeIntersectionTestHelpers;

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

	URoadTypeProfile* Profile = MakeSidewalkProfile(350.f, 200.f);

	const FVector HubPos(0, 60000, 0);
	const FFlexNodeId Hub = Subsystem->AddNode(HubPos);
	const FFlexNodeId ArmN = Subsystem->AddNode(HubPos + FVector(0, 4000, 0));
	const FFlexNodeId ArmS = Subsystem->AddNode(HubPos + FVector(0, -4000, 0));
	const FFlexNodeId ArmE = Subsystem->AddNode(HubPos + FVector(4000, 0, 0));
	const FFlexNodeId ArmW = Subsystem->AddNode(HubPos + FVector(-4000, 0, 0));

	Subsystem->AddSegment(Hub, ArmN, HubPos + FVector(0, 700, 0), HubPos + FVector(0, 3300, 0), Profile);
	Subsystem->AddSegment(ArmS, Hub, HubPos + FVector(0, -3300, 0), HubPos + FVector(0, -700, 0), Profile);
	Subsystem->AddSegment(Hub, ArmE, HubPos + FVector(700, 0, 0), HubPos + FVector(3300, 0, 0), Profile);
	Subsystem->AddSegment(ArmW, Hub, HubPos + FVector(-3300, 0, 0), HubPos + FVector(-700, 0, 0), Profile);

	const FFlexJunctionData* Junction = Subsystem->GetJunctionData(Hub);
	if (!TestNotNull(TEXT("Symmetric 4-way junction data was built"), Junction))
	{
		return false;
	}

	TestTrue(TEXT("Junction has a drivable polygon"), Junction->PolygonBoundary.Num() >= 3);
	TestTrue(TEXT("Junction polygon triangulated successfully (not discarded as invalid)"), Junction->PolygonTriangleIndices.Num() >= 3);

	// PolygonBoundary is flat (all Z equal, since this test scene is unbanked) -- project straight
	// to 2D (X,Y) to run the same simplicity check the builder itself uses.
	TArray<FVector2D> Polygon2D;
	Polygon2D.Reserve(Junction->PolygonBoundary.Num());
	for (const FVector& P : Junction->PolygonBoundary)
	{
		Polygon2D.Add(FVector2D(P.X, P.Y));
	}
	TestTrue(TEXT("Junction polygon boundary is simple (no self-crossing edges) even at default CurbReturnRadius"), FlexGeometry2D::IsSimplePolygon(Polygon2D));

	return true;
}

// Two approaches meeting at a shallow angle (like two carriageways of a divided road) must not
// get a curb-return sidewalk band/island bridging them, even though both have sidewalks -- only
// genuine corners get sidewalk treatment. The same two-approach setup at a normal angle (90
// degrees) must still get islands on both sides, proving the gate is angle-specific rather than
// disabling curb-return entirely.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexParallelApproachNoSidewalkTest, "FlexNetwork.Intersection.ParallelApproachNoSidewalk", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FFlexParallelApproachNoSidewalkTest::RunTest(const FString& Parameters)
{
	using namespace FlexSafeIntersectionTestHelpers;

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

	URoadTypeProfile* Profile = MakeSidewalkProfile(350.f, 200.f);

	// Shallow-angle "V": two approaches splaying apart by only ~10 degrees, like two carriageways
	// of a divided road forking slightly, or simply continuing in nearly the same direction.
	{
		const FVector HubPos(0, 70000, 0);
		const FFlexNodeId Hub = Subsystem->AddNode(HubPos);
		const FVector DirA = FVector(FMath::Cos(FMath::DegreesToRadians(85.f)), FMath::Sin(FMath::DegreesToRadians(85.f)), 0).GetSafeNormal();
		const FVector DirB = FVector(FMath::Cos(FMath::DegreesToRadians(95.f)), FMath::Sin(FMath::DegreesToRadians(95.f)), 0).GetSafeNormal();
		const FFlexNodeId ArmA = Subsystem->AddNode(HubPos + DirA * 4000.f);
		const FFlexNodeId ArmB = Subsystem->AddNode(HubPos + DirB * 4000.f);
		Subsystem->AddSegment(Hub, ArmA, HubPos + DirA * 700.f, HubPos + DirA * 3300.f, Profile);
		Subsystem->AddSegment(Hub, ArmB, HubPos + DirB * 700.f, HubPos + DirB * 3300.f, Profile);

		const FFlexJunctionData* Junction = Subsystem->GetJunctionData(Hub);
		if (TestNotNull(TEXT("Shallow-angle junction data was built"), Junction))
		{
			TestEqual(TEXT("Near-parallel approaches get no curb-return sidewalk islands"), Junction->CornerIslands.Num(), 0);
		}
	}

	// Same setup, but at a normal ~90-degree angle -- islands should appear on both sides now,
	// confirming the gate above is about the angle, not some other factor disabling islands.
	{
		const FVector HubPos(0, 80000, 0);
		const FFlexNodeId Hub = Subsystem->AddNode(HubPos);
		const FVector DirA = FVector(1, 0, 0);
		const FVector DirB = FVector(0, 1, 0);
		const FFlexNodeId ArmA = Subsystem->AddNode(HubPos + DirA * 4000.f);
		const FFlexNodeId ArmB = Subsystem->AddNode(HubPos + DirB * 4000.f);
		Subsystem->AddSegment(Hub, ArmA, HubPos + DirA * 700.f, HubPos + DirA * 3300.f, Profile);
		Subsystem->AddSegment(Hub, ArmB, HubPos + DirB * 700.f, HubPos + DirB * 3300.f, Profile);

		const FFlexJunctionData* Junction = Subsystem->GetJunctionData(Hub);
		if (TestNotNull(TEXT("Perpendicular junction data was built"), Junction))
		{
			TestEqual(TEXT("Perpendicular approaches get curb-return islands on both sides"), Junction->CornerIslands.Num(), 2);
		}
	}

	return true;
}

// Two roads crossing at a very acute angle need substantially more setback than an orthogonal
// junction. The old width-based reach cap cut all four roads but then discarded the crossing
// polygon as self-intersecting, leaving exactly the empty center shown in the regression report.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexAcuteCrossingSurfaceTest, "FlexNetwork.Intersection.AcuteCrossingProducesSurfaceAndSidewalks", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FFlexAcuteCrossingSurfaceTest::RunTest(const FString& Parameters)
{
	using namespace FlexSafeIntersectionTestHelpers;
	if (!GEditor) return false;
	UWorld* World = GEditor->GetEditorWorldContext().World();
	UFlexNetworkSubsystem* Subsystem = World ? World->GetSubsystem<UFlexNetworkSubsystem>() : nullptr;
	if (!TestNotNull(TEXT("FlexNetwork subsystem available"), Subsystem)) return false;

	URoadTypeProfile* Profile = MakeSidewalkProfile(350.f, 200.f);
	const FVector HubPos(0, 90000, 0);
	const FFlexNodeId Hub = Subsystem->AddNode(HubPos);
	Subsystem->BeginBatchUpdate();
	for (const float AngleDegrees : { -7.5f, 7.5f, 172.5f, 187.5f })
	{
		const float Radians = FMath::DegreesToRadians(AngleDegrees);
		const FVector Dir(FMath::Cos(Radians), FMath::Sin(Radians), 0.f);
		const FFlexNodeId Arm = Subsystem->AddNode(HubPos + Dir * 12000.f);
		Subsystem->AddSegment(Hub, Arm, HubPos + Dir * 3500.f, HubPos + Dir * 8500.f, Profile);
	}
	Subsystem->EndBatchUpdate();

	const FFlexJunctionData* Junction = Subsystem->GetJunctionData(Hub);
	if (!TestNotNull(TEXT("Acute crossing junction exists"), Junction)) return false;
	TestTrue(TEXT("Acute crossing has a surface boundary"), Junction->PolygonBoundary.Num() >= 4);
	TestTrue(TEXT("Acute crossing surface triangulates"), Junction->PolygonTriangleIndices.Num() >= 3);
	TestTrue(TEXT("Acute crossing retains smooth sidewalk returns"), Junction->CornerIslands.Num() >= 2);
	for (const TPair<FFlexSegmentId, float>& Trim : Junction->TrimArcLengthBySegment)
	{
		const FFlexRoadSegment* Segment = Subsystem->GetSegment(Trim.Key);
		if (Segment)
		{
			const float DistanceFromNode = Segment->StartNodeId == Hub ? Trim.Value : Segment->GetLength() - Trim.Value;
			TestTrue(TEXT("Acute crossing consumes available road length for its setback"), DistanceFromNode > Profile->GetOuterExtent());
		}
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
