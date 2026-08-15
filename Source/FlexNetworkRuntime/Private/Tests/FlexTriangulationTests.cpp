#include "Misc/AutomationTest.h"
#include "Math/FlexTriangulation.h"
#include "Math/FlexGeometry2D.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexTriangulationTest, "FlexNetwork.Math.ConcaveTriangulation", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexTriangulationTest::RunTest(const FString& Parameters)
{
	// A simple concave pentagon (one reflex vertex at index 2, pulled inward) -- representative
	// of the kind of shape a junction polygon with very unevenly-spaced approaches can produce,
	// which fan triangulation cannot handle correctly but ear-clipping can.
	const TArray<FVector2D> Polygon = {
		FVector2D(0, 0),
		FVector2D(4, 0),
		FVector2D(2, 2), // reflex vertex
		FVector2D(4, 4),
		FVector2D(0, 4)
	};

	TArray<int32> Triangles;
	const bool bSuccess = FlexTriangulation::EarClipTriangulate(Polygon, Triangles);
	TestTrue(TEXT("Triangulation succeeds on a simple concave polygon"), bSuccess);
	TestEqual(TEXT("A simple 5-vertex polygon triangulates into exactly 3 triangles"), Triangles.Num(), 9);

	// Every triangle's vertex indices must be valid and distinct.
	for (int32 i = 0; i + 2 < Triangles.Num(); i += 3)
	{
		const int32 A = Triangles[i], B = Triangles[i + 1], C = Triangles[i + 2];
		TestTrue(TEXT("Triangle indices are in range"), Polygon.IsValidIndex(A) && Polygon.IsValidIndex(B) && Polygon.IsValidIndex(C));
		TestTrue(TEXT("Triangle has three distinct vertices"), A != B && B != C && A != C);
	}

	// Correctness check: the triangulation's total area must equal the polygon's own area --
	// this fails if ear-clipping produces overlapping/missing triangles on a concave shape.
	float TriangleAreaSum = 0.f;
	for (int32 i = 0; i + 2 < Triangles.Num(); i += 3)
	{
		const FVector2D& A = Polygon[Triangles[i]];
		const FVector2D& B = Polygon[Triangles[i + 1]];
		const FVector2D& C = Polygon[Triangles[i + 2]];
		TriangleAreaSum += FMath::Abs(FVector2D::CrossProduct(B - A, C - A)) * 0.5f;
	}
	const float PolygonArea = FMath::Abs(FlexGeometry2D::SignedArea(Polygon));
	TestTrue(TEXT("Sum of triangle areas equals the polygon's area"), FMath::IsNearlyEqual(TriangleAreaSum, PolygonArea, 0.01f));

	// Degenerate input: fewer than 3 points must fail cleanly rather than crash.
	TArray<int32> DegenerateTriangles;
	const TArray<FVector2D> TwoPoints = { FVector2D(0, 0), FVector2D(1, 0) };
	TestFalse(TEXT("Triangulating fewer than 3 points fails cleanly"), FlexTriangulation::EarClipTriangulate(TwoPoints, DegenerateTriangles));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
