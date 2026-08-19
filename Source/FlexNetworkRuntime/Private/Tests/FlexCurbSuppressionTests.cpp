#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Mesh/FlexUnifiedRoadMeshBuilder.h"

namespace
{
	int32 CountTriangles(const TArray<FFlexMeshSectionData>& Sections)
	{
		int32 Count = 0;
		for (const FFlexMeshSectionData& Section : Sections)
		{
			Count += Section.Triangles.Num() / 3;
		}
		return Count;
	}

	double Cross2D(const FVector2D& A, const FVector2D& B, const FVector2D& C)
	{
		return (B.X - A.X) * (C.Y - A.Y) - (B.Y - A.Y) * (C.X - A.X);
	}

	bool TriangleContainsPoint2D(const FVector2D& Point, const FVector& A, const FVector& B, const FVector& C)
	{
		const FVector2D A2(A.X, A.Y);
		const FVector2D B2(B.X, B.Y);
		const FVector2D C2(C.X, C.Y);
		if (FMath::Abs(Cross2D(A2, B2, C2)) <= UE_DOUBLE_SMALL_NUMBER)
		{
			return false;
		}
		const double SideA = Cross2D(A2, B2, Point);
		const double SideB = Cross2D(B2, C2, Point);
		const double SideC = Cross2D(C2, A2, Point);
		const bool bHasNegative = SideA < -UE_DOUBLE_SMALL_NUMBER || SideB < -UE_DOUBLE_SMALL_NUMBER || SideC < -UE_DOUBLE_SMALL_NUMBER;
		const bool bHasPositive = SideA > UE_DOUBLE_SMALL_NUMBER || SideB > UE_DOUBLE_SMALL_NUMBER || SideC > UE_DOUBLE_SMALL_NUMBER;
		return !(bHasNegative && bHasPositive);
	}

	bool MeshContainsPoint2D(const TArray<FFlexMeshSectionData>& Sections, const FVector2D& Point)
	{
		for (const FFlexMeshSectionData& Section : Sections)
		{
			for (int32 Index = 0; Index + 2 < Section.Triangles.Num(); Index += 3)
			{
				const int32 A = Section.Triangles[Index];
				const int32 B = Section.Triangles[Index + 1];
				const int32 C = Section.Triangles[Index + 2];
				if (Section.Vertices.IsValidIndex(A) && Section.Vertices.IsValidIndex(B) && Section.Vertices.IsValidIndex(C)
					&& TriangleContainsPoint2D(Point, Section.Vertices[A], Section.Vertices[B], Section.Vertices[C]))
				{
					return true;
				}
			}
		}
		return false;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexCrosswalkCurbSuppressionTest,
	"FlexNetwork.Geometry.CrosswalkSuppressesCurbsOnly",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexCrosswalkCurbSuppressionTest::RunTest(const FString& Parameters)
{
	FFlexUnifiedRoadPolygonInput Road;
	Road.Boundary = {
		FVector(-500.f, -200.f, 0.f), FVector(500.f, -200.f, 0.f),
		FVector(500.f, 200.f, 0.f), FVector(-500.f, 200.f, 0.f)
	};
	Road.SidewalkWidth = 100.f;
	Road.CurbHeight = 15.f;

	const TArray<FFlexUnifiedRoadPolygonInput> Roads{ Road };
	const TArray<FFlexUnifiedRoadSuppressionInput> NoSuppressions;
	const FFlexUnifiedNetworkMeshResult Baseline = FFlexUnifiedRoadMeshBuilder::Build(Roads, NoSuppressions);

	FFlexUnifiedRoadSuppressionInput Crosswalk;
	Crosswalk.Boundary = {
		FVector(-150.f, -300.f, 0.f), FVector(150.f, -300.f, 0.f),
		FVector(150.f, 300.f, 0.f), FVector(-150.f, 300.f, 0.f)
	};
	Crosswalk.bSuppressSidewalks = false;
	Crosswalk.bSuppressCurbs = true;
	const TArray<FFlexUnifiedRoadSuppressionInput> CrosswalkSuppressions{ Crosswalk };
	const FFlexUnifiedNetworkMeshResult Result = FFlexUnifiedRoadMeshBuilder::Build(Roads, CrosswalkSuppressions);

	TestTrue(TEXT("Crosswalk removes at least one curb chord"), Result.CurbLines.Num() < Baseline.CurbLines.Num());
	TestEqual(TEXT("Curb-only suppression does not remove sidewalk triangles"),
		CountTriangles(Result.Sidewalks), CountTriangles(Baseline.Sidewalks));
	for (const TArray<FVector>& Line : Result.CurbLines)
	{
		if (Line.Num() >= 2)
		{
			const FVector Midpoint = (Line[0] + Line[1]) * 0.5f;
			TestFalse(TEXT("No surviving curb chord has its midpoint in the crosswalk clearance"),
				FMath::Abs(Midpoint.X) < 150.f && FMath::Abs(Midpoint.Y) < 300.f);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexMinimumGeneratedPolygonAreaTest,
	"FlexNetwork.Geometry.FiltersTinyBooleanComponents",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexMinimumGeneratedPolygonAreaTest::RunTest(const FString& Parameters)
{
	FFlexUnifiedRoadPolygonInput MainRoad;
	MainRoad.Boundary = {
		FVector(-500.f, -200.f, 0.f), FVector(500.f, -200.f, 0.f),
		FVector(500.f, 200.f, 0.f), FVector(-500.f, 200.f, 0.f)
	};

	FFlexUnifiedRoadPolygonInput TinySliver;
	TinySliver.Boundary = {
		FVector(1000.f, 0.f, 0.f), FVector(1025.f, 0.f, 0.f),
		FVector(1025.f, 25.f, 0.f), FVector(1000.f, 25.f, 0.f)
	};

	const TArray<FFlexUnifiedRoadPolygonInput> Roads{ MainRoad, TinySliver };
	const TArray<FFlexUnifiedRoadSuppressionInput> NoSuppressions;
	const FFlexUnifiedNetworkMeshResult Unfiltered = FFlexUnifiedRoadMeshBuilder::Build(Roads, NoSuppressions, 0.0);
	const FFlexUnifiedNetworkMeshResult Filtered = FFlexUnifiedRoadMeshBuilder::Build(Roads, NoSuppressions, 10000.0);

	TestTrue(TEXT("Area threshold removes the isolated 25cm boolean component"),
		CountTriangles(Filtered.Roadways) < CountTriangles(Unfiltered.Roadways));
	TestTrue(TEXT("The main road component remains after filtering"), CountTriangles(Filtered.Roadways) > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexMixedWindingRoadUnionTest,
	"FlexNetwork.Geometry.NormalizesMixedRoadWinding",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexMixedWindingRoadUnionTest::RunTest(const FString& Parameters)
{
	FFlexUnifiedRoadPolygonInput HorizontalRoad;
	HorizontalRoad.Boundary = {
		FVector(-500.f, -100.f, 0.f), FVector(500.f, -100.f, 0.f),
		FVector(500.f, 100.f, 0.f), FVector(-500.f, 100.f, 0.f)
	};
	HorizontalRoad.SidewalkWidth = 100.f;
	HorizontalRoad.CurbHeight = 15.f;

	FFlexUnifiedRoadPolygonInput VerticalRoad;
	// Deliberately clockwise. Without input normalization, Clipper's NonZero fill rule cancels
	// this road against the counterclockwise horizontal road throughout their overlap.
	VerticalRoad.Boundary = {
		FVector(-100.f, -500.f, 0.f), FVector(-100.f, 500.f, 0.f),
		FVector(100.f, 500.f, 0.f), FVector(100.f, -500.f, 0.f)
	};
	VerticalRoad.SidewalkWidth = 100.f;
	VerticalRoad.CurbHeight = 15.f;

	const TArray<FFlexUnifiedRoadPolygonInput> Roads{ HorizontalRoad, VerticalRoad };
	const TArray<FFlexUnifiedRoadSuppressionInput> NoSuppressions;
	const FFlexUnifiedNetworkMeshResult Result = FFlexUnifiedRoadMeshBuilder::Build(Roads, NoSuppressions);

	TestTrue(TEXT("Opposite-wound road footprints remain filled at their intersection"),
		MeshContainsPoint2D(Result.Roadways, FVector2D::ZeroVector));
	TestFalse(TEXT("The sidewalk difference does not fill the roadway intersection"),
		MeshContainsPoint2D(Result.Sidewalks, FVector2D::ZeroVector));
	return true;
}

#endif
