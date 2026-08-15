#include "Misc/AutomationTest.h"
#include "Math/FlexBezierMath.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexArcLengthTableTest, "FlexNetwork.Math.ArcLengthTable", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexArcLengthTableTest::RunTest(const FString& Parameters)
{
	// Straight-line curve: exact total length is known (the P0->P3 distance) regardless of subdivision.
	FFlexBezierCurve Straight;
	Straight.P0 = FVector(0, 0, 0);
	Straight.P1 = FVector(100, 0, 0);
	Straight.P2 = FVector(200, 0, 0);
	Straight.P3 = FVector(300, 0, 0);

	const FFlexArcLengthTable StraightTable = FFlexBezierMath::BuildArcLengthTable(Straight, 1.f, 10);
	TestTrue(TEXT("Table has at least the start/end samples"), StraightTable.Samples.Num() >= 2);
	TestTrue(TEXT("Straight-line arc length matches Euclidean distance"),
		FMath::IsNearlyEqual(StraightTable.GetTotalLength(), FVector::Dist(Straight.P0, Straight.P3), 0.5f));

	// Monotonicity, on a genuinely curved (non-degenerate) curve.
	FFlexBezierCurve Curved;
	Curved.P0 = FVector(0, 0, 0);
	Curved.P1 = FVector(0, 300, 0);
	Curved.P2 = FVector(300, 300, 0);
	Curved.P3 = FVector(300, 600, 0);
	const FFlexArcLengthTable CurvedTable = FFlexBezierMath::BuildArcLengthTable(Curved, 1.f, 12);

	for (int32 i = 1; i < CurvedTable.Samples.Num(); ++i)
	{
		TestTrue(TEXT("T is non-decreasing across samples"), CurvedTable.Samples[i].T >= CurvedTable.Samples[i - 1].T);
		TestTrue(TEXT("ArcLength strictly increases across samples"), CurvedTable.Samples[i].ArcLength > CurvedTable.Samples[i - 1].ArcLength);
	}

	// A curve is always at least as long as the straight-line distance between its endpoints, and
	// (for a curve that visibly bows away from that line, like this one) strictly longer.
	const float ChordLength = FVector::Dist(Curved.P0, Curved.P3);
	TestTrue(TEXT("Curved arc length exceeds the straight-line chord"), CurvedTable.GetTotalLength() > ChordLength);

	// Round-trip accuracy: ArcLengthToT then TToArcLength should return (approximately) the original arc length.
	const float TargetArcLength = CurvedTable.GetTotalLength() * 0.37f;
	const float T = FFlexBezierMath::ArcLengthToT(CurvedTable, TargetArcLength);
	TestTrue(TEXT("ArcLengthToT stays within [0,1]"), T >= 0.f && T <= 1.f);
	const float RoundTrippedArcLength = FFlexBezierMath::TToArcLength(CurvedTable, T);
	TestTrue(TEXT("ArcLengthToT -> TToArcLength round-trips within tolerance"),
		FMath::IsNearlyEqual(RoundTrippedArcLength, TargetArcLength, 3.f));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
