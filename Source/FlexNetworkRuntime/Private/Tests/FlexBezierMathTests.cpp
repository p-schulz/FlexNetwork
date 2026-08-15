#include "Misc/AutomationTest.h"
#include "Math/FlexBezierMath.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexBezierMathTest, "FlexNetwork.Math.BezierEvaluation", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexBezierMathTest::RunTest(const FString& Parameters)
{
	FFlexBezierCurve Curve;
	Curve.P0 = FVector(0, 0, 0);
	Curve.P1 = FVector(100, 0, 0);
	Curve.P2 = FVector(200, 0, 0);
	Curve.P3 = FVector(300, 0, 0);

	TestTrue(TEXT("Evaluate(0) == P0"), FFlexBezierMath::Evaluate(Curve, 0.f).Equals(Curve.P0, 0.01));
	TestTrue(TEXT("Evaluate(1) == P3"), FFlexBezierMath::Evaluate(Curve, 1.f).Equals(Curve.P3, 0.01));

	// Collinear, evenly-spaced control points make the cubic Bezier degenerate to the straight line P0->P3.
	TestTrue(TEXT("Straight, evenly-spaced control points evaluate exactly on the line at t=0.5"),
		FFlexBezierMath::Evaluate(Curve, 0.5f).Equals(FVector(150, 0, 0), 0.01));
	TestTrue(TEXT("Straight, evenly-spaced control points evaluate exactly on the line at t=0.25"),
		FFlexBezierMath::Evaluate(Curve, 0.25f).Equals(FVector(75, 0, 0), 0.01));

	const FVector Tangent = FFlexBezierMath::EvaluateDerivative(Curve, 0.5f).GetSafeNormal();
	TestTrue(TEXT("Tangent points along +X for a straight curve"), Tangent.Equals(FVector(1, 0, 0), 0.01));

	// Known-value check against the manual cubic Bernstein formula for a non-degenerate curve.
	FFlexBezierCurve Curved;
	Curved.P0 = FVector(0, 0, 0);
	Curved.P1 = FVector(0, 100, 0);
	Curved.P2 = FVector(100, 100, 0);
	Curved.P3 = FVector(100, 0, 0);
	const float T = 0.3f;
	const float U = 1.f - T;
	const FVector Expected = (U * U * U) * Curved.P0 + (3.f * U * U * T) * Curved.P1 + (3.f * U * T * T) * Curved.P2 + (T * T * T) * Curved.P3;
	TestTrue(TEXT("Evaluate matches the manual Bernstein polynomial formula"), FFlexBezierMath::Evaluate(Curved, T).Equals(Expected, 0.01));

	// De Casteljau split retraces the original curve exactly.
	FFlexBezierCurve Left, Right;
	FFlexBezierMath::Subdivide(Curved, 0.4f, Left, Right);
	TestTrue(TEXT("Subdivide: left piece starts at the original start"), Left.P0.Equals(Curved.P0, 0.01));
	TestTrue(TEXT("Subdivide: right piece ends at the original end"), Right.P3.Equals(Curved.P3, 0.01));
	TestTrue(TEXT("Subdivide: left/right pieces meet exactly at the split point"), Left.P3.Equals(Right.P0, 0.01));
	TestTrue(TEXT("Subdivide: split point matches direct evaluation at the same T"), Left.P3.Equals(FFlexBezierMath::Evaluate(Curved, 0.4f), 0.01));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
