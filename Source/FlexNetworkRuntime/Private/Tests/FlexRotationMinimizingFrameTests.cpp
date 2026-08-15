#include "Misc/AutomationTest.h"
#include "Math/FlexBezierMath.h"
#include "Math/FlexRotationMinimizingFrame.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexRotationMinimizingFrameTest, "FlexNetwork.Math.RotationMinimizingFrameContinuity", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexRotationMinimizingFrameTest::RunTest(const FString& Parameters)
{
	// An S-curve: the control polygon crosses itself in a way that gives the curve a genuine
	// curvature sign change (inflection point) partway along -- exactly the case a naive Frenet
	// frame (Right/Up read directly off the second derivative) flips 180 degrees at.
	FFlexBezierCurve Curve;
	Curve.P0 = FVector(0, 0, 0);
	Curve.P1 = FVector(100, 300, 0);
	Curve.P2 = FVector(200, -300, 0);
	Curve.P3 = FVector(300, 0, 0);

	// Confirm this curve really does have an inflection (signed curvature changes sign) before
	// trusting the "no flip" assertion below to mean anything.
	const FVector D1Low = FFlexBezierMath::EvaluateDerivative(Curve, 0.25f);
	const FVector D2Low = FFlexBezierMath::EvaluateSecondDerivative(Curve, 0.25f);
	const FVector D1High = FFlexBezierMath::EvaluateDerivative(Curve, 0.75f);
	const FVector D2High = FFlexBezierMath::EvaluateSecondDerivative(Curve, 0.75f);
	const float SignedCurvatureLow = FVector::CrossProduct(D1Low, D2Low).Z;
	const float SignedCurvatureHigh = FVector::CrossProduct(D1High, D2High).Z;
	TestTrue(TEXT("Precondition: test curve has a genuine curvature sign change"), SignedCurvatureLow * SignedCurvatureHigh < 0.f);

	const FFlexArcLengthTable Table = FFlexBezierMath::BuildArcLengthTable(Curve, 1.f, 12);
	const TArray<FFlexCurveFrame> Frames = FFlexRotationMinimizingFrame::ComputeFrames(Curve, Table, 5.f, FVector::UpVector);
	TestTrue(TEXT("Enough frames sampled to be a meaningful continuity check"), Frames.Num() > 10);

	for (int32 i = 0; i + 1 < Frames.Num(); ++i)
	{
		const float RightDot = FVector::DotProduct(Frames[i].Right, Frames[i + 1].Right);
		TestTrue(TEXT("Consecutive Right vectors stay close (no flip) across the inflection point"), RightDot > 0.9f);

		// Right/Up/Tangent should stay an orthonormal frame at every sample.
		TestTrue(TEXT("Right is unit length"), FMath::IsNearlyEqual(Frames[i].Right.Size(), 1.f, 0.01f));
		TestTrue(TEXT("Right is perpendicular to Tangent"), FMath::IsNearlyEqual(FVector::DotProduct(Frames[i].Right, Frames[i].Tangent), 0.f, 0.01f));
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
