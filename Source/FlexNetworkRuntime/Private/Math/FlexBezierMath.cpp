#include "Math/FlexBezierMath.h"
#include "Algo/BinarySearch.h"

namespace
{
	// A single midpoint flatness check can be fooled by a curve that is point-symmetric about the
	// interval's exact midpoint (a plain S-curve is the common case: its t=0.5 point can land
	// exactly on the P0-P1 chord even though the curve bulges well away from that chord on either
	// side). Forcing at least this many subdivision levels before the flatness check is allowed to
	// accept an interval guarantees a handful of off-center samples get evaluated too, which such a
	// symmetric curve can't simultaneously fool at every level.
	constexpr int32 kMinSubdivisionDepth = 3;
}

FVector FFlexBezierMath::Evaluate(const FFlexBezierCurve& Curve, float T)
{
	const float U = 1.f - T;
	const float UU = U * U;
	const float UUU = UU * U;
	const float TT = T * T;
	const float TTT = TT * T;

	return (UUU * Curve.P0) + (3.f * UU * T * Curve.P1) + (3.f * U * TT * Curve.P2) + (TTT * Curve.P3);
}

FVector FFlexBezierMath::EvaluateDerivative(const FFlexBezierCurve& Curve, float T)
{
	const float U = 1.f - T;
	return 3.f * U * U * (Curve.P1 - Curve.P0)
		+ 6.f * U * T * (Curve.P2 - Curve.P1)
		+ 3.f * T * T * (Curve.P3 - Curve.P2);
}

FVector FFlexBezierMath::EvaluateSecondDerivative(const FFlexBezierCurve& Curve, float T)
{
	const float U = 1.f - T;
	return 6.f * U * (Curve.P2 - 2.f * Curve.P1 + Curve.P0)
		+ 6.f * T * (Curve.P3 - 2.f * Curve.P2 + Curve.P1);
}

float FFlexBezierMath::EstimateCurvature(const FFlexBezierCurve& Curve, float T)
{
	const FVector D1 = EvaluateDerivative(Curve, T);
	const FVector D2 = EvaluateSecondDerivative(Curve, T);
	const float Speed = D1.Size();
	if (Speed <= KINDA_SMALL_NUMBER)
	{
		return 0.f;
	}
	const float CrossMag = FVector::CrossProduct(D1, D2).Size();
	return CrossMag / (Speed * Speed * Speed);
}

void FFlexBezierMath::SubdivideRecursive(const FFlexBezierCurve& Curve, float T0, float T1, const FVector& P0, const FVector& P1, int32 Depth, int32 MaxDepth, float ChordTolerance, float& InOutRunningLength, TArray<FFlexArcLengthSample>& OutSamples)
{
	const float TMid = (T0 + T1) * 0.5f;
	const FVector PMid = Evaluate(Curve, TMid);

	const float ChordDeviation = FMath::PointDistToSegment(PMid, P0, P1);

	if (Depth >= MaxDepth || (Depth >= kMinSubdivisionDepth && ChordDeviation <= ChordTolerance))
	{
		InOutRunningLength += FVector::Dist(P0, PMid) + FVector::Dist(PMid, P1);
		OutSamples.Add(FFlexArcLengthSample{ T1, InOutRunningLength });
		return;
	}

	SubdivideRecursive(Curve, T0, TMid, P0, PMid, Depth + 1, MaxDepth, ChordTolerance, InOutRunningLength, OutSamples);
	SubdivideRecursive(Curve, TMid, T1, PMid, P1, Depth + 1, MaxDepth, ChordTolerance, InOutRunningLength, OutSamples);
}

FFlexArcLengthTable FFlexBezierMath::BuildArcLengthTable(const FFlexBezierCurve& Curve, float ChordTolerance, int32 MaxDepth)
{
	FFlexArcLengthTable Table;
	Table.Samples.Add(FFlexArcLengthSample{ 0.f, 0.f });

	float RunningLength = 0.f;
	SubdivideRecursive(Curve, 0.f, 1.f, Curve.P0, Curve.P3, 0, MaxDepth, ChordTolerance, RunningLength, Table.Samples);

	return Table;
}

float FFlexBezierMath::ArcLengthToT(const FFlexArcLengthTable& Table, float ArcLength)
{
	if (!Table.IsValid())
	{
		return 0.f;
	}

	const float TotalLength = Table.GetTotalLength();
	ArcLength = FMath::Clamp(ArcLength, 0.f, TotalLength);

	const int32 UpperIndex = FMath::Clamp(
		Algo::UpperBoundBy(Table.Samples, ArcLength, [](const FFlexArcLengthSample& Sample) { return Sample.ArcLength; }),
		1, Table.Samples.Num() - 1);

	const FFlexArcLengthSample& A = Table.Samples[UpperIndex - 1];
	const FFlexArcLengthSample& B = Table.Samples[UpperIndex];
	const float Span = B.ArcLength - A.ArcLength;
	const float Alpha = Span > KINDA_SMALL_NUMBER ? (ArcLength - A.ArcLength) / Span : 0.f;
	return FMath::Lerp(A.T, B.T, Alpha);
}

float FFlexBezierMath::TToArcLength(const FFlexArcLengthTable& Table, float T)
{
	if (!Table.IsValid())
	{
		return 0.f;
	}

	T = FMath::Clamp(T, 0.f, 1.f);

	const int32 UpperIndex = FMath::Clamp(
		Algo::UpperBoundBy(Table.Samples, T, [](const FFlexArcLengthSample& Sample) { return Sample.T; }),
		1, Table.Samples.Num() - 1);

	const FFlexArcLengthSample& A = Table.Samples[UpperIndex - 1];
	const FFlexArcLengthSample& B = Table.Samples[UpperIndex];
	const float Span = B.T - A.T;
	const float Alpha = Span > KINDA_SMALL_NUMBER ? (T - A.T) / Span : 0.f;
	return FMath::Lerp(A.ArcLength, B.ArcLength, Alpha);
}

void FFlexBezierMath::Subdivide(const FFlexBezierCurve& Curve, float T, FFlexBezierCurve& OutLeft, FFlexBezierCurve& OutRight)
{
	const FVector P01 = FMath::Lerp(Curve.P0, Curve.P1, T);
	const FVector P12 = FMath::Lerp(Curve.P1, Curve.P2, T);
	const FVector P23 = FMath::Lerp(Curve.P2, Curve.P3, T);
	const FVector P012 = FMath::Lerp(P01, P12, T);
	const FVector P123 = FMath::Lerp(P12, P23, T);
	const FVector P0123 = FMath::Lerp(P012, P123, T);

	OutLeft.P0 = Curve.P0;
	OutLeft.P1 = P01;
	OutLeft.P2 = P012;
	OutLeft.P3 = P0123;

	OutRight.P0 = P0123;
	OutRight.P1 = P123;
	OutRight.P2 = P23;
	OutRight.P3 = Curve.P3;
}

float FFlexBezierMath::ApplyEase(EFlexElevationEase Ease, float Alpha)
{
	Alpha = FMath::Clamp(Alpha, 0.f, 1.f);
	switch (Ease)
	{
	case EFlexElevationEase::Linear:
		return Alpha;
	case EFlexElevationEase::EaseIn:
		return Alpha * Alpha;
	case EFlexElevationEase::EaseOut:
		return 1.f - (1.f - Alpha) * (1.f - Alpha);
	case EFlexElevationEase::EaseInOut:
	default:
		return Alpha * Alpha * (3.f - 2.f * Alpha); // smoothstep
	}
}
