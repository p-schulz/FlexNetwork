#pragma once

#include "CoreMinimal.h"
#include "FlexCurveTypes.h"

/**
 * Static cubic-Bezier math: point/derivative evaluation and the adaptive t<->arc-length lookup
 * table that every arc-length-based sampling operation (mesh extrusion, RMF stepping, prop
 * placement) builds on. The raw parameter t of a cubic Bezier is not proportional to distance
 * travelled, so anything that needs even spacing must go through the table built here.
 */
class FLEXNETWORKRUNTIME_API FFlexBezierMath
{
public:
	/** Evaluates the curve position at parameter T in [0,1] via the direct cubic Bernstein form. */
	static FVector Evaluate(const FFlexBezierCurve& Curve, float T);

	/** First derivative (unnormalized tangent) with respect to T. */
	static FVector EvaluateDerivative(const FFlexBezierCurve& Curve, float T);

	/** Second derivative with respect to T, used for curvature estimation. */
	static FVector EvaluateSecondDerivative(const FFlexBezierCurve& Curve, float T);

	/** Signed curvature magnitude (1/radius) at T. Returns 0 for a straight/degenerate curve. */
	static float EstimateCurvature(const FFlexBezierCurve& Curve, float T);

	/**
	 * Builds a monotonically-increasing t->arc-length table via adaptive recursive subdivision:
	 * a candidate interval is accepted once its midpoint deviates from the interval's chord by
	 * less than ChordTolerance (or MaxDepth is reached), otherwise it is split at its midpoint
	 * and each half is tested again.
	 */
	static FFlexArcLengthTable BuildArcLengthTable(const FFlexBezierCurve& Curve, float ChordTolerance = 2.f, int32 MaxDepth = 10);

	/** Maps an arc-length distance in [0, TotalLength] to the corresponding curve parameter T, via binary search + local linear interpolation in the table. */
	static float ArcLengthToT(const FFlexArcLengthTable& Table, float ArcLength);

	/** Maps a curve parameter T in [0,1] to the corresponding arc length, via binary search + local linear interpolation in the table. */
	static float TToArcLength(const FFlexArcLengthTable& Table, float T);

	/** Applies an easing curve to Alpha in [0,1]; used to blend node elevation along a segment instead of linear interpolation. */
	static float ApplyEase(EFlexElevationEase Ease, float Alpha);

	/** Exact De Casteljau split of Curve at parameter T into two cubic Beziers that together retrace the original curve -- used by SplitSegment to insert a node mid-curve without changing its shape. */
	static void Subdivide(const FFlexBezierCurve& Curve, float T, FFlexBezierCurve& OutLeft, FFlexBezierCurve& OutRight);

private:
	static void SubdivideRecursive(const FFlexBezierCurve& Curve, float T0, float T1, const FVector& P0, const FVector& P1, int32 Depth, int32 MaxDepth, float ChordTolerance, float& InOutRunningLength, TArray<FFlexArcLengthSample>& OutSamples);
};
