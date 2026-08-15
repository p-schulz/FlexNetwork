#pragma once

#include "CoreMinimal.h"
#include "FlexCurveTypes.h"

/** An orthonormal frame at one arc-length sample along a curve: Tangent (forward), Right (lateral), Up. */
struct FLEXNETWORKRUNTIME_API FFlexCurveFrame
{
	FVector Position = FVector::ZeroVector;
	FVector Tangent = FVector::ForwardVector;
	FVector Right = FVector::RightVector;
	FVector Up = FVector::UpVector;
	float ArcLength = 0.f;
};

/**
 * Rotation-minimizing frame (RMF) construction via the double reflection method (Wang, Juttler,
 * Zheng, Liu 2008). A naive Frenet frame (Right/Up derived directly from the curve's second
 * derivative every sample) flips 180 degrees at inflection points where curvature crosses zero,
 * producing a twisted mesh; the double reflection method instead propagates the frame from one
 * sample to the next via two mirror reflections, which stays continuous everywhere the tangent
 * itself is continuous, independent of curvature sign changes.
 */
class FLEXNETWORKRUNTIME_API FFlexRotationMinimizingFrame
{
public:
	/**
	 * Samples Curve at even arc-length steps (~SampleStep apart, always including both endpoints)
	 * using the supplied arc-length table, and returns one RMF-propagated frame per sample.
	 * ReferenceUp seeds the initial Right/Up axes at the first sample (typically the start node's
	 * up vector); it only matters for orientation about the tangent, not for RMF continuity.
	 */
	static TArray<FFlexCurveFrame> ComputeFrames(const FFlexBezierCurve& Curve, const FFlexArcLengthTable& Table, float SampleStep, const FVector& ReferenceUp = FVector::UpVector);

	/**
	 * Same propagation, but at an explicit (ascending) list of arc-length values instead of an
	 * even step -- used when a caller needs frames at specific points (a junction trim boundary,
	 * a single arbitrary query point) rather than a uniform sampling.
	 */
	static TArray<FFlexCurveFrame> ComputeFramesAtArcLengths(const FFlexBezierCurve& Curve, const FFlexArcLengthTable& Table, TConstArrayView<float> ArcLengths, const FVector& ReferenceUp = FVector::UpVector);
};
