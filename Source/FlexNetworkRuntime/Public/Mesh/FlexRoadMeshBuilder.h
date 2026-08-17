#pragma once

#include "CoreMinimal.h"
#include "FlexCurveTypes.h"
#include "RoadTypeProfile.h"
#include "Mesh/FlexMeshSectionData.h"
#include "Math/FlexRotationMinimizingFrame.h"

/**
 * Builds a segment's visible mesh (roadway strip + sidewalk offset strips) by walking the
 * segment's arc-length table in even steps, computing a rotation-minimizing frame at each
 * sample, and extruding the profile's lateral extents through those frames. Sidewalks reuse the
 * exact same frames as the roadway (offset further out laterally) rather than an independent
 * curve-offset computation, per the spec: a Bezier offset curve is not itself a Bezier curve in
 * general, so re-sampling the same RMF chain is both simpler and exactly consistent with the
 * road edge it has to butt up against.
 */
class FLEXNETWORKRUNTIME_API FFlexRoadMeshBuilder
{
public:
	/**
	 * TrimStartArcLength/TrimEndArcLength restrict extrusion to a sub-range of the curve (used to
	 * cut a segment's mesh off at a junction polygon boundary instead of extruding all the way to
	 * the node center); pass 0 and ArcLengthTable.GetTotalLength() for an untrimmed segment.
	 * Sidewalks are trimmed at the same points as the roadway -- the junction side is responsible
	 * for extending its own curb-return sidewalk bands out to meet that same point on each edge
	 * (see FFlexIntersectionBuilder::BuildJunctionCornersForRadius), so this trim doesn't need its
	 * own independent value.
	 */
	static FFlexSegmentMeshResult BuildSegmentMesh(
		const FFlexBezierCurve& Curve,
		const FFlexArcLengthTable& ArcLengthTable,
		const URoadTypeProfile* Profile,
		const FVector& ReferenceUp,
		float SampleStep,
		float TrimStartArcLength,
		float TrimEndArcLength);

	/** Frame at a single arc-length position, used both internally and by the subsystem's SampleSegmentAtArcLength query API. */
	static FFlexCurveFrame SampleFrameAtArcLength(const FFlexBezierCurve& Curve, const FFlexArcLengthTable& ArcLengthTable, float ArcLength, const FVector& ReferenceUp);

	/** Same even-step sampling BuildSegmentMesh uses internally, exposed so callers that need the raw frames (e.g. terrain conforming) stay exactly consistent with the mesh instead of re-deriving their own sampling. */
	static TArray<FFlexCurveFrame> BuildFramesForRange(const FFlexBezierCurve& Curve, const FFlexArcLengthTable& ArcLengthTable, const FVector& ReferenceUp, float SampleStep, float TrimStartArcLength, float TrimEndArcLength);

	static TArray<float> BuildSampleArcLengths(float TrimStart, float TrimEnd, float SampleStep);
};
