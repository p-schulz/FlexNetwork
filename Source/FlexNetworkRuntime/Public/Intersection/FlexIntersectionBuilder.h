#pragma once

#include "CoreMinimal.h"
#include "FlexNetworkTypes.h"
#include "FlexCurveTypes.h"
#include "RoadTypeProfile.h"
#include "Intersection/FlexLaneConnectorGraph.h"
#include "Mesh/FlexMeshSectionData.h"
#include "FlexIntersectionBuilder.generated.h"

/** One segment's data as seen from a specific node it's connected to -- the input the intersection builder needs per approach. */
USTRUCT()
struct FLEXNETWORKRUNTIME_API FFlexJunctionApproachInput
{
	GENERATED_BODY()

	UPROPERTY()
	FFlexSegmentId SegmentId;

	/** True if the node this junction is being built for is the segment's EndNodeId; false if it's the StartNodeId. */
	UPROPERTY()
	bool bNodeIsSegmentEnd = false;

	UPROPERTY()
	FFlexBezierCurve Curve;

	UPROPERTY()
	FFlexArcLengthTable ArcLengthTable;

	UPROPERTY()
	TObjectPtr<URoadTypeProfile> Profile = nullptr;
};

/**
 * Builds a junction's visible polygon (from sorted, pairwise-intersected/filleted approach outer
 * edges), the trim distance each approach's mesh should stop at, the invisible lane-connector
 * graph pathfinding actually uses, and crosswalk placements -- everything described in spec
 * section 1.5. Pure function of its inputs; the subsystem is what decides *when* to call this.
 */
class FLEXNETWORKRUNTIME_API FFlexIntersectionBuilder
{
public:
	/**
	 * True if this node's connectivity requires an explicit junction polygon: three or more
	 * connected segments, or exactly two meeting at a sharp angle or a significant width
	 * mismatch (rather than a smooth pass-through bend).
	 */
	static bool NeedsJunction(const TArray<FFlexJunctionApproachInput>& Approaches, float StraightThroughAngleToleranceDegrees = 30.f, float WidthMismatchTolerance = 50.f);

	static FFlexJunctionData BuildJunction(
		const FVector& NodePosition,
		const FVector& NodeUp,
		const TArray<FFlexJunctionApproachInput>& Approaches,
		float DefaultFilletRadius,
		float CrosswalkWidth,
		float CrosswalkMinClearance,
		float CurbReturnRadius,
		float ParallelApproachAngleToleranceDegrees,
		int32 FilletArcSegments = 8,
		int32 CurbReturnArcSegments = 12);

	static FFlexJunctionMeshResult BuildJunctionMesh(const FVector& NodeUp, const FFlexJunctionData& JunctionData, UMaterialInterface* SurfaceMaterial, UMaterialInterface* CrosswalkMaterial, UMaterialInterface* SidewalkMaterial, UMaterialInterface* MedianMaterial);
};
