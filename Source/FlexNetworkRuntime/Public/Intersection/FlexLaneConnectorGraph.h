#pragma once

#include "CoreMinimal.h"
#include "FlexNetworkTypes.h"
#include "FlexCurveTypes.h"
#include "FlexLaneConnectorGraph.generated.h"

/**
 * One drivable-lane-to-drivable-lane connection through a junction: a short Bezier curve from
 * where an incoming lane crosses the junction polygon boundary to where a legal outgoing lane
 * does. This -- not the junction's visible surface mesh -- is what pathfinding/traffic actually
 * follows through an intersection.
 */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FFlexLaneConnector
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	FFlexSegmentId FromSegment;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	int32 FromLaneIndex = INDEX_NONE;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	FFlexSegmentId ToSegment;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	int32 ToLaneIndex = INDEX_NONE;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	FFlexBezierCurve ConnectorCurve;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	float SpeedLimit = 0.f;
};

/** A rule-placed crosswalk/curb-cut strip where a sidewalk meets the junction polygon boundary. */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FFlexCrosswalkPlacement
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	FVector Center = FVector::ZeroVector;

	/** Unit direction a pedestrian walks along, crossing the road (perpendicular to the road this crosswalk crosses). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	FVector CrossingDirection = FVector::ForwardVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	float Width = 200.f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	float Length = 0.f;
};

/** Everything the intersection builder derives for one junction node: the visible polygon, per-approach trim distances, the lane-connector graph, and crosswalk placements. */
USTRUCT()
struct FLEXNETWORKRUNTIME_API FFlexJunctionData
{
	GENERATED_BODY()

	UPROPERTY()
	TArray<FVector> PolygonBoundary;

	UPROPERTY()
	TArray<int32> PolygonTriangleIndices;

	UPROPERTY()
	TMap<FFlexSegmentId, float> TrimArcLengthBySegment;

	UPROPERTY()
	TArray<FFlexLaneConnector> LaneConnectors;

	UPROPERTY()
	TArray<FFlexCrosswalkPlacement> Crosswalks;

	bool IsEmpty() const { return PolygonBoundary.Num() == 0; }
};
