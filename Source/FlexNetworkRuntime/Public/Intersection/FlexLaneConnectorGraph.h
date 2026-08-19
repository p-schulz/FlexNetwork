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

	/** Signed-lane travel direction change from entry to exit, in degrees (0=straight, 180=hairpin). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	float TurnAngleDegrees = 0.f;

	/** Convenience classification for consumers that apply special steering/speed behavior. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	bool bSharpTurn = false;
};

/** A rule-placed crosswalk/curb-cut strip where a sidewalk meets the junction polygon boundary. */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FFlexCrosswalkPlacement
{
	GENERATED_BODY()

	/** Approach whose roadway is crossed. Retained so complex regions can discard interior crossings. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	FFlexSegmentId SegmentId;

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

/**
 * A rounded corner refuge/median between two adjacent approaches' sidewalks, plus the constant-
 * width curved sidewalk band wrapped around its outside -- what turns "sidewalk stops abruptly at
 * the junction" into a proper curb return: the sidewalk sweeps around the island instead of just
 * ending. All three arcs are concentric (same Center/sweep, *decreasing* radius from
 * BandInnerArc -> BandOuterArc -> IslandOuterArc) -- the fillet Center sits on the far (pocket)
 * side of the curb line from the road, so moving away from the road (out of the pavement, into
 * the block corner) means moving *toward* Center, not away from it. They read as one continuously
 * curved feature rather than independently-rounded pieces that happen to be near each other.
 */
USTRUCT()
struct FLEXNETWORKRUNTIME_API FFlexJunctionCornerIsland
{
	GENERATED_BODY()

	/** Curb line -- coincides exactly with the junction polygon's own corner arc at this pairing. */
	UPROPERTY()
	TArray<FVector> BandInnerArc;

	/** Outer edge of the sidewalk band / inner edge of the landscaped island beyond it. */
	UPROPERTY()
	TArray<FVector> BandOuterArc;

	/** Outer edge of the landscaped island. */
	UPROPERTY()
	TArray<FVector> IslandOuterArc;

	/** Fillet circle center, shared by all three arcs. */
	UPROPERTY()
	FVector Center = FVector::ZeroVector;
};

/** Everything the intersection builder derives for one junction node: the visible polygon, per-approach trim distances, the lane-connector graph, crosswalk placements, and rounded sidewalk corner islands. */
USTRUCT()
struct FLEXNETWORKRUNTIME_API FFlexJunctionData
{
	GENERATED_BODY()

	UPROPERTY()
	TArray<FVector> PolygonBoundary;

	/**
	 * Parallel to PolygonBoundary -- true if the edge from this vertex to the next is genuine curb
	 * line, false if it's the "closing" edge between two *different* approaches' own edges (the
	 * short near-node edge connecting one approach's left curb point to its right one, needed to
	 * close the ring cleanly but running straight across the road rather than along it). Consumers
	 * that walk the boundary looking for curb features -- e.g. curbstone placement -- must skip the
	 * false edges, or they place a curbstone across the road at the trim boundary instead of along it.
	 */
	UPROPERTY()
	TArray<bool> PolygonEdgeIsCurbLine;

	UPROPERTY()
	TArray<int32> PolygonTriangleIndices;

	UPROPERTY()
	TMap<FFlexSegmentId, float> TrimArcLengthBySegment;

	UPROPERTY()
	TArray<FFlexLaneConnector> LaneConnectors;

	UPROPERTY()
	TArray<FFlexCrosswalkPlacement> Crosswalks;

	UPROPERTY()
	TArray<FFlexJunctionCornerIsland> CornerIslands;

	bool IsEmpty() const { return PolygonBoundary.Num() == 0; }
};
