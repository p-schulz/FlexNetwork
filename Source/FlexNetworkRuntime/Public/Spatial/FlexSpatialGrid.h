#pragma once

#include "CoreMinimal.h"
#include "FlexNetworkTypes.h"

/**
 * Uniform-grid spatial index over node positions and segment bounding boxes, in the network's
 * horizontal (X/Y) plane. Backs every snap/crossing query the subsystem and the drawing tool
 * perform (nearby-node snap, nearby-segment-midspan snap, drag-crosses-existing-segment split
 * detection) so those stay O(nearby items) instead of O(all nodes/segments) per mouse-move.
 */
class FLEXNETWORKRUNTIME_API FFlexSpatialGrid
{
public:
	explicit FFlexSpatialGrid(float InCellSize = 2000.f);

	void Clear();
	void SetCellSize(float InCellSize);

	void AddNode(FFlexNodeId Id, const FVector2D& Position);
	void RemoveNode(FFlexNodeId Id, const FVector2D& Position);
	void UpdateNode(FFlexNodeId Id, const FVector2D& OldPosition, const FVector2D& NewPosition);

	/** BoundsMin/Max should be the segment's Bezier control-point AABB (a cubic Bezier always lies within the convex hull of its control points, so their AABB is a valid conservative bound). */
	void AddSegment(FFlexSegmentId Id, const FVector2D& BoundsMin, const FVector2D& BoundsMax);
	void RemoveSegment(FFlexSegmentId Id, const FVector2D& BoundsMin, const FVector2D& BoundsMax);
	void UpdateSegment(FFlexSegmentId Id, const FVector2D& OldBoundsMin, const FVector2D& OldBoundsMax, const FVector2D& NewBoundsMin, const FVector2D& NewBoundsMax);

	/** Node IDs whose cell (or an adjacent cell, so nothing near a cell boundary is missed) falls within Radius of Center. Caller still needs an exact distance check on the returned candidates. */
	TArray<FFlexNodeId> QueryNodesNear(const FVector2D& Center, float Radius) const;

	/** Segment IDs whose bounding-box cell footprint overlaps a Center +/- Radius square. Conservative -- caller still needs an exact test against the actual curve. */
	TArray<FFlexSegmentId> QuerySegmentsNear(const FVector2D& Center, float Radius) const;

private:
	struct FCellKey
	{
		int32 X = 0;
		int32 Y = 0;

		bool operator==(const FCellKey& Other) const { return X == Other.X && Y == Other.Y; }
		friend uint32 GetTypeHash(const FCellKey& Key) { return HashCombine(::GetTypeHash(Key.X), ::GetTypeHash(Key.Y)); }
	};

	FCellKey CellOf(const FVector2D& Position) const;
	void ForEachCellInBounds(const FVector2D& BoundsMin, const FVector2D& BoundsMax, TFunctionRef<void(const FCellKey&)> Fn) const;

	float CellSize;
	TMap<FCellKey, TArray<FFlexNodeId>> NodeCells;
	TMap<FCellKey, TArray<FFlexSegmentId>> SegmentCells;
};
