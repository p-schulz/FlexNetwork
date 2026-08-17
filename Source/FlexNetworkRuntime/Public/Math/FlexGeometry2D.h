#pragma once

#include "CoreMinimal.h"

/**
 * 2D geometry helpers used by the intersection/junction builder: the junction polygon is
 * constructed by extrapolating each segment's outer cross-section edge as a ray toward the node
 * and intersecting (or filleting) pairs of angularly-adjacent rays, all worked out in the local
 * 2D plane of the junction (perpendicular to the node's up vector).
 */
namespace FlexGeometry2D
{
	/** Intersects two infinite lines (Origin + t*Direction). Returns false if (nearly) parallel. */
	FLEXNETWORKRUNTIME_API bool LineLineIntersection(const FVector2D& OriginA, const FVector2D& DirA, const FVector2D& OriginB, const FVector2D& DirB, FVector2D& OutPoint);

	/** Intersects two bounded segments [A0,A1] and [B0,B1]. Returns false if they don't cross within both segments' extents. */
	FLEXNETWORKRUNTIME_API bool SegmentSegmentIntersection(const FVector2D& A0, const FVector2D& A1, const FVector2D& B0, const FVector2D& B1, FVector2D& OutPoint, float& OutAlphaA, float& OutAlphaB);

	/** Twice the signed area of the polygon (positive = counter-clockwise winding). */
	FLEXNETWORKRUNTIME_API float SignedArea(TArrayView<const FVector2D> Polygon);

	/**
	 * Builds a rounded-corner arc between two rays that both emanate away from CornerPoint
	 * (DirAwayFromCornerA/B need not be normalized). Used when two adjacent segments' extrapolated
	 * outer edges don't meet at a usable convex angle for a sharp polygon corner. Returns false if
	 * the two directions are (nearly) collinear, in which case no corner rounding is meaningful.
	 */
	FLEXNETWORKRUNTIME_API bool ComputeFilletArc(const FVector2D& CornerPoint, const FVector2D& DirAwayFromCornerA, const FVector2D& DirAwayFromCornerB, float Radius, TArray<FVector2D>& OutArcPoints, int32 ArcSegments = 8);

	/**
	 * The circle-center/angle-range computation ComputeFilletArc does internally, exposed so a
	 * caller can sample *multiple* concentric radii around the same center/sweep -- e.g. a
	 * corner-island fill plus a constant-width sidewalk band wrapped around its outside, which
	 * only look like a single smoothly-curved feature if they share a center. Returns false on the
	 * same degenerate (collinear) cases ComputeFilletArc does.
	 */
	FLEXNETWORKRUNTIME_API bool ComputeFilletCenterAndSweep(const FVector2D& CornerPoint, const FVector2D& DirAwayFromCornerA, const FVector2D& DirAwayFromCornerB, float Radius, FVector2D& OutCenter, float& OutStartAngle, float& OutSweepAngle);

	/** Samples ArcSegments+1 points along a circular arc of the given Radius/Center, sweeping SweepAngle radians from StartAngle (as returned by ComputeFilletCenterAndSweep). */
	FLEXNETWORKRUNTIME_API void SampleArc(const FVector2D& Center, float StartAngle, float SweepAngle, float Radius, int32 ArcSegments, TArray<FVector2D>& OutPoints);

	/**
	 * True if no two non-adjacent edges of Polygon (treated as a closed loop) cross each other.
	 * An O(n^2) all-pairs check via SegmentSegmentIntersection -- fine for a junction polygon
	 * (at most a couple hundred vertices even with fine arc sampling) at the edit-time-only
	 * frequency this runs at. Not a complete simple-polygon test (perfectly overlapping/collinear
	 * coincident edges are a blind spot, since SegmentSegmentIntersection treats near-parallel
	 * segments as non-intersecting), but catches the actual crossing failures this is used for.
	 */
	FLEXNETWORKRUNTIME_API bool IsSimplePolygon(TArrayView<const FVector2D> Polygon);
}
