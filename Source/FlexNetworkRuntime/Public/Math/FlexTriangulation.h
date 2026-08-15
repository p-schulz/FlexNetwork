#pragma once

#include "CoreMinimal.h"

/**
 * Ear-clipping triangulation for simple (non-self-intersecting) 2D polygons, including concave
 * ones -- required for junction polygons, which are frequently concave (e.g. a 4-way junction
 * with very mismatched road widths). Implemented in-house rather than fan triangulation because
 * fan triangulation only works for convex polygons, which junction polygons are not guaranteed
 * to be; O(n^2) worst case, which is fine at junction-polygon vertex counts (single digits to
 * low tens).
 */
namespace FlexTriangulation
{
	/**
	 * Triangulates Polygon (any winding order accepted; internally normalized to CCW). Appends
	 * three indices per triangle (into Polygon) to OutTriangleIndices, in the same (CCW) winding
	 * as the normalized polygon. Returns false if the polygon is degenerate or clipping stalls
	 * (e.g. self-intersecting input) -- callers should treat that as "no usable junction surface"
	 * rather than crash.
	 */
	FLEXNETWORKRUNTIME_API bool EarClipTriangulate(TArrayView<const FVector2D> Polygon, TArray<int32>& OutTriangleIndices);
}
