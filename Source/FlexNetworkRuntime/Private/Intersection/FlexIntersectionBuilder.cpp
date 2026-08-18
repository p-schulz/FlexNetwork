#include "Intersection/FlexIntersectionBuilder.h"
#include "Math/FlexBezierMath.h"
#include "Math/FlexGeometry2D.h"
#include "Math/FlexTriangulation.h"
#include "Mesh/FlexRoadMeshBuilder.h"
#include "Algo/Sort.h"
#include "Algo/Accumulate.h"

namespace
{
	/** Outward-from-node unit tangent (3D) for one approach. */
	FVector GetOutwardTangent(const FFlexJunctionApproachInput& Approach)
	{
		const FVector Tangent = Approach.bNodeIsSegmentEnd
			? FFlexBezierMath::EvaluateDerivative(Approach.Curve, 1.f)
			: FFlexBezierMath::EvaluateDerivative(Approach.Curve, 0.f);
		const FVector Normalized = Tangent.GetSafeNormal(UE_SMALL_NUMBER, FVector::ForwardVector);
		return Approach.bNodeIsSegmentEnd ? -Normalized : Normalized;
	}

	struct FLocalBasis
	{
		FVector AxisX;
		FVector AxisY;
		FVector Up;

		FVector2D To2D(const FVector& WorldOffset) const { return FVector2D(FVector::DotProduct(WorldOffset, AxisX), FVector::DotProduct(WorldOffset, AxisY)); }
		FVector To3D(const FVector& Origin, const FVector2D& Local) const { return Origin + AxisX * Local.X + AxisY * Local.Y; }
	};

	FLocalBasis MakeLocalBasis(const FVector& Up)
	{
		FLocalBasis Basis;
		Basis.Up = Up.GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
		FVector AxisX = FVector::VectorPlaneProject(FVector::ForwardVector, Basis.Up);
		if (AxisX.IsNearlyZero())
		{
			AxisX = FVector::VectorPlaneProject(FVector::RightVector, Basis.Up);
		}
		Basis.AxisX = AxisX.GetSafeNormal(UE_SMALL_NUMBER, FVector::ForwardVector);
		Basis.AxisY = FVector::CrossProduct(Basis.Up, Basis.AxisX).GetSafeNormal(UE_SMALL_NUMBER, FVector::RightVector);
		return Basis;
	}

	struct FSortedApproach
	{
		int32 ApproachIndex = INDEX_NONE;
		FVector2D Direction2D = FVector2D::ZeroVector;
		float Angle = 0.f;
		float OuterExtent = 0.f;
		float RoadwayHalfWidth = 0.f;
		float SidewalkWidth = 0.f;
		float CurbHeight = 0.f;
	};

	bool IsDrivableLane(const FRoadLaneDescriptor& Lane)
	{
		return Lane.IsDrivable() && Lane.Direction != EFlexLaneDirection::None;
	}

	// Physical direction of travel for a lane entering (Incoming) or leaving (Outgoing) the
	// junction at this approach is determined purely by which end of the segment touches the
	// node -- Forward/Backward on the lane just picks which of those two roles it plays.
	void GetLaneRoles(const FRoadLaneDescriptor& Lane, bool bNodeIsSegmentEnd, bool& bOutIncoming, bool& bOutOutgoing)
	{
		bOutIncoming = false;
		bOutOutgoing = false;
		if (!IsDrivableLane(Lane))
		{
			return;
		}

		switch (Lane.Direction)
		{
		case EFlexLaneDirection::Forward:
			bOutIncoming = bNodeIsSegmentEnd;
			bOutOutgoing = !bNodeIsSegmentEnd;
			break;
		case EFlexLaneDirection::Backward:
			bOutIncoming = !bNodeIsSegmentEnd;
			bOutOutgoing = bNodeIsSegmentEnd;
			break;
		case EFlexLaneDirection::Bidirectional:
			bOutIncoming = true;
			bOutOutgoing = true;
			break;
		default:
			break;
		}
	}

	/**
	 * One angularly-adjacent pair's raw, unclipped contribution to the drivable polygon ring --
	 * points ordered from the ApproachA-facing end to the ApproachB-facing end (a single point for
	 * a sharp corner or the straight-through/near-parallel fallback, several for a curb-return or
	 * DefaultFilletRadius arc). "Raw" because it's built purely from this one pair's own geometry,
	 * before either approach's final trim distance (the max reach across *both* of its corners) is
	 * known -- see the extension pass in BuildJunctionCornersForRadius.
	 */
	struct FRawCorner
	{
		TArray<FVector2D> Points;
		int32 ApproachA = INDEX_NONE;
		int32 ApproachB = INDEX_NONE;
	};

	struct FJunctionCornerBuildResult
	{
		TArray<FVector2D> Polygon2D;
		// Parallel to Polygon2D -- true if the edge from this vertex to the next is genuine curb
		// line (part of one RawCorner's own arc/point), false if it's the "closing" edge between
		// two *different* approaches' own edges (see the extension pass below). Curbstone
		// generation needs this distinction: a closing edge runs straight across the road at the
		// trim boundary, not along a curb, and must not get a curbstone.
		TArray<bool> Polygon2DEdgeIsCurbLine;
		TMap<int32, float> TrimDistance2DByApproachIndex;
		// One entry per angularly-adjacent pair (so Sorted.Num() entries), collected during the
		// main loop and consumed -- extended out to each side's final trim distance, then flattened
		// into Polygon2D in ring order -- right after it, before any of the CornerIslands work below.
		TArray<FRawCorner> RawCorners;
		TArray<FFlexJunctionCornerIsland> CornerIslands;
		// Parallel to CornerIslands -- which two approaches (by index into the Approaches array
		// passed to BuildJunction) each island's band was built between. Needed after the main loop
		// to extend an island's arcs out to the approach's own final trim distance (which isn't
		// known until every pair touching that approach has been visited).
		TArray<int32> CornerIslandApproachA;
		TArray<int32> CornerIslandApproachB;
		TArray<float> CornerIslandAvgCurbHeight;
	};

	/**
	 * Guaranteed-simple fallback for acute multi-road crossings. It wraps the curb corners of every
	 * trimmed approach in a convex hull. The detailed pairwise ring is preferable (it preserves
	 * concave pockets), but a hull is a much better last resort than returning no surface after the
	 * roads have already been cut back. Same-approach hull edges are road-mouth closing edges;
	 * edges between approaches are genuine curb boundary.
	 */
	void BuildApproachMouthHull(const TArray<FSortedApproach>& Sorted, const TArray<FFlexJunctionApproachInput>& Approaches,
		TMap<int32, float>& InOutTrimDistance, TArray<FVector2D>& OutHull, TArray<bool>& OutEdgeIsCurb)
	{
		struct FHullPoint { FVector2D P; int32 ApproachIndex = INDEX_NONE; };
		TArray<FHullPoint> Points;
		for (const FSortedApproach& Entry : Sorted)
		{
			const float Available = Approaches.IsValidIndex(Entry.ApproachIndex)
				? Approaches[Entry.ApproachIndex].ArcLengthTable.GetTotalLength() * 0.45f : 0.f;
			float& Trim = InOutTrimDistance.FindOrAdd(Entry.ApproachIndex, Entry.OuterExtent);
			Trim = FMath::Clamp(FMath::Max(Trim, Entry.RoadwayHalfWidth), 0.f, Available);
			const FVector2D Left(-Entry.Direction2D.Y, Entry.Direction2D.X);
			const FVector2D Center = Entry.Direction2D * Trim;
			Points.Add({ Center + Left * Entry.RoadwayHalfWidth, Entry.ApproachIndex });
			Points.Add({ Center - Left * Entry.RoadwayHalfWidth, Entry.ApproachIndex });
		}
		Points.Sort([](const FHullPoint& A, const FHullPoint& B)
		{
			return A.P.X < B.P.X || (FMath::IsNearlyEqual(A.P.X, B.P.X) && A.P.Y < B.P.Y);
		});
		for (int32 i = Points.Num() - 1; i > 0; --i)
		{
			if (Points[i].P.Equals(Points[i - 1].P, KINDA_SMALL_NUMBER)) Points.RemoveAt(i);
		}
		if (Points.Num() < 3) return;
		auto Cross = [](const FVector2D& O, const FVector2D& A, const FVector2D& B)
		{
			return (A.X - O.X) * (B.Y - O.Y) - (A.Y - O.Y) * (B.X - O.X);
		};
		TArray<FHullPoint> Hull;
		for (const FHullPoint& Point : Points)
		{
			while (Hull.Num() >= 2 && Cross(Hull[Hull.Num() - 2].P, Hull.Last().P, Point.P) <= KINDA_SMALL_NUMBER) Hull.Pop();
			Hull.Add(Point);
		}
		const int32 LowerCount = Hull.Num();
		for (int32 i = Points.Num() - 2; i >= 0; --i)
		{
			while (Hull.Num() > LowerCount && Cross(Hull[Hull.Num() - 2].P, Hull.Last().P, Points[i].P) <= KINDA_SMALL_NUMBER) Hull.Pop();
			Hull.Add(Points[i]);
		}
		Hull.Pop(); // repeated first point
		OutHull.Reserve(Hull.Num());
		OutEdgeIsCurb.Reserve(Hull.Num());
		for (int32 i = 0; i < Hull.Num(); ++i)
		{
			OutHull.Add(Hull[i].P);
			OutEdgeIsCurb.Add(Hull[i].ApproachIndex != Hull[(i + 1) % Hull.Num()].ApproachIndex);
		}
	}

	/**
	 * Builds one node's full ring of corners (sharp / DefaultFilletRadius-fillet / curb-return,
	 * per angularly-adjacent pair) at a single attempted curb-return radius. Pure function of its
	 * inputs and side-effect-free -- BuildJunction calls this repeatedly at different radii
	 * (bisecting down from the configured CurbReturnRadius whenever the full radius produces a
	 * self-intersecting polygon) until the result is simple, so every attempt must be fully
	 * self-contained and discardable.
	 */
	FJunctionCornerBuildResult BuildJunctionCornersForRadius(
		const TArray<FSortedApproach>& Sorted,
		const TArray<FFlexJunctionApproachInput>& Approaches,
		const FLocalBasis& Basis,
		const FVector& NodePosition,
		float CurbReturnRadiusAttempt,
		float DefaultFilletRadius,
		float ParallelApproachAngleToleranceDegrees,
		int32 FilletArcSegments,
		int32 CurbReturnArcSegments)
	{
		FJunctionCornerBuildResult Out;
		const int32 N = Sorted.Num();

		auto UpdateTrim = [&Out](int32 ApproachIndex, const FVector2D& CornerPoint, const FVector2D& Origin, const FVector2D& Dir)
		{
			const float Forward = FVector2D::DotProduct(CornerPoint - Origin, Dir);
			float& Existing = Out.TrimDistance2DByApproachIndex.FindOrAdd(ApproachIndex, 0.f);
			Existing = FMath::Max(Existing, Forward);
		};

		for (int32 i = 0; i < N; ++i)
		{
			const FSortedApproach& A = Sorted[i];
			const FSortedApproach& B = Sorted[(i + 1) % N];

			// A's boundary edge on the side facing B ("right" of A's outward direction) and B's
			// boundary edge on the side facing A ("left" of B's outward direction).
			const FVector2D RightPerpA(A.Direction2D.Y, -A.Direction2D.X);
			const FVector2D LeftPerpB(-B.Direction2D.Y, B.Direction2D.X);

			// RoadwayHalfWidth (the curb line), not OuterExtent (curb line + sidewalk width): the
			// drivable surface's own boundary always sits at the curb line, same as the curb-return
			// path below uses for this same purpose. Using the wider OuterExtent here whenever a
			// corner falls back to the sharp/fillet path (e.g. gated by
			// ParallelApproachAngleToleranceDegrees or MaxCornerReach, even though both flanking
			// roads do have sidewalks) pushed the drivable polygon's boundary out into where the
			// sidewalk needs to sit, and -- since a neighboring corner at the same node might still
			// be using the narrower curb-return boundary -- produced a mismatched, notched edge
			// between the two reference widths at the node where they met.
			const FVector2D OriginA = RightPerpA * A.RoadwayHalfWidth;
			const FVector2D OriginB = LeftPerpB * B.RoadwayHalfWidth;

			bool bBuiltPolygonCorner = false;
			// This pair's raw (unclipped) contribution to the drivable ring, ordered from the
			// A-facing end to the B-facing end -- collected rather than appended to Out.Polygon2D
			// directly, since it still needs extending out to each approach's own *final* trim
			// distance (the max reach across both of its corners) in the pass after this loop.
			TArray<FVector2D> CornerPoints;

			// A near-parallel pair (e.g. two carriageways of a divided road) never gets curb-return
			// sidewalk treatment, regardless of radius -- sidewalks only bridge genuine corners. This
			// also happens to be exactly the degenerate input (HalfTheta -> 0) that made the
			// curb-return fillet math's tangent distance blow up, so gating it out here is both a
			// semantic fix and a robustness one. The *other* extreme is just as degenerate: a pair
			// that's nearly straight-through (e.g. the two long arms of a T-junction, ~180 degrees
			// apart) sends HalfTheta -> 90 degrees, which collapses TangentDist = Radius/tan(HalfTheta)
			// to ~0 -- the "corner" point lands almost exactly on the node's own center instead of
			// out at the curb line, biting a large wrong notch out of the polygon. Neither extreme is
			// a real corner to round off; both are gated the same way, symmetrically.
			const float CosTheta = FMath::Clamp(FVector2D::DotProduct(A.Direction2D, B.Direction2D), -1.f, 1.f);
			const float AngleDeg = FMath::RadiansToDegrees(FMath::Acos(CosTheta));
			// Two-approach shallow forks still use the configured parallel gate. At a real multi-road
			// crossing, however, the acute wedge is a genuine intersection corner and needs a long
			// sidewalk return; treating it as parallel is what left the screenshot's empty center.
			const float EffectiveParallelTolerance = N >= 3 ? 2.f : ParallelApproachAngleToleranceDegrees;
			const bool bIsGenuineCorner = AngleDeg >= EffectiveParallelTolerance && AngleDeg <= (180.f - EffectiveParallelTolerance);
			const bool bBothHaveSidewalks = A.SidewalkWidth > KINDA_SMALL_NUMBER && B.SidewalkWidth > KINDA_SMALL_NUMBER
				&& bIsGenuineCorner && CurbReturnRadiusAttempt > KINDA_SMALL_NUMBER;

			if (bBothHaveSidewalks)
			{
				// The curb-return fillet needs the *physical* corner where A's and B's near (facing-
				// each-other) edges would sharply meet -- the opposite pairing from OriginA/OriginB
				// above (which deliberately use the far edges so consecutive corners trace the drivable
				// polygon directly; that far-edge intersection sits nowhere near this pair's actual
				// curbside corner, which is what made earlier islands balloon in toward the junction
				// center instead of tucking into the corner pocket).
				//
				// Offset by RoadwayHalfWidth here, not OuterExtent: this corner (and the curb arc built
				// from it below) becomes both the drivable polygon's own boundary *and* the sidewalk
				// band's inner edge -- i.e. the curb line, where pavement ends and sidewalk begins, same
				// as everywhere else the pavement/sidewalk edge sits.
				const FVector2D LeftPerpA(-A.Direction2D.Y, A.Direction2D.X);
				const FVector2D RightPerpB(B.Direction2D.Y, -B.Direction2D.X);
				const FVector2D NearOriginA = LeftPerpA * A.RoadwayHalfWidth;
				const FVector2D NearOriginB = RightPerpB * B.RoadwayHalfWidth;
				FVector2D NearCornerRef;
				if (!FlexGeometry2D::LineLineIntersection(NearOriginA, A.Direction2D, NearOriginB, B.Direction2D, NearCornerRef))
				{
					NearCornerRef = (NearOriginA + NearOriginB) * 0.5f;
				}

				// A corner with sidewalks on both sides gets a long, smooth curb-return sweep instead
				// of a sharp pavement point -- and the sidewalk band + landscaped island below reuse
				// this exact same center/sweep so all three read as one continuously curved feature.
				//
				// This per-pair cap bounds the attempted radius by the flanking segments' genuinely
				// available length. Acute crossings are intentionally not capped by road width: their
				// valid miter/curb return can require many road widths of setback. This is a fast-path,
				// not the correctness
				// mechanism -- it does NOT by itself guarantee a simple polygon (an ordinary symmetric
				// 90-degree 4-way at default settings already overlaps at the node's center without
				// tripping either bound here), so the caller validates the *whole* resulting polygon
				// and bisects CurbReturnRadiusAttempt down if needed. This cap just means fewer
				// bisection iterations are typically needed for already-tight/narrow cases.
				float EffectiveCurbRadius = CurbReturnRadiusAttempt;
				{
					const float HalfTheta = FMath::Acos(CosTheta) * 0.5f;
					if (HalfTheta > KINDA_SMALL_NUMBER)
					{
						const float ShorterFlankingLength = FMath::Min(Approaches[A.ApproachIndex].ArcLengthTable.GetTotalLength(), Approaches[B.ApproachIndex].ArcLengthTable.GetTotalLength());
						const float LengthBasedCap = FMath::Max(ShorterFlankingLength * 0.45f, 1.f);
						// Acute turns legitimately need a long setback. The usable 45% of the shorter
						// approach is the real constraint; road width is not an availability limit.
						const float MaxTangentDist = LengthBasedCap;
						const float TanHalfTheta = FMath::Tan(HalfTheta);
						if (TanHalfTheta > KINDA_SMALL_NUMBER && EffectiveCurbRadius / TanHalfTheta > MaxTangentDist)
						{
							EffectiveCurbRadius = MaxTangentDist * TanHalfTheta;
						}
					}
				}

				FVector2D Center2D;
				float StartAngle = 0.f, SweepAngle = 0.f;
				if (FlexGeometry2D::ComputeFilletCenterAndSweep(NearCornerRef, A.Direction2D, B.Direction2D, EffectiveCurbRadius, Center2D, StartAngle, SweepAngle))
				{
					TArray<FVector2D> CurbArc;
					FlexGeometry2D::SampleArc(Center2D, StartAngle, SweepAngle, EffectiveCurbRadius, CurbReturnArcSegments, CurbArc);
					for (const FVector2D& P : CurbArc)
					{
						UpdateTrim(A.ApproachIndex, P, FVector2D::ZeroVector, A.Direction2D);
						UpdateTrim(B.ApproachIndex, P, FVector2D::ZeroVector, B.Direction2D);
					}
					CornerPoints = CurbArc;
					bBuiltPolygonCorner = true;

					const float AvgSidewalkWidth = (A.SidewalkWidth + B.SidewalkWidth) * 0.5f;
					const float AvgCurbHeight = (A.CurbHeight + B.CurbHeight) * 0.5f;
					const FVector CurbOffset = Basis.Up * AvgCurbHeight;

					// The fillet Center sits on the far (pocket) side of the curb line from the
					// road -- i.e. moving *away* from the road, out of the pavement and into the
					// block corner, means moving toward Center, not away from it. So the sidewalk
					// band and island step the radius *down* from the curb line, not up (a bigger
					// radius from this same Center swings back the other way, toward the node's own
					// origin -- into the intersection, not away from it, which is what made the
					// bands reach the wrong way).
					const float BandOuterRadius = FMath::Max(EffectiveCurbRadius - AvgSidewalkWidth, 0.f);
					const float IslandOuterRadius = FMath::Max(EffectiveCurbRadius - AvgSidewalkWidth * 2.f, 0.f);
					TArray<FVector2D> OuterArc, IslandOuterArc;
					FlexGeometry2D::SampleArc(Center2D, StartAngle, SweepAngle, BandOuterRadius, CurbReturnArcSegments, OuterArc);
					FlexGeometry2D::SampleArc(Center2D, StartAngle, SweepAngle, IslandOuterRadius, CurbReturnArcSegments, IslandOuterArc);

					FFlexJunctionCornerIsland Island;
					Island.Center = Basis.To3D(NodePosition, Center2D) + CurbOffset;
					Island.BandInnerArc.Reserve(CurbArc.Num());
					Island.BandOuterArc.Reserve(OuterArc.Num());
					Island.IslandOuterArc.Reserve(IslandOuterArc.Num());
					for (const FVector2D& P : CurbArc) { Island.BandInnerArc.Add(Basis.To3D(NodePosition, P) + CurbOffset); }
					for (const FVector2D& P : OuterArc) { Island.BandOuterArc.Add(Basis.To3D(NodePosition, P) + CurbOffset); }
					for (const FVector2D& P : IslandOuterArc) { Island.IslandOuterArc.Add(Basis.To3D(NodePosition, P) + CurbOffset); }
					Out.CornerIslands.Add(MoveTemp(Island));
					Out.CornerIslandApproachA.Add(A.ApproachIndex);
					Out.CornerIslandApproachB.Add(B.ApproachIndex);
					Out.CornerIslandAvgCurbHeight.Add(AvgCurbHeight);
				}
			}

			if (!bBuiltPolygonCorner && !bIsGenuineCorner)
			{
				// Not a real corner to round off at all (near-parallel or near-straight-through --
				// see the comment on bIsGenuineCorner above) -- both the sharp-corner intersection
				// and any fillet are degenerate for this angle, so skip straight to the plain
				// curb-line offset. For a genuinely straight pair this is already the geometrically
				// correct answer (no rounding needed, OriginA and OriginB coincide), not just a
				// bounded fallback.
				const FVector2D Fallback = (OriginA + OriginB) * 0.5f;
				UpdateTrim(A.ApproachIndex, Fallback, FVector2D::ZeroVector, A.Direction2D);
				UpdateTrim(B.ApproachIndex, Fallback, FVector2D::ZeroVector, B.Direction2D);
				CornerPoints = { Fallback };
			}
			else if (!bBuiltPolygonCorner)
			{
				// Neither the sharp corner nor the DefaultFilletRadius fillet below has any
				// inherent bound on how far it can reach -- both grow like 1/tan(HalfTheta/2), same
				// as the curb-return fillet did, and a near-parallel pair gated out of curb-return
				// above (or simply two roads with no sidewalks meeting at a shallow angle) is
				// exactly the input that sends that arbitrarily far out. Cap how far either is
				// allowed to reach using the actual available flanking-road length. If neither fits,
				// the bounded sharp bevel below still connects both curb mouths without an unbounded
				// spike; the final hull fallback guarantees a surface even for a pathological ring.
				float MaxCornerReach = TNumericLimits<float>::Max();
				{
					const float HalfTheta = FMath::Acos(CosTheta) * 0.5f;
					if (HalfTheta > KINDA_SMALL_NUMBER)
					{
						const float ShorterFlankingLength = FMath::Min(Approaches[A.ApproachIndex].ArcLengthTable.GetTotalLength(), Approaches[B.ApproachIndex].ArcLengthTable.GetTotalLength());
						const float LengthBasedCap = FMath::Max(ShorterFlankingLength * 0.45f, 1.f);
						MaxCornerReach = LengthBasedCap;
					}
				}

				FVector2D Corner;
				bool bUsedSharpCorner = false;
				if (FlexGeometry2D::LineLineIntersection(OriginA, A.Direction2D, OriginB, B.Direction2D, Corner))
				{
					const float ForwardA = FVector2D::DotProduct(Corner - OriginA, A.Direction2D);
					const float ForwardB = FVector2D::DotProduct(Corner - OriginB, B.Direction2D);
					// A clean corner must lie "ahead of" the node along both boundary rays (behind
					// either one means too acute an angle for the sharp intersection to be usable),
					// and within MaxCornerReach of it (too far means the same acute-angle blowup).
					if (ForwardA >= 0.f && ForwardB >= 0.f && ForwardA <= MaxCornerReach && ForwardB <= MaxCornerReach)
					{
						bUsedSharpCorner = true;
					}
				}

				if (bUsedSharpCorner)
				{
					UpdateTrim(A.ApproachIndex, Corner, FVector2D::ZeroVector, A.Direction2D);
					UpdateTrim(B.ApproachIndex, Corner, FVector2D::ZeroVector, B.Direction2D);
					CornerPoints = { Corner };
				}
				else
				{
					// The fillet's own corner-reference point must be the *curb-line* corner
					// (OriginA/OriginB, same as the sharp-corner check just above), not the node's
					// raw center -- using FVector2D::ZeroVector here (a since-fixed bug) pulled the
					// whole arc toward the road's centerline instead of anchoring it at the true
					// offset corner, pinching the polygon into a narrow dart. Reuse Corner if the
					// intersection above found one (even if it failed the Forward/MaxCornerReach
					// checks, it's still a valid geometric reference point), else the same simple
					// midpoint used as the ultimate fallback below.
					const FVector2D FilletCornerRef = FlexGeometry2D::LineLineIntersection(OriginA, A.Direction2D, OriginB, B.Direction2D, Corner) ? Corner : (OriginA + OriginB) * 0.5f;

					const float Radius = DefaultFilletRadius;
					const float HalfTheta = FMath::Acos(CosTheta) * 0.5f;
					const float TanHalfTheta = FMath::Tan(HalfTheta);
					const bool bFilletFitsWithinCap = HalfTheta > KINDA_SMALL_NUMBER && TanHalfTheta > KINDA_SMALL_NUMBER && (Radius / TanHalfTheta) <= MaxCornerReach;
					TArray<FVector2D> Arc;
					if (bFilletFitsWithinCap && FlexGeometry2D::ComputeFilletArc(FilletCornerRef, A.Direction2D, B.Direction2D, Radius, Arc, FilletArcSegments))
					{
						for (const FVector2D& P : Arc)
						{
							UpdateTrim(A.ApproachIndex, P, FVector2D::ZeroVector, A.Direction2D);
							UpdateTrim(B.ApproachIndex, P, FVector2D::ZeroVector, B.Direction2D);
						}
						CornerPoints = Arc;
					}
					else
					{
						// A sharp/acute turn can put the theoretical miter far beyond the usable
						// intersection footprint. Preserve the turn as a bounded sharp bevel instead
						// of collapsing both curb edges to one midpoint (which pinched the surface and
						// could leave one approach uncovered). Each endpoint stays on its own curb ray.
						FVector2D SharpIntersection;
						float ReachA = 0.f;
						float ReachB = 0.f;
						if (FlexGeometry2D::LineLineIntersection(OriginA, A.Direction2D, OriginB, B.Direction2D, SharpIntersection))
						{
							ReachA = FMath::Clamp(FVector2D::DotProduct(SharpIntersection - OriginA, A.Direction2D), 0.f, MaxCornerReach);
							ReachB = FMath::Clamp(FVector2D::DotProduct(SharpIntersection - OriginB, B.Direction2D), 0.f, MaxCornerReach);
						}
						const FVector2D BevelA = OriginA + A.Direction2D * ReachA;
						const FVector2D BevelB = OriginB + B.Direction2D * ReachB;
						UpdateTrim(A.ApproachIndex, BevelA, FVector2D::ZeroVector, A.Direction2D);
						UpdateTrim(B.ApproachIndex, BevelB, FVector2D::ZeroVector, B.Direction2D);
						CornerPoints.Add(BevelA);
						if (!BevelA.Equals(BevelB, KINDA_SMALL_NUMBER))
						{
							CornerPoints.Add(BevelB);
						}
					}
				}
			}

			Out.RawCorners.Add(FRawCorner{ MoveTemp(CornerPoints), A.ApproachIndex, B.ApproachIndex });
		}

		// Each pair's raw contribution only reaches as far as *that pair's own* geometry does --
		// but an approach's final trim distance (below) is the max reach across *both* of its
		// corners, so whichever of the two corners reached less far needs its near/far end
		// extended by a straight point to actually meet the polygon edge the other corner (or the
		// approach's own straight roadway) settles on; otherwise the ring falls short there,
		// leaving a real gap in the drivable surface on that corner's shorter side.
		for (FRawCorner& Raw : Out.RawCorners)
		{
			if (Raw.Points.Num() == 0)
			{
				continue;
			}

			const FSortedApproach* AEntry = Sorted.FindByPredicate([&](const FSortedApproach& S) { return S.ApproachIndex == Raw.ApproachA; });
			const FSortedApproach* BEntry = Sorted.FindByPredicate([&](const FSortedApproach& S) { return S.ApproachIndex == Raw.ApproachB; });
			if (!AEntry || !BEntry)
			{
				continue;
			}

			const float TrimA = Out.TrimDistance2DByApproachIndex.FindChecked(AEntry->ApproachIndex);
			const float GapA = TrimA - FVector2D::DotProduct(Raw.Points[0], AEntry->Direction2D);
			if (GapA > KINDA_SMALL_NUMBER)
			{
				Raw.Points.Insert(Raw.Points[0] + AEntry->Direction2D * GapA, 0);
			}

			const float TrimB = Out.TrimDistance2DByApproachIndex.FindChecked(BEntry->ApproachIndex);
			const float GapB = TrimB - FVector2D::DotProduct(Raw.Points.Last(), BEntry->Direction2D);
			if (GapB > KINDA_SMALL_NUMBER)
			{
				Raw.Points.Add(Raw.Points.Last() + BEntry->Direction2D * GapB);
			}

			Out.Polygon2D.Append(Raw.Points);
			for (int32 k = 0; k < Raw.Points.Num(); ++k)
			{
				// Every edge within this corner's own points is curb line; the edge leaving the
				// last one (to the next RawCorner's first point, appended by the next iteration --
				// or wrapping back to RawCorners[0] after the last) is the closing edge instead.
				Out.Polygon2DEdgeIsCurbLine.Add(k + 1 < Raw.Points.Num());
			}
		}

		// A curb-return corner's band only covers its own arc sweep -- unlike the drivable polygon
		// (one continuous ring, so the straight run along an approach's own curb line between its
		// two corners comes for free as a polygon edge), each island here is built in isolation.
		// An approach's *final* trim distance isn't known until every pair touching it has been
		// visited (it's the max reach across both of that approach's corners, and for the very
		// first approach in the ring the "previous" corner is the *last* pair processed) -- so only
		// now, after the full loop above, extend each island's three concentric arcs with one
		// straight point on whichever side(s) fall short of that approach's own resolved trim
		// distance. All three arcs get the same extension (not just the two AppendQuad reads from
		// for the sidewalk band) so BandOuterArc and IslandOuterArc -- read together by the
		// CornerIslands AppendBand call -- stay index-aligned; the tangent-point *angle* from a
		// shared center depends only on line direction, not radius, so all three extend by the
		// exact same amount.
		for (int32 IslandIdx = 0; IslandIdx < Out.CornerIslands.Num(); ++IslandIdx)
		{
			FFlexJunctionCornerIsland& Island = Out.CornerIslands[IslandIdx];
			if (Island.BandInnerArc.Num() == 0 || Island.BandOuterArc.Num() != Island.BandInnerArc.Num() || Island.IslandOuterArc.Num() != Island.BandInnerArc.Num())
			{
				continue;
			}

			const FSortedApproach* AEntry = Sorted.FindByPredicate([&](const FSortedApproach& S) { return S.ApproachIndex == Out.CornerIslandApproachA[IslandIdx]; });
			const FSortedApproach* BEntry = Sorted.FindByPredicate([&](const FSortedApproach& S) { return S.ApproachIndex == Out.CornerIslandApproachB[IslandIdx]; });
			if (!AEntry || !BEntry)
			{
				continue;
			}

			const FVector CurbOffset = Basis.Up * Out.CornerIslandAvgCurbHeight[IslandIdx];
			const float TrimA = Out.TrimDistance2DByApproachIndex.FindChecked(AEntry->ApproachIndex);
			const float TrimB = Out.TrimDistance2DByApproachIndex.FindChecked(BEntry->ApproachIndex);

			const FVector2D FrontInner2D = Basis.To2D(Island.BandInnerArc[0] - CurbOffset - NodePosition);
			const float GapA = TrimA - FVector2D::DotProduct(FrontInner2D, AEntry->Direction2D);
			if (GapA > KINDA_SMALL_NUMBER)
			{
				const FVector2D FrontOuter2D = Basis.To2D(Island.BandOuterArc[0] - CurbOffset - NodePosition);
				const FVector2D FrontIsland2D = Basis.To2D(Island.IslandOuterArc[0] - CurbOffset - NodePosition);
				Island.BandInnerArc.Insert(Basis.To3D(NodePosition, FrontInner2D + AEntry->Direction2D * GapA) + CurbOffset, 0);
				Island.BandOuterArc.Insert(Basis.To3D(NodePosition, FrontOuter2D + AEntry->Direction2D * GapA) + CurbOffset, 0);
				Island.IslandOuterArc.Insert(Basis.To3D(NodePosition, FrontIsland2D + AEntry->Direction2D * GapA) + CurbOffset, 0);
			}

			const FVector2D BackInner2D = Basis.To2D(Island.BandInnerArc.Last() - CurbOffset - NodePosition);
			const float GapB = TrimB - FVector2D::DotProduct(BackInner2D, BEntry->Direction2D);
			if (GapB > KINDA_SMALL_NUMBER)
			{
				const FVector2D BackOuter2D = Basis.To2D(Island.BandOuterArc.Last() - CurbOffset - NodePosition);
				const FVector2D BackIsland2D = Basis.To2D(Island.IslandOuterArc.Last() - CurbOffset - NodePosition);
				Island.BandInnerArc.Add(Basis.To3D(NodePosition, BackInner2D + BEntry->Direction2D * GapB) + CurbOffset);
				Island.BandOuterArc.Add(Basis.To3D(NodePosition, BackOuter2D + BEntry->Direction2D * GapB) + CurbOffset);
				Island.IslandOuterArc.Add(Basis.To3D(NodePosition, BackIsland2D + BEntry->Direction2D * GapB) + CurbOffset);
			}
		}

		return Out;
	}
}

bool FFlexIntersectionBuilder::NeedsJunction(const TArray<FFlexJunctionApproachInput>& Approaches, float StraightThroughAngleToleranceDegrees, float WidthMismatchTolerance)
{
	if (Approaches.Num() >= 3)
	{
		return true;
	}
	if (Approaches.Num() < 2)
	{
		return false;
	}

	const FVector DirA = GetOutwardTangent(Approaches[0]);
	const FVector DirB = GetOutwardTangent(Approaches[1]);
	const float CosAngle = FMath::Clamp(FVector::DotProduct(DirA, DirB), -1.f, 1.f);
	const float AngleDeg = FMath::RadiansToDegrees(FMath::Acos(CosAngle));
	// A smooth pass-through bend has the two approaches pointing roughly opposite (~180 degrees apart).
	const float DeviationFromStraight = FMath::Abs(180.f - AngleDeg);
	if (DeviationFromStraight > StraightThroughAngleToleranceDegrees)
	{
		return true;
	}

	const float WidthA = Approaches[0].Profile ? Approaches[0].Profile->GetRoadwayHalfWidth() : 0.f;
	const float WidthB = Approaches[1].Profile ? Approaches[1].Profile->GetRoadwayHalfWidth() : 0.f;
	if (FMath::Abs(WidthA - WidthB) > WidthMismatchTolerance)
	{
		return true;
	}

	return false;
}

FFlexJunctionData FFlexIntersectionBuilder::BuildJunction(const FVector& NodePosition, const FVector& NodeUp, const TArray<FFlexJunctionApproachInput>& Approaches, float DefaultFilletRadius, float CrosswalkWidth, float CrosswalkMinClearance, float CurbReturnRadius, float ParallelApproachAngleToleranceDegrees, int32 FilletArcSegments, int32 CurbReturnArcSegments)
{
	FFlexJunctionData Result;
	const int32 N = Approaches.Num();
	if (N < 2)
	{
		return Result;
	}

	const FLocalBasis Basis = MakeLocalBasis(NodeUp);

	TArray<FSortedApproach> Sorted;
	Sorted.Reserve(N);
	for (int32 i = 0; i < N; ++i)
	{
		FSortedApproach Entry;
		Entry.ApproachIndex = i;
		Entry.Direction2D = Basis.To2D(GetOutwardTangent(Approaches[i])).GetSafeNormal();
		Entry.Angle = FMath::Atan2(Entry.Direction2D.Y, Entry.Direction2D.X);
		Entry.OuterExtent = Approaches[i].Profile ? Approaches[i].Profile->GetOuterExtent() : 0.f;
		Entry.RoadwayHalfWidth = Approaches[i].Profile ? Approaches[i].Profile->GetRoadwayHalfWidth() : 0.f;
		Entry.SidewalkWidth = Approaches[i].Profile ? Approaches[i].Profile->SidewalkWidth : 0.f;
		Entry.CurbHeight = Approaches[i].Profile ? Approaches[i].Profile->CurbHeight : 0.f;
		Sorted.Add(Entry);
	}
	Algo::SortBy(Sorted, [](const FSortedApproach& A) { return A.Angle; });

	// Builds the full ring of corners at RadiusAttempt and checks whether the result is usable
	// (at least a triangle's worth of points, and no two non-adjacent edges crossing each other).
	auto TryRadius = [&](float RadiusAttempt) { return BuildJunctionCornersForRadius(Sorted, Approaches, Basis, NodePosition, RadiusAttempt, DefaultFilletRadius, ParallelApproachAngleToleranceDegrees, FilletArcSegments, CurbReturnArcSegments); };
	auto IsValidResult = [](const FJunctionCornerBuildResult& R) { return R.Polygon2D.Num() >= 3 && FlexGeometry2D::IsSimplePolygon(R.Polygon2D); };

	// Fast path: the configured CurbReturnRadius works as-is (the overwhelmingly common case for
	// reasonably-spaced intersections).
	FJunctionCornerBuildResult CornerResult = TryRadius(CurbReturnRadius);
	// Keep the full-radius sidewalk returns available for the hull fallback. Polygon validity is a
	// ring-order problem; each individual return can still be valid and is needed to bridge the
	// longer acute-angle road cuts smoothly.
	const TArray<FFlexJunctionCornerIsland> FullRadiusCornerIslands = CornerResult.CornerIslands;
	const TMap<int32, float> FullRadiusTrimDistances = CornerResult.TrimDistance2DByApproachIndex;
	if (!IsValidResult(CornerResult))
	{
		// It doesn't -- even an ordinary symmetric 90-degree 4-way at default settings can
		// self-intersect at the node's center (the four corners' fillet circles all reach in far
		// enough to overlap each other), and no per-pair heuristic cap can predict that in
		// advance, since it depends on how many approaches share the node, not any one pair's own
		// width/angle. Bisect the curb-return radius down between a known-bad (CurbReturnRadius)
		// and a known-good (0, which forces every corner through the sharp/DefaultFilletRadius
		// path below -- unaffected by this radius and so always at least as safe) bound. This
		// converges close to the true max safe radius for this specific node in a handful of
		// attempts, rather than a fixed decay schedule that can land two nearly-identical
		// neighboring intersections on opposite sides of the threshold with visibly different
		// final radii.
		FJunctionCornerBuildResult ZeroResult = TryRadius(0.f);
		if (IsValidResult(ZeroResult))
		{
			float Lo = 0.f, Hi = CurbReturnRadius;
			FJunctionCornerBuildResult BestValid = MoveTemp(ZeroResult);
			for (int32 Attempt = 0; Attempt < 5; ++Attempt)
			{
				const float Mid = (Lo + Hi) * 0.5f;
				FJunctionCornerBuildResult MidResult = TryRadius(Mid);
				if (IsValidResult(MidResult))
				{
					BestValid = MoveTemp(MidResult);
					Lo = Mid;
				}
				else
				{
					Hi = Mid;
				}
			}
			CornerResult = MoveTemp(BestValid);
		}
		else
		{
			// Preserve the zero-radius attempt's useful per-approach trims and sidewalk returns;
			// its invalid detailed ring is replaced by the guaranteed-simple mouth hull below.
			CornerResult = MoveTemp(ZeroResult);
		}
	}

	if (!IsValidResult(CornerResult))
	{
		CornerResult.CornerIslands = FullRadiusCornerIslands;
		for (const TPair<int32, float>& Pair : FullRadiusTrimDistances)
		{
			float& Trim = CornerResult.TrimDistance2DByApproachIndex.FindOrAdd(Pair.Key, 0.f);
			Trim = FMath::Max(Trim, Pair.Value);
		}
		CornerResult.Polygon2D.Reset();
		CornerResult.Polygon2DEdgeIsCurbLine.Reset();
		BuildApproachMouthHull(Sorted, Approaches, CornerResult.TrimDistance2DByApproachIndex,
			CornerResult.Polygon2D, CornerResult.Polygon2DEdgeIsCurbLine);
	}

	TArray<FVector2D> Polygon2D = MoveTemp(CornerResult.Polygon2D);
	TMap<int32, float> TrimDistance2DByApproachIndex = MoveTemp(CornerResult.TrimDistance2DByApproachIndex);
	Result.CornerIslands = MoveTemp(CornerResult.CornerIslands);
	Result.PolygonEdgeIsCurbLine = MoveTemp(CornerResult.Polygon2DEdgeIsCurbLine);

	if (Polygon2D.Num() < 3)
	{
		return Result;
	}

	// Convert to world space and triangulate.
	Result.PolygonBoundary.Reserve(Polygon2D.Num());
	for (const FVector2D& P : Polygon2D)
	{
		Result.PolygonBoundary.Add(Basis.To3D(NodePosition, P));
	}
	// EarClipTriangulate does not itself detect self-intersection -- its only checks are vertex
	// convexity and point-in-triangle, both against *vertices*, not edges, so it can (and for the
	// rare case the bisection above still couldn't resolve, will) return true on self-intersecting
	// input and produce a garbled-but-rendered sliver mesh rather than cleanly failing. Check
	// simplicity explicitly first rather than trusting triangulation's return value for it.
	if (!FlexGeometry2D::IsSimplePolygon(Polygon2D) || !FlexTriangulation::EarClipTriangulate(Polygon2D, Result.PolygonTriangleIndices))
	{
		// A self-intersecting/degenerate polygon (e.g. an unusually tight cluster of approaches)
		// leaves ear-clipping with a partial-at-best triangle list -- discard it rather than render
		// a garbled surface; every other piece of junction data (trims, crosswalks, lane
		// connectors, corner islands) is still valid and unaffected.
		Result.PolygonTriangleIndices.Reset();
	}
	else
	{
		// EarClipTriangulate forces its *2D* input to wind CCW (positive signed area) before
		// clipping -- correct for its own ear-detection math, but combined with this basis
		// (AxisX,AxisY,Up right-handed, i.e. AxisX x AxisY = Up) that lands the resulting 3D
		// triangles wound the way this engine's rasterizer treats as back-facing, not front --
		// confirmed empirically (surface was visible from underneath, not from above). Flip every
		// triangle's winding here, once, after triangulation rather than touching
		// EarClipTriangulate itself (which needs its own internally-consistent convention to
		// detect ears/containment correctly, independent of what the renderer wants).
		for (int32 i = 0; i + 2 < Result.PolygonTriangleIndices.Num(); i += 3)
		{
			Swap(Result.PolygonTriangleIndices[i + 1], Result.PolygonTriangleIndices[i + 2]);
		}
	}

	// Resolve per-approach trim arc-lengths (in the approach's own segment parameterization) and
	// place crosswalks/curb-cuts.
	for (const FSortedApproach& Entry : Sorted)
	{
		const FFlexJunctionApproachInput& Approach = Approaches[Entry.ApproachIndex];
		const float SegmentLength = Approach.ArcLengthTable.GetTotalLength();
		const float* FoundTrim2D = TrimDistance2DByApproachIndex.Find(Entry.ApproachIndex);
		float TrimDistance2D = FoundTrim2D ? *FoundTrim2D : Entry.OuterExtent;
		TrimDistance2D = FMath::Max(TrimDistance2D, Entry.OuterExtent * 0.5f);
		TrimDistance2D = FMath::Clamp(TrimDistance2D, 0.f, SegmentLength * 0.45f);

		const float TrimArcLength = Approach.bNodeIsSegmentEnd ? (SegmentLength - TrimDistance2D) : TrimDistance2D;
		Result.TrimArcLengthBySegment.Add(Approach.SegmentId, TrimArcLength);

		if (Approach.Profile && Approach.Profile->SidewalkWidth > KINDA_SMALL_NUMBER)
		{
			const float ClearedDistance2D = FMath::Max(TrimDistance2D, CrosswalkMinClearance);
			const FFlexCurveFrame Frame = FFlexRoadMeshBuilder::SampleFrameAtArcLength(Approach.Curve, Approach.ArcLengthTable, Approach.bNodeIsSegmentEnd ? SegmentLength - ClearedDistance2D : ClearedDistance2D, NodeUp);

			FFlexCrosswalkPlacement Crosswalk;
			Crosswalk.Center = Frame.Position;
			Crosswalk.CrossingDirection = Frame.Right;
			Crosswalk.Width = CrosswalkWidth;
			Crosswalk.Length = Approach.Profile->GetRoadwayHalfWidth() * 2.f;
			Result.Crosswalks.Add(Crosswalk);
		}
	}

	// Lane connectors: every legal incoming-lane -> outgoing-lane pair across distinct approaches.
	struct FResolvedLane
	{
		int32 ApproachIndex;
		int32 LaneIndex;
		FVector Position;
		FVector Tangent; // direction of travel at this connector endpoint
		float SpeedLimit;
	};
	TArray<FResolvedLane> IncomingLanes;
	TArray<FResolvedLane> OutgoingLanes;

	for (const FSortedApproach& Entry : Sorted)
	{
		const FFlexJunctionApproachInput& Approach = Approaches[Entry.ApproachIndex];
		if (!Approach.Profile)
		{
			continue;
		}
		const float TrimArcLength = Result.TrimArcLengthBySegment.FindChecked(Approach.SegmentId);
		const FFlexCurveFrame Frame = FFlexRoadMeshBuilder::SampleFrameAtArcLength(Approach.Curve, Approach.ArcLengthTable, TrimArcLength, NodeUp);

		const FVector InwardDir = Approach.bNodeIsSegmentEnd ? Frame.Tangent : -Frame.Tangent;
		const FVector OutwardDir = -InwardDir;

		const TArray<FRoadLaneDescriptor>& Lanes = Approach.Profile->Lanes;
		for (int32 LaneIndex = 0; LaneIndex < Lanes.Num(); ++LaneIndex)
		{
			bool bIncoming = false, bOutgoing = false;
			GetLaneRoles(Lanes[LaneIndex], Approach.bNodeIsSegmentEnd, bIncoming, bOutgoing);
			if (!bIncoming && !bOutgoing)
			{
				continue;
			}

			const FVector LanePos = Frame.Position + Frame.Right * Lanes[LaneIndex].LateralOffset;

			if (bIncoming)
			{
				IncomingLanes.Add(FResolvedLane{ Entry.ApproachIndex, LaneIndex, LanePos, InwardDir, Lanes[LaneIndex].SpeedLimit });
			}
			if (bOutgoing)
			{
				OutgoingLanes.Add(FResolvedLane{ Entry.ApproachIndex, LaneIndex, LanePos, OutwardDir, Lanes[LaneIndex].SpeedLimit });
			}
		}
	}

	for (const FResolvedLane& In : IncomingLanes)
	{
		for (const FResolvedLane& Out : OutgoingLanes)
		{
			if (In.ApproachIndex == Out.ApproachIndex)
			{
				// No U-turn back onto the same approach; a real through-connection between two
				// different approaches covers the ordinary 2-approach sharp-junction case.
				continue;
			}

			const float Dist = FVector::Dist(In.Position, Out.Position);
			const float DirectionDot = FMath::Clamp(FVector::DotProduct(In.Tangent.GetSafeNormal(), Out.Tangent.GetSafeNormal()), -1.f, 1.f);
			const float TurnAngleDegrees = FMath::RadiansToDegrees(FMath::Acos(DirectionDot));
			// Straight movements keep long handles for a broad smooth sweep. Tight turns shorten
			// progressively so their control points remain inside the junction rather than crossing
			// the opposite curb or producing a loop at acute intersections.
			const float Sharpness = FMath::Clamp(TurnAngleDegrees / 180.f, 0.f, 1.f);
			const float HandleScale = FMath::Lerp(0.45f, 0.18f, Sharpness);
			const float HandleLength = FMath::Max(Dist * HandleScale, 1.f);

			FFlexLaneConnector Connector;
			Connector.FromSegment = Approaches[In.ApproachIndex].SegmentId;
			Connector.FromLaneIndex = In.LaneIndex;
			Connector.ToSegment = Approaches[Out.ApproachIndex].SegmentId;
			Connector.ToLaneIndex = Out.LaneIndex;
			Connector.ConnectorCurve.P0 = In.Position;
			Connector.ConnectorCurve.P1 = In.Position + In.Tangent * HandleLength;
			Connector.ConnectorCurve.P2 = Out.Position - Out.Tangent * HandleLength;
			Connector.ConnectorCurve.P3 = Out.Position;
			Connector.SpeedLimit = FMath::Min(In.SpeedLimit, Out.SpeedLimit);
			Connector.TurnAngleDegrees = TurnAngleDegrees;
			Connector.bSharpTurn = TurnAngleDegrees >= 100.f;
			Result.LaneConnectors.Add(Connector);
		}
	}

	return Result;
}

FFlexJunctionMeshResult FFlexIntersectionBuilder::BuildJunctionMesh(const FVector& NodeUp, const FFlexJunctionData& JunctionData, UMaterialInterface* SurfaceMaterial, UMaterialInterface* CrosswalkMaterial, UMaterialInterface* SidewalkMaterial, UMaterialInterface* MedianMaterial)
{
	FFlexJunctionMeshResult Result;
	if (JunctionData.IsEmpty())
	{
		return Result;
	}

	const FVector Up = NodeUp.GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
	const FLocalBasis Basis = MakeLocalBasis(Up);

	Result.Surface.Material = SurfaceMaterial;
	Result.Surface.Vertices = JunctionData.PolygonBoundary;
	Result.Surface.Triangles = JunctionData.PolygonTriangleIndices;
	Result.Surface.Normals.Init(Up, JunctionData.PolygonBoundary.Num());
	Result.Surface.VertexColors.Init(FColor::White, JunctionData.PolygonBoundary.Num());
	const FVector JunctionCentroid = JunctionData.PolygonBoundary.Num() > 0
		? Algo::Accumulate(JunctionData.PolygonBoundary, FVector::ZeroVector) / JunctionData.PolygonBoundary.Num()
		: FVector::ZeroVector;
	for (const FVector& Vertex : JunctionData.PolygonBoundary)
	{
		const FVector2D Local2D = Basis.To2D(Vertex - JunctionCentroid) / 100.f; // meters
		Result.Surface.UV0.Add(Local2D);
		Result.Surface.Tangents.Add(FProcMeshTangent(Basis.AxisX, false));
	}

	Result.Crosswalks.Material = CrosswalkMaterial;
	for (const FFlexCrosswalkPlacement& Crosswalk : JunctionData.Crosswalks)
	{
		const FVector Along = Crosswalk.CrossingDirection.GetSafeNormal(UE_SMALL_NUMBER, Basis.AxisX);
		const FVector Across = FVector::CrossProduct(Up, Along).GetSafeNormal(UE_SMALL_NUMBER, Basis.AxisY);
		const FVector HalfAlong = Along * (Crosswalk.Length * 0.5f);
		const FVector HalfAcross = Across * (Crosswalk.Width * 0.5f);
		const FVector ZOffset = Up * 0.5f; // avoid z-fighting with the surface section

		const FVector A = Crosswalk.Center - HalfAlong - HalfAcross + ZOffset;
		const FVector B = Crosswalk.Center + HalfAlong - HalfAcross + ZOffset;
		const FVector C = Crosswalk.Center + HalfAlong + HalfAcross + ZOffset;
		const FVector D = Crosswalk.Center - HalfAlong + HalfAcross + ZOffset;

		// Reversed traversal (A,D,C,B instead of A,B,C,D) -- confirmed empirically (see the Surface
		// triangulation comment above) that this basis's "forward" winding renders back-facing here.
		Result.Crosswalks.AppendQuad(A, D, C, B, Up, Along, FVector2D(0.f, 0.f), FVector2D(0.f, 1.f), FVector2D(1.f, 1.f), FVector2D(1.f, 0.f));
	}

	// Sidewalk corner bands and the landscaped islands they wrap around: both are quad-stripped
	// between two concentric arcs (inner->outer), one arc-index-pair at a time.
	auto AppendBand = [&Up](FFlexMeshSectionData& Section, const TArray<FVector>& InnerArc, const TArray<FVector>& OuterArc)
	{
		const int32 Count = FMath::Min(InnerArc.Num(), OuterArc.Num());
		for (int32 i = 0; i + 1 < Count; ++i)
		{
			const FVector& A0 = InnerArc[i];
			const FVector& A1 = InnerArc[i + 1];
			const FVector& B0 = OuterArc[i];
			const FVector& B1 = OuterArc[i + 1];
			const FVector Tangent = (A1 - A0).GetSafeNormal(UE_SMALL_NUMBER, FVector::ForwardVector);

			// The arc's sweep direction (and so the CW/CCW-ness of walking Inner[i]->Inner[i+1])
			// flips from one junction corner to the next depending on approach geometry, which
			// would otherwise silently invert half the bands' winding if the vertex order below
			// were assumed fixed -- derive it from the *actual* first triangle the default order
			// below would produce (A0,A1,B1) instead. Confirmed empirically (see the Surface
			// triangulation comment above) that a positive dot here renders back-facing, not front.
			const FVector FaceNormal = FVector::CrossProduct(A1 - A0, B1 - A0);
			if (FVector::DotProduct(FaceNormal, Up) > 0.f)
			{
				Section.AppendQuad(A0, B0, B1, A1, Up, Tangent, FVector2D(static_cast<float>(i), 0.f), FVector2D(static_cast<float>(i), 1.f), FVector2D(static_cast<float>(i + 1), 1.f), FVector2D(static_cast<float>(i + 1), 0.f));
			}
			else
			{
				Section.AppendQuad(A0, A1, B1, B0, Up, Tangent, FVector2D(static_cast<float>(i), 0.f), FVector2D(static_cast<float>(i + 1), 0.f), FVector2D(static_cast<float>(i + 1), 1.f), FVector2D(static_cast<float>(i), 1.f));
			}
		}
	};

	Result.SidewalkCorners.Material = SidewalkMaterial;
	Result.CornerIslands.Material = MedianMaterial ? MedianMaterial : SidewalkMaterial;
	for (const FFlexJunctionCornerIsland& Island : JunctionData.CornerIslands)
	{
		AppendBand(Result.SidewalkCorners, Island.BandInnerArc, Island.BandOuterArc);
		AppendBand(Result.CornerIslands, Island.BandOuterArc, Island.IslandOuterArc);
	}

	return Result;
}
