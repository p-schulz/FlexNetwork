#include "Mesh/FlexUnifiedRoadMeshBuilder.h"

#include "CompGeom/Delaunay2.h"
#include "Curve/GeneralPolygon2.h"
#include "Curve/PolygonIntersectionUtils.h"
#include "Curve/PolygonOffsetUtils.h"
#include "Materials/MaterialInterface.h"
#include "Polygon2.h"

namespace
{
	using namespace UE::Geometry;

	constexpr double MinEdgeLengthSquared = 0.01;
	constexpr float UvScale = 0.01f;
	constexpr double MaxCurbEdgeLength = 100.0;
	constexpr double SidewalkWidthGroupTolerance = 0.1; // cm
	constexpr double SidewalkOffsetMiterLimit = 2.0;
	constexpr double SidewalkOffsetMaxStepsPerRadian = 6.0;
	constexpr double SidewalkOffsetRoundScale = 0.001;

	struct FSurfaceSupport
	{
		FVector A = FVector::ZeroVector;
		FVector B = FVector::ZeroVector;
		const FFlexUnifiedRoadPolygonInput* Input = nullptr;
	};

	struct FResolvedSurface
	{
		float Z = 0.f;
		float SidewalkWidth = 0.f;
		float CurbHeight = 0.f;
		const FFlexUnifiedRoadPolygonInput* Input = nullptr;
	};

	bool MakePolygon(TConstArrayView<FVector> Boundary, FGeneralPolygon2d& OutPolygon)
	{
		TArray<FVector2d> Vertices;
		Vertices.Reserve(Boundary.Num());
		for (const FVector& Point : Boundary)
		{
			const FVector2d Candidate(Point.X, Point.Y);
			if (Vertices.IsEmpty() || FVector2d::DistSquared(Vertices.Last(), Candidate) > MinEdgeLengthSquared)
			{
				Vertices.Add(Candidate);
			}
		}
		if (Vertices.Num() > 2 && FVector2d::DistSquared(Vertices[0], Vertices.Last()) <= MinEdgeLengthSquared)
		{
			Vertices.Pop();
		}
		if (Vertices.Num() < 3)
		{
			return false;
		}

		FPolygon2d Outer(MoveTemp(Vertices));
		if (FMath::Abs(Outer.SignedArea()) <= UE_DOUBLE_SMALL_NUMBER)
		{
			return false;
		}
		// GeometryAlgorithms uses Clipper's NonZero fill rule. Oppositely-wound subject paths
		// cancel where they overlap, so every standalone road/junction/suppression footprint must
		// enter the boolean pass with the same outer-ring orientation.
		if (Outer.IsClockwise())
		{
			Outer.Reverse();
		}
		OutPolygon = FGeneralPolygon2d(Outer);
		return true;
	}

	void AddSupports(const FFlexUnifiedRoadPolygonInput& Input, TArray<FSurfaceSupport>& OutSupports)
	{
		for (int32 Index = 0; Index < Input.Boundary.Num(); ++Index)
		{
			const FVector& A = Input.Boundary[Index];
			const FVector& B = Input.Boundary[(Index + 1) % Input.Boundary.Num()];
			if (FVector2D::DistSquared(FVector2D(A.X, A.Y), FVector2D(B.X, B.Y)) > MinEdgeLengthSquared)
			{
				OutSupports.Add({ A, B, &Input });
			}
		}
	}

	FResolvedSurface ResolveSurface(const FVector2d& Point, TConstArrayView<FSurfaceSupport> Supports)
	{
		FResolvedSurface Result;
		double BestDistanceSquared = TNumericLimits<double>::Max();
		for (const FSurfaceSupport& Support : Supports)
		{
			const FVector2d A(Support.A.X, Support.A.Y);
			const FVector2d B(Support.B.X, Support.B.Y);
			const FVector2d AB = B - A;
			const double LengthSquared = AB.SquaredLength();
			const double Alpha = LengthSquared > UE_DOUBLE_SMALL_NUMBER
				? FMath::Clamp((Point - A).Dot(AB) / LengthSquared, 0.0, 1.0)
				: 0.0;
			const FVector2d Closest = A + AB * Alpha;
			const double DistanceSquared = FVector2d::DistSquared(Point, Closest);
			if (DistanceSquared < BestDistanceSquared)
			{
				BestDistanceSquared = DistanceSquared;
				Result.Z = FMath::Lerp(Support.A.Z, Support.B.Z, static_cast<float>(Alpha));
				Result.SidewalkWidth = Support.Input ? Support.Input->SidewalkWidth : 0.f;
				Result.CurbHeight = Support.Input ? Support.Input->CurbHeight : 0.f;
				Result.Input = Support.Input;
			}
		}
		return Result;
	}

	UMaterialInterface* ResolveMaterial(const FResolvedSurface& Surface, bool bRaised)
	{
		if (!Surface.Input)
		{
			return nullptr;
		}
		return bRaised ? Surface.Input->SidewalkMaterial : Surface.Input->RoadMaterial;
	}

	FFlexMeshSectionData& FindOrAddSection(TArray<FFlexMeshSectionData>& Sections, UMaterialInterface* Material)
	{
		if (FFlexMeshSectionData* Existing = Sections.FindByPredicate([Material](const FFlexMeshSectionData& Section)
		{
			return Section.Material == Material;
		}))
		{
			return *Existing;
		}
		FFlexMeshSectionData& Added = Sections.AddDefaulted_GetRef();
		Added.Material = Material;
		return Added;
	}

	void AppendTriangulatedPolygon(const FGeneralPolygon2d& Polygon, TConstArrayView<FSurfaceSupport> Supports, bool bRaised, TArray<FFlexMeshSectionData>& OutSections)
	{
		FDelaunay2 Triangulator;
		TArray<FIndex3i> Triangles;
		TArray<FVector2d> Vertices2d;
		Triangulator.Triangulate(Polygon, &Triangles, &Vertices2d, true);
		if (Triangles.IsEmpty() || Vertices2d.IsEmpty())
		{
			return;
		}

		for (const FIndex3i& Triangle : Triangles)
		{
			if (!Vertices2d.IsValidIndex(Triangle.A) || !Vertices2d.IsValidIndex(Triangle.B) || !Vertices2d.IsValidIndex(Triangle.C))
			{
				continue;
			}
			FVector2d PointA = Vertices2d[Triangle.A];
			FVector2d PointB = Vertices2d[Triangle.B];
			FVector2d PointC = Vertices2d[Triangle.C];
			FResolvedSurface SurfaceA = ResolveSurface(PointA, Supports);
			FResolvedSurface SurfaceB = ResolveSurface(PointB, Supports);
			FResolvedSurface SurfaceC = ResolveSurface(PointC, Supports);
			const FVector2d Centroid = (PointA + PointB + PointC) / 3.0;
			const FResolvedSurface MaterialSurface = ResolveSurface(Centroid, Supports);
			FFlexMeshSectionData& OutSection = FindOrAddSection(OutSections, ResolveMaterial(MaterialSurface, bRaised));
			FVector A(PointA.X, PointA.Y, SurfaceA.Z + (bRaised ? SurfaceA.CurbHeight : 0.f));
			FVector B(PointB.X, PointB.Y, SurfaceB.Z + (bRaised ? SurfaceB.CurbHeight : 0.f));
			FVector C(PointC.X, PointC.Y, SurfaceC.Z + (bRaised ? SurfaceC.CurbHeight : 0.f));
			// ProceduralMeshComponent uses the opposite face convention from the GeometryCore
			// triangulator here; match the existing classic junction builder's explicit flip.
			if (FVector::CrossProduct(B - A, C - A).Z > 0.f)
			{
				Swap(B, C);
				Swap(PointB, PointC);
			}
			const FVector SurfaceNormal = -FVector::CrossProduct(B - A, C - A).GetSafeNormal(UE_SMALL_NUMBER, -FVector::UpVector);
			const FVector SurfaceTangent = (B - A).GetSafeNormal(UE_SMALL_NUMBER, FVector::ForwardVector);
			OutSection.AppendTriangle(A, B, C, SurfaceNormal, SurfaceTangent,
				FVector2D(PointA.X * UvScale, PointA.Y * UvScale),
				FVector2D(PointB.X * UvScale, PointB.Y * UvScale),
				FVector2D(PointC.X * UvScale, PointC.Y * UvScale));
		}
	}

	void AppendTriangulatedPolygons(TConstArrayView<FGeneralPolygon2d> Polygons, TConstArrayView<FSurfaceSupport> Supports, bool bRaised, TArray<FFlexMeshSectionData>& OutSections)
	{
		for (const FGeneralPolygon2d& Polygon : Polygons)
		{
			AppendTriangulatedPolygon(Polygon, Supports, bRaised, OutSections);
		}
	}

	void FilterTinyPolygonComponents(TArray<FGeneralPolygon2d>& Polygons, double MinimumArea)
	{
		if (MinimumArea <= UE_DOUBLE_SMALL_NUMBER)
		{
			return;
		}
		for (int32 PolygonIndex = Polygons.Num() - 1; PolygonIndex >= 0; --PolygonIndex)
		{
			FGeneralPolygon2d& Polygon = Polygons[PolygonIndex];
			if (FMath::Abs(Polygon.GetOuter().SignedArea()) < MinimumArea)
			{
				Polygons.RemoveAtSwap(PolygonIndex);
			}
		}
	}

	void FilterTinyPolygonFeatures(TArray<FGeneralPolygon2d>& Polygons, double MinimumArea)
	{
		FilterTinyPolygonComponents(Polygons, MinimumArea);
		for (FGeneralPolygon2d& Polygon : Polygons)
		{
			Polygon.FilterHoles([MinimumArea](const FPolygon2d& Hole)
			{
				return MinimumArea > UE_DOUBLE_SMALL_NUMBER && FMath::Abs(Hole.SignedArea()) < MinimumArea;
			});
		}
	}

	void SimplifyPolygonRings(TArray<FGeneralPolygon2d>& Polygons)
	{
		// Boolean and offset operations can retain long runs that are collinear to sub-millimetre
		// precision. Feeding those redundant constraints into Delaunay can produce degenerate
		// alternating edge triangles. Simplify far below any visible geometry tolerance.
		constexpr double ClusterTolerance = 0.1;       // cm
		constexpr double LineDeviationTolerance = 0.1; // cm
		for (int32 PolygonIndex = Polygons.Num() - 1; PolygonIndex >= 0; --PolygonIndex)
		{
			FGeneralPolygon2d& Polygon = Polygons[PolygonIndex];
			FPolygon2d Outer = Polygon.GetOuter();
			Outer.Simplify(ClusterTolerance, LineDeviationTolerance);
			if (Outer.VertexCount() < 3 || FMath::Abs(Outer.SignedArea()) <= UE_DOUBLE_SMALL_NUMBER)
			{
				Polygons.RemoveAtSwap(PolygonIndex);
				continue;
			}
			if (Outer.IsClockwise())
			{
				Outer.Reverse();
			}
			FGeneralPolygon2d Simplified(Outer);
			for (const FPolygon2d& SourceHole : Polygon.GetHoles())
			{
				FPolygon2d Hole = SourceHole;
				Hole.Simplify(ClusterTolerance, LineDeviationTolerance);
				if (Hole.VertexCount() >= 3 && FMath::Abs(Hole.SignedArea()) > UE_DOUBLE_SMALL_NUMBER)
				{
					if (!Hole.IsClockwise())
					{
						Hole.Reverse();
					}
					// Recheck containment after simplification instead of forwarding a malformed hole
					// into the next NonZero boolean or constrained triangulation pass.
					Simplified.AddHole(MoveTemp(Hole), true, true);
				}
			}
			Polygon = MoveTemp(Simplified);
		}
	}

	bool IsSuppressed(const FVector2d& Point, TConstArrayView<FGeneralPolygon2d> SuppressionPolygons)
	{
		for (const FGeneralPolygon2d& Polygon : SuppressionPolygons)
		{
			if (Polygon.Contains(Point))
			{
				return true;
			}
		}
		return false;
	}

	struct FSidewalkWidthGroup
	{
		double Width = 0.0;
		TArray<FGeneralPolygon2d> Polygons;
	};

	void AddSidewalkSourcePolygon(const FGeneralPolygon2d& Polygon, double Width, TArray<FSidewalkWidthGroup>& OutGroups)
	{
		if (Width <= KINDA_SMALL_NUMBER)
		{
			return;
		}

		FSidewalkWidthGroup* Group = OutGroups.FindByPredicate([Width](const FSidewalkWidthGroup& Candidate)
		{
			return FMath::IsNearlyEqual(Candidate.Width, Width, SidewalkWidthGroupTolerance);
		});
		if (!Group)
		{
			Group = &OutGroups.AddDefaulted_GetRef();
			Group->Width = Width;
		}
		Group->Polygons.Add(Polygon);
	}

	void BuildOffsetSidewalkPolygons(TConstArrayView<FSidewalkWidthGroup> WidthGroups, double MinimumPolygonArea,
		TArray<FGeneralPolygon2d>& OutBufferedPolygons)
	{
		for (const FSidewalkWidthGroup& Group : WidthGroups)
		{
			if (Group.Polygons.IsEmpty())
			{
				continue;
			}

			// Union first so internal segment and junction seams never become offset boundaries.
			// Separate width groups retain road-profile-specific sidewalk widths; their buffers are
			// merged below before the complete unified roadway is subtracted.
			TArray<FGeneralPolygon2d> UnifiedGroupPolygons;
			PolygonsUnion(Group.Polygons, UnifiedGroupPolygons, true);
			FilterTinyPolygonFeatures(UnifiedGroupPolygons, MinimumPolygonArea);
			SimplifyPolygonRings(UnifiedGroupPolygons);
			if (UnifiedGroupPolygons.IsEmpty())
			{
				continue;
			}

			TArray<FGeneralPolygon2d> OffsetPolygons;
			const bool bOffsetSucceeded = PolygonsOffset(
				Group.Width,
				UnifiedGroupPolygons,
				OffsetPolygons,
				false,
				SidewalkOffsetMiterLimit,
				EPolygonOffsetJoinType::Round,
				EPolygonOffsetEndType::Polygon,
				SidewalkOffsetMaxStepsPerRadian,
				SidewalkOffsetRoundScale);
			if (bOffsetSucceeded)
			{
				OutBufferedPolygons.Append(MoveTemp(OffsetPolygons));
			}
		}
	}

	void AppendCurbEdge(const FVector2d& A2d, const FVector2d& B2d, const FVector2d& Outward, TConstArrayView<FSurfaceSupport> Supports, TArray<FFlexMeshSectionData>& OutSections, TArray<TArray<FVector>>& OutCurbLines)
	{
		const FResolvedSurface SurfaceA = ResolveSurface(A2d, Supports);
		const FResolvedSurface SurfaceB = ResolveSurface(B2d, Supports);
		if (FMath::Max(SurfaceA.CurbHeight, SurfaceB.CurbHeight) <= KINDA_SMALL_NUMBER)
		{
			return;
		}
		const FVector A(A2d.X, A2d.Y, SurfaceA.Z);
		const FVector B(B2d.X, B2d.Y, SurfaceB.Z);
		const FVector ATop = A + FVector::UpVector * SurfaceA.CurbHeight;
		const FVector BTop = B + FVector::UpVector * SurfaceB.CurbHeight;
		const FVector Normal(Outward.X, Outward.Y, 0.f);
		const FVector Tangent = (B - A).GetSafeNormal(UE_SMALL_NUMBER, FVector::ForwardVector);
		const float Length = FVector::Distance(A, B) * UvScale;
		TArray<FVector> CurbLine;
		// Normalize line direction so a spline mesh's local +Y (world Up x forward) always points
		// away from the roadway. ApplyCurbstones can then place an arbitrary-pivot mesh wholly on
		// the sidewalk side instead of letting half of its width straddle the road edge.
		if (FVector::DotProduct(FVector::CrossProduct(FVector::UpVector, Tangent), Normal) >= 0.f)
		{
			CurbLine.Add(A);
			CurbLine.Add(B);
		}
		else
		{
			CurbLine.Add(B);
			CurbLine.Add(A);
		}
		OutCurbLines.Add(MoveTemp(CurbLine));
		const FResolvedSurface MidSurface = ResolveSurface((A2d + B2d) * 0.5, Supports);
		UMaterialInterface* Material = MidSurface.Input
			? (MidSurface.Input->CurbMaterial ? MidSurface.Input->CurbMaterial : MidSurface.Input->SidewalkMaterial)
			: nullptr;
		FFlexMeshSectionData& OutSection = FindOrAddSection(OutSections, Material);

		const FVector Cross = FVector::CrossProduct(B - A, BTop - A);
		if (FVector::DotProduct(Cross, Normal) > 0.f)
		{
			OutSection.AppendQuad(A, ATop, BTop, B, Normal, Tangent,
				FVector2D(0.f, 0.f), FVector2D(0.f, 1.f), FVector2D(Length, 1.f), FVector2D(Length, 0.f));
		}
		else
		{
			OutSection.AppendQuad(A, B, BTop, ATop, Normal, Tangent,
				FVector2D(0.f, 0.f), FVector2D(Length, 0.f), FVector2D(Length, 1.f), FVector2D(0.f, 1.f));
		}
	}

	void BuildBoundaryCurbs(const FGeneralPolygon2d& RoadPolygon, TConstArrayView<FSurfaceSupport> Supports,
		TConstArrayView<FGeneralPolygon2d> SuppressionPolygons, TArray<FFlexMeshSectionData>& OutCurbs,
		TArray<TArray<FVector>>& OutCurbLines)
	{
		const bool bOuterClockwise = RoadPolygon.OuterIsClockwise();
		auto VisitRing = [&](const FPolygon2d& Ring)
		{
			const TArray<FVector2d>& Vertices = Ring.GetVertices();
			for (int32 Index = 0; Index < Vertices.Num(); ++Index)
			{
				const FVector2d A = Vertices[Index];
				const FVector2d B = Vertices[(Index + 1) % Vertices.Num()];
				const FVector2d Edge = B - A;
				if (Edge.SquaredLength() <= MinEdgeLengthSquared)
				{
					continue;
				}
				const FVector2d Left(-Edge.Y, Edge.X);
				const FVector2d Outward = (bOuterClockwise ? Left : -Left).GetSafeNormal();
				const int32 CurbSteps = FMath::Max(1, FMath::CeilToInt(Edge.Length() / MaxCurbEdgeLength));
				for (int32 Step = 0; Step < CurbSteps; ++Step)
				{
					const double AlphaA = static_cast<double>(Step) / CurbSteps;
					const double AlphaB = static_cast<double>(Step + 1) / CurbSteps;
					const FVector2d CurbA = FMath::Lerp(A, B, AlphaA);
					const FVector2d CurbB = FMath::Lerp(A, B, AlphaB);
					if (!IsSuppressed((CurbA + CurbB) * 0.5, SuppressionPolygons))
					{
						AppendCurbEdge(CurbA, CurbB, Outward, Supports, OutCurbs, OutCurbLines);
					}
				}
			}
		};

		VisitRing(RoadPolygon.GetOuter());
		for (const FPolygon2d& Hole : RoadPolygon.GetHoles())
		{
			VisitRing(Hole);
		}
	}
}

FFlexUnifiedNetworkMeshResult FFlexUnifiedRoadMeshBuilder::Build(TConstArrayView<FFlexUnifiedRoadPolygonInput> SurfaceInputs, TConstArrayView<FFlexUnifiedRoadSuppressionInput> SuppressionInputs, double MinimumPolygonArea)
{
	FFlexUnifiedNetworkMeshResult Result;
	if (SurfaceInputs.IsEmpty())
	{
		return Result;
	}

	TSet<int32> Layers;
	for (const FFlexUnifiedRoadPolygonInput& Input : SurfaceInputs)
	{
		Layers.Add(Input.ElevationLayer);
	}

	for (const int32 Layer : Layers)
	{
		TArray<FGeneralPolygon2d> InputPolygons;
		TArray<FSurfaceSupport> Supports;
		TArray<FSidewalkWidthGroup> SidewalkWidthGroups;
		for (const FFlexUnifiedRoadPolygonInput& Input : SurfaceInputs)
		{
			if (Input.ElevationLayer != Layer)
			{
				continue;
			}
			FGeneralPolygon2d Polygon;
			if (MakePolygon(Input.Boundary, Polygon))
			{
				AddSidewalkSourcePolygon(Polygon, Input.SidewalkWidth, SidewalkWidthGroups);
				InputPolygons.Add(MoveTemp(Polygon));
				AddSupports(Input, Supports);
			}
		}
		if (InputPolygons.IsEmpty())
		{
			continue;
		}

		TArray<FGeneralPolygon2d> UnifiedRoadPolygons;
		PolygonsUnion(InputPolygons, UnifiedRoadPolygons, true);
		FilterTinyPolygonFeatures(UnifiedRoadPolygons, MinimumPolygonArea);
		SimplifyPolygonRings(UnifiedRoadPolygons);
		AppendTriangulatedPolygons(UnifiedRoadPolygons, Supports, false, Result.Roadways);

		TArray<FGeneralPolygon2d> SidewalkSuppressionPolygons;
		TArray<FGeneralPolygon2d> CurbSuppressionPolygons;
		for (const FFlexUnifiedRoadSuppressionInput& Input : SuppressionInputs)
		{
			if (Input.ElevationLayer == Layer)
			{
				FGeneralPolygon2d Polygon;
				if (MakePolygon(Input.Boundary, Polygon))
				{
					if (Input.bSuppressSidewalks)
					{
						SidewalkSuppressionPolygons.Add(Polygon);
					}
					if (Input.bSuppressCurbs)
					{
						CurbSuppressionPolygons.Add(MoveTemp(Polygon));
					}
				}
			}
		}
		auto UnionSuppressionPolygons = [](TArray<FGeneralPolygon2d>& Polygons)
		{
			if (Polygons.Num() > 1)
			{
				TArray<FGeneralPolygon2d> Unified;
				PolygonsUnion(Polygons, Unified, true);
				Polygons = MoveTemp(Unified);
			}
		};
		UnionSuppressionPolygons(SidewalkSuppressionPolygons);
		UnionSuppressionPolygons(CurbSuppressionPolygons);

		for (const FGeneralPolygon2d& Polygon : UnifiedRoadPolygons)
		{
			BuildBoundaryCurbs(Polygon, Supports, CurbSuppressionPolygons, Result.Curbs, Result.CurbLines);
		}

		TArray<FGeneralPolygon2d> BufferedRoadPolygons;
		BuildOffsetSidewalkPolygons(SidewalkWidthGroups, MinimumPolygonArea, BufferedRoadPolygons);
		if (!BufferedRoadPolygons.IsEmpty())
		{
			TArray<FGeneralPolygon2d> UnifiedSidewalkPolygons;
			PolygonsUnion(BufferedRoadPolygons, UnifiedSidewalkPolygons, true);
			TArray<FGeneralPolygon2d> OutsideRoadPolygons;
			PolygonsDifference(UnifiedSidewalkPolygons, UnifiedRoadPolygons, OutsideRoadPolygons);
			FilterTinyPolygonFeatures(OutsideRoadPolygons, MinimumPolygonArea);
			if (!SidewalkSuppressionPolygons.IsEmpty())
			{
				TArray<FGeneralPolygon2d> UnsuppressedSidewalkPolygons;
				PolygonsDifference(OutsideRoadPolygons, SidewalkSuppressionPolygons, UnsuppressedSidewalkPolygons);
				OutsideRoadPolygons = MoveTemp(UnsuppressedSidewalkPolygons);
			}
			// Suppression holes are intentional. Only discard tiny components created by the final
			// subtraction; do not fill a small user/configuration-defined suppression region.
			FilterTinyPolygonComponents(OutsideRoadPolygons, MinimumPolygonArea);
			SimplifyPolygonRings(OutsideRoadPolygons);
			AppendTriangulatedPolygons(OutsideRoadPolygons, Supports, true, Result.Sidewalks);
		}
	}

	return Result;
}
