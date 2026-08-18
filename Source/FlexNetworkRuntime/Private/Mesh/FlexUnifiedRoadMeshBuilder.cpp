#include "Mesh/FlexUnifiedRoadMeshBuilder.h"

#include "CompGeom/Delaunay2.h"
#include "Curve/GeneralPolygon2.h"
#include "Curve/PolygonIntersectionUtils.h"
#include "Materials/MaterialInterface.h"
#include "Polygon2.h"

namespace
{
	using namespace UE::Geometry;

	constexpr double MinEdgeLengthSquared = 0.01;
	constexpr float UvScale = 0.01f;
	constexpr double MaxCurbEdgeLength = 100.0;

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

	void AddStripPolygon(const FVector2d& A, const FVector2d& B, const FVector2d& Offset, TArray<FGeneralPolygon2d>& OutPolygons)
	{
		TArray<FVector2d> Vertices{ A, B, B + Offset, A + Offset };
		FGeneralPolygon2d Polygon;
		if (MakePolygon(TArray<FVector>{
			FVector(Vertices[0].X, Vertices[0].Y, 0.f), FVector(Vertices[1].X, Vertices[1].Y, 0.f),
			FVector(Vertices[2].X, Vertices[2].Y, 0.f), FVector(Vertices[3].X, Vertices[3].Y, 0.f) }, Polygon))
		{
			OutPolygons.Add(MoveTemp(Polygon));
		}
	}

	void AddCornerPolygon(const FVector2d& Center, const FVector2d& Outward, double Radius, TArray<FGeneralPolygon2d>& OutPolygons)
	{
		if (Radius <= UE_DOUBLE_SMALL_NUMBER)
		{
			return;
		}
		TArray<FVector> Points;
		constexpr int32 Steps = 8;
		Points.Reserve(Steps);
		const double BaseAngle = FMath::Atan2(Outward.Y, Outward.X);
		for (int32 Step = 0; Step < Steps; ++Step)
		{
			const double Angle = BaseAngle + UE_TWO_PI * static_cast<double>(Step) / Steps;
			Points.Add(FVector(Center.X + FMath::Cos(Angle) * Radius, Center.Y + FMath::Sin(Angle) * Radius, 0.f));
		}
		FGeneralPolygon2d Polygon;
		if (MakePolygon(Points, Polygon))
		{
			OutPolygons.Add(MoveTemp(Polygon));
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
		CurbLine.Add(A);
		CurbLine.Add(B);
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

	void BuildBoundaryGeometry(const FGeneralPolygon2d& RoadPolygon, TConstArrayView<FSurfaceSupport> Supports, TConstArrayView<FGeneralPolygon2d> SuppressionPolygons, TArray<FGeneralPolygon2d>& OutRawSidewalkPolygons, TArray<FFlexMeshSectionData>& OutCurbs, TArray<TArray<FVector>>& OutCurbLines)
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
				const FVector2d Midpoint = (A + B) * 0.5;
				const FResolvedSurface Surface = ResolveSurface(Midpoint, Supports);
				if (Surface.SidewalkWidth > KINDA_SMALL_NUMBER)
				{
					AddStripPolygon(A, B, Outward * Surface.SidewalkWidth, OutRawSidewalkPolygons);
					AddCornerPolygon(A, Outward, Surface.SidewalkWidth, OutRawSidewalkPolygons);
				}
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

FFlexUnifiedNetworkMeshResult FFlexUnifiedRoadMeshBuilder::Build(TConstArrayView<FFlexUnifiedRoadPolygonInput> SurfaceInputs, TConstArrayView<FFlexUnifiedRoadSuppressionInput> SuppressionInputs)
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
		for (const FFlexUnifiedRoadPolygonInput& Input : SurfaceInputs)
		{
			if (Input.ElevationLayer != Layer)
			{
				continue;
			}
			FGeneralPolygon2d Polygon;
			if (MakePolygon(Input.Boundary, Polygon))
			{
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
		AppendTriangulatedPolygons(UnifiedRoadPolygons, Supports, false, Result.Roadways);

		TArray<FGeneralPolygon2d> SuppressionPolygons;
		for (const FFlexUnifiedRoadSuppressionInput& Input : SuppressionInputs)
		{
			if (Input.ElevationLayer == Layer)
			{
				FGeneralPolygon2d Polygon;
				if (MakePolygon(Input.Boundary, Polygon))
				{
					SuppressionPolygons.Add(MoveTemp(Polygon));
				}
			}
		}
		if (SuppressionPolygons.Num() > 1)
		{
			TArray<FGeneralPolygon2d> UnifiedSuppression;
			PolygonsUnion(SuppressionPolygons, UnifiedSuppression, true);
			SuppressionPolygons = MoveTemp(UnifiedSuppression);
		}

		TArray<FGeneralPolygon2d> RawSidewalkPolygons;
		for (const FGeneralPolygon2d& Polygon : UnifiedRoadPolygons)
		{
			BuildBoundaryGeometry(Polygon, Supports, SuppressionPolygons, RawSidewalkPolygons, Result.Curbs, Result.CurbLines);
		}
		if (!RawSidewalkPolygons.IsEmpty())
		{
			TArray<FGeneralPolygon2d> UnifiedSidewalkPolygons;
			PolygonsUnion(RawSidewalkPolygons, UnifiedSidewalkPolygons, true);
			TArray<FGeneralPolygon2d> OutsideRoadPolygons;
			PolygonsDifference(UnifiedSidewalkPolygons, UnifiedRoadPolygons, OutsideRoadPolygons);
			if (!SuppressionPolygons.IsEmpty())
			{
				TArray<FGeneralPolygon2d> UnsuppressedSidewalkPolygons;
				PolygonsDifference(OutsideRoadPolygons, SuppressionPolygons, UnsuppressedSidewalkPolygons);
				OutsideRoadPolygons = MoveTemp(UnsuppressedSidewalkPolygons);
			}
			AppendTriangulatedPolygons(OutsideRoadPolygons, Supports, true, Result.Sidewalks);
		}
	}

	return Result;
}
