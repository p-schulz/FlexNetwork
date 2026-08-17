#include "Math/FlexGeometry2D.h"

bool FlexGeometry2D::LineLineIntersection(const FVector2D& OriginA, const FVector2D& DirA, const FVector2D& OriginB, const FVector2D& DirB, FVector2D& OutPoint)
{
	const float Denom = FVector2D::CrossProduct(DirA, DirB);
	if (FMath::Abs(Denom) <= KINDA_SMALL_NUMBER)
	{
		return false;
	}

	const FVector2D Delta = OriginB - OriginA;
	const float T = FVector2D::CrossProduct(Delta, DirB) / Denom;
	OutPoint = OriginA + DirA * T;
	return true;
}

bool FlexGeometry2D::SegmentSegmentIntersection(const FVector2D& A0, const FVector2D& A1, const FVector2D& B0, const FVector2D& B1, FVector2D& OutPoint, float& OutAlphaA, float& OutAlphaB)
{
	const FVector2D DirA = A1 - A0;
	const FVector2D DirB = B1 - B0;
	const float Denom = FVector2D::CrossProduct(DirA, DirB);
	if (FMath::Abs(Denom) <= KINDA_SMALL_NUMBER)
	{
		return false;
	}

	const FVector2D Delta = B0 - A0;
	const float AlphaA = FVector2D::CrossProduct(Delta, DirB) / Denom;
	const float AlphaB = FVector2D::CrossProduct(Delta, DirA) / Denom;

	if (AlphaA < 0.f || AlphaA > 1.f || AlphaB < 0.f || AlphaB > 1.f)
	{
		return false;
	}

	OutAlphaA = AlphaA;
	OutAlphaB = AlphaB;
	OutPoint = A0 + DirA * AlphaA;
	return true;
}

float FlexGeometry2D::SignedArea(TArrayView<const FVector2D> Polygon)
{
	float Area = 0.f;
	const int32 N = Polygon.Num();
	for (int32 i = 0; i < N; ++i)
	{
		const FVector2D& A = Polygon[i];
		const FVector2D& B = Polygon[(i + 1) % N];
		Area += FVector2D::CrossProduct(A, B);
	}
	return Area * 0.5f;
}

bool FlexGeometry2D::ComputeFilletCenterAndSweep(const FVector2D& CornerPoint, const FVector2D& DirAwayFromCornerA, const FVector2D& DirAwayFromCornerB, float Radius, FVector2D& OutCenter, float& OutStartAngle, float& OutSweepAngle)
{
	const FVector2D UA = DirAwayFromCornerA.GetSafeNormal();
	const FVector2D UB = DirAwayFromCornerB.GetSafeNormal();
	if (UA.IsNearlyZero() || UB.IsNearlyZero())
	{
		return false;
	}

	float CosTheta = FVector2D::DotProduct(UA, UB);
	CosTheta = FMath::Clamp(CosTheta, -1.f, 1.f);
	const float Theta = FMath::Acos(CosTheta);

	// Nearly straight (Theta ~ PI, edges continue through the corner) or nearly folded back on
	// itself (Theta ~ 0): no well-defined convex corner to round off.
	if (Theta < KINDA_SMALL_NUMBER || Theta > PI - KINDA_SMALL_NUMBER)
	{
		return false;
	}

	const float HalfTheta = Theta * 0.5f;
	const float TangentDist = Radius / FMath::Tan(HalfTheta);
	const float CenterDist = Radius / FMath::Sin(HalfTheta);

	const FVector2D TangentA = CornerPoint + UA * TangentDist;
	const FVector2D TangentB = CornerPoint + UB * TangentDist;
	const FVector2D Bisector = (UA + UB).GetSafeNormal();
	OutCenter = CornerPoint + Bisector * CenterDist;

	const float AngleA = FMath::Atan2(TangentA.Y - OutCenter.Y, TangentA.X - OutCenter.X);
	const float AngleB = FMath::Atan2(TangentB.Y - OutCenter.Y, TangentB.X - OutCenter.X);

	OutStartAngle = AngleA;
	OutSweepAngle = FMath::FindDeltaAngleRadians(AngleA, AngleB);
	return true;
}

void FlexGeometry2D::SampleArc(const FVector2D& Center, float StartAngle, float SweepAngle, float Radius, int32 ArcSegments, TArray<FVector2D>& OutPoints)
{
	OutPoints.Reset();
	ArcSegments = FMath::Max(ArcSegments, 1);
	OutPoints.Reserve(ArcSegments + 1);
	for (int32 i = 0; i <= ArcSegments; ++i)
	{
		const float Alpha = static_cast<float>(i) / static_cast<float>(ArcSegments);
		const float Angle = StartAngle + SweepAngle * Alpha;
		OutPoints.Add(Center + FVector2D(FMath::Cos(Angle), FMath::Sin(Angle)) * Radius);
	}
}

bool FlexGeometry2D::IsSimplePolygon(TArrayView<const FVector2D> Polygon)
{
	const int32 N = Polygon.Num();
	if (N < 3)
	{
		return false;
	}

	for (int32 i = 0; i < N; ++i)
	{
		const int32 NextI = (i + 1) % N;
		for (int32 j = i + 1; j < N; ++j)
		{
			const int32 NextJ = (j + 1) % N;
			// Skip adjacent edges (they legitimately share an endpoint) and the wraparound pair
			// where edge j's own next vertex is edge i's start.
			if (j == i || j == NextI || NextJ == i)
			{
				continue;
			}

			FVector2D IntersectionPoint;
			float AlphaA = 0.f, AlphaB = 0.f;
			if (SegmentSegmentIntersection(Polygon[i], Polygon[NextI], Polygon[j], Polygon[NextJ], IntersectionPoint, AlphaA, AlphaB))
			{
				return false;
			}
		}
	}

	return true;
}

bool FlexGeometry2D::ComputeFilletArc(const FVector2D& CornerPoint, const FVector2D& DirAwayFromCornerA, const FVector2D& DirAwayFromCornerB, float Radius, TArray<FVector2D>& OutArcPoints, int32 ArcSegments)
{
	OutArcPoints.Reset();

	FVector2D Center;
	float StartAngle, SweepAngle;
	if (!ComputeFilletCenterAndSweep(CornerPoint, DirAwayFromCornerA, DirAwayFromCornerB, Radius, Center, StartAngle, SweepAngle))
	{
		return false;
	}

	SampleArc(Center, StartAngle, SweepAngle, Radius, ArcSegments, OutArcPoints);
	return true;
}
