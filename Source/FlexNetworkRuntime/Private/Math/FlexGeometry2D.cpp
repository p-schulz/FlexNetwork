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

bool FlexGeometry2D::ComputeFilletArc(const FVector2D& CornerPoint, const FVector2D& DirAwayFromCornerA, const FVector2D& DirAwayFromCornerB, float Radius, TArray<FVector2D>& OutArcPoints, int32 ArcSegments)
{
	OutArcPoints.Reset();

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
	const FVector2D Center = CornerPoint + Bisector * CenterDist;

	const float AngleA = FMath::Atan2(TangentA.Y - Center.Y, TangentA.X - Center.X);
	const float AngleB = FMath::Atan2(TangentB.Y - Center.Y, TangentB.X - Center.X);
	const float Delta = FMath::FindDeltaAngleRadians(AngleA, AngleB);

	ArcSegments = FMath::Max(ArcSegments, 1);
	OutArcPoints.Reserve(ArcSegments + 1);
	for (int32 i = 0; i <= ArcSegments; ++i)
	{
		const float Alpha = static_cast<float>(i) / static_cast<float>(ArcSegments);
		const float Angle = AngleA + Delta * Alpha;
		OutArcPoints.Add(Center + FVector2D(FMath::Cos(Angle), FMath::Sin(Angle)) * Radius);
	}

	return true;
}
