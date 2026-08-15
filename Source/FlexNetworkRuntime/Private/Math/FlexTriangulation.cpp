#include "Math/FlexTriangulation.h"
#include "Math/FlexGeometry2D.h"
#include "Algo/Reverse.h"

namespace
{
	bool IsConvexVertex(const FVector2D& Prev, const FVector2D& Curr, const FVector2D& Next)
	{
		return FVector2D::CrossProduct(Curr - Prev, Next - Curr) > 0.f;
	}

	bool PointInTriangle(const FVector2D& P, const FVector2D& A, const FVector2D& B, const FVector2D& C)
	{
		const float D0 = FVector2D::CrossProduct(B - A, P - A);
		const float D1 = FVector2D::CrossProduct(C - B, P - B);
		const float D2 = FVector2D::CrossProduct(A - C, P - C);

		const bool bHasNeg = (D0 < 0.f) || (D1 < 0.f) || (D2 < 0.f);
		const bool bHasPos = (D0 > 0.f) || (D1 > 0.f) || (D2 > 0.f);
		return !(bHasNeg && bHasPos);
	}
}

bool FlexTriangulation::EarClipTriangulate(TArrayView<const FVector2D> Polygon, TArray<int32>& OutTriangleIndices)
{
	OutTriangleIndices.Reset();

	const int32 N = Polygon.Num();
	if (N < 3)
	{
		return false;
	}

	TArray<int32> Remaining;
	Remaining.Reserve(N);
	for (int32 i = 0; i < N; ++i)
	{
		Remaining.Add(i);
	}

	if (FlexGeometry2D::SignedArea(Polygon) < 0.f)
	{
		Algo::Reverse(Remaining);
	}

	const int32 GuardLimit = N * N + 16;
	int32 GuardCounter = 0;

	while (Remaining.Num() > 3 && GuardCounter++ < GuardLimit)
	{
		const int32 Count = Remaining.Num();
		bool bClippedEar = false;

		for (int32 i = 0; i < Count; ++i)
		{
			const int32 PrevIdx = Remaining[(i + Count - 1) % Count];
			const int32 CurrIdx = Remaining[i];
			const int32 NextIdx = Remaining[(i + 1) % Count];

			const FVector2D& Prev = Polygon[PrevIdx];
			const FVector2D& Curr = Polygon[CurrIdx];
			const FVector2D& Next = Polygon[NextIdx];

			if (!IsConvexVertex(Prev, Curr, Next))
			{
				continue;
			}

			bool bAnyOtherPointInside = false;
			for (int32 Other : Remaining)
			{
				if (Other == PrevIdx || Other == CurrIdx || Other == NextIdx)
				{
					continue;
				}
				if (PointInTriangle(Polygon[Other], Prev, Curr, Next))
				{
					bAnyOtherPointInside = true;
					break;
				}
			}
			if (bAnyOtherPointInside)
			{
				continue;
			}

			OutTriangleIndices.Add(PrevIdx);
			OutTriangleIndices.Add(CurrIdx);
			OutTriangleIndices.Add(NextIdx);
			Remaining.RemoveAt(i);
			bClippedEar = true;
			break;
		}

		if (!bClippedEar)
		{
			// No convex, unobstructed ear found -- degenerate or self-intersecting input.
			return false;
		}
	}

	if (Remaining.Num() == 3)
	{
		OutTriangleIndices.Add(Remaining[0]);
		OutTriangleIndices.Add(Remaining[1]);
		OutTriangleIndices.Add(Remaining[2]);
	}

	return true;
}
