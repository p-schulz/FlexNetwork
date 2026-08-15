#include "Terrain/FlexLandscapeConformer.h"
#include "EngineUtils.h"
#include "Landscape.h"
#include "LandscapeInfo.h"

#if WITH_EDITOR
#include "LandscapeEdit.h"
#include "LandscapeDataAccess.h"
#endif

namespace
{
	float DistanceAndHeightOnPolyline(const TArray<FFlexCurveFrame>& Frames, const FVector2D& Point2D, float& OutRoadWorldZ)
	{
		float BestDistSq = MAX_flt;
		float BestZ = 0.f;
		for (int32 i = 0; i + 1 < Frames.Num(); ++i)
		{
			const FVector2D A(Frames[i].Position.X, Frames[i].Position.Y);
			const FVector2D B(Frames[i + 1].Position.X, Frames[i + 1].Position.Y);
			const FVector2D AB = B - A;
			const float LenSq = AB.SizeSquared();
			float T = LenSq > KINDA_SMALL_NUMBER ? (FVector2D::DotProduct(Point2D - A, AB) / LenSq) : 0.f;
			T = FMath::Clamp(T, 0.f, 1.f);
			const FVector2D Closest = A + AB * T;
			const float DistSq = FVector2D::DistSquared(Point2D, Closest);
			if (DistSq < BestDistSq)
			{
				BestDistSq = DistSq;
				BestZ = FMath::Lerp(Frames[i].Position.Z, Frames[i + 1].Position.Z, T);
			}
		}
		OutRoadWorldZ = BestZ;
		return Frames.Num() >= 2 ? FMath::Sqrt(BestDistSq) : MAX_flt;
	}
}

ALandscape* FFlexLandscapeConformer::FindLandscape(UWorld* World)
{
	if (!World)
	{
		return nullptr;
	}
	for (TActorIterator<ALandscape> It(World); It; ++It)
	{
		return *It;
	}
	return nullptr;
}

#if WITH_EDITOR

void FFlexLandscapeConformer::RestoreSavedRegion(UWorld* World, FFlexSegmentId SegmentId)
{
	FSavedHeightRegion Saved;
	if (!SavedRegionsBySegment.RemoveAndCopyValue(SegmentId, Saved))
	{
		return;
	}

	ALandscape* Landscape = FindLandscape(World);
	if (!Landscape)
	{
		return;
	}
	ULandscapeInfo* Info = Landscape->GetLandscapeInfo();
	if (!Info)
	{
		return;
	}

	FLandscapeEditDataInterface EditInterface(Info);
	int32 X1 = Saved.MinX, Y1 = Saved.MinY, X2 = Saved.MaxX, Y2 = Saved.MaxY;
	EditInterface.SetHeightData(X1, Y1, X2, Y2, Saved.OriginalHeights.GetData(), 0, /*InCalcNormals=*/ true);
}

void FFlexLandscapeConformer::ConformSegment(UWorld* World, FFlexSegmentId SegmentId, const TArray<FFlexCurveFrame>& Frames, float RoadHalfWidth, float Margin, float FalloffDistance)
{
	if (Frames.Num() < 2)
	{
		return;
	}

	ALandscape* Landscape = FindLandscape(World);
	if (!Landscape)
	{
		return;
	}
	ULandscapeInfo* Info = Landscape->GetLandscapeInfo();
	if (!Info)
	{
		return;
	}

	// Repaint from a clean slate: undo whatever footprint this segment left last time before
	// computing its (possibly moved/resized) new footprint, so a segment that shrinks or shifts
	// doesn't leave a stale flattened patch behind.
	RestoreSavedRegion(World, SegmentId);

	const float TotalHalfExtent = RoadHalfWidth + Margin + FalloffDistance;
	const FTransform& LandscapeTransform = Landscape->GetActorTransform();

	FVector2D WorldMin(TNumericLimits<float>::Max(), TNumericLimits<float>::Max());
	FVector2D WorldMax(TNumericLimits<float>::Lowest(), TNumericLimits<float>::Lowest());
	for (const FFlexCurveFrame& Frame : Frames)
	{
		for (float Side : { -1.f, 1.f })
		{
			const FVector P = Frame.Position + Frame.Right * (TotalHalfExtent * Side);
			WorldMin.X = FMath::Min(WorldMin.X, static_cast<float>(P.X));
			WorldMin.Y = FMath::Min(WorldMin.Y, static_cast<float>(P.Y));
			WorldMax.X = FMath::Max(WorldMax.X, static_cast<float>(P.X));
			WorldMax.Y = FMath::Max(WorldMax.Y, static_cast<float>(P.Y));
		}
	}

	const FVector LocalMin = LandscapeTransform.InverseTransformPosition(FVector(WorldMin.X, WorldMin.Y, 0.f));
	const FVector LocalMax = LandscapeTransform.InverseTransformPosition(FVector(WorldMax.X, WorldMax.Y, 0.f));

	int32 MinX = FMath::FloorToInt32(FMath::Min(LocalMin.X, LocalMax.X));
	int32 MinY = FMath::FloorToInt32(FMath::Min(LocalMin.Y, LocalMax.Y));
	int32 MaxX = FMath::CeilToInt32(FMath::Max(LocalMin.X, LocalMax.X));
	int32 MaxY = FMath::CeilToInt32(FMath::Max(LocalMin.Y, LocalMax.Y));

	int32 LandscapeMinX, LandscapeMinY, LandscapeMaxX, LandscapeMaxY;
	if (!Info->GetLandscapeExtent(LandscapeMinX, LandscapeMinY, LandscapeMaxX, LandscapeMaxY))
	{
		return;
	}
	MinX = FMath::Clamp(MinX, LandscapeMinX, LandscapeMaxX);
	MinY = FMath::Clamp(MinY, LandscapeMinY, LandscapeMaxY);
	MaxX = FMath::Clamp(MaxX, LandscapeMinX, LandscapeMaxX);
	MaxY = FMath::Clamp(MaxY, LandscapeMinY, LandscapeMaxY);
	if (MaxX <= MinX || MaxY <= MinY)
	{
		// The segment's footprint doesn't actually overlap this landscape.
		return;
	}

	const int32 Width = MaxX - MinX + 1;
	const int32 Height = MaxY - MinY + 1;

	TArray<uint16> OriginalHeights;
	OriginalHeights.SetNumUninitialized(Width * Height);
	{
		FLandscapeEditDataInterface ReadInterface(Info);
		int32 X1 = MinX, Y1 = MinY, X2 = MaxX, Y2 = MaxY;
		ReadInterface.GetHeightData(X1, Y1, X2, Y2, OriginalHeights.GetData(), 0);
	}

	TArray<uint16> NewHeights = OriginalHeights;
	for (int32 Y = MinY; Y <= MaxY; ++Y)
	{
		for (int32 X = MinX; X <= MaxX; ++X)
		{
			const int32 Index = (Y - MinY) * Width + (X - MinX);
			const uint16 OriginalPacked = OriginalHeights[Index];

			const FVector VertexWorld = LandscapeTransform.TransformPosition(FVector(static_cast<float>(X), static_cast<float>(Y), LandscapeDataAccess::GetLocalHeight(OriginalPacked)));

			float RoadWorldZ = 0.f;
			const float Dist = DistanceAndHeightOnPolyline(Frames, FVector2D(VertexWorld.X, VertexWorld.Y), RoadWorldZ);

			if (Dist > RoadHalfWidth + Margin + FalloffDistance)
			{
				continue; // Untouched -- outside this segment's influence entirely.
			}

			float Alpha = 1.f;
			if (Dist > RoadHalfWidth + Margin)
			{
				const float FalloffAlpha = (Dist - (RoadHalfWidth + Margin)) / FMath::Max(FalloffDistance, KINDA_SMALL_NUMBER);
				Alpha = 1.f - FMath::SmoothStep(0.f, 1.f, FalloffAlpha);
			}

			const float OriginalWorldZ = VertexWorld.Z;
			const float BlendedWorldZ = FMath::Lerp(OriginalWorldZ, RoadWorldZ, Alpha);

			const FVector BlendedLocal = LandscapeTransform.InverseTransformPosition(FVector(VertexWorld.X, VertexWorld.Y, BlendedWorldZ));
			NewHeights[Index] = LandscapeDataAccess::GetTexHeight(BlendedLocal.Z);
		}
	}

	{
		FLandscapeEditDataInterface WriteInterface(Info);
		int32 X1 = MinX, Y1 = MinY, X2 = MaxX, Y2 = MaxY;
		WriteInterface.SetHeightData(X1, Y1, X2, Y2, NewHeights.GetData(), 0, /*InCalcNormals=*/ true);
	}

	FSavedHeightRegion& Saved = SavedRegionsBySegment.Add(SegmentId);
	Saved.MinX = MinX;
	Saved.MinY = MinY;
	Saved.MaxX = MaxX;
	Saved.MaxY = MaxY;
	Saved.OriginalHeights = MoveTemp(OriginalHeights);
}

void FFlexLandscapeConformer::RemoveSegmentConforming(UWorld* World, FFlexSegmentId SegmentId)
{
	RestoreSavedRegion(World, SegmentId);
}

#else // !WITH_EDITOR

void FFlexLandscapeConformer::ConformSegment(UWorld* World, FFlexSegmentId SegmentId, const TArray<FFlexCurveFrame>& Frames, float RoadHalfWidth, float Margin, float FalloffDistance)
{
	// Landscape heightmap editing (even via FLandscapeEditDataInterface, which this class relies
	// on) is an editor-time-only capability in UE5 -- there is no supported non-destructive way
	// to repaint a landscape at runtime in a packaged game. Projects that need runtime terrain
	// deformation should supply their own IFlexTerrainConformer over a runtime-capable terrain
	// system instead.
}

void FFlexLandscapeConformer::RemoveSegmentConforming(UWorld* World, FFlexSegmentId SegmentId)
{
}

void FFlexLandscapeConformer::RestoreSavedRegion(UWorld* World, FFlexSegmentId SegmentId)
{
}

#endif // WITH_EDITOR
