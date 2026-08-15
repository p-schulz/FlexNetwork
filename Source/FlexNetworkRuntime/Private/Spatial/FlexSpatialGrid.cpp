#include "Spatial/FlexSpatialGrid.h"

FFlexSpatialGrid::FFlexSpatialGrid(float InCellSize)
	: CellSize(FMath::Max(InCellSize, 1.f))
{
}

void FFlexSpatialGrid::Clear()
{
	NodeCells.Reset();
	SegmentCells.Reset();
}

void FFlexSpatialGrid::SetCellSize(float InCellSize)
{
	CellSize = FMath::Max(InCellSize, 1.f);
}

FFlexSpatialGrid::FCellKey FFlexSpatialGrid::CellOf(const FVector2D& Position) const
{
	return FCellKey{ FMath::FloorToInt32(Position.X / CellSize), FMath::FloorToInt32(Position.Y / CellSize) };
}

void FFlexSpatialGrid::ForEachCellInBounds(const FVector2D& BoundsMin, const FVector2D& BoundsMax, TFunctionRef<void(const FCellKey&)> Fn) const
{
	const FCellKey MinCell = CellOf(BoundsMin);
	const FCellKey MaxCell = CellOf(BoundsMax);
	for (int32 Y = MinCell.Y; Y <= MaxCell.Y; ++Y)
	{
		for (int32 X = MinCell.X; X <= MaxCell.X; ++X)
		{
			Fn(FCellKey{ X, Y });
		}
	}
}

void FFlexSpatialGrid::AddNode(FFlexNodeId Id, const FVector2D& Position)
{
	NodeCells.FindOrAdd(CellOf(Position)).AddUnique(Id);
}

void FFlexSpatialGrid::RemoveNode(FFlexNodeId Id, const FVector2D& Position)
{
	const FCellKey Key = CellOf(Position);
	if (TArray<FFlexNodeId>* Cell = NodeCells.Find(Key))
	{
		Cell->RemoveSingleSwap(Id);
		if (Cell->Num() == 0)
		{
			NodeCells.Remove(Key);
		}
	}
}

void FFlexSpatialGrid::UpdateNode(FFlexNodeId Id, const FVector2D& OldPosition, const FVector2D& NewPosition)
{
	if (CellOf(OldPosition) == CellOf(NewPosition))
	{
		return;
	}
	RemoveNode(Id, OldPosition);
	AddNode(Id, NewPosition);
}

void FFlexSpatialGrid::AddSegment(FFlexSegmentId Id, const FVector2D& BoundsMin, const FVector2D& BoundsMax)
{
	ForEachCellInBounds(BoundsMin, BoundsMax, [this, Id](const FCellKey& Key)
	{
		SegmentCells.FindOrAdd(Key).AddUnique(Id);
	});
}

void FFlexSpatialGrid::RemoveSegment(FFlexSegmentId Id, const FVector2D& BoundsMin, const FVector2D& BoundsMax)
{
	ForEachCellInBounds(BoundsMin, BoundsMax, [this, Id](const FCellKey& Key)
	{
		if (TArray<FFlexSegmentId>* Cell = SegmentCells.Find(Key))
		{
			Cell->RemoveSingleSwap(Id);
			if (Cell->Num() == 0)
			{
				SegmentCells.Remove(Key);
			}
		}
	});
}

void FFlexSpatialGrid::UpdateSegment(FFlexSegmentId Id, const FVector2D& OldBoundsMin, const FVector2D& OldBoundsMax, const FVector2D& NewBoundsMin, const FVector2D& NewBoundsMax)
{
	RemoveSegment(Id, OldBoundsMin, OldBoundsMax);
	AddSegment(Id, NewBoundsMin, NewBoundsMax);
}

TArray<FFlexNodeId> FFlexSpatialGrid::QueryNodesNear(const FVector2D& Center, float Radius) const
{
	TArray<FFlexNodeId> Result;
	const FVector2D Extent(Radius, Radius);
	ForEachCellInBounds(Center - Extent, Center + Extent, [this, &Result](const FCellKey& Key)
	{
		if (const TArray<FFlexNodeId>* Cell = NodeCells.Find(Key))
		{
			Result.Append(*Cell);
		}
	});
	return Result;
}

TArray<FFlexSegmentId> FFlexSpatialGrid::QuerySegmentsNear(const FVector2D& Center, float Radius) const
{
	TSet<FFlexSegmentId> ResultSet;
	const FVector2D Extent(Radius, Radius);
	ForEachCellInBounds(Center - Extent, Center + Extent, [this, &ResultSet](const FCellKey& Key)
	{
		if (const TArray<FFlexSegmentId>* Cell = SegmentCells.Find(Key))
		{
			ResultSet.Append(*Cell);
		}
	});
	return ResultSet.Array();
}
