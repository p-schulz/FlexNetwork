#include "RoadTypeProfile.h"
#include "Algo/Sort.h"

float URoadTypeProfile::GetRoadwayMinOffset() const
{
	if (Lanes.IsEmpty())
	{
		return LateralOffset;
	}
	float MinOffset = TNumericLimits<float>::Max();
	for (const FRoadLaneDescriptor& Lane : Lanes)
	{
		MinOffset = FMath::Min(MinOffset, LateralOffset + FMath::Min(Lane.GetInnerEdge(), Lane.GetOuterEdge()));
	}
	return MinOffset;
}

float URoadTypeProfile::GetRoadwayMaxOffset() const
{
	if (Lanes.IsEmpty())
	{
		return LateralOffset;
	}
	float MaxOffset = TNumericLimits<float>::Lowest();
	for (const FRoadLaneDescriptor& Lane : Lanes)
	{
		MaxOffset = FMath::Max(MaxOffset, LateralOffset + FMath::Max(Lane.GetInnerEdge(), Lane.GetOuterEdge()));
	}
	return MaxOffset;
}

float URoadTypeProfile::GetRoadwayWidth() const
{
	return FMath::Max(0.f, GetRoadwayMaxOffset() - GetRoadwayMinOffset());
}

float URoadTypeProfile::GetRoadwayHalfWidth() const
{
	return GetRoadwayWidth() * 0.5f;
}

float URoadTypeProfile::GetRoadwayCenterOffset() const
{
	return (GetRoadwayMinOffset() + GetRoadwayMaxOffset()) * 0.5f;
}

float URoadTypeProfile::GetOuterExtent() const
{
	return FMath::Max(FMath::Abs(GetRoadwayMinOffset() - SidewalkWidth), FMath::Abs(GetRoadwayMaxOffset() + SidewalkWidth));
}

TArray<FRoadLaneDescriptor> URoadTypeProfile::GetLanesSortedByOffset() const
{
	TArray<FRoadLaneDescriptor> Sorted = Lanes;
	Algo::SortBy(Sorted, [](const FRoadLaneDescriptor& Lane) { return Lane.LateralOffset; });
	return Sorted;
}
