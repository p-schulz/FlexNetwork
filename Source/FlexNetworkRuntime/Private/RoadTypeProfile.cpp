#include "RoadTypeProfile.h"
#include "Algo/Sort.h"

float URoadTypeProfile::GetRoadwayHalfWidth() const
{
	float MaxAbsOuter = 0.f;
	for (const FRoadLaneDescriptor& Lane : Lanes)
	{
		MaxAbsOuter = FMath::Max(MaxAbsOuter, FMath::Abs(Lane.GetOuterEdge()));
		MaxAbsOuter = FMath::Max(MaxAbsOuter, FMath::Abs(Lane.GetInnerEdge()));
	}
	return MaxAbsOuter;
}

float URoadTypeProfile::GetOuterExtent() const
{
	return GetRoadwayHalfWidth() + SidewalkWidth;
}

TArray<FRoadLaneDescriptor> URoadTypeProfile::GetLanesSortedByOffset() const
{
	TArray<FRoadLaneDescriptor> Sorted = Lanes;
	Algo::SortBy(Sorted, [](const FRoadLaneDescriptor& Lane) { return Lane.LateralOffset; });
	return Sorted;
}
