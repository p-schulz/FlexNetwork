#include "Osm/FlexOsmBuildingFootprints.h"

#include "Osm/FlexOsmGraphBuilder.h"
#include "Osm/OsmDataAsset.h"

namespace
{
	bool IsTruthyTagValue(const FString& Value)
	{
		const FString Lower = Value.ToLower();
		return !Lower.IsEmpty() && Lower != TEXT("no") && Lower != TEXT("false") && Lower != TEXT("0");
	}

	bool HasBuildingTag(const TMap<FString, FString>& Tags)
	{
		const FString* Building = Tags.Find(TEXT("building"));
		const FString* Part = Tags.Find(TEXT("building:part"));
		return (Building && IsTruthyTagValue(*Building)) || (Part && IsTruthyTagValue(*Part));
	}

	bool IsBuildingPart(const TMap<FString, FString>& Tags)
	{
		const FString* Part = Tags.Find(TEXT("building:part"));
		return Part && IsTruthyTagValue(*Part);
	}

	TArray<TArray<int64>> StitchPolylinesIntoRings(const TArray<TArray<int64>>& Polylines)
	{
		TArray<TArray<int64>> Rings;
		TArray<bool> Used;
		Used.Init(false, Polylines.Num());

		for (int32 StartIndex = 0; StartIndex < Polylines.Num(); ++StartIndex)
		{
			if (Used[StartIndex] || Polylines[StartIndex].Num() < 2)
			{
				continue;
			}

			Used[StartIndex] = true;
			TArray<int64> Ring = Polylines[StartIndex];
			bool bExtended = true;
			while (bExtended && Ring.Last() != Ring[0])
			{
				bExtended = false;
				for (int32 Index = 0; Index < Polylines.Num(); ++Index)
				{
					if (Used[Index] || Polylines[Index].Num() < 2)
					{
						continue;
					}

					const TArray<int64>& Candidate = Polylines[Index];
					if (Candidate[0] == Ring.Last())
					{
						for (int32 PointIndex = 1; PointIndex < Candidate.Num(); ++PointIndex)
						{
							Ring.Add(Candidate[PointIndex]);
						}
					}
					else if (Candidate.Last() == Ring.Last())
					{
						for (int32 PointIndex = Candidate.Num() - 2; PointIndex >= 0; --PointIndex)
						{
							Ring.Add(Candidate[PointIndex]);
						}
					}
					else
					{
						continue;
					}

					Used[Index] = true;
					bExtended = true;
					break;
				}
			}

			if (Ring.Num() >= 4 && Ring.Last() == Ring[0])
			{
				Ring.Pop(EAllowShrinking::No);
				Rings.Add(MoveTemp(Ring));
			}
		}
		return Rings;
	}

	bool ResolveProjectedRing(
		const TArray<int64>& NodeIds,
		const UOsmDataAsset& Asset,
		double OriginLatitude,
		double OriginLongitude,
		TArray<FVector2D>& OutRing)
	{
		OutRing.Reset(NodeIds.Num());
		for (const int64 NodeId : NodeIds)
		{
			const FOsmNode* Node = Asset.Nodes.Find(NodeId);
			if (!Node)
			{
				OutRing.Reset();
				return false;
			}
			OutRing.Add(FFlexOsmGraphBuilder::ProjectLatLonToLocalCm(
				Node->Latitude, Node->Longitude, OriginLatitude, OriginLongitude) / 100.0);
		}
		return OutRing.Num() >= 3;
	}

	double SignedArea(const TArray<FVector2D>& Ring)
	{
		double AreaTwice = 0.0;
		for (int32 Index = 0; Index < Ring.Num(); ++Index)
		{
			const FVector2D& A = Ring[Index];
			const FVector2D& B = Ring[(Index + 1) % Ring.Num()];
			AreaTwice += A.X * B.Y - B.X * A.Y;
		}
		return AreaTwice * 0.5;
	}

	bool PointInRing(const FVector2D& Point, const TArray<FVector2D>& Ring)
	{
		bool bInside = false;
		for (int32 Index = 0, Previous = Ring.Num() - 1; Index < Ring.Num(); Previous = Index++)
		{
			const FVector2D& A = Ring[Index];
			const FVector2D& B = Ring[Previous];
			if (((A.Y > Point.Y) != (B.Y > Point.Y)) &&
				(Point.X < (B.X - A.X) * (Point.Y - A.Y) / (B.Y - A.Y) + A.X))
			{
				bInside = !bInside;
			}
		}
		return bInside;
	}
}

bool FFlexOsmBuildingFootprints::ExtractProjected(
	const UOsmDataAsset* OsmAsset,
	const FFlexOsmImportSettings& ImportSettings,
	TArray<FFlexOsmBuildingFootprint>& OutFootprints,
	FString& OutError)
{
	OutFootprints.Reset();
	OutError.Reset();
	if (!OsmAsset)
	{
		OutError = TEXT("No OSM asset was selected.");
		return false;
	}

	double OriginLatitude = 0.0;
	double OriginLongitude = 0.0;
	if (!FFlexOsmGraphBuilder::ResolveOrigin(*OsmAsset, ImportSettings, OriginLatitude, OriginLongitude))
	{
		OutError = TEXT("The OSM asset contains no usable origin or nodes.");
		return false;
	}

	TSet<int64> RelationMemberWayIds;
	for (const TPair<int64, FOsmRelation>& RelationPair : OsmAsset->Relations)
	{
		const FOsmRelation& Relation = RelationPair.Value;
		const FString* Type = Relation.Tags.Find(TEXT("type"));
		if (!Type || !Type->Equals(TEXT("multipolygon"), ESearchCase::IgnoreCase) || !HasBuildingTag(Relation.Tags))
		{
			continue;
		}

		TArray<TArray<int64>> OuterPolylines;
		TArray<TArray<int64>> InnerPolylines;
		for (const FOsmRelationMember& Member : Relation.Members)
		{
			if (Member.Type != EOsmElementType::Way)
			{
				continue;
			}
			const FOsmWay* Way = OsmAsset->Ways.Find(Member.Ref);
			if (!Way)
			{
				continue;
			}
			RelationMemberWayIds.Add(Member.Ref);
			(Member.Role.Equals(TEXT("inner"), ESearchCase::IgnoreCase) ? InnerPolylines : OuterPolylines).Add(Way->NodeRefs);
		}

		TArray<FFlexOsmBuildingFootprint> RelationFootprints;
		for (const TArray<int64>& RingIds : StitchPolylinesIntoRings(OuterPolylines))
		{
			FFlexOsmBuildingFootprint Footprint;
			if (!ResolveProjectedRing(RingIds, *OsmAsset, OriginLatitude, OriginLongitude, Footprint.OuterRingMeters))
			{
				continue;
			}
			Footprint.OsmId = RelationPair.Key;
			Footprint.SourceType = TEXT("relation");
			Footprint.Tags = Relation.Tags;
			Footprint.bIsBuildingPart = IsBuildingPart(Relation.Tags);
			RelationFootprints.Add(MoveTemp(Footprint));
		}

		for (const TArray<int64>& RingIds : StitchPolylinesIntoRings(InnerPolylines))
		{
			TArray<FVector2D> InnerRing;
			if (!ResolveProjectedRing(RingIds, *OsmAsset, OriginLatitude, OriginLongitude, InnerRing))
			{
				continue;
			}

			int32 BestOuter = INDEX_NONE;
			double BestArea = TNumericLimits<double>::Max();
			for (int32 Index = 0; Index < RelationFootprints.Num(); ++Index)
			{
				if (!PointInRing(InnerRing[0], RelationFootprints[Index].OuterRingMeters))
				{
					continue;
				}
				const double Area = FMath::Abs(SignedArea(RelationFootprints[Index].OuterRingMeters));
				if (Area < BestArea)
				{
					BestArea = Area;
					BestOuter = Index;
				}
			}
			if (BestOuter != INDEX_NONE)
			{
				FFlexOsmBuildingRing Hole;
				Hole.PointsMeters = MoveTemp(InnerRing);
				RelationFootprints[BestOuter].Holes.Add(MoveTemp(Hole));
			}
		}

		OutFootprints.Append(MoveTemp(RelationFootprints));
	}

	for (const TPair<int64, FOsmWay>& WayPair : OsmAsset->Ways)
	{
		const FOsmWay& Way = WayPair.Value;
		if (RelationMemberWayIds.Contains(WayPair.Key) || !HasBuildingTag(Way.Tags) ||
			Way.NodeRefs.Num() < 4 || Way.NodeRefs[0] != Way.NodeRefs.Last())
		{
			continue;
		}

		TArray<int64> RingIds = Way.NodeRefs;
		RingIds.Pop(EAllowShrinking::No);
		FFlexOsmBuildingFootprint Footprint;
		if (!ResolveProjectedRing(RingIds, *OsmAsset, OriginLatitude, OriginLongitude, Footprint.OuterRingMeters))
		{
			continue;
		}
		Footprint.OsmId = WayPair.Key;
		Footprint.SourceType = TEXT("way");
		Footprint.Tags = Way.Tags;
		Footprint.bIsBuildingPart = IsBuildingPart(Way.Tags);
		OutFootprints.Add(MoveTemp(Footprint));
	}

	return true;
}
