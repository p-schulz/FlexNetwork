#include "Osm/FlexOsmGraphBuilder.h"
#include "Osm/OsmDataAsset.h"
#include "FlexNetworkSubsystem.h"
#include "RoadTypeProfile.h"
#include "Misc/ScopeExit.h"

namespace
{
	bool IsMatchedHighwayWay(const FOsmWay& Way, const FFlexOsmImportSettings& Settings)
	{
		const FString* HighwayTag = Way.Tags.Find(TEXT("highway"));
		return HighwayTag && Settings.HighwayTags.Contains(*HighwayTag);
	}

	bool IsMatchedRailwayWay(const FOsmWay& Way, const FFlexOsmImportSettings& Settings)
	{
		const FString* RailwayTag = Way.Tags.Find(TEXT("railway"));
		return RailwayTag && Settings.RailwayTags.Contains(*RailwayTag);
	}

	void CollectMatchingWayIds(const UOsmDataAsset& OsmAsset, const FFlexOsmImportSettings& Settings, TArray<int64>& OutWayIds)
	{
		OutWayIds.Reset();
		for (const TPair<int64, FOsmWay>& Pair : OsmAsset.Ways)
		{
			if (IsMatchedHighwayWay(Pair.Value, Settings) || IsMatchedRailwayWay(Pair.Value, Settings))
			{
				OutWayIds.Add(Pair.Key);
			}
		}

		// TMap iteration order is not a stable import contract. Sorting makes the fallback origin and
		// all downstream way processing deterministic for roads, buildings, and imagery alike.
		OutWayIds.Sort();
	}

	bool GetTrafficControlType(const FOsmNode& Node, EFlexTrafficControlType& OutType)
	{
		const FString Highway = Node.Tags.FindRef(TEXT("highway")).TrimStartAndEnd().ToLower();
		if (Highway == TEXT("traffic_signals"))
		{
			OutType = EFlexTrafficControlType::TrafficLight;
			return true;
		}
		if (Highway == TEXT("stop"))
		{
			OutType = EFlexTrafficControlType::StopSign;
			return true;
		}
		if (Highway == TEXT("give_way"))
		{
			OutType = EFlexTrafficControlType::YieldSign;
			return true;
		}
		return false;
	}

	bool TrafficControlAppliesToWayDirection(const FOsmNode& Node, const bool bForward)
	{
		FString Direction = Node.Tags.FindRef(TEXT("traffic_signals:direction"));
		if (Direction.IsEmpty())
		{
			Direction = Node.Tags.FindRef(TEXT("direction"));
		}
		Direction = Direction.TrimStartAndEnd().ToLower();
		if (Direction.IsEmpty() || Direction == TEXT("both") || Direction == TEXT("all"))
		{
			return true;
		}
		return bForward ? Direction == TEXT("forward") : Direction == TEXT("backward");
	}

	/** Minimal union-find (disjoint set), used for road proximity clusters and intersection contraction. */
	struct FUnionFind
	{
		TArray<int32> Parent;

		explicit FUnionFind(int32 Num)
		{
			Parent.SetNumUninitialized(Num);
			for (int32 i = 0; i < Num; ++i)
			{
				Parent[i] = i;
			}
		}

		int32 Find(int32 i)
		{
			while (Parent[i] != i)
			{
				Parent[i] = Parent[Parent[i]];
				i = Parent[i];
			}
			return i;
		}

		void Union(int32 A, int32 B)
		{
			A = Find(A);
			B = Find(B);
			if (A != B)
			{
				Parent[A] = B;
			}
		}
	};

	void ApplyTaggedSpeedLimit(const FOsmWay& Way, float& InOutSpeedLimitKmh)
	{
		if (const FString* MaxSpeedStr = Way.Tags.Find(TEXT("maxspeed")))
		{
			// Handles plain "50", "50 mph", and (best-effort, falls back to the default) tagging
			// schemes like "DE:zone30" -- pull the leading number, treat as mph if "mph" appears.
			const FString Trimmed = MaxSpeedStr->TrimStartAndEnd();
			FString NumberPart;
			for (TCHAR Ch : Trimmed)
			{
				if (FChar::IsDigit(Ch))
				{
					NumberPart.AppendChar(Ch);
				}
				else if (!NumberPart.IsEmpty())
				{
					break;
				}
			}
			if (!NumberPart.IsEmpty())
			{
				float Value = FCString::Atof(*NumberPart);
				if (Trimmed.Contains(TEXT("mph")))
				{
					Value *= 1.60934f;
				}
				if (Value > 0.f)
				{
					InOutSpeedLimitKmh = Value;
				}
			}
		}
	}

	FFlexOsmGraphBuilder::FLaneSignature DeriveLaneSignature(const FOsmWay& Way, const FFlexOsmImportSettings& Settings)
	{
		FFlexOsmGraphBuilder::FLaneSignature Signature;
		Signature.HighwayTag = Way.Tags.FindRef(TEXT("highway"));
		Signature.LaneWidth = Settings.DefaultLaneWidth;
		Signature.SpeedLimitKmh = Settings.DefaultSpeedLimitKmh;
		Signature.SidewalkWidth = Settings.SidewalkWidth;

		if (IsMatchedRailwayWay(Way, Settings))
		{
			Signature.bIsRailway = true;
			Signature.RailwayTag = Way.Tags.FindRef(TEXT("railway"));
			Signature.HighwayTag.Reset();
			Signature.SpeedLimitKmh = Signature.RailwayTag == TEXT("tram")
				? Settings.DefaultSpeedLimitKmh : Settings.DefaultRailSpeedLimitKmh;
			Signature.SidewalkWidth = 0.f;
			Signature.RailGauge = Settings.DefaultRailGauge;
			Signature.RailWidth = Settings.RailWidth;
			Signature.RailTopWidth = Settings.RailTopWidth;
			Signature.RailHeight = Settings.RailHeight;
			Signature.bUseGroovedRailProfile = Signature.RailwayTag == TEXT("tram");
			Signature.RailGrooveWidth = Settings.TramGrooveWidth;
			Signature.RailGrooveDepth = Settings.TramGrooveDepth;
			Signature.RailGrooveInwardOffset = Settings.TramGrooveInwardOffset;
			Signature.RailBooleanOverlap = Settings.RailBooleanOverlap;
			Signature.RailTrackSpacing = Settings.DefaultRailTrackSpacing;
			Signature.RailTrackCount = 1;
			if (const FString* TracksTag = Way.Tags.Find(TEXT("tracks")))
			{
				Signature.RailTrackCount = FMath::Max(1, FCString::Atoi(**TracksTag));
			}
			if (const FString* GaugeTag = Way.Tags.Find(TEXT("gauge")))
			{
				// OSM railway gauge is expressed in millimeters. A semicolon-separated multi-gauge
				// value naturally resolves to its first numeric gauge for this simple mesh pass.
				const float GaugeMillimeters = FCString::Atof(**GaugeTag);
				if (GaugeMillimeters > 0.f)
				{
					Signature.RailGauge = GaugeMillimeters * 0.1f;
				}
			}
			const FString OneWayTag = Way.Tags.FindRef(TEXT("oneway")).TrimStartAndEnd().ToLower();
			Signature.bRailReverse = OneWayTag == TEXT("-1") || OneWayTag == TEXT("reverse");
			Signature.bRailOneWay = Signature.bRailReverse || OneWayTag == TEXT("yes")
				|| OneWayTag == TEXT("1") || OneWayTag == TEXT("true");
			Signature.LaneWidth = Signature.RailGauge + Signature.RailWidth * 2.f;
			ApplyTaggedSpeedLimit(Way, Signature.SpeedLimitKmh);
			return Signature;
		}

		const FString OneWayTag = Way.Tags.FindRef(TEXT("oneway")).TrimStartAndEnd().ToLower();
		const bool bReverseOneWay = OneWayTag == TEXT("-1") || OneWayTag == TEXT("reverse");
		const bool bExplicitTwoWay = OneWayTag == TEXT("no") || OneWayTag == TEXT("0") || OneWayTag == TEXT("false");
		const bool bRoundabout = Way.Tags.FindRef(TEXT("junction")).Equals(TEXT("roundabout"), ESearchCase::IgnoreCase);
		const bool bOneWay = bReverseOneWay || OneWayTag == TEXT("yes") || OneWayTag == TEXT("1")
			|| OneWayTag == TEXT("true") || (bRoundabout && !bExplicitTwoWay);

		int32 TotalLanes = Settings.DefaultLaneCount;
		if (const FString* LanesStr = Way.Tags.Find(TEXT("lanes")))
		{
			const int32 Parsed = FCString::Atoi(**LanesStr);
			if (Parsed > 0)
			{
				TotalLanes = Parsed;
			}
		}

		const FString* FwdStr = Way.Tags.Find(TEXT("lanes:forward"));
		const FString* BwdStr = Way.Tags.Find(TEXT("lanes:backward"));

		if (bOneWay)
		{
			Signature.ForwardLanes = bReverseOneWay ? 0 : TotalLanes;
			Signature.BackwardLanes = bReverseOneWay ? TotalLanes : 0;
		}
		else if (FwdStr || BwdStr)
		{
			Signature.ForwardLanes = FwdStr ? FMath::Max(0, FCString::Atoi(**FwdStr)) : FMath::Max(1, TotalLanes - (BwdStr ? FCString::Atoi(**BwdStr) : 0));
			Signature.BackwardLanes = BwdStr ? FMath::Max(0, FCString::Atoi(**BwdStr)) : FMath::Max(1, TotalLanes - Signature.ForwardLanes);
		}
		else
		{
			Signature.ForwardLanes = FMath::Max(1, FMath::CeilToInt(TotalLanes * 0.5f));
			Signature.BackwardLanes = FMath::Max(1, TotalLanes - Signature.ForwardLanes);
		}

		float WidthMeters = 0.f;
		if (const FString* WidthStr = Way.Tags.Find(TEXT("width")))
		{
			WidthMeters = FCString::Atof(**WidthStr);
		}
		const int32 TotalLaneCountForWidth = FMath::Max(1, Signature.ForwardLanes + Signature.BackwardLanes);
		if (WidthMeters > 0.f)
		{
			Signature.LaneWidth = (WidthMeters * 100.f) / static_cast<float>(TotalLaneCountForWidth);
		}
		if (const FString* LaneWidthStr = Way.Tags.Find(TEXT("lane_width")))
		{
			const float LaneWidthMeters = FCString::Atof(**LaneWidthStr);
			if (LaneWidthMeters > 0.f)
			{
				Signature.LaneWidth = LaneWidthMeters * 100.f;
			}
		}
		else if (const FString* LaneWidthsStr = Way.Tags.Find(TEXT("width:lanes")))
		{
			TArray<FString> LaneWidths;
			LaneWidthsStr->ParseIntoArray(LaneWidths, TEXT("|"), true);
			if (LaneWidths.Num() <= 1)
			{
				LaneWidthsStr->ParseIntoArray(LaneWidths, TEXT(";"), true);
			}
			float WidthSumMeters = 0.f;
			int32 ValidWidths = 0;
			for (const FString& LaneWidthValue : LaneWidths)
			{
				const float LaneWidthMeters = FCString::Atof(*LaneWidthValue);
				if (LaneWidthMeters > 0.f)
				{
					WidthSumMeters += LaneWidthMeters;
					++ValidWidths;
				}
			}
			if (ValidWidths > 0)
			{
				Signature.LaneWidth = WidthSumMeters * 100.f / static_cast<float>(ValidWidths);
			}
		}

		ApplyTaggedSpeedLimit(Way, Signature.SpeedLimitKmh);

		return Signature;
	}

	/**
	 * Derives a way's target elevation type and height offset (cm, relative to ground datum) from
	 * its bridge/tunnel/layer tags. An explicit layer=<n> is authoritative when present (its sign
	 * already encodes above/below ground per OSM convention); bridge/tunnel without one falls back
	 * to Settings' default height/depth. A way with none of these tags stays Ground at offset 0.
	 */
	void ComputeWayElevation(const FOsmWay& Way, const FFlexOsmImportSettings& Settings, EFlexRoadElevationType& OutType, float& OutOffsetZ)
	{
		int32 Layer = 0;
		bool bHasLayer = false;
		if (const FString* LayerStr = Way.Tags.Find(TEXT("layer")))
		{
			Layer = FCString::Atoi(**LayerStr);
			bHasLayer = (Layer != 0);
		}

		const FString BridgeTag = Way.Tags.FindRef(TEXT("bridge"));
		const bool bIsBridge = !BridgeTag.IsEmpty() && BridgeTag != TEXT("no");
		const FString TunnelTag = Way.Tags.FindRef(TEXT("tunnel"));
		const bool bIsTunnel = !TunnelTag.IsEmpty() && TunnelTag != TEXT("no");

		if (bIsBridge)
		{
			OutType = EFlexRoadElevationType::Bridge;
			OutOffsetZ = bHasLayer ? Layer * Settings.LayerHeightStep : Settings.DefaultBridgeHeight;
		}
		else if (bIsTunnel)
		{
			OutType = EFlexRoadElevationType::Tunnel;
			OutOffsetZ = bHasLayer ? Layer * Settings.LayerHeightStep : -Settings.DefaultTunnelDepth;
		}
		else if (bHasLayer)
		{
			OutType = EFlexRoadElevationType::Elevated;
			OutOffsetZ = Layer * Settings.LayerHeightStep;
		}
		else
		{
			OutType = EFlexRoadElevationType::Ground;
			OutOffsetZ = 0.f;
		}
	}

	/**
	 * Derives the lateral shift (cm, +right of the way's own digitized direction) needed to move a
	 * way's nodes from where OSM says it was actually traced to FlexNetwork's own lane-profile
	 * origin, given a placement=<right_of|left_of|middle_of>:<lane> tag (falling back to
	 * placement:forward, then placement:backward, if the direction-agnostic tag isn't present).
	 * OSM numbers lanes 1..N left-to-right across the *whole* way regardless of direction;
	 * FlexNetwork's own LateralOffset 0 sits at the boundary between BackwardLanes (left) and
	 * ForwardLanes (right) -- see ConfigureProfileFromLaneSignature -- i.e. BackwardLanes
	 * lane-widths in from the way's left edge. A way traced left_of/at the left of that origin
	 * needs shifting right to land back on it, and vice versa, which is where the sign below falls
	 * out of naturally rather than being asserted. With no placement tag the digitized way is
	 * assumed to be the roadway center, so an
	 * asymmetric lane pack (notably one-way roads) receives the offset needed to center it.
	 */
	float ComputeWayPlacementOffset(const FOsmWay& Way, const FFlexOsmGraphBuilder::FLaneSignature& Signature)
	{
		const float CenteredLanePackOffset =
			(static_cast<float>(Signature.BackwardLanes - Signature.ForwardLanes) * Signature.LaneWidth) * 0.5f;
		FString PlacementTag = Way.Tags.FindRef(TEXT("placement"));
		if (PlacementTag.IsEmpty())
		{
			PlacementTag = Way.Tags.FindRef(TEXT("placement:forward"));
		}
		if (PlacementTag.IsEmpty())
		{
			PlacementTag = Way.Tags.FindRef(TEXT("placement:backward"));
		}
		if (PlacementTag.IsEmpty())
		{
			return CenteredLanePackOffset;
		}
		PlacementTag = PlacementTag.TrimStartAndEnd().ToLower();

		FString EdgeKind, LaneStr;
		if (!PlacementTag.Split(TEXT(":"), &EdgeKind, &LaneStr))
		{
			return CenteredLanePackOffset;
		}
		const int32 Lane = FCString::Atoi(*LaneStr);
		if (Lane <= 0)
		{
			return CenteredLanePackOffset;
		}

		// Distance from the roadway's left edge (lane 1's own left edge) to the tagged point.
		float DistFromLeftEdge;
		if (EdgeKind == TEXT("right_of"))
		{
			DistFromLeftEdge = static_cast<float>(Lane) * Signature.LaneWidth;
		}
		else if (EdgeKind == TEXT("left_of"))
		{
			DistFromLeftEdge = static_cast<float>(Lane - 1) * Signature.LaneWidth;
		}
		else if (EdgeKind == TEXT("middle_of"))
		{
			DistFromLeftEdge = (static_cast<float>(Lane) - 0.5f) * Signature.LaneWidth;
		}
		else
		{
			return CenteredLanePackOffset; // Unrecognized placement kind (e.g. "transition").
		}

		const float FlexOriginFromLeftEdge = static_cast<float>(Signature.BackwardLanes) * Signature.LaneWidth;
		const float WayOffsetFromFlexOrigin = DistFromLeftEdge - FlexOriginFromLeftEdge;
		return -WayOffsetFromFlexOrigin;
	}

	struct FCollapsedIntersectionApproach
	{
		FFlexSegmentId SegmentId;
		FFlexNodeId NodeId;
		URoadTypeProfile* Profile = nullptr;
		EFlexRoadElevationType ElevationType = EFlexRoadElevationType::Ground;
		FFlexElevationProfile ElevationProfile;
		FVector HeadingTowardCenter = FVector::ForwardVector;
		FVector NeighborTangentHandle = FVector::ZeroVector;
		float CenterTangentHandleLength = 1.f;
		bool bWasSegmentStart = false;
		bool bProfileIsOneWay = false;
	};

	bool IsOneWayProfile(const URoadTypeProfile* Profile)
	{
		bool bHasForward = false;
		bool bHasBackward = false;
		bool bHasBidirectional = false;
		if (Profile)
		{
			for (const FRoadLaneDescriptor& Lane : Profile->Lanes)
			{
				if (!Lane.IsDrivable())
				{
					continue;
				}
				bHasForward |= Lane.Direction == EFlexLaneDirection::Forward;
				bHasBackward |= Lane.Direction == EFlexLaneDirection::Backward;
				bHasBidirectional |= Lane.Direction == EFlexLaneDirection::Bidirectional;
			}
		}
		return !bHasBidirectional && bHasForward != bHasBackward;
	}

	/**
	 * Pairs a consolidated intersection's approaches using a
	 * minimum-cost perfect matching. Pair cost is dominated by continuation angle, with small
	 * penalties for cross-section differences. Endpoint direction is part of the constraint for
	 * one-way profiles. Paired approaches receive a common through-axis at the center, but remain
	 * attached to that node so the junction builder can produce every legal turning connector.
	 */
	bool AlignCollapsedIntersectionThroughRoads(UFlexNetworkSubsystem& Subsystem, FFlexNodeId CenterNodeId,
		float MinimumContinuationAngleDegrees)
	{
		const FFlexRoadNode* CenterNode = Subsystem.GetNode(CenterNodeId);
		if (!CenterNode || CenterNode->ConnectedSegments.Num() < 4
			|| (CenterNode->ConnectedSegments.Num() & 1) != 0
			|| CenterNode->ConnectedSegments.Num() > 16)
		{
			return false;
		}

		TArray<FCollapsedIntersectionApproach> Approaches;
		Approaches.Reserve(CenterNode->ConnectedSegments.Num());
		TSet<FFlexNodeId> UniqueNeighborNodes;
		for (const FFlexSegmentId SegmentId : CenterNode->ConnectedSegments)
		{
			const FFlexRoadSegment* Segment = Subsystem.GetSegment(SegmentId);
			if (!Segment || !Segment->Profile)
			{
				return false;
			}

			FCollapsedIntersectionApproach& Approach = Approaches.AddDefaulted_GetRef();
			Approach.SegmentId = SegmentId;
			Approach.Profile = Segment->Profile;
			Approach.ElevationType = Segment->ElevationType;
			Approach.ElevationProfile = Segment->ElevationProfile;
			Approach.bProfileIsOneWay = IsOneWayProfile(Segment->Profile);
			if (Segment->EndNodeId == CenterNodeId)
			{
				Approach.NodeId = Segment->StartNodeId;
				Approach.HeadingTowardCenter = (Segment->Curve.P1 - Segment->Curve.P0).GetSafeNormal();
				Approach.NeighborTangentHandle = Segment->Curve.P1;
				Approach.CenterTangentHandleLength = FVector::Distance(Segment->Curve.P3, Segment->Curve.P2);
				Approach.bWasSegmentStart = true;
			}
			else if (Segment->StartNodeId == CenterNodeId)
			{
				Approach.NodeId = Segment->EndNodeId;
				Approach.HeadingTowardCenter = (Segment->Curve.P2 - Segment->Curve.P3).GetSafeNormal();
				Approach.NeighborTangentHandle = Segment->Curve.P2;
				Approach.CenterTangentHandleLength = FVector::Distance(Segment->Curve.P0, Segment->Curve.P1);
				Approach.bWasSegmentStart = false;
			}
			else
			{
				return false;
			}

			if (!Approach.NodeId.IsValid() || Approach.HeadingTowardCenter.IsNearlyZero()
				|| UniqueNeighborNodes.Contains(Approach.NodeId))
			{
				return false;
			}
			UniqueNeighborNodes.Add(Approach.NodeId);
		}

		// Stable angular order makes equal-cost matchings deterministic across TSet iteration orders.
		Approaches.Sort([](const FCollapsedIntersectionApproach& A, const FCollapsedIntersectionApproach& B)
		{
			const float AngleA = FMath::Atan2(A.HeadingTowardCenter.Y, A.HeadingTowardCenter.X);
			const float AngleB = FMath::Atan2(B.HeadingTowardCenter.Y, B.HeadingTowardCenter.X);
			return !FMath::IsNearlyEqual(AngleA, AngleB) ? AngleA < AngleB : A.NodeId.Index < B.NodeId.Index;
		});

		constexpr float InvalidPairCost = 1.0e30f;
		const int32 NumApproaches = Approaches.Num();
		TArray<float> PairCosts;
		PairCosts.Init(InvalidPairCost, NumApproaches * NumApproaches);
		for (int32 AIndex = 0; AIndex < NumApproaches; ++AIndex)
		{
			for (int32 BIndex = AIndex + 1; BIndex < NumApproaches; ++BIndex)
			{
				const FCollapsedIntersectionApproach& A = Approaches[AIndex];
				const FCollapsedIntersectionApproach& B = Approaches[BIndex];
				const float HeadingDot = FVector::DotProduct(A.HeadingTowardCenter, B.HeadingTowardCenter);
				const float ContinuationAngle = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(HeadingDot, -1.f, 1.f)));
				if (ContinuationAngle + KINDA_SMALL_NUMBER < MinimumContinuationAngleDegrees
					|| A.ElevationType != B.ElevationType
					|| ((A.bProfileIsOneWay || B.bProfileIsOneWay) && A.bWasSegmentStart == B.bWasSegmentStart))
				{
					continue;
				}

				const float WidthA = A.Profile->GetRoadwayWidth();
				const float WidthB = B.Profile->GetRoadwayWidth();
				const float WidthPenalty = FMath::Abs(WidthA - WidthB) / FMath::Max(FMath::Max(WidthA, WidthB), 1.f);
				const float LanePenalty = 0.02f * FMath::Abs(A.Profile->Lanes.Num() - B.Profile->Lanes.Num());
				PairCosts[AIndex * NumApproaches + BIndex] = 1.f + HeadingDot + 0.15f * WidthPenalty + LanePenalty;
			}
		}

		TMap<uint32, float> CostByMask;
		TMap<uint32, int32> ChoiceByMask;
		TFunction<float(uint32)> SolveMatching;
		SolveMatching = [&](uint32 Mask) -> float
		{
			if (Mask == 0)
			{
				return 0.f;
			}
			if (const float* Cached = CostByMask.Find(Mask))
			{
				return *Cached;
			}

			int32 FirstIndex = 0;
			while ((Mask & (1u << FirstIndex)) == 0)
			{
				++FirstIndex;
			}
			float BestCost = InvalidPairCost;
			int32 BestPartner = INDEX_NONE;
			for (int32 PartnerIndex = FirstIndex + 1; PartnerIndex < NumApproaches; ++PartnerIndex)
			{
				if ((Mask & (1u << PartnerIndex)) == 0)
				{
					continue;
				}
				const float PairCost = PairCosts[FirstIndex * NumApproaches + PartnerIndex];
				if (PairCost >= InvalidPairCost)
				{
					continue;
				}
				const uint32 RemainingMask = Mask & ~(1u << FirstIndex) & ~(1u << PartnerIndex);
				const float RemainingCost = SolveMatching(RemainingMask);
				if (RemainingCost < InvalidPairCost && PairCost + RemainingCost < BestCost)
				{
					BestCost = PairCost + RemainingCost;
					BestPartner = PartnerIndex;
				}
			}
			CostByMask.Add(Mask, BestCost);
			ChoiceByMask.Add(Mask, BestPartner);
			return BestCost;
		};

		const uint32 FullMask = (1u << NumApproaches) - 1u;
		if (SolveMatching(FullMask) >= InvalidPairCost)
		{
			return false;
		}

		TArray<FIntPoint> MatchedPairs;
		uint32 RemainingMask = FullMask;
		while (RemainingMask != 0)
		{
			int32 FirstIndex = 0;
			while ((RemainingMask & (1u << FirstIndex)) == 0)
			{
				++FirstIndex;
			}
			const int32 PartnerIndex = ChoiceByMask.FindRef(RemainingMask);
			if (PartnerIndex == INDEX_NONE)
			{
				return false;
			}
			MatchedPairs.Emplace(FirstIndex, PartnerIndex);
			RemainingMask &= ~(1u << FirstIndex);
			RemainingMask &= ~(1u << PartnerIndex);
		}

		const FVector CenterPosition = CenterNode->Position;
		bool bAllAligned = true;
		for (const FIntPoint& Pair : MatchedPairs)
		{
			const FCollapsedIntersectionApproach& A = Approaches[Pair.X];
			const FCollapsedIntersectionApproach& B = Approaches[Pair.Y];
			FVector AxisTowardCenter = (A.HeadingTowardCenter - B.HeadingTowardCenter).GetSafeNormal();
			if (AxisTowardCenter.IsNearlyZero())
			{
				bAllAligned = false;
				continue;
			}
			if (FVector::DotProduct(AxisTowardCenter, A.HeadingTowardCenter) < 0.f)
			{
				AxisTowardCenter *= -1.f;
			}

			auto AlignApproach = [&](const FCollapsedIntersectionApproach& Approach, const FVector& InwardAxis)
			{
				const FFlexRoadNode* NeighborNode = Subsystem.GetNode(Approach.NodeId);
				if (!NeighborNode)
				{
					return false;
				}
				const float ChordLength = FVector::Distance(NeighborNode->Position, CenterPosition);
				const float CenterHandleLength = FMath::Clamp(Approach.CenterTangentHandleLength,
					1.f, FMath::Max(1.f, ChordLength * 0.45f));
				const FVector CenterHandle = CenterPosition - InwardAxis * CenterHandleLength;
				return Approach.bWasSegmentStart
					? Subsystem.SetSegmentCurve(Approach.SegmentId, Approach.NeighborTangentHandle, CenterHandle)
					: Subsystem.SetSegmentCurve(Approach.SegmentId, CenterHandle, Approach.NeighborTangentHandle);
			};

			bAllAligned &= AlignApproach(A, AxisTowardCenter);
			bAllAligned &= AlignApproach(B, -AxisTowardCenter);
		}
		return bAllAligned;
	}
}

FString FFlexOsmGraphBuilder::FLaneSignature::ToKey() const
{
	if (bIsRailway)
	{
		return FString::Printf(TEXT("Rail_%s_T%d_G%d_RW%d_RT%d_RH%d_GW%d_GD%d_GO%d_BO%d_TS%d_S%d_%s"), *RailwayTag, RailTrackCount,
			FMath::RoundToInt(RailGauge), FMath::RoundToInt(RailWidth), FMath::RoundToInt(RailTopWidth), FMath::RoundToInt(RailHeight),
			FMath::RoundToInt(RailGrooveWidth), FMath::RoundToInt(RailGrooveDepth), FMath::RoundToInt(RailGrooveInwardOffset),
			FMath::RoundToInt(RailBooleanOverlap * 10.f),
			FMath::RoundToInt(RailTrackSpacing), FMath::RoundToInt(SpeedLimitKmh),
			bRailOneWay ? (bRailReverse ? TEXT("Reverse") : TEXT("Forward")) : TEXT("Both"));
	}
	const int32 RoundedOffset = FMath::RoundToInt(LateralOffset);
	return FString::Printf(TEXT("%s_F%d_B%d_W%d_S%d_O%c%d"), *HighwayTag, ForwardLanes, BackwardLanes,
		FMath::RoundToInt(LaneWidth), FMath::RoundToInt(SpeedLimitKmh), RoundedOffset < 0 ? TEXT('N') : TEXT('P'), FMath::Abs(RoundedOffset));
}

void FFlexOsmGraphBuilder::ConfigureProfileFromLaneSignature(URoadTypeProfile& Profile, const FLaneSignature& Signature)
{
	Profile.Lanes.Reset();
	Profile.LateralOffset = Signature.LateralOffset;

	const float SpeedLimitCmPerSec = Signature.SpeedLimitKmh * 100000.f / 3600.f;
	if (Signature.bIsRailway)
	{
		Profile.bIsRailProfile = true;
		Profile.RailGauge = Signature.RailGauge;
		Profile.RailWidth = Signature.RailWidth;
		Profile.RailTopWidth = Signature.RailTopWidth;
		Profile.RailHeight = Signature.RailHeight;
		Profile.bUseGroovedRailProfile = Signature.bUseGroovedRailProfile;
		Profile.RailGrooveWidth = Signature.RailGrooveWidth;
		Profile.RailGrooveDepth = Signature.RailGrooveDepth;
		Profile.RailGrooveInwardOffset = Signature.RailGrooveInwardOffset;
		Profile.RailBooleanOverlap = Signature.RailBooleanOverlap;
		Profile.SidewalkWidth = 0.f;
		Profile.CurbHeight = 0.f;
		for (int32 TrackIndex = 0; TrackIndex < Signature.RailTrackCount; ++TrackIndex)
		{
			FRoadLaneDescriptor Track;
			Track.LaneName = *FString::Printf(TEXT("Rail_%d"), TrackIndex);
			Track.Width = Signature.RailGauge + Signature.RailWidth * 2.f;
			Track.LateralOffset = (static_cast<float>(TrackIndex) - static_cast<float>(Signature.RailTrackCount - 1) * 0.5f)
				* Signature.RailTrackSpacing;
			Track.Type = EFlexLaneType::Rail;
			Track.Direction = Signature.bRailOneWay
				? (Signature.bRailReverse ? EFlexLaneDirection::Backward : EFlexLaneDirection::Forward)
				: EFlexLaneDirection::Bidirectional;
			Track.SpeedLimit = SpeedLimitCmPerSec;
			Profile.Lanes.Add(Track);
		}
		Profile.MaxGrade = 0.04f;
		Profile.MinTurnRadius = Signature.RailwayTag == TEXT("tram") ? 1800.f : 10000.f;
		return;
	}

	Profile.bIsRailProfile = false;
	Profile.bUseGroovedRailProfile = false;

	for (int32 i = 0; i < Signature.ForwardLanes; ++i)
	{
		FRoadLaneDescriptor Lane;
		Lane.LaneName = *FString::Printf(TEXT("Forward_%d"), i);
		Lane.Width = Signature.LaneWidth;
		Lane.LateralOffset = Signature.LaneWidth * (static_cast<float>(i) + 0.5f);
		Lane.Type = EFlexLaneType::Vehicle;
		Lane.Direction = EFlexLaneDirection::Forward;
		Lane.SpeedLimit = SpeedLimitCmPerSec;
		Profile.Lanes.Add(Lane);
	}
	for (int32 i = 0; i < Signature.BackwardLanes; ++i)
	{
		FRoadLaneDescriptor Lane;
		Lane.LaneName = *FString::Printf(TEXT("Backward_%d"), i);
		Lane.Width = Signature.LaneWidth;
		Lane.LateralOffset = -Signature.LaneWidth * (static_cast<float>(i) + 0.5f);
		Lane.Type = EFlexLaneType::Vehicle;
		Lane.Direction = EFlexLaneDirection::Backward;
		Lane.SpeedLimit = SpeedLimitCmPerSec;
		Profile.Lanes.Add(Lane);
	}

	Profile.SidewalkWidth = Signature.SidewalkWidth;
	Profile.CurbHeight = 15.f;
	Profile.MaxGrade = 0.08f;
	Profile.MinTurnRadius = 800.f;
}

FVector2D FFlexOsmGraphBuilder::ProjectLatLonToLocalCm(double Lat, double Lon, double OriginLatDeg, double OriginLonDeg)
{
	constexpr double kEarthRadiusCm = 6378137.0 * 100.0;
	const double OriginLatRad = FMath::DegreesToRadians(OriginLatDeg);
	const double NorthCm = FMath::DegreesToRadians(Lat - OriginLatDeg) * kEarthRadiusCm;
	const double EastCm = FMath::DegreesToRadians(Lon - OriginLonDeg) * FMath::Cos(OriginLatRad) * kEarthRadiusCm;
	// Unreal's horizontal map convention is X=North/forward, Y=East/right. Feeding ordinary GIS
	// X=East/Y=North through unchanged reflects the map and reverses geographic left/right.
	return FVector2D(NorthCm, EastCm);
}

void FFlexOsmGraphBuilder::UnprojectLocalCmToLatLon(double LocalX, double LocalY, double OriginLatDeg, double OriginLonDeg, double& OutLat, double& OutLon)
{
	constexpr double kEarthRadiusCm = 6378137.0 * 100.0;
	const double OriginLatRad = FMath::DegreesToRadians(OriginLatDeg);
	OutLat = OriginLatDeg + FMath::RadiansToDegrees(LocalX / kEarthRadiusCm);
	OutLon = OriginLonDeg + FMath::RadiansToDegrees(LocalY / (kEarthRadiusCm * FMath::Cos(OriginLatRad)));
}

bool FFlexOsmGraphBuilder::ResolveOrigin(const UOsmDataAsset& OsmAsset, const FFlexOsmImportSettings& Settings, double& OutOriginLat, double& OutOriginLon)
{
	if (Settings.bUseOriginOverride)
	{
		OutOriginLat = Settings.OriginLatLon.X;
		OutOriginLon = Settings.OriginLatLon.Y;
		return true;
	}
	if (OsmAsset.Bounds.bIsValid)
	{
		const FVector2D Center = OsmAsset.Bounds.GetCenter();
		OutOriginLat = Center.X;
		OutOriginLon = Center.Y;
		return true;
	}
	TArray<int64> MatchingWayIds;
	CollectMatchingWayIds(OsmAsset, Settings, MatchingWayIds);
	for (const int64 WayId : MatchingWayIds)
	{
		const FOsmWay& Way = OsmAsset.Ways.FindChecked(WayId);
		for (const int64 NodeRef : Way.NodeRefs)
		{
			if (const FOsmNode* Node = OsmAsset.Nodes.Find(NodeRef))
			{
				OutOriginLat = Node->Latitude;
				OutOriginLon = Node->Longitude;
				return true;
			}
		}
	}

	// Building-only OSM extracts may contain no matching highway at all. They still need a useful,
	// deterministic local origin when no override or bounds element was supplied.
	TArray<int64> NodeIds;
	OsmAsset.Nodes.GetKeys(NodeIds);
	NodeIds.Sort();
	if (!NodeIds.IsEmpty())
	{
		const FOsmNode& Node = OsmAsset.Nodes.FindChecked(NodeIds[0]);
		OutOriginLat = Node.Latitude;
		OutOriginLon = Node.Longitude;
		return true;
	}
	return false;
}

bool FFlexOsmGraphBuilder::ComputeMatchingRoadExtent(const UOsmDataAsset& OsmAsset, const FFlexOsmImportSettings& Settings, double& OutOriginLat, double& OutOriginLon, FVector2D& OutMinLocalCm, FVector2D& OutMaxLocalCm)
{
	TArray<int64> MatchingWayIds;
	CollectMatchingWayIds(OsmAsset, Settings, MatchingWayIds);
	if (MatchingWayIds.Num() == 0)
	{
		return false;
	}

	double OriginLat = 0.0, OriginLon = 0.0;
	if (!ResolveOrigin(OsmAsset, Settings, OriginLat, OriginLon))
	{
		return false;
	}

	FVector2D MinLocal(TNumericLimits<double>::Max(), TNumericLimits<double>::Max());
	FVector2D MaxLocal(TNumericLimits<double>::Lowest(), TNumericLimits<double>::Lowest());
	bool bAnyNode = false;
	for (int64 WayId : MatchingWayIds)
	{
		const FOsmWay& Way = OsmAsset.Ways.FindChecked(WayId);
		for (int64 NodeRef : Way.NodeRefs)
		{
			const FOsmNode* OsmNode = OsmAsset.Nodes.Find(NodeRef);
			if (!OsmNode)
			{
				continue;
			}
			const FVector2D Local = ProjectLatLonToLocalCm(OsmNode->Latitude, OsmNode->Longitude, OriginLat, OriginLon);
			MinLocal.X = FMath::Min(MinLocal.X, Local.X);
			MinLocal.Y = FMath::Min(MinLocal.Y, Local.Y);
			MaxLocal.X = FMath::Max(MaxLocal.X, Local.X);
			MaxLocal.Y = FMath::Max(MaxLocal.Y, Local.Y);
			bAnyNode = true;
		}
	}
	if (!bAnyNode)
	{
		return false;
	}

	OutOriginLat = OriginLat;
	OutOriginLon = OriginLon;
	OutMinLocalCm = MinLocal;
	OutMaxLocalCm = MaxLocal;
	return true;
}

FFlexOsmGraphBuilder::FImportResult FFlexOsmGraphBuilder::BuildFromOsm(UFlexNetworkSubsystem& Subsystem, const UOsmDataAsset& OsmAsset, const FFlexOsmImportSettings& Settings, TFunctionRef<URoadTypeProfile*(const FLaneSignature&)> ResolveProfile)
{
	FImportResult Result;

	// 1. Filter ways by configured highway and railway tags.
	TArray<int64> MatchingWayIds;
	CollectMatchingWayIds(OsmAsset, Settings, MatchingWayIds);

	if (MatchingWayIds.Num() == 0)
	{
		Result.Warnings.Add(TEXT("No ways matched the configured HighwayTags or RailwayTags filters."));
		return Result;
	}

	// 2. Resolve the one shared projection origin used by roads, buildings, and imagery.
	double OriginLat = 0.0, OriginLon = 0.0;
	const bool bOriginFound = ResolveOrigin(OsmAsset, Settings, OriginLat, OriginLon);

	// 3. Collect every node referenced by a matching way.
	TArray<int64> RelevantNodeIds;
	TMap<int64, int32> NodeIndexByOsmId;
	TArray<FVector2D> RawPositions;
	TSet<int32> RailwayNodeIndices;
	TMap<int32, TSet<int64>> MatchingWaysByNodeIndex;

	for (int64 WayId : MatchingWayIds)
	{
		const FOsmWay& Way = OsmAsset.Ways.FindChecked(WayId);
		const bool bRailwayWay = IsMatchedRailwayWay(Way, Settings);
		for (int64 NodeRef : Way.NodeRefs)
		{
			int32 NodeIndex = INDEX_NONE;
			if (const int32* ExistingIndex = NodeIndexByOsmId.Find(NodeRef))
			{
				NodeIndex = *ExistingIndex;
			}
			else
			{
				const FOsmNode* OsmNode = OsmAsset.Nodes.Find(NodeRef);
				if (!OsmNode)
				{
					continue; // Referenced node wasn't in the asset (e.g. a partial/clipped extract).
				}
				NodeIndex = RelevantNodeIds.Add(NodeRef);
				NodeIndexByOsmId.Add(NodeRef, NodeIndex);
				RawPositions.Add(FVector2D::ZeroVector);
			}
			if (bRailwayWay)
			{
				RailwayNodeIndices.Add(NodeIndex);
			}
			MatchingWaysByNodeIndex.FindOrAdd(NodeIndex).Add(WayId);
		}
	}

	if (!bOriginFound)
	{
		Result.Warnings.Add(TEXT("Could not resolve a projection origin (no referenced way had a node with coordinates)."));
		return Result;
	}

	for (int32 i = 0; i < RelevantNodeIds.Num(); ++i)
	{
		const FOsmNode& OsmNode = OsmAsset.Nodes.FindChecked(RelevantNodeIds[i]);
		RawPositions[i] = ProjectLatLonToLocalCm(OsmNode.Latitude, OsmNode.Longitude, OriginLat, OriginLon);
	}

	// 4. Cluster road-only nodes within JunctionMergeRadius via a lightweight spatial hash + union-find.
	// (A one-shot bulk-clustering pass like this doesn't reuse FFlexSpatialGrid -- that class is
	// shaped around the live, incrementally-updated graph's node/segment IDs, not a disposable
	// index into a temporary array, so a small local grid is simpler here than forcing the fit.)
	FUnionFind UnionFind(RelevantNodeIds.Num());
	if (Settings.JunctionMergeRadius > KINDA_SMALL_NUMBER)
	{
		const float CellSize = FMath::Max(Settings.JunctionMergeRadius, 1.f);
		TMap<FIntPoint, TArray<int32>> Cells;
		for (int32 i = 0; i < RawPositions.Num(); ++i)
		{
			const FIntPoint Cell(FMath::FloorToInt32(RawPositions[i].X / CellSize), FMath::FloorToInt32(RawPositions[i].Y / CellSize));
			Cells.FindOrAdd(Cell).Add(i);
		}

		const float RadiusSq = FMath::Square(Settings.JunctionMergeRadius);
		auto SharesMatchingWay = [&MatchingWaysByNodeIndex](int32 A, int32 B)
		{
			const TSet<int64>* WaysA = MatchingWaysByNodeIndex.Find(A);
			const TSet<int64>* WaysB = MatchingWaysByNodeIndex.Find(B);
			if (!WaysA || !WaysB)
			{
				return false;
			}
			const TSet<int64>* Smaller = WaysA->Num() <= WaysB->Num() ? WaysA : WaysB;
			const TSet<int64>* Larger = Smaller == WaysA ? WaysB : WaysA;
			for (const int64 WayId : *Smaller)
			{
				if (Larger->Contains(WayId))
				{
					return true;
				}
			}
			return false;
		};
		for (int32 i = 0; i < RawPositions.Num(); ++i)
		{
			const FIntPoint Cell(FMath::FloorToInt32(RawPositions[i].X / CellSize), FMath::FloorToInt32(RawPositions[i].Y / CellSize));
			for (int32 DY = -1; DY <= 1; ++DY)
			{
				for (int32 DX = -1; DX <= 1; ++DX)
				{
					const TArray<int32>* Bucket = Cells.Find(Cell + FIntPoint(DX, DY));
					if (!Bucket)
					{
						continue;
					}
					for (int32 j : *Bucket)
					{
						// Railway geometry often has parallel tracks or an embedded tram line within a
						// road well inside the road junction merge radius. Only identical OSM node refs
						// are shared across those networks; proximity merging remains road-only.
						if (j > i && !RailwayNodeIndices.Contains(i) && !RailwayNodeIndices.Contains(j)
							&& !SharesMatchingWay(i, j)
							&& FVector2D::DistSquared(RawPositions[i], RawPositions[j]) <= RadiusSq)
						{
							UnionFind.Union(i, j);
						}
					}
				}
			}
		}
	}

	// 5. Each cluster's representative position is the centroid of its members.
	TMap<int32, FVector2D> ClusterPositionSum;
	TMap<int32, int32> ClusterMemberCount;
	for (int32 i = 0; i < RawPositions.Num(); ++i)
	{
		const int32 Root = UnionFind.Find(i);
		ClusterPositionSum.FindOrAdd(Root) += RawPositions[i];
		ClusterMemberCount.FindOrAdd(Root) += 1;
	}
	for (const TPair<int32, int32>& Pair : ClusterMemberCount)
	{
		if (Pair.Value >= 2)
		{
			++Result.NumJunctionsMerged;
		}
	}

	// 6. Detect compact, at-grade networks of junction nodes which describe one physical
	// intersection. Do not contract their distinct portals: divided carriageways rely on the
	// original positions/headings. The retained nodes are registered as one shared surface region
	// after graph creation; short internal links remain routing-only inside that region.
	FUnionFind FinalUnionFind(RelevantNodeIds.Num());
	for (int32 i = 0; i < RelevantNodeIds.Num(); ++i)
	{
		FinalUnionFind.Union(i, UnionFind.Find(i));
	}
	TArray<TArray<int32>> ComplexIntersectionRootComponents;
	TSet<int32> ComplexIntersectionInteriorShapeRoots;

	if (Settings.bCollapseComplexIntersectionInteriors
		&& Settings.ComplexIntersectionInternalEdgeLength > KINDA_SMALL_NUMBER
		&& Settings.ComplexIntersectionMaxDiameter > KINDA_SMALL_NUMBER)
	{
		struct FClusterGraphEdge
		{
			int32 A = INDEX_NONE;
			int32 B = INDEX_NONE;
			float Length = 0.f;
		};
		struct FJunctionGraphConnection
		{
			int32 A = INDEX_NONE;
			int32 B = INDEX_NONE;
			float Length = 0.f;
			TArray<int32> PathRoots;
		};
		auto MakeEdgeKey = [](int32 A, int32 B) -> uint64
		{
			const uint32 MinRoot = static_cast<uint32>(FMath::Min(A, B));
			const uint32 MaxRoot = static_cast<uint32>(FMath::Max(A, B));
			return (static_cast<uint64>(MinRoot) << 32) | MaxRoot;
		};
		auto ClusterPosition = [&ClusterPositionSum, &ClusterMemberCount](int32 Root) -> FVector2D
		{
			return ClusterPositionSum.FindChecked(Root) / static_cast<float>(ClusterMemberCount.FindChecked(Root));
		};

		TMap<uint64, FClusterGraphEdge> AtGradeEdges;
		TMap<int32, TSet<int32>> AtGradeNeighbors;
		TSet<int32> RootsWithNonGroundWays;
		TSet<int32> RootsWithRailwayWays;
		for (int32 RailwayNodeIndex : RailwayNodeIndices)
		{
			RootsWithRailwayWays.Add(UnionFind.Find(RailwayNodeIndex));
		}
		for (int64 WayId : MatchingWayIds)
		{
			const FOsmWay& Way = OsmAsset.Ways.FindChecked(WayId);
			if (IsMatchedRailwayWay(Way, Settings))
			{
				continue; // Switches and parallel tracks are not road intersection interiors.
			}
			EFlexRoadElevationType ElevationType;
			float UnusedOffsetZ;
			ComputeWayElevation(Way, Settings, ElevationType, UnusedOffsetZ);
			if (ElevationType != EFlexRoadElevationType::Ground)
			{
				for (int64 NodeRef : Way.NodeRefs)
				{
					if (const int32* NodeIndex = NodeIndexByOsmId.Find(NodeRef))
					{
						RootsWithNonGroundWays.Add(UnionFind.Find(*NodeIndex));
					}
				}
				continue;
			}

			int32 PreviousRoot = INDEX_NONE;
			for (int64 NodeRef : Way.NodeRefs)
			{
				const int32* NodeIndex = NodeIndexByOsmId.Find(NodeRef);
				if (!NodeIndex)
				{
					continue;
				}
				const int32 Root = UnionFind.Find(*NodeIndex);
				if (PreviousRoot != INDEX_NONE && Root != PreviousRoot)
				{
					AtGradeNeighbors.FindOrAdd(PreviousRoot).Add(Root);
					AtGradeNeighbors.FindOrAdd(Root).Add(PreviousRoot);
					const float Length = FVector2D::Distance(ClusterPosition(PreviousRoot), ClusterPosition(Root));
					const uint64 Key = MakeEdgeKey(PreviousRoot, Root);
					if (FClusterGraphEdge* ExistingEdge = AtGradeEdges.Find(Key))
					{
						ExistingEdge->Length = FMath::Min(ExistingEdge->Length, Length);
					}
					else
					{
						AtGradeEdges.Add(Key, FClusterGraphEdge{ PreviousRoot, Root, Length });
					}
				}
				PreviousRoot = Root;
			}
		}

		TSet<int32> JunctionRoots;
		for (const TPair<int32, TSet<int32>>& Pair : AtGradeNeighbors)
		{
			if (Pair.Value.Num() >= 3 && !RootsWithNonGroundWays.Contains(Pair.Key)
				&& !RootsWithRailwayWays.Contains(Pair.Key))
			{
				JunctionRoots.Add(Pair.Key);
			}
		}

		// Compress degree-2 OSM shape points between junctions into one logical connection for
		// region detection, while retaining every root in PathRoots for later portal registration.
		// This lets the proximity pass preserve authored way geometry without preventing a short
		// multi-point internal link from identifying one compact physical intersection.
		TMap<uint64, FJunctionGraphConnection> JunctionConnections;
		for (const int32 StartRoot : JunctionRoots)
		{
			const TSet<int32>* StartNeighbors = AtGradeNeighbors.Find(StartRoot);
			if (!StartNeighbors)
			{
				continue;
			}
			for (const int32 FirstNeighbor : *StartNeighbors)
			{
				int32 Previous = StartRoot;
				int32 Current = FirstNeighbor;
				float PathLength = 0.f;
				TArray<int32> PathRoots{ StartRoot };
				TSet<int32> Visited;
				Visited.Add(StartRoot);
				bool bValidPath = true;
				while (true)
				{
					const FClusterGraphEdge* Edge = AtGradeEdges.Find(MakeEdgeKey(Previous, Current));
					if (!Edge || Visited.Contains(Current))
					{
						bValidPath = false;
						break;
					}
					PathLength += Edge->Length;
					PathRoots.Add(Current);
					Visited.Add(Current);
					if (JunctionRoots.Contains(Current))
					{
						break;
					}
					const TSet<int32>* Neighbors = AtGradeNeighbors.Find(Current);
					if (!Neighbors || Neighbors->Num() != 2 || RootsWithNonGroundWays.Contains(Current)
						|| RootsWithRailwayWays.Contains(Current))
					{
						bValidPath = false;
						break;
					}
					int32 Next = INDEX_NONE;
					for (const int32 Candidate : *Neighbors)
					{
						if (Candidate != Previous)
						{
							Next = Candidate;
							break;
						}
					}
					if (Next == INDEX_NONE)
					{
						bValidPath = false;
						break;
					}
					Previous = Current;
					Current = Next;
				}

				if (!bValidPath || Current == StartRoot || !JunctionRoots.Contains(Current))
				{
					continue;
				}
				const uint64 Key = MakeEdgeKey(StartRoot, Current);
				FJunctionGraphConnection* Existing = JunctionConnections.Find(Key);
				if (!Existing || PathLength < Existing->Length)
				{
					FJunctionGraphConnection Connection;
					Connection.A = StartRoot;
					Connection.B = Current;
					Connection.Length = PathLength;
					Connection.PathRoots = MoveTemp(PathRoots);
					JunctionConnections.Add(Key, MoveTemp(Connection));
				}
			}
		}

		FUnionFind CandidateUnionFind(RelevantNodeIds.Num());
		for (int32 i = 0; i < RelevantNodeIds.Num(); ++i)
		{
			CandidateUnionFind.Union(i, UnionFind.Find(i));
		}
		for (const TPair<uint64, FJunctionGraphConnection>& Pair : JunctionConnections)
		{
			const FJunctionGraphConnection& Edge = Pair.Value;
			if (Edge.Length <= Settings.ComplexIntersectionInternalEdgeLength
				&& JunctionRoots.Contains(Edge.A) && JunctionRoots.Contains(Edge.B))
			{
				CandidateUnionFind.Union(Edge.A, Edge.B);
			}
		}

		TMap<int32, TArray<int32>> CandidateComponents;
		for (int32 JunctionRoot : JunctionRoots)
		{
			CandidateComponents.FindOrAdd(CandidateUnionFind.Find(JunctionRoot)).Add(JunctionRoot);
		}

		const float MaxDiameterSquared = FMath::Square(Settings.ComplexIntersectionMaxDiameter);
		for (const TPair<int32, TArray<int32>>& Pair : CandidateComponents)
		{
			const TArray<int32>& Component = Pair.Value;
			if (Component.Num() < 2)
			{
				continue;
			}
			bool bWithinDiameter = true;
			for (int32 AIndex = 0; AIndex < Component.Num() && bWithinDiameter; ++AIndex)
			{
				for (int32 BIndex = AIndex + 1; BIndex < Component.Num(); ++BIndex)
				{
					if (FVector2D::DistSquared(ClusterPosition(Component[AIndex]), ClusterPosition(Component[BIndex])) > MaxDiameterSquared)
					{
						bWithinDiameter = false;
						break;
					}
				}
			}
			if (!bWithinDiameter)
			{
				continue;
			}

			TArray<int32> RegionRoots = Component;
			for (const TPair<uint64, FJunctionGraphConnection>& ConnectionPair : JunctionConnections)
			{
				const FJunctionGraphConnection& Connection = ConnectionPair.Value;
				if (Connection.Length <= Settings.ComplexIntersectionInternalEdgeLength
					&& Component.Contains(Connection.A) && Component.Contains(Connection.B))
				{
					for (const int32 PathRoot : Connection.PathRoots)
					{
						RegionRoots.AddUnique(PathRoot);
						if (!Component.Contains(PathRoot))
						{
							ComplexIntersectionInteriorShapeRoots.Add(PathRoot);
						}
					}
				}
			}
			ComplexIntersectionRootComponents.Add(MoveTemp(RegionRoots));
			++Result.NumComplexIntersectionsCollapsed;
		}
	}

	TMap<int32, FVector2D> FinalPositionSum;
	TMap<int32, int32> FinalMemberCount;
	// Average primary proximity-cluster representatives, not every raw member. This keeps any
	// intentionally merged junction centered without allowing a dense run of curve points to
	// dominate its position. Complex-region portals themselves are never unioned here.
	for (const TPair<int32, int32>& Pair : ClusterMemberCount)
	{
		const int32 Root = FinalUnionFind.Find(Pair.Key);
		FinalPositionSum.FindOrAdd(Root) += ClusterPositionSum.FindChecked(Pair.Key) / static_cast<float>(Pair.Value);
		FinalMemberCount.FindOrAdd(Root) += 1;
	}

	// 7. FlexNetwork nodes are created lazily, one per final cluster, the first time a way references it.
	TMap<int32, FFlexNodeId> ClusterToFlexNode;
	// Z/ElevationType only take effect the first time a given cluster is turned into a
	// Flex node (i.e. when this way is the one creating it) -- a cluster already placed by an
	// earlier way keeps whatever position/height/type that way gave it, which is what lets a later
	// way's elevation ramp read the shared boundary node's *existing* height as its entry point.
	// Lateral placement belongs to each way's profile and therefore never moves this shared node.
	auto GetOrCreateFlexNode = [&Subsystem, &FinalUnionFind, &ClusterToFlexNode, &FinalPositionSum, &FinalMemberCount, &Result](int32 NodeIndex, float Z, EFlexRoadElevationType ElevationType) -> FFlexNodeId
	{
		const int32 Root = FinalUnionFind.Find(NodeIndex);
		if (const FFlexNodeId* Existing = ClusterToFlexNode.Find(Root))
		{
			return *Existing;
		}
		const FVector2D Centroid = FinalPositionSum.FindChecked(Root) / static_cast<float>(FinalMemberCount.FindChecked(Root));
		const FFlexNodeId NewNodeId = Subsystem.AddNode(FVector(Centroid.X, Centroid.Y, Z), ElevationType);
		ClusterToFlexNode.Add(Root, NewNodeId);
		++Result.NumNodesCreated;
		return NewNodeId;
	};

	// 8. Walk each way, resolving its lane profile and building a smooth chain of segments.
	TMap<FString, URoadTypeProfile*> ProfileCache;
	struct FImportedWaySegment
	{
		int64 WayId = 0;
		int64 StartOsmNodeId = 0;
		int64 EndOsmNodeId = 0;
		FFlexSegmentId SegmentId;
		FFlexNodeId StartNodeId;
		FFlexNodeId EndNodeId;
		TObjectPtr<URoadTypeProfile> Profile = nullptr;
	};
	TArray<FImportedWaySegment> ImportedWaySegments;

	Subsystem.BeginBatchUpdate();
	ON_SCOPE_EXIT
	{
		// Nested batches are supported. For a direct runtime import this is the one and only
		// RebuildDirty trigger; the editor wraps visualization-mode changes in an outer batch, in
		// which case this merely returns control to that outer transaction.
		Subsystem.EndBatchUpdate();
	};

	for (int64 WayId : MatchingWayIds)
	{
		const FOsmWay& Way = OsmAsset.Ways.FindChecked(WayId);
		if (Way.NodeRefs.Num() < 2)
		{
			continue;
		}

		// Resolve the way's shape to a chain of node-array indices, collapsing consecutive
		// duplicates (two adjacent shape points landing in the same merge cluster).
		TArray<int32> VertexIndices;
		TArray<int64> VertexOsmNodeIds;
		VertexIndices.Reserve(Way.NodeRefs.Num());
		VertexOsmNodeIds.Reserve(Way.NodeRefs.Num());
		int32 LastRoot = INDEX_NONE;
		for (int64 NodeRef : Way.NodeRefs)
		{
			const int32* Index = NodeIndexByOsmId.Find(NodeRef);
			if (!Index)
			{
				continue;
			}
			const int32 Root = FinalUnionFind.Find(*Index);
			if (Root == LastRoot)
			{
				continue;
			}
			VertexIndices.Add(*Index);
			VertexOsmNodeIds.Add(NodeRef);
			LastRoot = Root;
		}
		if (VertexIndices.Num() < 2)
		{
			continue;
		}

		FLaneSignature Signature = DeriveLaneSignature(Way, Settings);
		if (!Signature.bIsRailway)
		{
			Signature.LateralOffset = ComputeWayPlacementOffset(Way, Signature);
		}
		URoadTypeProfile* Profile = nullptr;
		if (URoadTypeProfile** Cached = ProfileCache.Find(Signature.ToKey()))
		{
			Profile = *Cached;
		}
		else
		{
			Profile = ResolveProfile(Signature);
			if (Profile)
			{
				ProfileCache.Add(Signature.ToKey(), Profile);
				++Result.NumDistinctLaneSignatures;
			}
		}
		if (!Profile)
		{
			Result.Warnings.Add(FString::Printf(TEXT("No profile resolved for way %lld (signature %s) -- skipped."), WayId, *Signature.ToKey()));
			continue;
		}

		const int32 NumVerts = VertexIndices.Num();

		EFlexRoadElevationType ThisWayElevationType;
		float ThisWayOffsetZ;
		ComputeWayElevation(Way, Settings, ThisWayElevationType, ThisWayOffsetZ);

		// Ramp entry height: if this way's first vertex's cluster was already placed by an earlier
		// way (a shared boundary node), ramp in from *that* node's existing height; otherwise this
		// way starts a fresh chain and simply begins flat at its own target elevation.
		const int32 FirstRoot = FinalUnionFind.Find(VertexIndices[0]);
		float EntryOffsetZ = ThisWayOffsetZ;
		if (const FFlexNodeId* ExistingFirst = ClusterToFlexNode.Find(FirstRoot))
		{
			if (const FFlexRoadNode* ExistingNode = Subsystem.GetNode(*ExistingFirst))
			{
				EntryOffsetZ = ExistingNode->Position.Z;
			}
		}
		const bool bNeedsRamp = !FMath::IsNearlyEqual(EntryOffsetZ, ThisWayOffsetZ, 0.01f);

		TArray<FVector> VertexPositions;
		VertexPositions.Reserve(NumVerts);
		for (int32 i = 0; i < NumVerts; ++i)
		{
			const int32 Idx = VertexIndices[i];
			const int32 Root = FinalUnionFind.Find(Idx);
			const FVector2D Position = FinalPositionSum.FindChecked(Root) / static_cast<float>(FinalMemberCount.FindChecked(Root));
			VertexPositions.Add(FVector(Position.X, Position.Y, 0.f));
		}

		// Cumulative 2D distance along this way's own shape points -- the ramp eases height from
		// EntryOffsetZ to ThisWayOffsetZ over ElevationTransitionLength of *this* distance (clamped
		// to the way's own total length, so a short way still always reaches its target by its last
		// node, keeping the next way's own ramp-in well-defined).
		TArray<float> CumulativeDist;
		CumulativeDist.SetNumUninitialized(NumVerts);
		CumulativeDist[0] = 0.f;
		for (int32 i = 1; i < NumVerts; ++i)
		{
			CumulativeDist[i] = CumulativeDist[i - 1] + FVector::Dist(VertexPositions[i - 1], VertexPositions[i]);
		}
		const float EffectiveRampLength = bNeedsRamp ? FMath::Clamp(Settings.ElevationTransitionLength, KINDA_SMALL_NUMBER, FMath::Max(CumulativeDist.Last(), KINDA_SMALL_NUMBER)) : 0.f;

		for (int32 i = 0; i < NumVerts; ++i)
		{
			const float Alpha = bNeedsRamp ? FMath::Clamp(CumulativeDist[i] / EffectiveRampLength, 0.f, 1.f) : 1.f;
			VertexPositions[i].Z = FMath::Lerp(EntryOffsetZ, ThisWayOffsetZ, FMath::SmoothStep(0.f, 1.f, Alpha));
		}

		// Catmull-Rom-derived tangents: each interior handle points along (next - prev), giving a
		// smooth curve through every shape point instead of a jagged chain of straight segments.
		// Now that VertexPositions carries real (ramped) Z, this also gives the curve itself a
		// smooth 3D slope through the transition instead of just kinking at the endpoints.
		auto NodeElevationTypeAt = [bNeedsRamp, EffectiveRampLength, &CumulativeDist, ThisWayElevationType](int32 VertexIndex) -> EFlexRoadElevationType
		{
			return (bNeedsRamp && CumulativeDist[VertexIndex] < EffectiveRampLength - KINDA_SMALL_NUMBER) ? EFlexRoadElevationType::Ramp : ThisWayElevationType;
		};

		FFlexNodeId PrevFlexNode = GetOrCreateFlexNode(VertexIndices[0], VertexPositions[0].Z, NodeElevationTypeAt(0));
		for (int32 i = 0; i + 1 < NumVerts; ++i)
		{
			const FFlexNodeId NextFlexNode = GetOrCreateFlexNode(VertexIndices[i + 1], VertexPositions[i + 1].Z, NodeElevationTypeAt(i + 1));

			const FVector& PCurr = VertexPositions[i];
			const FVector& PNext = VertexPositions[i + 1];
			const FVector& PPrev = (i > 0) ? VertexPositions[i - 1] : PCurr;
			const FVector& PNextNext = (i + 2 < NumVerts) ? VertexPositions[i + 2] : PNext;

			const FVector FallbackDir = (PNext - PCurr).GetSafeNormal(UE_SMALL_NUMBER, FVector::ForwardVector);
			const FVector StartTangentDir = (PNext - PPrev).GetSafeNormal(UE_SMALL_NUMBER, FallbackDir);
			const FVector EndTangentDir = (PNextNext - PCurr).GetSafeNormal(UE_SMALL_NUMBER, FallbackDir);
			const float HandleLength = FMath::Max(FVector::Dist(PCurr, PNext) / 3.f, 1.f);

			const FVector StartHandle = PCurr + StartTangentDir * HandleLength;
			const FVector EndHandle = PNext - EndTangentDir * HandleLength;

			const EFlexRoadElevationType SegmentElevationType = (bNeedsRamp && CumulativeDist[i + 1] <= EffectiveRampLength + KINDA_SMALL_NUMBER) ? EFlexRoadElevationType::Ramp : ThisWayElevationType;

			const FFlexSegmentId CreatedSegmentId = Subsystem.AddSegment(
				PrevFlexNode, NextFlexNode, StartHandle, EndHandle, Profile, SegmentElevationType);
			if (CreatedSegmentId.IsValid())
			{
				FImportedWaySegment& Imported = ImportedWaySegments.AddDefaulted_GetRef();
				Imported.WayId = WayId;
				Imported.StartOsmNodeId = VertexOsmNodeIds[i];
				Imported.EndOsmNodeId = VertexOsmNodeIds[i + 1];
				Imported.SegmentId = CreatedSegmentId;
				Imported.StartNodeId = PrevFlexNode;
				Imported.EndNodeId = NextFlexNode;
				Imported.Profile = Profile;
				++Result.NumSegmentsCreated;
			}

			PrevFlexNode = NextFlexNode;
		}

		++Result.NumWaysImported;
		if (Signature.bIsRailway)
		{
			++Result.NumRailwayWaysImported;
		}
	}

	// Remove degree-2 OSM shape nodes lying only on a consumed intersection-interior path. Join the
	// two adjacent directed segments with the original outer Bezier handles, which keeps each
	// portal's authored heading while eliminating visual/routing control points in the asphalt
	// interior. Nodes where way direction or profile changes are deliberately retained.
	for (const int32 ShapeRoot : ComplexIntersectionInteriorShapeRoots)
	{
		const int32 FinalRoot = FinalUnionFind.Find(ShapeRoot);
		const FFlexNodeId* ShapeNodeIdPtr = ClusterToFlexNode.Find(FinalRoot);
		const FFlexRoadNode* ShapeNode = ShapeNodeIdPtr ? Subsystem.GetNode(*ShapeNodeIdPtr) : nullptr;
		if (!ShapeNode || ShapeNode->ConnectedSegments.Num() != 2)
		{
			continue;
		}

		const FFlexSegmentId FirstId = ShapeNode->ConnectedSegments[0];
		const FFlexSegmentId SecondId = ShapeNode->ConnectedSegments[1];
		const FFlexRoadSegment* FirstPtr = Subsystem.GetSegment(FirstId);
		const FFlexRoadSegment* SecondPtr = Subsystem.GetSegment(SecondId);
		if (!FirstPtr || !SecondPtr || FirstPtr->Profile.Get() != SecondPtr->Profile.Get()
			|| FirstPtr->ElevationType != SecondPtr->ElevationType)
		{
			continue;
		}
		const FFlexRoadSegment* Incoming = FirstPtr->EndNodeId == *ShapeNodeIdPtr ? FirstPtr
			: (SecondPtr->EndNodeId == *ShapeNodeIdPtr ? SecondPtr : nullptr);
		const FFlexRoadSegment* Outgoing = FirstPtr->StartNodeId == *ShapeNodeIdPtr ? FirstPtr
			: (SecondPtr->StartNodeId == *ShapeNodeIdPtr ? SecondPtr : nullptr);
		if (!Incoming || !Outgoing || Incoming == Outgoing)
		{
			continue;
		}

		const FFlexSegmentId IncomingId = Incoming == FirstPtr ? FirstId : SecondId;
		const FFlexSegmentId OutgoingId = Outgoing == FirstPtr ? FirstId : SecondId;
		const FFlexNodeId StartNodeId = Incoming->StartNodeId;
		const FFlexNodeId EndNodeId = Outgoing->EndNodeId;
		const FVector StartHandle = Incoming->Curve.P1;
		const FVector EndHandle = Outgoing->Curve.P2;
		URoadTypeProfile* Profile = Incoming->Profile.Get();
		const EFlexRoadElevationType ElevationType = Incoming->ElevationType;
		const FFlexElevationProfile ElevationProfile = Incoming->ElevationProfile;
		if (!Subsystem.RemoveSegment(IncomingId) || !Subsystem.RemoveSegment(OutgoingId)
			|| !Subsystem.RemoveNode(*ShapeNodeIdPtr)
			|| !Subsystem.AddSegment(StartNodeId, EndNodeId, StartHandle, EndHandle, Profile,
				ElevationType, ElevationProfile).IsValid())
		{
			Result.Warnings.Add(TEXT("Could not simplify one complex-intersection interior shape node."));
			continue;
		}
		ClusterToFlexNode.Remove(FinalRoot);
		--Result.NumNodesCreated;
		--Result.NumSegmentsCreated;
	}

	// Register the retained portal nodes only after every way has created its graph nodes. The
	// subsystem consumes links whose endpoints share this region into one physical surface while
	// leaving those links authoritative for one-way routing and turn connectivity.
	for (const TArray<int32>& Component : ComplexIntersectionRootComponents)
	{
		TArray<FFlexNodeId> MemberNodes;
		for (int32 PrimaryRoot : Component)
		{
			if (const FFlexNodeId* NodeId = ClusterToFlexNode.Find(FinalUnionFind.Find(PrimaryRoot)))
			{
				MemberNodes.AddUnique(*NodeId);
			}
		}

		if (Subsystem.RegisterComplexIntersectionRegion(MemberNodes) == INDEX_NONE)
		{
			Result.Warnings.Add(TEXT("A detected complex intersection did not retain enough valid portal nodes to register its shared surface."));
		}
	}

	// OSM point controls become directed records attached to the exact imported approach. Missing
	// direction means both way directions, represented as separate records so MassTraffic can
	// associate each one with one inbound ZoneGraph side. Records whose short source segment was
	// consumed while simplifying a complex-intersection interior are intentionally skipped; those
	// points sit inside the shared surface rather than at a valid stop portal.
	if (Settings.bImportTrafficControls)
	{
		TSet<FString> ImportedControlKeys;
		for (const FImportedWaySegment& Imported : ImportedWaySegments)
		{
			const FFlexRoadSegment* Segment = Subsystem.GetSegment(Imported.SegmentId);
			if (!Segment || !Imported.Profile || Imported.Profile->bIsRailProfile)
			{
				continue;
			}

			auto AddEndpointControl = [&](const int64 OsmNodeId, const bool bForward,
				const FFlexNodeId JunctionNodeId, const float AnchorFraction)
			{
				const FOsmNode* OsmNode = OsmAsset.Nodes.Find(OsmNodeId);
				EFlexTrafficControlType Type;
				if (!OsmNode || !GetTrafficControlType(*OsmNode, Type)
					|| !TrafficControlAppliesToWayDirection(*OsmNode, bForward))
				{
					return;
				}

				const FString DirectionName = bForward ? TEXT("forward") : TEXT("backward");
				const FString SourceId = FString::Printf(TEXT("OSM/node/%lld/way/%lld/%s"),
					OsmNodeId, Imported.WayId, *DirectionName);
				if (ImportedControlKeys.Contains(SourceId))
				{
					return;
				}

				FFlexTrafficSignal Signal;
				Signal.Type = Type;
				Signal.AnchorSegmentId = Imported.SegmentId;
				Signal.AnchorFraction = AnchorFraction;
				Signal.ControlledApproachSegmentId = Imported.SegmentId;
				Signal.ControlledJunctionNodeId = JunctionNodeId;
				Signal.LateralOffset = (bForward
					? Imported.Profile->GetRoadwayMaxOffset()
					: -Imported.Profile->GetRoadwayMinOffset())
					+ Settings.TrafficControlRoadEdgeClearance;
				Signal.SourceId = SourceId;
				if (Subsystem.AddTrafficSignal(Signal).IsValid())
				{
					ImportedControlKeys.Add(SourceId);
					++Result.NumTrafficControlsImported;
				}
			};

			// Forward traffic reaches the segment's end; backward traffic reaches its start.
			AddEndpointControl(Imported.EndOsmNodeId, true, Imported.EndNodeId, 1.f);
			AddEndpointControl(Imported.StartOsmNodeId, false, Imported.StartNodeId, 0.f);
		}
	}

	return Result;
}
