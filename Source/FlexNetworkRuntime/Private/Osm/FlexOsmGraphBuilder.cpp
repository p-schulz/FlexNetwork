#include "Osm/FlexOsmGraphBuilder.h"
#include "Osm/OsmDataAsset.h"
#include "FlexNetworkSubsystem.h"
#include "RoadTypeProfile.h"
#include "Misc/ScopeExit.h"

namespace
{
	/** Minimal union-find (disjoint set), used to cluster OSM nodes within JunctionMergeRadius of each other. */
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

	FFlexOsmGraphBuilder::FLaneSignature DeriveLaneSignature(const FOsmWay& Way, const FFlexOsmImportSettings& Settings)
	{
		FFlexOsmGraphBuilder::FLaneSignature Signature;
		Signature.HighwayTag = Way.Tags.FindRef(TEXT("highway"));
		Signature.LaneWidth = Settings.DefaultLaneWidth;
		Signature.SpeedLimitKmh = Settings.DefaultSpeedLimitKmh;
		Signature.SidewalkWidth = Settings.SidewalkWidth;

		const FString OneWayTag = Way.Tags.FindRef(TEXT("oneway"));
		const bool bOneWay = (OneWayTag == TEXT("yes") || OneWayTag == TEXT("1") || OneWayTag == TEXT("true"));

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
			Signature.ForwardLanes = TotalLanes;
			Signature.BackwardLanes = 0;
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
					Signature.SpeedLimitKmh = Value;
				}
			}
		}

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
	 * out of naturally rather than being asserted. Returns 0 for a way with no placement tag at
	 * all (its digitized line is assumed to already be correct).
	 */
	float ComputeWayPlacementOffset(const FOsmWay& Way, const FFlexOsmGraphBuilder::FLaneSignature& Signature)
	{
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
			return 0.f;
		}

		FString EdgeKind, LaneStr;
		if (!PlacementTag.Split(TEXT(":"), &EdgeKind, &LaneStr))
		{
			return 0.f; // Malformed (missing ":<lane>") -- nothing sensible to derive.
		}
		const int32 Lane = FCString::Atoi(*LaneStr);
		if (Lane <= 0)
		{
			return 0.f;
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
			return 0.f; // Unrecognized placement kind (e.g. "transition") -- nothing sensible to derive.
		}

		const float FlexOriginFromLeftEdge = static_cast<float>(Signature.BackwardLanes) * Signature.LaneWidth;
		const float WayOffsetFromFlexOrigin = DistFromLeftEdge - FlexOriginFromLeftEdge;
		return -WayOffsetFromFlexOrigin;
	}
}

FString FFlexOsmGraphBuilder::FLaneSignature::ToKey() const
{
	return FString::Printf(TEXT("%s_F%d_B%d_W%d"), *HighwayTag, ForwardLanes, BackwardLanes, FMath::RoundToInt(LaneWidth));
}

void FFlexOsmGraphBuilder::ConfigureProfileFromLaneSignature(URoadTypeProfile& Profile, const FLaneSignature& Signature)
{
	Profile.Lanes.Reset();

	const float SpeedLimitCmPerSec = Signature.SpeedLimitKmh * 100000.f / 3600.f;

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
	const double X = FMath::DegreesToRadians(Lon - OriginLonDeg) * FMath::Cos(OriginLatRad) * kEarthRadiusCm;
	const double Y = FMath::DegreesToRadians(Lat - OriginLatDeg) * kEarthRadiusCm;
	return FVector2D(X, Y);
}

void FFlexOsmGraphBuilder::UnprojectLocalCmToLatLon(double LocalX, double LocalY, double OriginLatDeg, double OriginLonDeg, double& OutLat, double& OutLon)
{
	constexpr double kEarthRadiusCm = 6378137.0 * 100.0;
	const double OriginLatRad = FMath::DegreesToRadians(OriginLatDeg);
	OutLat = OriginLatDeg + FMath::RadiansToDegrees(LocalY / kEarthRadiusCm);
	OutLon = OriginLonDeg + FMath::RadiansToDegrees(LocalX / (kEarthRadiusCm * FMath::Cos(OriginLatRad)));
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
	for (const TPair<int64, FOsmNode>& Pair : OsmAsset.Nodes)
	{
		OutOriginLat = Pair.Value.Latitude;
		OutOriginLon = Pair.Value.Longitude;
		return true;
	}
	return false;
}

bool FFlexOsmGraphBuilder::ComputeMatchingRoadExtent(const UOsmDataAsset& OsmAsset, const FFlexOsmImportSettings& Settings, double& OutOriginLat, double& OutOriginLon, FVector2D& OutMinLocalCm, FVector2D& OutMaxLocalCm)
{
	TArray<int64> MatchingWayIds;
	for (const TPair<int64, FOsmWay>& Pair : OsmAsset.Ways)
	{
		const FString* HighwayTag = Pair.Value.Tags.Find(TEXT("highway"));
		if (HighwayTag && Settings.HighwayTags.Contains(*HighwayTag))
		{
			MatchingWayIds.Add(Pair.Key);
		}
	}
	if (MatchingWayIds.Num() == 0)
	{
		return false;
	}

	double OriginLat = 0.0, OriginLon = 0.0;
	bool bOriginFound = false;
	if (Settings.bUseOriginOverride)
	{
		OriginLat = Settings.OriginLatLon.X;
		OriginLon = Settings.OriginLatLon.Y;
		bOriginFound = true;
	}
	else if (OsmAsset.Bounds.bIsValid)
	{
		const FVector2D Center = OsmAsset.Bounds.GetCenter();
		OriginLat = Center.X;
		OriginLon = Center.Y;
		bOriginFound = true;
	}
	else
	{
		// Matches BuildFromOsm's own third fallback exactly (the first node referenced by a
		// *matching* way) -- deliberately not ResolveOrigin's own third fallback (first node in
		// OsmAsset.Nodes' iteration order, which needn't even be reachable from a matching way).
		for (int64 WayId : MatchingWayIds)
		{
			const FOsmWay& Way = OsmAsset.Ways.FindChecked(WayId);
			for (int64 NodeRef : Way.NodeRefs)
			{
				if (const FOsmNode* OsmNode = OsmAsset.Nodes.Find(NodeRef))
				{
					OriginLat = OsmNode->Latitude;
					OriginLon = OsmNode->Longitude;
					bOriginFound = true;
					break;
				}
			}
			if (bOriginFound)
			{
				break;
			}
		}
	}
	if (!bOriginFound)
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

	// 1. Filter ways by highway tag.
	TArray<int64> MatchingWayIds;
	for (const TPair<int64, FOsmWay>& Pair : OsmAsset.Ways)
	{
		const FString* HighwayTag = Pair.Value.Tags.Find(TEXT("highway"));
		if (HighwayTag && Settings.HighwayTags.Contains(*HighwayTag))
		{
			MatchingWayIds.Add(Pair.Key);
		}
	}

	if (MatchingWayIds.Num() == 0)
	{
		Result.Warnings.Add(TEXT("No ways matched the configured HighwayTags filter."));
		return Result;
	}

	// 2. Projection origin: explicit override, else the file's <bounds> midpoint (its first
	// element, when present), else the first referenced node encountered as a last resort for
	// files without a <bounds> element. Settings.bUseOriginOverride/OsmAsset.Bounds cases are
	// shared with ResolveOrigin above (any other importer wanting to line up with this one should
	// call that instead of reimplementing this priority order) -- the third fallback here is
	// intentionally *not* shared, since it needs the highway-tag-filtered node set below rather
	// than just the first node in the asset's map.
	double OriginLat = 0.0, OriginLon = 0.0;
	bool bOriginFound = false;
	if (Settings.bUseOriginOverride)
	{
		OriginLat = Settings.OriginLatLon.X;
		OriginLon = Settings.OriginLatLon.Y;
		bOriginFound = true;
	}
	else if (OsmAsset.Bounds.bIsValid)
	{
		const FVector2D Center = OsmAsset.Bounds.GetCenter();
		OriginLat = Center.X;
		OriginLon = Center.Y;
		bOriginFound = true;
	}

	// 3. Collect every node referenced by a matching way.
	TArray<int64> RelevantNodeIds;
	TMap<int64, int32> NodeIndexByOsmId;
	TArray<FVector2D> RawPositions;

	for (int64 WayId : MatchingWayIds)
	{
		const FOsmWay& Way = OsmAsset.Ways.FindChecked(WayId);
		for (int64 NodeRef : Way.NodeRefs)
		{
			if (NodeIndexByOsmId.Contains(NodeRef))
			{
				continue;
			}
			const FOsmNode* OsmNode = OsmAsset.Nodes.Find(NodeRef);
			if (!OsmNode)
			{
				continue; // Referenced node wasn't in the asset (e.g. a partial/clipped extract).
			}
			if (!bOriginFound)
			{
				OriginLat = OsmNode->Latitude;
				OriginLon = OsmNode->Longitude;
				bOriginFound = true;
			}
			const int32 Index = RelevantNodeIds.Add(NodeRef);
			NodeIndexByOsmId.Add(NodeRef, Index);
			RawPositions.Add(FVector2D::ZeroVector);
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

	// 4. Cluster nodes within JunctionMergeRadius via a lightweight spatial hash + union-find.
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
						if (j > i && FVector2D::DistSquared(RawPositions[i], RawPositions[j]) <= RadiusSq)
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

	// 6. FlexNetwork nodes are created lazily, one per cluster, the first time a way references it.
	TMap<int32, FFlexNodeId> ClusterToFlexNode;
	// XYShift/Z/ElevationType only take effect the first time a given cluster is turned into a
	// Flex node (i.e. when this way is the one creating it) -- a cluster already placed by an
	// earlier way keeps whatever position/height/type that way gave it, which is what lets a later
	// way's elevation ramp read the shared boundary node's *existing* height as its entry point
	// (see the per-way loop below). A placement-tag correction is likewise only meaningful for the
	// way that's actually creating the node -- at a junction shared by multiple ways it's
	// inherently ambiguous which one's correction should win, so whichever gets there first does.
	auto GetOrCreateFlexNode = [&Subsystem, &UnionFind, &ClusterToFlexNode, &ClusterPositionSum, &ClusterMemberCount, &Result](int32 NodeIndex, const FVector2D& XYShift, float Z, EFlexRoadElevationType ElevationType) -> FFlexNodeId
	{
		const int32 Root = UnionFind.Find(NodeIndex);
		if (const FFlexNodeId* Existing = ClusterToFlexNode.Find(Root))
		{
			return *Existing;
		}
		const FVector2D Centroid = ClusterPositionSum.FindChecked(Root) / static_cast<float>(ClusterMemberCount.FindChecked(Root)) + XYShift;
		const FFlexNodeId NewNodeId = Subsystem.AddNode(FVector(Centroid.X, Centroid.Y, Z), ElevationType);
		ClusterToFlexNode.Add(Root, NewNodeId);
		++Result.NumNodesCreated;
		return NewNodeId;
	};

	// 7. Walk each way, resolving its lane profile and building a smooth chain of segments.
	TMap<FString, URoadTypeProfile*> ProfileCache;

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
		VertexIndices.Reserve(Way.NodeRefs.Num());
		int32 LastRoot = INDEX_NONE;
		for (int64 NodeRef : Way.NodeRefs)
		{
			const int32* Index = NodeIndexByOsmId.Find(NodeRef);
			if (!Index)
			{
				continue;
			}
			const int32 Root = UnionFind.Find(*Index);
			if (Root == LastRoot)
			{
				continue;
			}
			VertexIndices.Add(*Index);
			LastRoot = Root;
		}
		if (VertexIndices.Num() < 2)
		{
			continue;
		}

		const FLaneSignature Signature = DeriveLaneSignature(Way, Settings);
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
		const int32 FirstRoot = UnionFind.Find(VertexIndices[0]);
		float EntryOffsetZ = ThisWayOffsetZ;
		if (const FFlexNodeId* ExistingFirst = ClusterToFlexNode.Find(FirstRoot))
		{
			if (const FFlexRoadNode* ExistingNode = Subsystem.GetNode(*ExistingFirst))
			{
				EntryOffsetZ = ExistingNode->Position.Z;
			}
		}
		const bool bNeedsRamp = !FMath::IsNearlyEqual(EntryOffsetZ, ThisWayOffsetZ, 0.01f);

		// A placement=<right_of|left_of|middle_of>:<lane> tag means this way wasn't digitized along
		// the road's true centerline -- shift every node it creates sideways (perpendicular to the
		// way's own local direction at that point) by however far off-center OSM says it was
		// traced, so the generated cross-section actually sits where the road really is.
		const float PlacementShiftCm = ComputeWayPlacementOffset(Way, Signature);

		TArray<FVector> VertexPositions;
		VertexPositions.Reserve(NumVerts);
		TArray<FVector2D> VertexShift2D;
		VertexShift2D.Reserve(NumVerts);
		for (int32 i = 0; i < NumVerts; ++i)
		{
			const int32 Idx = VertexIndices[i];
			FVector2D Shift = FVector2D::ZeroVector;
			if (!FMath::IsNearlyZero(PlacementShiftCm))
			{
				const FVector2D& PCurr2D = RawPositions[Idx];
				const FVector2D& PPrev2D = (i > 0) ? RawPositions[VertexIndices[i - 1]] : PCurr2D;
				const FVector2D& PNext2D = (i + 1 < NumVerts) ? RawPositions[VertexIndices[i + 1]] : PCurr2D;
				const FVector2D RawTangent2D = PNext2D - PPrev2D;
				const FVector2D TangentDir2D = RawTangent2D.IsNearlyZero(UE_SMALL_NUMBER) ? FVector2D(1.f, 0.f) : RawTangent2D.GetSafeNormal();
				const FVector2D RightDir2D(TangentDir2D.Y, -TangentDir2D.X);
				Shift = RightDir2D * PlacementShiftCm;
			}
			VertexShift2D.Add(Shift);
			VertexPositions.Add(FVector(RawPositions[Idx].X + Shift.X, RawPositions[Idx].Y + Shift.Y, 0.f));
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

		FFlexNodeId PrevFlexNode = GetOrCreateFlexNode(VertexIndices[0], VertexShift2D[0], VertexPositions[0].Z, NodeElevationTypeAt(0));
		for (int32 i = 0; i + 1 < NumVerts; ++i)
		{
			const FFlexNodeId NextFlexNode = GetOrCreateFlexNode(VertexIndices[i + 1], VertexShift2D[i + 1], VertexPositions[i + 1].Z, NodeElevationTypeAt(i + 1));

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

			if (Subsystem.AddSegment(PrevFlexNode, NextFlexNode, StartHandle, EndHandle, Profile, SegmentElevationType).IsValid())
			{
				++Result.NumSegmentsCreated;
			}

			PrevFlexNode = NextFlexNode;
		}

		++Result.NumWaysImported;
	}

	return Result;
}
