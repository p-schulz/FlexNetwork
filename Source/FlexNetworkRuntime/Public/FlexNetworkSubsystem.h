#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "FlexNetworkTypes.h"
#include "FlexRoadNode.h"
#include "FlexRoadSegment.h"
#include "FlexNetworkBakeTypes.h"
#include "Traffic/FlexTrafficSignal.h"
#include "Intersection/FlexLaneConnectorGraph.h"
#include "Spatial/FlexSpatialGrid.h"
#include "Terrain/IFlexTerrainConformer.h"
#include "Export/IFlexNetworkExporter.h"
#include "FlexNetworkSubsystem.generated.h"

class AFlexNetworkMeshActor;
class AFlexNetworkSegmentActor;
class UFlexNetworkSettings;
struct FFlexSegmentMeshResult;
struct FFlexJunctionMeshResult;
struct FFlexUnifiedNetworkMeshResult;
struct FFlexMeshSectionData;
struct FFlexUnifiedRoadPolygonInput;

DECLARE_MULTICAST_DELEGATE_TwoParams(FOnRoadNetworkChangedNative, const TArray<FFlexNodeId>& /*ChangedNodes*/, const TArray<FFlexSegmentId>& /*ChangedSegments*/);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnRoadNetworkChangedDynamic, const TArray<FFlexNodeId>&, ChangedNodes, const TArray<FFlexSegmentId>&, ChangedSegments);
DECLARE_MULTICAST_DELEGATE(FOnFlexTrafficSignalsChangedNative);
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnFlexTrafficSignalsChangedDynamic);

/** One point where a proposed new curve crosses an existing segment; drives the auto-split-on-crossing behavior from spec 1.7. */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FFlexSegmentCrossing
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	FFlexSegmentId ExistingSegmentId;

	/** Arc length along the existing segment where the crossing occurs -- pass straight to SplitSegment. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	float ArcLengthOnExistingSegment = 0.f;

	/** Arc length along the proposed curve (not yet a graph segment) where the crossing occurs -- lets a caller building a new road split it at the same point the existing road was split, so both roads actually meet at a shared junction node instead of just visually overlapping. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	float ArcLengthOnProposedCurve = 0.f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	FVector WorldPoint = FVector::ZeroVector;
};

/**
 * Owns the FlexNetwork planar graph (nodes + Bezier segments), and is the single source of truth
 * every other stage (mesh, junction, terrain, lane-connector) derives from. Exposes a mutation
 * API for graph edits, a query API for the traffic simulation/editor tool, and drives incremental
 * rebuilds so a single edit only touches the segments/junctions it actually affects.
 *
 * A UWorldSubsystem rather than a UGameInstanceSubsystem: the graph (and its generated meshes)
 * are naturally per-world data -- PIE, a dedicated server, and the editor's persistent level each
 * want their own independent network state, which UWorldSubsystem gives for free.
 */
UCLASS()
class FLEXNETWORKRUNTIME_API UFlexNetworkSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;

	// ---------------------------------------------------------------- Batch updates

	/**
	 * Suppresses RebuildDirty() from actually running (it just keeps accumulating the dirty set)
	 * until a matching EndBatchUpdate brings the nesting count back to zero, at which point one
	 * combined rebuild covers everything touched in between. Without this, bulk-loading something
	 * like an OSM import -- hundreds or thousands of AddSegment calls -- would trigger a full
	 * incremental-rebuild pass (arc-length tables, junction polygons, mesh generation) after
	 * *every single call*, which is wasted work multiplied by the size of the import. Calls nest;
	 * always pair with EndBatchUpdate (e.g. via a scope guard) even on early-out/error paths.
	 */
	void BeginBatchUpdate();
	void EndBatchUpdate();

	// ---------------------------------------------------------------- Mutation API

	FFlexNodeId AddNode(const FVector& Position, EFlexRoadElevationType ElevationType = EFlexRoadElevationType::Ground, const FVector& UpVector = FVector::UpVector);

	/** StartTangentHandle/EndTangentHandle are absolute world positions (FFlexBezierCurve::P1/P2), not offsets. */
	FFlexSegmentId AddSegment(FFlexNodeId StartNodeId, FFlexNodeId EndNodeId, const FVector& StartTangentHandle, const FVector& EndTangentHandle, URoadTypeProfile* Profile, EFlexRoadElevationType ElevationType = EFlexRoadElevationType::Ground, const FFlexElevationProfile& ElevationProfile = FFlexElevationProfile());

	/** Removes the authoritative graph and every transient representation in this world. */
	void ClearNetwork();

	/** Reconstructs authored graph data stored by AFlexNetworkBakeActor and rebuilds all derivations. */
	int32 LoadBakedNetwork(const TArray<FFlexBakedNode>& BakedNodes, const TArray<FFlexBakedSegment>& BakedSegments, const TArray<FFlexBakedTrafficSignal>& BakedSignals, EFlexNetworkVisualizationMode BakedVisualizationMode);
	/** Compatibility overload for bake snapshots created before traffic signals were authoritative. */
	int32 LoadBakedNetwork(const TArray<FFlexBakedNode>& BakedNodes, const TArray<FFlexBakedSegment>& BakedSegments, EFlexNetworkVisualizationMode BakedVisualizationMode);

	/** Marks distinct retained nodes as routing portals of one shared complex-intersection surface. */
	int32 RegisterComplexIntersectionRegion(TConstArrayView<FFlexNodeId> MemberNodeIds);

	/** Removes Segment and detaches it from both endpoint nodes (the nodes themselves are kept, even if left with zero connections). */
	bool RemoveSegment(FFlexSegmentId SegmentId);

	/** Removes Node and cascades to remove every segment still connected to it. */
	bool RemoveNode(FFlexNodeId NodeId);

	bool SetNodePosition(FFlexNodeId NodeId, const FVector& NewPosition);
	/** Rotates the node's local up vector and every connected Bezier endpoint tangent about the node position. */
	bool RotateNode(FFlexNodeId NodeId, const FQuat& DeltaRotation);
	bool SetNodeElevationType(FFlexNodeId NodeId, EFlexRoadElevationType NewType);
	bool SetSegmentProfile(FFlexSegmentId SegmentId, URoadTypeProfile* NewProfile);
	bool SetSegmentCurve(FFlexSegmentId SegmentId, const FVector& StartTangentHandle, const FVector& EndTangentHandle);

	// ---------------------------------------------------------------- Traffic controls

	/** Adds one directed control. Returns an invalid GUID if either graph attachment is invalid. */
	FGuid AddTrafficSignal(const FFlexTrafficSignal& Signal);
	bool UpdateTrafficSignal(const FFlexTrafficSignal& Signal);
	bool RemoveTrafficSignal(const FGuid& SignalId);
	void ClearTrafficSignals();
	const TMap<FGuid, FFlexTrafficSignal>& GetAllTrafficSignals() const { return TrafficSignals; }
	const FFlexTrafficSignal* GetTrafficSignal(const FGuid& SignalId) const { return TrafficSignals.Find(SignalId); }
	bool ResolveTrafficSignal(const FFlexTrafficSignal& Signal, FFlexResolvedTrafficSignal& OutResolved) const;

	/** Splits Segment at ArcLength (De Casteljau exact split), inserting a new node there. Returns the new node's id, or an invalid id if SegmentId/ArcLength are invalid. */
	FFlexNodeId SplitSegment(FFlexSegmentId SegmentId, float ArcLength);

	// ---------------------------------------------------------------- Terrain (bulk editor commands)

	/** Force-reconforms terrain under every Ground segment to the current road heights, regardless of dirty state -- backs the toolkit's "Conform Terrain To Roads" command (e.g. to re-apply after the landscape was hand-edited since the last rebuild). Bridge/Elevated/Tunnel/Ramp segments are untouched, same as the automatic per-mutation conforming. */
	void ConformAllTerrainToRoads();

	/** Samples terrain height under every Ground node and snaps it there, draping the network onto the landscape's actual surface -- backs the toolkit's "Fit Roads To Terrain" command. Bridge/Elevated/Tunnel/Ramp nodes are left untouched (their elevation is deliberately offset from ground, not meant to be re-snapped to it). No-op per node if there's no terrain conformer or no landscape under it. */
	void FitNodesToTerrain();

	// ---------------------------------------------------------------- Snap / crossing queries (drives the editor drawing tool)

	bool FindNearestNode(const FVector& WorldPosition, float Radius, FFlexNodeId& OutNodeId) const;

	/** Nearest point on any existing segment's curve within Radius -- used to snap a drag endpoint onto a segment's midspan for splitting. */
	bool FindNearestSegmentPoint(const FVector& WorldPosition, float Radius, FFlexSegmentId& OutSegmentId, float& OutArcLength, FVector& OutPointOnCurve) const;

	/** Every existing segment a proposed new curve crosses, so the caller can split each one and wire the new segment through the resulting nodes. */
	TArray<FFlexSegmentCrossing> FindCrossings(const FFlexBezierCurve& ProposedCurve) const;

	/** A reasonable outward tangent-handle direction for a new segment starting at an existing node, continuing smoothly from whatever's already connected there (spec 1.2: tangents should stay aligned at shared nodes). Returns a fallback (world forward) at an isolated node. */
	FVector SuggestOutgoingTangentDirection(FFlexNodeId NodeId) const;

	/**
	 * Checks min length, min turn radius (curvature), and self-intersection for a proposed curve
	 * against Profile's constraints (or UFlexNetworkSettings' defaults) -- the live validity check
	 * the drawing tool runs every frame while dragging.
	 */
	bool ValidateProposedSegment(const FFlexBezierCurve& Curve, const URoadTypeProfile* Profile, FText& OutReason) const;

	// ---------------------------------------------------------------- Query API

	const FFlexRoadNode* GetNode(FFlexNodeId NodeId) const { return Nodes.Find(NodeId); }
	const FFlexRoadSegment* GetSegment(FFlexSegmentId SegmentId) const { return Segments.Find(SegmentId); }
	const TMap<FFlexNodeId, FFlexRoadNode>& GetAllNodes() const { return Nodes; }
	const TMap<FFlexSegmentId, FFlexRoadSegment>& GetAllSegments() const { return Segments; }
	TArray<FFlexSegmentId> GetConnectedSegments(FFlexNodeId NodeId) const;
	const FFlexJunctionData* GetJunctionData(FFlexNodeId NodeId) const { return JunctionDataByNode.Find(NodeId); }
	TArray<FFlexLaneConnector> GetLaneConnectorsAtNode(FFlexNodeId NodeId) const;

	/** Builds the current visible segment result from graph data, including trims from both endpoint junctions. */
	bool BuildSegmentMeshResult(FFlexSegmentId SegmentId, FFlexSegmentMeshResult& OutResult) const;

	/** Resolves both endpoint trims from the authoritative cached junction data. */
	bool GetSegmentTrimRange(FFlexSegmentId SegmentId, float& OutTrimStart, float& OutTrimEnd) const;

	/** Builds all visible layers of a cached junction using the same materials and geometry as the actor renderer. */
	bool BuildJunctionMeshResult(FFlexNodeId NodeId, FFlexJunctionMeshResult& OutResult) const;

	/**
	 * Builds rails grouped by profile across the complete graph. Outer solids and tram groove
	 * cutters are unified before subtraction so switch/intersection geometry has no segment caps.
	 */
	void BuildRailMeshResults(TArray<FFlexMeshSectionData>& OutResults) const;

	/** Position/tangent/right/up at ArcLength along Segment -- lane positions use Profile.LateralOffset + Lane.LateralOffset. */
	FFlexCurveFrame SampleSegmentAtArcLength(FFlexSegmentId SegmentId, float ArcLength) const;

	AFlexNetworkMeshActor* GetMeshActor() const { return MeshActor; }
	AFlexNetworkSegmentActor* GetSegmentActor(FFlexSegmentId SegmentId) const;
	EFlexNetworkVisualizationMode GetVisualizationMode() const { return VisualizationMode; }

	/** Switches representation and rebuilds all existing hand-authored or OSM-imported segments. */
	void SetVisualizationMode(EFlexNetworkVisualizationMode NewMode);

	/** Marks the complete graph dirty and rebuilds the currently selected representation. */
	void RebuildAllNetworkGeometry();

	// ---------------------------------------------------------------- Extension points

	/** Takes ownership. Pass nullptr to disable terrain conforming entirely. Defaults to an FFlexLandscapeConformer. */
	void SetTerrainConformer(TUniquePtr<IFlexTerrainConformer> InConformer) { TerrainConformer = MoveTemp(InConformer); }

	void RegisterExporter(TSharedPtr<IFlexNetworkExporter> Exporter);
	void UnregisterExporter(const TSharedPtr<IFlexNetworkExporter>& Exporter);

	// ---------------------------------------------------------------- Change notification

	/** Native (C++-only) delegate -- prefer this from other native modules (e.g. the traffic sim) over the dynamic one below. */
	FOnRoadNetworkChangedNative OnRoadNetworkChanged;

	UPROPERTY(BlueprintAssignable, Category = "FlexNetwork")
	FOnRoadNetworkChangedDynamic OnRoadNetworkChangedBP;

	FOnFlexTrafficSignalsChangedNative OnTrafficSignalsChanged;

	UPROPERTY(BlueprintAssignable, Category = "FlexNetwork|Traffic Signals")
	FOnFlexTrafficSignalsChangedDynamic OnTrafficSignalsChangedBP;

private:
	TFlexIdAllocator<FFlexNodeId> NodeIdAllocator;
	TFlexIdAllocator<FFlexSegmentId> SegmentIdAllocator;

	// UPROPERTY here (rather than a plain TMap) matters: FFlexRoadSegment holds a
	// TObjectPtr<URoadTypeProfile>, which needs reflection to be GC-visible -- without it the
	// referenced profile assets could be collected out from under still-live segments.
	UPROPERTY()
	TMap<FFlexNodeId, FFlexRoadNode> Nodes;

	UPROPERTY()
	TMap<FFlexSegmentId, FFlexRoadSegment> Segments;

	UPROPERTY()
	TMap<FFlexNodeId, FFlexJunctionData> JunctionDataByNode;

	/** Authoritative controls; MassTraffic assets and any future visual actors are projections. */
	UPROPERTY()
	TMap<FGuid, FFlexTrafficSignal> TrafficSignals;

	FFlexSpatialGrid SpatialGrid;

	TUniquePtr<IFlexTerrainConformer> TerrainConformer;
	TArray<TSharedPtr<IFlexNetworkExporter>> Exporters;

	UPROPERTY(Transient)
	TObjectPtr<AFlexNetworkMeshActor> MeshActor;

	/** Lightweight spline/PCG sources, one per graph segment. They intentionally contain no mesh geometry. */
	UPROPERTY(Transient)
	TMap<FFlexSegmentId, TObjectPtr<AFlexNetworkSegmentActor>> SegmentActors;

	EFlexNetworkVisualizationMode VisualizationMode = EFlexNetworkVisualizationMode::GeneratedGeometry;
	/** Segment actor currently responsible for the single profile-wide PCG rail result. */
	FFlexSegmentId RailPCGOwner = FFlexSegmentId::Invalid();

	TSet<FFlexNodeId> DirtyNodes;
	TSet<FFlexSegmentId> DirtySegments;
	int32 BatchDepth = 0;
	bool bTrafficSignalsChangedDuringBatch = false;
	int32 NextComplexIntersectionRegionIndex = 0;

	AFlexNetworkMeshActor* GetOrCreateMeshActor();
	AFlexNetworkSegmentActor* GetOrCreateSegmentActor(FFlexSegmentId SegmentId);
	const UFlexNetworkSettings* GetSettings() const;

	FVector2D NodeSegmentBoundsMin(const FFlexRoadSegment& Segment) const;
	FVector2D NodeSegmentBoundsMax(const FFlexRoadSegment& Segment) const;
	void AddSegmentToSpatialGrid(FFlexSegmentId Id, const FFlexRoadSegment& Segment);
	void RemoveSegmentFromSpatialGrid(FFlexSegmentId Id, const FFlexRoadSegment& Segment);

	void DetachSegmentFromNode(FFlexNodeId NodeId, FFlexSegmentId SegmentId);
	bool ValidateTrafficSignalAttachments(const FFlexTrafficSignal& Signal) const;
	void BroadcastTrafficSignalsChanged();

	/**
	 * Recomputes arc-length tables, junction polygons/lane-connectors and terrain conforming for
	 * everything touched since the last call. Classic rendering then rebuilds its unified footprint
	 * globally because exposed boundary ownership can change across former component boundaries;
	 * segment actors/PCG sources remain incremental. Every mutation method above ends here.
	 */
	void RebuildDirty();
	FFlexUnifiedNetworkMeshResult BuildUnifiedClassicMeshResult() const;
	bool IsInternalComplexIntersectionSegment(const FFlexRoadSegment& Segment) const;
	bool BuildComplexIntersectionRegionSurface(int32 RegionIndex, FFlexUnifiedRoadPolygonInput& OutSurface) const;
	bool IsComplexIntersectionRegionOwner(FFlexNodeId NodeId) const;

	TArray<struct FFlexJunctionApproachInput> BuildApproachInputs(FFlexNodeId NodeId) const;
};
