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
#include "Mesh/FlexMeshSectionData.h"
#include "FlexNetworkSubsystem.generated.h"

class AActor;
class AFlexNetworkMeshActor;
class AFlexNetworkSegmentActor;
class UFlexNetworkSettings;
struct FFlexSegmentMeshResult;
struct FFlexJunctionMeshResult;
struct FFlexUnifiedRoadPolygonInput;

/**
 * One connected road-graph component's cached unified road/sidewalk/curb mesh slice, plus the
 * exact node membership it was built from -- BuildUnifiedClassicMeshResult compares this against
 * the current rebuild's membership to detect a component split/merge and force a fresh build
 * rather than risk reusing a slice that no longer corresponds to the same physical set of roads.
 */
struct FLEXNETWORKRUNTIME_API FFlexCachedRoadComponent
{
	TSet<FFlexNodeId> MemberNodeIds;
	TArray<FFlexMeshSectionData> Roadways;
	TArray<FFlexMeshSectionData> Sidewalks;
	TArray<FFlexMeshSectionData> Curbs;
	TArray<TArray<FVector>> CurbLines;
};

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
	 * Builds rails grouped by profile across the complete graph. For each distinct rail profile,
	 * derives a topology-first TrackGraph/RailGraph (see Rail/FlexTrackGraphBuilder.h,
	 * Rail/FlexTrackJunctionSolver.h, Rail/FlexRailGraphBuilder.h) and sweeps its edges -- switches,
	 * turnouts, and crossings are resolved as rail topology before any mesh is built, rather than
	 * relying on mesh-boolean unions to keep junction geometry watertight.
	 */
	void BuildRailMeshResults(TArray<FFlexMeshSectionData>& OutResults) const;

	/**
	 * Builds road-marking geometry graph-wide, grouped one section per distinct material actually
	 * used: solid lines between opposite-direction lanes, dashed lines between same-direction lanes
	 * (or a two-lane road's sole boundary), dashed guide lines on the left border of qualifying
	 * lanes through a junction (by default just the leftmost incoming lane's left-turn/straight
	 * movements -- see UFlexNetworkSettings::bMarkingIntersectionLeftmostLaneOnly), short dashes
	 * along both long edges of every crosswalk, and one divider line at every
	 * UFlexNetworkSettings::MarkingParkingSpotSpacing interval along a Parking-type lane. A no-op if
	 * UFlexNetworkSettings::bGenerateRoadMarkings is off, or if no profile has any marking material
	 * configured. See Mesh/FlexRoadMarkingBuilder.h for the per-case generation logic.
	 */
	void BuildRoadMarkingMeshResults(TArray<FFlexMeshSectionData>& OutResults) const;

	/**
	 * Builds bike-lane overlay geometry graph-wide, grouped one section per distinct
	 * BikeLaneMaterial actually used: a thin strip raised UFlexNetworkSettings::BikeLaneVerticalOffset
	 * above the ordinary roadway surface for each contiguous run of Bike-type lanes in a segment's
	 * profile (see FFlexRoadMeshBuilder::AppendBikeLaneOverlay). A no-op if no profile has a
	 * BikeLaneMaterial configured.
	 */
	void BuildBikeLaneMeshResults(TArray<FFlexMeshSectionData>& OutResults) const;

	/** Same reasoning as BuildBikeLaneMeshResults, for Parking-type lanes and ParkingLaneMaterial instead. */
	void BuildParkingLaneMeshResults(TArray<FFlexMeshSectionData>& OutResults) const;

	/** Curb walls (see FFlexRoadMeshBuilder::AppendParkingLaneCurbs) around every Parking-type lane run, graph-wide, for every profile with URoadTypeProfile::bGenerateParkingLaneCurbs on. */
	void BuildParkingLaneCurbMeshResults(TArray<FFlexMeshSectionData>& OutResults) const;

	/**
	 * Sidewalk tree-patch (tree pit) top and curb-wall geometry graph-wide, for every profile with
	 * URoadTypeProfile::bGenerateSidewalkTreePatches on. See FFlexRoadMeshBuilder::AppendSidewalkTreePatches.
	 */
	void BuildSidewalkTreePatchMeshResults(TArray<FFlexMeshSectionData>& OutTopResults, TArray<FFlexMeshSectionData>& OutWallResults) const;

	/**
	 * Builds median-lane raised top and curb-wall geometry graph-wide, grouped one section per
	 * distinct material actually used: OutTopResults uses each profile's MedianMaterial, OutWallResults
	 * uses CurbMaterial (falling back to SidewalkMaterial) -- see FFlexRoadMeshBuilder::AppendMedianOverlay.
	 */
	void BuildMedianMeshResults(TArray<FFlexMeshSectionData>& OutTopResults, TArray<FFlexMeshSectionData>& OutWallResults) const;

	/**
	 * On-demand actor-spawning pass (see FRoadLaneDescriptor::LaneActors/FFlexLaneActorSpawnEntry):
	 * destroys every actor a previous call spawned, then walks every non-rail segment's lanes and, for
	 * each FFlexLaneActorSpawnEntry with a valid ActorClass, spawns one instance every
	 * SpacingDistance along the segment's trimmed span, positioned LaneEdgeOffset out from the lane's
	 * own outer edge and oriented along the lane's direction of travel plus HeadingOffsetDegrees
	 * (both optionally jittered per bUseOffsetAndHeadingVariance). Unlike the mesh-generation passes
	 * above, this is never called automatically from RebuildDirty() -- call it explicitly (see
	 * UFlexNetworkEdModeSettings::GenerateLaneActors) whenever the graph or its profiles' lane-actor
	 * configuration has changed. Returns how many actors were spawned.
	 */
	int32 GenerateLaneActors();

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

	/** Actors spawned by the most recent GenerateLaneActors() call, destroyed wholesale at the start of the next one. */
	UPROPERTY(Transient)
	TArray<TObjectPtr<AActor>> SpawnedLaneActors;

	EFlexNetworkVisualizationMode VisualizationMode = EFlexNetworkVisualizationMode::GeneratedGeometry;
	/** Segment actor currently responsible for the single profile-wide PCG rail result. */
	FFlexSegmentId RailPCGOwner = FFlexSegmentId::Invalid();

	TSet<FFlexNodeId> DirtyNodes;
	TSet<FFlexSegmentId> DirtySegments;
	int32 BatchDepth = 0;

	/**
	 * Set whenever a segment is removed since the last RebuildDirty() call. A removed segment can
	 * no longer be looked up, so it can never appear as a rail/non-rail hit in a dirty-segment scope
	 * check -- without this, deleting a network's only rail segment (say) would leave
	 * CachedRailSections permanently stale. RebuildDirty() forces a full (unscoped) rail/marking
	 * recompute whenever this is set, then clears it.
	 */
	bool bSegmentRemovedSinceLastRebuild = false;

	bool bTrafficSignalsChangedDuringBatch = false;
	int32 NextComplexIntersectionRegionIndex = 0;

	/**
	 * Last-built rail/marking mesh results, reused by BuildUnifiedClassicMeshResult whenever the
	 * current rebuild's dirty segment scope doesn't touch anything rail/marking-relevant (e.g.
	 * editing an ordinary road segment with no rail neighbors never needs to recompute rail
	 * geometry). Always fully recomputed when DirtySegmentsScope is null (unknown/full scope),
	 * which is exactly what every dirty node/segment in the graph being marked dirty produces --
	 * so RebuildAllNetworkGeometry's full rebuild behavior is unaffected by construction.
	 */
	UPROPERTY(Transient)
	TArray<FFlexMeshSectionData> CachedRailSections;

	UPROPERTY(Transient)
	TArray<FFlexMeshSectionData> CachedMarkingSections;

	/** Same reuse strategy as CachedMarkingSections, gated by the same "any dirty non-rail segment" scope check -- bike lane overlays are, like markings, a per-segment additive pass over the same non-rail segments. */
	UPROPERTY(Transient)
	TArray<FFlexMeshSectionData> CachedBikeLaneSections;

	/** Same reuse strategy as CachedBikeLaneSections, for Parking-type lane overlay geometry. */
	UPROPERTY(Transient)
	TArray<FFlexMeshSectionData> CachedParkingLaneSections;

	/** Same reuse strategy as CachedBikeLaneSections, for parking-lane curb wall geometry. */
	UPROPERTY(Transient)
	TArray<FFlexMeshSectionData> CachedParkingLaneCurbSections;

	/** Same reuse strategy as CachedBikeLaneSections, for sidewalk tree-patch top/wall geometry. */
	UPROPERTY(Transient)
	TArray<FFlexMeshSectionData> CachedTreePatchTopSections;

	UPROPERTY(Transient)
	TArray<FFlexMeshSectionData> CachedTreePatchWallSections;

	/** Same reuse strategy as CachedBikeLaneSections, for median top/wall geometry. */
	UPROPERTY(Transient)
	TArray<FFlexMeshSectionData> CachedMedianTopSections;

	UPROPERTY(Transient)
	TArray<FFlexMeshSectionData> CachedMedianWallSections;

	/** One entry per connected road-graph component last seen by BuildUnifiedClassicMeshResult, keyed by that component's canonical (smallest-member-node-ID) key. Not a UPROPERTY: FFlexCachedRoadComponent's own TObjectPtr-bearing FFlexMeshSectionData entries are already kept alive by Segments' Profile references, and this cache is wholesale-replaced (never incrementally mutated) every rebuild, so nothing here needs to survive a GC pass on its own. */
	TMap<FFlexNodeId, FFlexCachedRoadComponent> CachedRoadComponents;

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
	 * per connected road-graph component (see BuildUnifiedClassicMeshResult/CachedRoadComponents) --
	 * only components a touched node/segment actually belongs to are recomputed, since exposed
	 * boundary ownership can change across former sub-boundaries *within* a component but never
	 * reaches into a disconnected one; segment actors/PCG sources remain incremental too. Every
	 * mutation method above ends here.
	 */
	void RebuildDirty();

	/**
	 * DirtySegmentsScope, when non-null, restricts recomputation two ways:
	 *  - Rail/marking mesh: only recomputed when the scope contains a rail-profile / an ordinary
	 *    road-profile segment respectively; otherwise CachedRailSections/CachedMarkingSections are
	 *    reused unchanged.
	 *  - Road/sidewalk/curb mesh: computed per connected road-graph component (see
	 *    CachedRoadComponents); a component is only rebuilt when the scope contains one of its
	 *    member segments (or its cached membership no longer matches -- a split/merge always forces
	 *    a rebuild regardless of scope).
	 * Pass nullptr (the default) to force everything to recompute, which is what every caller
	 * outside RebuildDirty's own incremental path should do.
	 */
	FFlexUnifiedNetworkMeshResult BuildUnifiedClassicMeshResult(const TSet<FFlexSegmentId>* DirtySegmentsScope = nullptr);
	bool IsInternalComplexIntersectionSegment(const FFlexRoadSegment& Segment) const;
	bool BuildComplexIntersectionRegionSurface(int32 RegionIndex, FFlexUnifiedRoadPolygonInput& OutSurface) const;
	bool IsComplexIntersectionRegionOwner(FFlexNodeId NodeId) const;

	TArray<struct FFlexJunctionApproachInput> BuildApproachInputs(FFlexNodeId NodeId) const;
};
