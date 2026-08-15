#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "FlexNetworkTypes.h"
#include "FlexRoadNode.h"
#include "FlexRoadSegment.h"
#include "Intersection/FlexLaneConnectorGraph.h"
#include "Spatial/FlexSpatialGrid.h"
#include "Terrain/IFlexTerrainConformer.h"
#include "Export/IFlexNetworkExporter.h"
#include "FlexNetworkSubsystem.generated.h"

class AFlexNetworkMeshActor;
class UFlexNetworkSettings;

DECLARE_MULTICAST_DELEGATE_TwoParams(FOnRoadNetworkChangedNative, const TArray<FFlexNodeId>& /*ChangedNodes*/, const TArray<FFlexSegmentId>& /*ChangedSegments*/);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnRoadNetworkChangedDynamic, const TArray<FFlexNodeId>&, ChangedNodes, const TArray<FFlexSegmentId>&, ChangedSegments);

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

	// ---------------------------------------------------------------- Mutation API

	FFlexNodeId AddNode(const FVector& Position, EFlexRoadElevationType ElevationType = EFlexRoadElevationType::Ground, const FVector& UpVector = FVector::UpVector);

	/** StartTangentHandle/EndTangentHandle are absolute world positions (FFlexBezierCurve::P1/P2), not offsets. */
	FFlexSegmentId AddSegment(FFlexNodeId StartNodeId, FFlexNodeId EndNodeId, const FVector& StartTangentHandle, const FVector& EndTangentHandle, URoadTypeProfile* Profile, EFlexRoadElevationType ElevationType = EFlexRoadElevationType::Ground);

	/** Removes Segment and detaches it from both endpoint nodes (the nodes themselves are kept, even if left with zero connections). */
	bool RemoveSegment(FFlexSegmentId SegmentId);

	/** Removes Node and cascades to remove every segment still connected to it. */
	bool RemoveNode(FFlexNodeId NodeId);

	bool SetNodePosition(FFlexNodeId NodeId, const FVector& NewPosition);
	bool SetNodeElevationType(FFlexNodeId NodeId, EFlexRoadElevationType NewType);
	bool SetSegmentProfile(FFlexSegmentId SegmentId, URoadTypeProfile* NewProfile);
	bool SetSegmentCurve(FFlexSegmentId SegmentId, const FVector& StartTangentHandle, const FVector& EndTangentHandle);

	/** Splits Segment at ArcLength (De Casteljau exact split), inserting a new node there. Returns the new node's id, or an invalid id if SegmentId/ArcLength are invalid. */
	FFlexNodeId SplitSegment(FFlexSegmentId SegmentId, float ArcLength);

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

	/** Position/tangent/right/up at ArcLength along Segment -- what the traffic sim samples to drive a vehicle along a lane (offset laterally by the lane's LateralOffset). */
	FFlexCurveFrame SampleSegmentAtArcLength(FFlexSegmentId SegmentId, float ArcLength) const;

	AFlexNetworkMeshActor* GetMeshActor() const { return MeshActor; }

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

	FFlexSpatialGrid SpatialGrid;

	TUniquePtr<IFlexTerrainConformer> TerrainConformer;
	TArray<TSharedPtr<IFlexNetworkExporter>> Exporters;

	UPROPERTY(Transient)
	TObjectPtr<AFlexNetworkMeshActor> MeshActor;

	TSet<FFlexNodeId> DirtyNodes;
	TSet<FFlexSegmentId> DirtySegments;

	AFlexNetworkMeshActor* GetOrCreateMeshActor();
	const UFlexNetworkSettings* GetSettings() const;

	FVector2D NodeSegmentBoundsMin(const FFlexRoadSegment& Segment) const;
	FVector2D NodeSegmentBoundsMax(const FFlexRoadSegment& Segment) const;
	void AddSegmentToSpatialGrid(FFlexSegmentId Id, const FFlexRoadSegment& Segment);
	void RemoveSegmentFromSpatialGrid(FFlexSegmentId Id, const FFlexRoadSegment& Segment);

	void DetachSegmentFromNode(FFlexNodeId NodeId, FFlexSegmentId SegmentId);

	/** Recomputes arc-length tables, junction polygons/lane-connectors, meshes, and terrain conforming for everything touched since the last call, then fires OnRoadNetworkChanged. This is the one place all of that happens -- every mutation method above ends by calling it. */
	void RebuildDirty();

	TArray<struct FFlexJunctionApproachInput> BuildApproachInputs(FFlexNodeId NodeId) const;
};
