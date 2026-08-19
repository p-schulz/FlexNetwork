#pragma once

#include "CoreMinimal.h"
#include "Math/FlexRotationMinimizingFrame.h"

/**
 * What kind of physical rail an FFlexRailEdge represents. SwitchBlade and Frog are declared for
 * the design doc's "Detailed" quality tier (real switch-blade/frog meshes) -- FFlexRailGraphBuilder
 * does not emit them yet; Phase 1 ("TopologyCorrect") only ever produces Normal, Shared, and
 * Crossing, with Crossing rendered as a simple trimmed gap by FFlexRailMeshBuilder.
 */
enum class ERailEdgeType : uint8
{
	Normal,
	Shared,
	SwitchBlade,
	Frog,
	Crossing
};

struct FLEXNETWORKRUNTIME_API FFlexRailNode
{
	FVector Position = FVector::ZeroVector;
	TArray<int32> IncomingEdges;
	TArray<int32> OutgoingEdges;
};

/**
 * One physical rail edge: a densely sampled, already left/right-offset rail-head centerline in
 * world space, with orientation frames carried through from the source segment/movement so
 * FFlexRailMeshBuilder can sweep it without needing to re-derive tangent frames from raw points.
 */
struct FLEXNETWORKRUNTIME_API FFlexRailEdge
{
	int32 StartNode = INDEX_NONE;
	int32 EndNode = INDEX_NONE;

	TArray<FFlexCurveFrame> Frames;
	ERailEdgeType Type = ERailEdgeType::Normal;

	/** Which physical rail (of the track's two) this edge belongs to -- needed to mirror the groove-cutter offset correctly per side. */
	bool bLeftRail = false;

	/** Index/indices, into the raw rail-path list FFlexRailGraphBuilder built internally, that contributed to this edge -- more than one only for Shared. Diagnostic/debug use only. */
	TArray<int32> SourcePathIndices;
};

/**
 * The design doc's FRailGraph ("which physical pieces of steel exist"), built by
 * FFlexRailGraphBuilder from a solved FTrackGraph. Distinct rail nodes at the same spatial point
 * (e.g. two rails that cross without connecting) are represented as separate FFlexRailNode
 * entries, never merged -- see FFlexRailGraphBuilder's crossing-detection pass.
 */
struct FLEXNETWORKRUNTIME_API FFlexRailGraph
{
	TArray<FFlexRailNode> Nodes;
	TArray<FFlexRailEdge> Edges;
};
