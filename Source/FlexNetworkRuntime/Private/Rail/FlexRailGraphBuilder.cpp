#include "Rail/FlexRailGraphBuilder.h"

#include "Algo/Reverse.h"
#include "FlexNetworkSubsystem.h"
#include "FlexRoadNode.h"
#include "FlexRoadSegment.h"
#include "RoadTypeProfile.h"
#include "Math/FlexBezierMath.h"
#include "Math/FlexGeometry2D.h"
#include "Math/FlexRotationMinimizingFrame.h"
#include "Mesh/FlexRoadMeshBuilder.h"

namespace
{
	enum class EAnchorKind : uint8 { None, OrdinaryEnd, JunctionPort };

	/**
	 * Identifies where one end of a raw rail path sits: either the plain road-graph node it
	 * terminates/continues at (OrdinaryEnd -- a dead end, or a bend that isn't a rail junction), or
	 * a specific port of a specific junction (JunctionPort). Two anchors compare equal by identity
	 * (node + port index), not by position, which is what lets the merge/graph-node logic below
	 * connect edges correctly even where floating-point position comparison would be fragile.
	 */
	struct FRailPathAnchor
	{
		EAnchorKind Kind = EAnchorKind::None;
		FFlexNodeId NodeId;
		int32 PortIndex = INDEX_NONE;

		bool SharesJunctionPortWith(const FRailPathAnchor& Other) const
		{
			return Kind == EAnchorKind::JunctionPort && Other.Kind == EAnchorKind::JunctionPort
				&& NodeId == Other.NodeId && PortIndex == Other.PortIndex;
		}
	};

	struct FRawRailPath
	{
		TArray<FFlexCurveFrame> Frames;
		FRailPathAnchor StartAnchor;
		FRailPathAnchor EndAnchor;
		bool bLeftRail = false;

		// Populated by the merge pass. A "merge cut" is a natural (forward) index into Frames: for
		// a start-anchored merge the shared span is Frames[0..MergeCutAtStart]; for an end-anchored
		// merge it's Frames[MergeCutAtEnd..Last]. The lower-index path in a merged pair owns the
		// emitted Shared edge; its partner suppresses its own copy of that span.
		int32 MergeCutAtStart = INDEX_NONE;
		int32 MergeCutAtEnd = INDEX_NONE;
		bool bSuppressStartSpanEdge = false;
		bool bSuppressEndSpanEdge = false;
		TArray<int32> MergePartnerIndices;

		// Populated by the crossing pass: natural indices (the earlier point of the crossing
		// segment pair) at which this path is cut, with no node shared with the other path.
		TArray<int32> CrossingCutIndices;
	};

	struct FAnchorSlotKey
	{
		FFlexNodeId NodeId;
		int32 PortIndex = INDEX_NONE;
		bool bLeftRail = false;

		bool operator==(const FAnchorSlotKey& Other) const
		{
			return NodeId == Other.NodeId && PortIndex == Other.PortIndex && bLeftRail == Other.bLeftRail;
		}
		friend uint32 GetTypeHash(const FAnchorSlotKey& Key)
		{
			return HashCombine(HashCombine(GetTypeHash(Key.NodeId), GetTypeHash(Key.PortIndex)), GetTypeHash(Key.bLeftRail));
		}
	};

	struct FAnchorSlotEntry
	{
		int32 PathIndex = INDEX_NONE;
		bool bAtPathStart = false;
	};

	struct FJunctionPortKey
	{
		FFlexNodeId NodeId;
		int32 PortIndex = INDEX_NONE;

		bool operator==(const FJunctionPortKey& Other) const
		{
			return NodeId == Other.NodeId && PortIndex == Other.PortIndex;
		}
		friend uint32 GetTypeHash(const FJunctionPortKey& Key)
		{
			return HashCombine(GetTypeHash(Key.NodeId), GetTypeHash(Key.PortIndex));
		}
	};

	const FFlexTrackJunction* FindJunction(TArrayView<const FFlexTrackJunction> Junctions, FFlexNodeId NodeId)
	{
		for (const FFlexTrackJunction& Junction : Junctions)
		{
			if (Junction.NodeId == NodeId)
			{
				return &Junction;
			}
		}
		return nullptr;
	}

	int32 FindPortIndex(const FFlexTrackJunction& Junction, FFlexSegmentId SegmentId, bool bAtSegmentEnd)
	{
		for (int32 Index = 0; Index < Junction.Ports.Num(); ++Index)
		{
			const FFlexTrackPort& Port = Junction.Ports[Index];
			if (Port.SegmentId == SegmentId && Port.bAtSegmentEnd == bAtSegmentEnd)
			{
				return Index;
			}
		}
		return INDEX_NONE;
	}

	/** Appends the left/right raw rail paths offset from one already-trimmed span of frames. */
	void AppendRailPairFromFrames(TConstArrayView<FFlexCurveFrame> Frames, float Gauge, float RailWidth,
		const FRailPathAnchor& StartAnchor, const FRailPathAnchor& EndAnchor, TArray<FRawRailPath>& OutPaths)
	{
		if (Frames.Num() < 2)
		{
			return;
		}
		// Matches the existing profile semantics used elsewhere in the plugin (e.g. the previous
		// FFlexRailMeshBuilder): RailWidth is the rail's own base width, so the two rail centerlines
		// sit half a gauge plus half a rail-width out from the track centerline on each side.
		const float RailCenterOffset = (Gauge + RailWidth) * 0.5f;
		for (const float Side : { -1.f, 1.f })
		{
			FRawRailPath RailPath;
			RailPath.bLeftRail = Side < 0.f;
			RailPath.StartAnchor = StartAnchor;
			RailPath.EndAnchor = EndAnchor;
			RailPath.Frames.Reserve(Frames.Num());
			for (const FFlexCurveFrame& Frame : Frames)
			{
				FFlexCurveFrame Offset = Frame;
				Offset.Position = Frame.Position + Frame.Right * (Side * RailCenterOffset);
				RailPath.Frames.Add(Offset);
			}
			OutPaths.Add(MoveTemp(RailPath));
		}
	}

	TArray<FFlexCurveFrame> ReversedCopy(const TArray<FFlexCurveFrame>& In)
	{
		TArray<FFlexCurveFrame> Out = In;
		Algo::Reverse(Out);
		return Out;
	}

	/** How many leading samples of A and B (both starting at the same shared anchor point) stay within tolerance of each other. */
	int32 FindCommonPrefixLength(TConstArrayView<FFlexCurveFrame> A, TConstArrayView<FFlexCurveFrame> B, float MergeTolerance, float AngleToleranceDegrees)
	{
		const int32 MaxCheck = FMath::Min(A.Num(), B.Num());
		int32 Count = MaxCheck > 0 ? 1 : 0;
		for (int32 Index = 1; Index < MaxCheck; ++Index)
		{
			if (FVector::DistSquared(A[Index].Position, B[Index].Position) > FMath::Square(MergeTolerance))
			{
				break;
			}
			const float CosAngle = FVector::DotProduct(A[Index].Tangent.GetSafeNormal(), B[Index].Tangent.GetSafeNormal());
			const float AngleDegrees = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(CosAngle, -1.f, 1.f)));
			if (AngleDegrees > AngleToleranceDegrees)
			{
				break;
			}
			Count = Index + 1;
		}
		return Count;
	}

	/** Finds the first genuine (large-angle) intersection between two raw polylines and records a crossing cut on each. No shared node is created; FFlexRailMeshBuilder trims a visual gap on both sides instead. */
	bool FindAndRecordCrossing(FRawRailPath& PathA, FRawRailPath& PathB, float CrossingAngleToleranceDegrees)
	{
		for (int32 i = 0; i + 1 < PathA.Frames.Num(); ++i)
		{
			for (int32 j = 0; j + 1 < PathB.Frames.Num(); ++j)
			{
				const FVector2D A0(PathA.Frames[i].Position.X, PathA.Frames[i].Position.Y);
				const FVector2D A1(PathA.Frames[i + 1].Position.X, PathA.Frames[i + 1].Position.Y);
				const FVector2D B0(PathB.Frames[j].Position.X, PathB.Frames[j].Position.Y);
				const FVector2D B1(PathB.Frames[j + 1].Position.X, PathB.Frames[j + 1].Position.Y);

				FVector2D IntersectionPoint;
				float AlphaA = 0.f, AlphaB = 0.f;
				if (!FlexGeometry2D::SegmentSegmentIntersection(A0, A1, B0, B1, IntersectionPoint, AlphaA, AlphaB))
				{
					continue;
				}

				const float CosAngle = FVector::DotProduct(PathA.Frames[i].Tangent.GetSafeNormal(), PathB.Frames[j].Tangent.GetSafeNormal());
				const float RawAngleDegrees = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(CosAngle, -1.f, 1.f)));
				// Undirected line angle: two tangents pointing nearly opposite ways (~180 degrees)
				// are just as parallel as two pointing nearly the same way (~0 degrees) -- our raw
				// paths have no consistent travel direction, so only the line angle is meaningful.
				const float UndirectedAngleDegrees = FMath::Min(RawAngleDegrees, 180.f - RawAngleDegrees);
				if (UndirectedAngleDegrees < CrossingAngleToleranceDegrees)
				{
					continue; // Near-parallel/tangential touch, not a real crossing.
				}

				PathA.CrossingCutIndices.AddUnique(i);
				PathB.CrossingCutIndices.AddUnique(j);
				return true;
			}
		}
		return false;
	}
}

FFlexRailGraph FFlexRailGraphBuilder::Build(
	const FFlexTrackGraph& TrackGraph,
	TArrayView<const FFlexTrackJunction> Junctions,
	const UFlexNetworkSubsystem& Network,
	float SampleStep,
	float MergeToleranceCm,
	float AngleToleranceDegrees,
	float CrossingAngleToleranceDegrees)
{
	TArray<FRawRailPath> RawPaths;

	// ---- Ordinary track: every rail segment's junction-trimmed span, offset into two rails. ----
	for (const FFlexTrackSegmentRef& Track : TrackGraph.Tracks)
	{
		const FFlexRoadSegment* Segment = Network.GetSegment(Track.SegmentId);
		if (!Segment || !Segment->Profile || !Segment->ArcLengthTable.IsValid())
		{
			continue;
		}
		const float SegmentLength = Segment->GetLength();

		float TrimStart = 0.f;
		FRailPathAnchor StartAnchor;
		StartAnchor.NodeId = Segment->StartNodeId;
		if (const FFlexTrackJunction* Junction = FindJunction(Junctions, Segment->StartNodeId))
		{
			const int32 PortIndex = FindPortIndex(*Junction, Track.SegmentId, false);
			if (PortIndex != INDEX_NONE)
			{
				TrimStart = Junction->Ports[PortIndex].TrimArcLength;
				StartAnchor.Kind = EAnchorKind::JunctionPort;
				StartAnchor.PortIndex = PortIndex;
			}
		}
		if (StartAnchor.Kind == EAnchorKind::None)
		{
			StartAnchor.Kind = EAnchorKind::OrdinaryEnd;
		}

		float TrimEnd = SegmentLength;
		FRailPathAnchor EndAnchor;
		EndAnchor.NodeId = Segment->EndNodeId;
		if (const FFlexTrackJunction* Junction = FindJunction(Junctions, Segment->EndNodeId))
		{
			const int32 PortIndex = FindPortIndex(*Junction, Track.SegmentId, true);
			if (PortIndex != INDEX_NONE)
			{
				TrimEnd = Junction->Ports[PortIndex].TrimArcLength;
				EndAnchor.Kind = EAnchorKind::JunctionPort;
				EndAnchor.PortIndex = PortIndex;
			}
		}
		if (EndAnchor.Kind == EAnchorKind::None)
		{
			EndAnchor.Kind = EAnchorKind::OrdinaryEnd;
		}

		if (TrimEnd - TrimStart < KINDA_SMALL_NUMBER)
		{
			continue; // Fully consumed by junction trims on both ends.
		}

		const FFlexRoadNode* StartNode = Network.GetNode(Segment->StartNodeId);
		const TArray<FFlexCurveFrame> Frames = FFlexRoadMeshBuilder::BuildFramesForRange(
			Segment->Curve, Segment->ArcLengthTable, StartNode ? StartNode->UpVector : FVector::UpVector,
			SampleStep, TrimStart, TrimEnd);
		AppendRailPairFromFrames(Frames, Track.Gauge, Track.RailWidth, StartAnchor, EndAnchor, RawPaths);
	}

	// ---- Junction movements: each solved movement between two ports offsets into two rails. ----
	for (const FFlexTrackJunction& Junction : Junctions)
	{
		for (const FFlexTrackMovement& Movement : Junction.Movements)
		{
			if (!Junction.Ports.IsValidIndex(Movement.FromPortIndex) || !Junction.Ports.IsValidIndex(Movement.ToPortIndex))
			{
				continue;
			}
			const FFlexTrackPort& FromPort = Junction.Ports[Movement.FromPortIndex];
			const FFlexArcLengthTable Table = FFlexBezierMath::BuildArcLengthTable(Movement.Centerline);
			if (!Table.IsValid())
			{
				continue;
			}
			const TArray<FFlexCurveFrame> Frames = FFlexRotationMinimizingFrame::ComputeFrames(Movement.Centerline, Table, SampleStep, FromPort.Up);

			FRailPathAnchor StartAnchor;
			StartAnchor.Kind = EAnchorKind::JunctionPort;
			StartAnchor.NodeId = Junction.NodeId;
			StartAnchor.PortIndex = Movement.FromPortIndex;

			FRailPathAnchor EndAnchor;
			EndAnchor.Kind = EAnchorKind::JunctionPort;
			EndAnchor.NodeId = Junction.NodeId;
			EndAnchor.PortIndex = Movement.ToPortIndex;

			AppendRailPairFromFrames(Frames, FromPort.Gauge, FromPort.RailWidth, StartAnchor, EndAnchor, RawPaths);
		}
	}

	// ---- Merge pass: exactly two same-side paths sharing one junction port fold their common leading span into one Shared edge. ----
	TMap<FAnchorSlotKey, TArray<FAnchorSlotEntry>> SlotsByKey;
	for (int32 Index = 0; Index < RawPaths.Num(); ++Index)
	{
		const FRawRailPath& Path = RawPaths[Index];
		if (Path.StartAnchor.Kind == EAnchorKind::JunctionPort)
		{
			SlotsByKey.FindOrAdd(FAnchorSlotKey{ Path.StartAnchor.NodeId, Path.StartAnchor.PortIndex, Path.bLeftRail }).Add(FAnchorSlotEntry{ Index, true });
		}
		if (Path.EndAnchor.Kind == EAnchorKind::JunctionPort)
		{
			SlotsByKey.FindOrAdd(FAnchorSlotKey{ Path.EndAnchor.NodeId, Path.EndAnchor.PortIndex, Path.bLeftRail }).Add(FAnchorSlotEntry{ Index, false });
		}
	}
	for (const TPair<FAnchorSlotKey, TArray<FAnchorSlotEntry>>& SlotPair : SlotsByKey)
	{
		const TArray<FAnchorSlotEntry>& Entries = SlotPair.Value;
		if (Entries.Num() != 2)
		{
			continue; // 0/1: nothing to merge. 3+: complex slip switch -- left unmerged, see class comment.
		}

		FRawRailPath& PathA = RawPaths[Entries[0].PathIndex];
		FRawRailPath& PathB = RawPaths[Entries[1].PathIndex];
		const TArray<FFlexCurveFrame> OrderedA = Entries[0].bAtPathStart ? PathA.Frames : ReversedCopy(PathA.Frames);
		const TArray<FFlexCurveFrame> OrderedB = Entries[1].bAtPathStart ? PathB.Frames : ReversedCopy(PathB.Frames);

		const int32 PrefixLength = FindCommonPrefixLength(OrderedA, OrderedB, MergeToleranceCm, AngleToleranceDegrees);
		if (PrefixLength < 2)
		{
			continue; // No meaningful shared span beyond the single coincident anchor point.
		}

		if (Entries[0].bAtPathStart)
		{
			PathA.MergeCutAtStart = PrefixLength - 1;
		}
		else
		{
			PathA.MergeCutAtEnd = PathA.Frames.Num() - PrefixLength;
		}
		PathA.MergePartnerIndices.Add(Entries[1].PathIndex);

		if (Entries[1].bAtPathStart)
		{
			PathB.MergeCutAtStart = PrefixLength - 1;
			PathB.bSuppressStartSpanEdge = true;
		}
		else
		{
			PathB.MergeCutAtEnd = PathB.Frames.Num() - PrefixLength;
			PathB.bSuppressEndSpanEdge = true;
		}
	}

	// ---- Crossing pass: paths at the same junction that don't share a port but geometrically cross are split independently. ----
	TMap<FFlexNodeId, TArray<int32>> PathIndicesByJunctionNode;
	for (int32 Index = 0; Index < RawPaths.Num(); ++Index)
	{
		const FRawRailPath& Path = RawPaths[Index];
		if (Path.StartAnchor.Kind == EAnchorKind::JunctionPort)
		{
			PathIndicesByJunctionNode.FindOrAdd(Path.StartAnchor.NodeId).AddUnique(Index);
		}
		if (Path.EndAnchor.Kind == EAnchorKind::JunctionPort)
		{
			PathIndicesByJunctionNode.FindOrAdd(Path.EndAnchor.NodeId).AddUnique(Index);
		}
	}
	for (const TPair<FFlexNodeId, TArray<int32>>& NodeGroup : PathIndicesByJunctionNode)
	{
		const TArray<int32>& Indices = NodeGroup.Value;
		for (int32 Ai = 0; Ai < Indices.Num(); ++Ai)
		{
			for (int32 Bi = Ai + 1; Bi < Indices.Num(); ++Bi)
			{
				FRawRailPath& PathA = RawPaths[Indices[Ai]];
				FRawRailPath& PathB = RawPaths[Indices[Bi]];
				const bool bSharesAnchor =
					PathA.StartAnchor.SharesJunctionPortWith(PathB.StartAnchor) ||
					PathA.StartAnchor.SharesJunctionPortWith(PathB.EndAnchor) ||
					PathA.EndAnchor.SharesJunctionPortWith(PathB.StartAnchor) ||
					PathA.EndAnchor.SharesJunctionPortWith(PathB.EndAnchor);
				if (bSharesAnchor)
				{
					continue; // Handled (or eligible to be handled) by the merge pass instead.
				}
				FindAndRecordCrossing(PathA, PathB, CrossingAngleToleranceDegrees);
			}
		}
	}

	// ---- Slice every raw path at its recorded cuts and emit the final graph. ----
	FFlexRailGraph Graph;
	TMap<FFlexNodeId, int32> OrdinaryNodeIndexByRoadNode;
	TMap<FJunctionPortKey, int32> JunctionPortNodeIndex;

	auto ResolveNodeIndex = [&Graph, &OrdinaryNodeIndexByRoadNode, &JunctionPortNodeIndex](const FRailPathAnchor& Anchor, const FVector& Position) -> int32
	{
		if (Anchor.Kind == EAnchorKind::JunctionPort)
		{
			const FJunctionPortKey Key{ Anchor.NodeId, Anchor.PortIndex };
			if (const int32* Existing = JunctionPortNodeIndex.Find(Key))
			{
				return *Existing;
			}
			FFlexRailNode NewNode;
			NewNode.Position = Position;
			const int32 NewIndex = Graph.Nodes.Add(NewNode);
			JunctionPortNodeIndex.Add(Key, NewIndex);
			return NewIndex;
		}
		if (Anchor.Kind == EAnchorKind::OrdinaryEnd)
		{
			if (const int32* Existing = OrdinaryNodeIndexByRoadNode.Find(Anchor.NodeId))
			{
				return *Existing;
			}
			FFlexRailNode NewNode;
			NewNode.Position = Position;
			const int32 NewIndex = Graph.Nodes.Add(NewNode);
			OrdinaryNodeIndexByRoadNode.Add(Anchor.NodeId, NewIndex);
			return NewIndex;
		}
		// An interior cut (merge remainder boundary or crossing point): always a fresh, unshared node.
		FFlexRailNode NewNode;
		NewNode.Position = Position;
		return Graph.Nodes.Add(NewNode);
	};

	for (int32 PathIndex = 0; PathIndex < RawPaths.Num(); ++PathIndex)
	{
		FRawRailPath& Path = RawPaths[PathIndex];
		if (Path.Frames.Num() < 2)
		{
			continue;
		}
		const int32 LastFrameIndex = Path.Frames.Num() - 1;

		// A path with only two frames (start and end, no interior sample) has no valid interior
		// index to cut at -- leave it as one whole span regardless of any recorded merge/crossing.
		TArray<int32> Cuts;
		if (LastFrameIndex >= 2)
		{
			if (Path.MergeCutAtStart != INDEX_NONE)
			{
				Cuts.AddUnique(FMath::Clamp(Path.MergeCutAtStart, 1, LastFrameIndex - 1));
			}
			if (Path.MergeCutAtEnd != INDEX_NONE)
			{
				Cuts.AddUnique(FMath::Clamp(Path.MergeCutAtEnd, 1, LastFrameIndex - 1));
			}
			for (int32 CrossingIndex : Path.CrossingCutIndices)
			{
				Cuts.AddUnique(FMath::Clamp(CrossingIndex, 1, LastFrameIndex - 1));
			}
			Cuts.Sort();
		}

		TArray<int32> Boundaries;
		Boundaries.Add(0);
		Boundaries.Append(Cuts);
		Boundaries.Add(LastFrameIndex);
		for (int32 i = Boundaries.Num() - 1; i > 0; --i)
		{
			if (Boundaries[i] == Boundaries[i - 1])
			{
				Boundaries.RemoveAt(i);
			}
		}

		for (int32 Span = 0; Span + 1 < Boundaries.Num(); ++Span)
		{
			const int32 First = Boundaries[Span];
			const int32 Last = Boundaries[Span + 1];
			const bool bIsStartSpan = (First == 0);
			const bool bIsEndSpan = (Last == LastFrameIndex);

			const bool bOwnsStartMerge = bIsStartSpan && Path.MergeCutAtStart != INDEX_NONE && !Path.bSuppressStartSpanEdge && Last <= Path.MergeCutAtStart;
			const bool bOwnsEndMerge = bIsEndSpan && Path.MergeCutAtEnd != INDEX_NONE && !Path.bSuppressEndSpanEdge && First >= Path.MergeCutAtEnd;

			const bool bIsSuppressedStart = bIsStartSpan && Path.bSuppressStartSpanEdge && Path.MergeCutAtStart != INDEX_NONE && Last <= Path.MergeCutAtStart;
			const bool bIsSuppressedEnd = bIsEndSpan && Path.bSuppressEndSpanEdge && Path.MergeCutAtEnd != INDEX_NONE && First >= Path.MergeCutAtEnd;
			if (bIsSuppressedStart || bIsSuppressedEnd)
			{
				continue; // Already emitted by the merge partner that owns this shared span.
			}

			FFlexRailEdge Edge;
			Edge.Frames.Append(&Path.Frames[First], Last - First + 1);
			Edge.SourcePathIndices.Add(PathIndex);
			Edge.bLeftRail = Path.bLeftRail;

			if (bOwnsStartMerge || bOwnsEndMerge)
			{
				Edge.Type = ERailEdgeType::Shared;
				Edge.SourcePathIndices.Append(Path.MergePartnerIndices);
			}
			else if (Path.CrossingCutIndices.Contains(First) || Path.CrossingCutIndices.Contains(Last))
			{
				Edge.Type = ERailEdgeType::Crossing;
			}
			else
			{
				Edge.Type = ERailEdgeType::Normal;
			}

			const FRailPathAnchor StartAnchorForNode = bIsStartSpan ? Path.StartAnchor : FRailPathAnchor{};
			const FRailPathAnchor EndAnchorForNode = bIsEndSpan ? Path.EndAnchor : FRailPathAnchor{};
			Edge.StartNode = ResolveNodeIndex(StartAnchorForNode, Path.Frames[First].Position);
			Edge.EndNode = ResolveNodeIndex(EndAnchorForNode, Path.Frames[Last].Position);

			const int32 EdgeIndex = Graph.Edges.Add(Edge);
			Graph.Nodes[Edge.StartNode].OutgoingEdges.Add(EdgeIndex);
			Graph.Nodes[Edge.EndNode].IncomingEdges.Add(EdgeIndex);
		}
	}

	return Graph;
}
