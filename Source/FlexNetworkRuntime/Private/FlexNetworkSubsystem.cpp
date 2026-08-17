#include "FlexNetworkSubsystem.h"
#include "FlexNetworkSettings.h"
#include "FlexNetworkMeshActor.h"
#include "Math/FlexBezierMath.h"
#include "Math/FlexGeometry2D.h"
#include "Mesh/FlexRoadMeshBuilder.h"
#include "Intersection/FlexIntersectionBuilder.h"
#include "Terrain/FlexLandscapeConformer.h"
#include "Async/ParallelFor.h"
#include "Engine/World.h"

namespace
{
	// Chooses tangent-handle Z so the curve's own Z-component naturally forms the desired ease
	// shape (see FlexNetworkSubsystem.h's class comment for why this needs no separate sampling
	// pass): a cubic Bezier's derivative at t=0 is proportional to (P1-P0), and at t=1 to
	// (P3-P2), so levelling a handle with its endpoint gives that end a zero vertical slope --
	// exactly a smoothstep-style ease -- for free.
	void ApplyElevationEase(FFlexRoadSegment& Segment)
	{
		const double Z0 = Segment.Curve.P0.Z;
		const double Z3 = Segment.Curve.P3.Z;

		switch (Segment.ElevationProfile.Ease)
		{
		case EFlexElevationEase::Linear:
			Segment.Curve.P1.Z = FMath::Lerp(Z0, Z3, 1.0 / 3.0);
			Segment.Curve.P2.Z = FMath::Lerp(Z0, Z3, 2.0 / 3.0);
			break;
		case EFlexElevationEase::EaseIn:
			Segment.Curve.P1.Z = Z0;
			Segment.Curve.P2.Z = FMath::Lerp(Z0, Z3, 2.0 / 3.0);
			break;
		case EFlexElevationEase::EaseOut:
			Segment.Curve.P1.Z = FMath::Lerp(Z0, Z3, 1.0 / 3.0);
			Segment.Curve.P2.Z = Z3;
			break;
		case EFlexElevationEase::EaseInOut:
		default:
			Segment.Curve.P1.Z = Z0;
			Segment.Curve.P2.Z = Z3;
			break;
		}
	}
}

void UFlexNetworkSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	TerrainConformer = MakeUnique<FFlexLandscapeConformer>();
}

void UFlexNetworkSubsystem::Deinitialize()
{
	if (MeshActor)
	{
		MeshActor->Destroy();
		MeshActor = nullptr;
	}
	TerrainConformer.Reset();
	Exporters.Reset();
	Super::Deinitialize();
}

const UFlexNetworkSettings* UFlexNetworkSubsystem::GetSettings() const
{
	return GetDefault<UFlexNetworkSettings>();
}

AFlexNetworkMeshActor* UFlexNetworkSubsystem::GetOrCreateMeshActor()
{
	if (!MeshActor)
	{
		if (UWorld* World = GetWorld())
		{
			FActorSpawnParameters Params;
			Params.ObjectFlags |= RF_Transient;
			MeshActor = World->SpawnActor<AFlexNetworkMeshActor>(Params);
		}
	}
	return MeshActor;
}

FVector2D UFlexNetworkSubsystem::NodeSegmentBoundsMin(const FFlexRoadSegment& Segment) const
{
	double MinX = Segment.Curve.P0.X, MinY = Segment.Curve.P0.Y;
	for (const FVector& P : { Segment.Curve.P1, Segment.Curve.P2, Segment.Curve.P3 })
	{
		MinX = FMath::Min(MinX, static_cast<double>(P.X));
		MinY = FMath::Min(MinY, static_cast<double>(P.Y));
	}
	return FVector2D(MinX, MinY);
}

FVector2D UFlexNetworkSubsystem::NodeSegmentBoundsMax(const FFlexRoadSegment& Segment) const
{
	double MaxX = Segment.Curve.P0.X, MaxY = Segment.Curve.P0.Y;
	for (const FVector& P : { Segment.Curve.P1, Segment.Curve.P2, Segment.Curve.P3 })
	{
		MaxX = FMath::Max(MaxX, static_cast<double>(P.X));
		MaxY = FMath::Max(MaxY, static_cast<double>(P.Y));
	}
	return FVector2D(MaxX, MaxY);
}

void UFlexNetworkSubsystem::AddSegmentToSpatialGrid(FFlexSegmentId Id, const FFlexRoadSegment& Segment)
{
	SpatialGrid.AddSegment(Id, NodeSegmentBoundsMin(Segment), NodeSegmentBoundsMax(Segment));
}

void UFlexNetworkSubsystem::RemoveSegmentFromSpatialGrid(FFlexSegmentId Id, const FFlexRoadSegment& Segment)
{
	SpatialGrid.RemoveSegment(Id, NodeSegmentBoundsMin(Segment), NodeSegmentBoundsMax(Segment));
}

void UFlexNetworkSubsystem::DetachSegmentFromNode(FFlexNodeId NodeId, FFlexSegmentId SegmentId)
{
	if (FFlexRoadNode* Node = Nodes.Find(NodeId))
	{
		Node->ConnectedSegments.RemoveSingleSwap(SegmentId);
	}
}

// ---------------------------------------------------------------- Mutation API

FFlexNodeId UFlexNetworkSubsystem::AddNode(const FVector& Position, EFlexRoadElevationType ElevationType, const FVector& UpVector)
{
	const FFlexNodeId Id = NodeIdAllocator.Allocate();

	FFlexRoadNode Node;
	Node.Position = Position;
	Node.UpVector = UpVector.GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
	Node.ElevationType = ElevationType;
	Node.RoleFlags = static_cast<uint8>(EFlexNodeRole::Endpoint);
	Nodes.Add(Id, Node);

	SpatialGrid.AddNode(Id, FVector2D(Position.X, Position.Y));
	return Id;
}

FFlexSegmentId UFlexNetworkSubsystem::AddSegment(FFlexNodeId StartNodeId, FFlexNodeId EndNodeId, const FVector& StartTangentHandle, const FVector& EndTangentHandle, URoadTypeProfile* Profile, EFlexRoadElevationType ElevationType)
{
	FFlexRoadNode* StartNode = Nodes.Find(StartNodeId);
	FFlexRoadNode* EndNode = Nodes.Find(EndNodeId);
	if (!StartNode || !EndNode || !Profile)
	{
		return FFlexSegmentId::Invalid();
	}

	FFlexRoadSegment Segment;
	Segment.StartNodeId = StartNodeId;
	Segment.EndNodeId = EndNodeId;
	Segment.Curve.P0 = StartNode->Position;
	Segment.Curve.P1 = StartTangentHandle;
	Segment.Curve.P2 = EndTangentHandle;
	Segment.Curve.P3 = EndNode->Position;
	Segment.Profile = Profile;
	Segment.ElevationType = ElevationType;
	Segment.bDirty = true;
	ApplyElevationEase(Segment);

	const FFlexSegmentId Id = SegmentIdAllocator.Allocate();
	AddSegmentToSpatialGrid(Id, Segment);
	Segments.Add(Id, Segment);

	StartNode->ConnectedSegments.Add(Id);
	EndNode->ConnectedSegments.Add(Id);

	DirtySegments.Add(Id);
	DirtyNodes.Add(StartNodeId);
	DirtyNodes.Add(EndNodeId);
	RebuildDirty();
	return Id;
}

bool UFlexNetworkSubsystem::RemoveSegment(FFlexSegmentId SegmentId)
{
	FFlexRoadSegment* Segment = Segments.Find(SegmentId);
	if (!Segment)
	{
		return false;
	}

	RemoveSegmentFromSpatialGrid(SegmentId, *Segment);
	if (TerrainConformer)
	{
		TerrainConformer->RemoveSegmentConforming(GetWorld(), SegmentId);
	}
	if (MeshActor)
	{
		MeshActor->RemoveSegmentMesh(SegmentId);
	}

	const FFlexNodeId StartId = Segment->StartNodeId;
	const FFlexNodeId EndId = Segment->EndNodeId;

	DetachSegmentFromNode(StartId, SegmentId);
	DetachSegmentFromNode(EndId, SegmentId);

	Segments.Remove(SegmentId);
	SegmentIdAllocator.Free(SegmentId);
	DirtySegments.Remove(SegmentId);

	DirtyNodes.Add(StartId);
	DirtyNodes.Add(EndId);
	RebuildDirty();
	return true;
}

bool UFlexNetworkSubsystem::RemoveNode(FFlexNodeId NodeId)
{
	FFlexRoadNode* Node = Nodes.Find(NodeId);
	if (!Node)
	{
		return false;
	}

	// Copy first: RemoveSegment mutates this node's ConnectedSegments array as it detaches each one.
	const TArray<FFlexSegmentId> Connected = Node->ConnectedSegments;
	for (FFlexSegmentId SegId : Connected)
	{
		RemoveSegment(SegId);
	}

	// Node may have been freed by a reentrant call if this ever becomes recursive; re-find defensively.
	if (FFlexRoadNode* StillThere = Nodes.Find(NodeId))
	{
		SpatialGrid.RemoveNode(NodeId, FVector2D(StillThere->Position.X, StillThere->Position.Y));
	}

	if (JunctionDataByNode.Contains(NodeId))
	{
		if (MeshActor)
		{
			MeshActor->RemoveJunctionMesh(NodeId);
		}
		JunctionDataByNode.Remove(NodeId);
	}

	Nodes.Remove(NodeId);
	NodeIdAllocator.Free(NodeId);
	DirtyNodes.Remove(NodeId);
	return true;
}

bool UFlexNetworkSubsystem::SetNodePosition(FFlexNodeId NodeId, const FVector& NewPosition)
{
	FFlexRoadNode* Node = Nodes.Find(NodeId);
	if (!Node)
	{
		return false;
	}

	const FVector2D OldPos2D(Node->Position.X, Node->Position.Y);
	Node->Position = NewPosition;
	SpatialGrid.UpdateNode(NodeId, OldPos2D, FVector2D(NewPosition.X, NewPosition.Y));

	for (FFlexSegmentId SegId : Node->ConnectedSegments)
	{
		FFlexRoadSegment* Segment = Segments.Find(SegId);
		if (!Segment)
		{
			continue;
		}

		RemoveSegmentFromSpatialGrid(SegId, *Segment);

		// Keep the tangent handle's planform (X/Y) offset from its endpoint so the curve's shape
		// moves rigidly with the node; Z is re-derived by ApplyElevationEase below rather than
		// preserved, since it depends on both endpoints' (possibly just-changed) heights.
		if (Segment->StartNodeId == NodeId)
		{
			const FVector Offset = Segment->Curve.P1 - Segment->Curve.P0;
			Segment->Curve.P0 = NewPosition;
			Segment->Curve.P1 = FVector(NewPosition.X + Offset.X, NewPosition.Y + Offset.Y, Segment->Curve.P1.Z);
		}
		if (Segment->EndNodeId == NodeId)
		{
			const FVector Offset = Segment->Curve.P2 - Segment->Curve.P3;
			Segment->Curve.P3 = NewPosition;
			Segment->Curve.P2 = FVector(NewPosition.X + Offset.X, NewPosition.Y + Offset.Y, Segment->Curve.P2.Z);
		}
		ApplyElevationEase(*Segment);

		AddSegmentToSpatialGrid(SegId, *Segment);
		DirtySegments.Add(SegId);
	}

	DirtyNodes.Add(NodeId);
	RebuildDirty();
	return true;
}

bool UFlexNetworkSubsystem::SetNodeElevationType(FFlexNodeId NodeId, EFlexRoadElevationType NewType)
{
	FFlexRoadNode* Node = Nodes.Find(NodeId);
	if (!Node)
	{
		return false;
	}
	Node->ElevationType = NewType;
	DirtyNodes.Add(NodeId);
	RebuildDirty();
	return true;
}

bool UFlexNetworkSubsystem::SetSegmentProfile(FFlexSegmentId SegmentId, URoadTypeProfile* NewProfile)
{
	FFlexRoadSegment* Segment = Segments.Find(SegmentId);
	if (!Segment || !NewProfile)
	{
		return false;
	}
	Segment->Profile = NewProfile;
	DirtySegments.Add(SegmentId);
	DirtyNodes.Add(Segment->StartNodeId);
	DirtyNodes.Add(Segment->EndNodeId);
	RebuildDirty();
	return true;
}

bool UFlexNetworkSubsystem::SetSegmentCurve(FFlexSegmentId SegmentId, const FVector& StartTangentHandle, const FVector& EndTangentHandle)
{
	FFlexRoadSegment* Segment = Segments.Find(SegmentId);
	if (!Segment)
	{
		return false;
	}

	RemoveSegmentFromSpatialGrid(SegmentId, *Segment);
	Segment->Curve.P1 = StartTangentHandle;
	Segment->Curve.P2 = EndTangentHandle;
	ApplyElevationEase(*Segment);
	AddSegmentToSpatialGrid(SegmentId, *Segment);

	DirtySegments.Add(SegmentId);
	DirtyNodes.Add(Segment->StartNodeId);
	DirtyNodes.Add(Segment->EndNodeId);
	RebuildDirty();
	return true;
}

FFlexNodeId UFlexNetworkSubsystem::SplitSegment(FFlexSegmentId SegmentId, float ArcLength)
{
	FFlexRoadSegment* Segment = Segments.Find(SegmentId);
	if (!Segment || !Segment->ArcLengthTable.IsValid())
	{
		return FFlexNodeId::Invalid();
	}

	const float ClampedArcLength = FMath::Clamp(ArcLength, 0.f, Segment->GetLength());
	const float T = FFlexBezierMath::ArcLengthToT(Segment->ArcLengthTable, ClampedArcLength);

	FFlexBezierCurve Left, Right;
	FFlexBezierMath::Subdivide(Segment->Curve, T, Left, Right);

	const FFlexNodeId OldStartId = Segment->StartNodeId;
	const FFlexNodeId OldEndId = Segment->EndNodeId;
	URoadTypeProfile* Profile = Segment->Profile;
	const EFlexRoadElevationType ElevationType = Segment->ElevationType;
	const FFlexElevationProfile ElevationProfile = Segment->ElevationProfile;
	const FVector SplitUp = Nodes.Contains(OldStartId) ? Nodes.FindChecked(OldStartId).UpVector : FVector::UpVector;

	const FFlexNodeId NewNodeId = AddNode(Left.P3, ElevationType, SplitUp);

	DetachSegmentFromNode(OldEndId, SegmentId);
	RemoveSegmentFromSpatialGrid(SegmentId, *Segment);

	Segment->EndNodeId = NewNodeId;
	Segment->Curve = Left;
	Segment->bDirty = true;
	AddSegmentToSpatialGrid(SegmentId, *Segment);
	if (FFlexRoadNode* NewNode = Nodes.Find(NewNodeId))
	{
		NewNode->ConnectedSegments.Add(SegmentId);
	}

	FFlexRoadSegment NewSegment;
	NewSegment.StartNodeId = NewNodeId;
	NewSegment.EndNodeId = OldEndId;
	NewSegment.Curve = Right;
	NewSegment.Profile = Profile;
	NewSegment.ElevationType = ElevationType;
	NewSegment.ElevationProfile = ElevationProfile;
	NewSegment.bDirty = true;

	const FFlexSegmentId NewSegmentId = SegmentIdAllocator.Allocate();
	AddSegmentToSpatialGrid(NewSegmentId, NewSegment);
	Segments.Add(NewSegmentId, NewSegment);

	if (FFlexRoadNode* NewNode = Nodes.Find(NewNodeId))
	{
		NewNode->ConnectedSegments.Add(NewSegmentId);
	}
	if (FFlexRoadNode* EndNode = Nodes.Find(OldEndId))
	{
		EndNode->ConnectedSegments.Add(NewSegmentId);
	}

	DirtySegments.Add(SegmentId);
	DirtySegments.Add(NewSegmentId);
	DirtyNodes.Add(OldStartId);
	DirtyNodes.Add(NewNodeId);
	DirtyNodes.Add(OldEndId);
	RebuildDirty();

	return NewNodeId;
}

// ---------------------------------------------------------------- Terrain (bulk editor commands)

void UFlexNetworkSubsystem::ConformAllTerrainToRoads()
{
	if (!TerrainConformer)
	{
		return;
	}
	// Reuses RebuildDirty()'s existing per-segment conforming (and its Ground-only guard) rather
	// than duplicating the frame-building/conforming call here -- marking everything dirty and
	// letting the normal rebuild path run is simpler and can't drift out of sync with it.
	for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Segments)
	{
		DirtySegments.Add(Pair.Key);
	}
	RebuildDirty();
}

void UFlexNetworkSubsystem::FitNodesToTerrain()
{
	if (!TerrainConformer)
	{
		return;
	}

	UWorld* World = GetWorld();
	BeginBatchUpdate();
	for (TPair<FFlexNodeId, FFlexRoadNode>& Pair : Nodes)
	{
		if (Pair.Value.ElevationType != EFlexRoadElevationType::Ground)
		{
			continue;
		}
		float SampledHeight = 0.f;
		if (TerrainConformer->SampleHeight(World, FVector2D(Pair.Value.Position.X, Pair.Value.Position.Y), SampledHeight))
		{
			SetNodePosition(Pair.Key, FVector(Pair.Value.Position.X, Pair.Value.Position.Y, SampledHeight));
		}
	}
	EndBatchUpdate();
}

// ---------------------------------------------------------------- Snap / crossing queries

bool UFlexNetworkSubsystem::FindNearestNode(const FVector& WorldPosition, float Radius, FFlexNodeId& OutNodeId) const
{
	const FVector2D Pos2D(WorldPosition.X, WorldPosition.Y);
	const TArray<FFlexNodeId> Candidates = SpatialGrid.QueryNodesNear(Pos2D, Radius);

	float BestDistSq = FMath::Square(Radius);
	bool bFound = false;
	for (FFlexNodeId Id : Candidates)
	{
		const FFlexRoadNode* Node = Nodes.Find(Id);
		if (!Node)
		{
			continue;
		}
		const float DistSq = FVector2D::DistSquared(Pos2D, FVector2D(Node->Position.X, Node->Position.Y));
		if (DistSq <= BestDistSq)
		{
			BestDistSq = DistSq;
			OutNodeId = Id;
			bFound = true;
		}
	}
	return bFound;
}

bool UFlexNetworkSubsystem::FindNearestSegmentPoint(const FVector& WorldPosition, float Radius, FFlexSegmentId& OutSegmentId, float& OutArcLength, FVector& OutPointOnCurve) const
{
	const FVector2D Pos2D(WorldPosition.X, WorldPosition.Y);
	const TArray<FFlexSegmentId> Candidates = SpatialGrid.QuerySegmentsNear(Pos2D, Radius);

	float BestDistSq = FMath::Square(Radius);
	bool bFound = false;

	for (FFlexSegmentId Id : Candidates)
	{
		const FFlexRoadSegment* Segment = Segments.Find(Id);
		if (!Segment || !Segment->ArcLengthTable.IsValid())
		{
			continue;
		}

		for (const FFlexArcLengthSample& Sample : Segment->ArcLengthTable.Samples)
		{
			const FVector P = FFlexBezierMath::Evaluate(Segment->Curve, Sample.T);
			const float DistSq = FVector2D::DistSquared(Pos2D, FVector2D(P.X, P.Y));
			if (DistSq <= BestDistSq)
			{
				BestDistSq = DistSq;
				OutSegmentId = Id;
				OutArcLength = Sample.ArcLength;
				OutPointOnCurve = P;
				bFound = true;
			}
		}
	}
	return bFound;
}

TArray<FFlexSegmentCrossing> UFlexNetworkSubsystem::FindCrossings(const FFlexBezierCurve& ProposedCurve) const
{
	TArray<FFlexSegmentCrossing> Result;

	FVector BoundsMin = ProposedCurve.P0;
	FVector BoundsMax = ProposedCurve.P0;
	for (const FVector& P : { ProposedCurve.P1, ProposedCurve.P2, ProposedCurve.P3 })
	{
		BoundsMin = BoundsMin.ComponentMin(P);
		BoundsMax = BoundsMax.ComponentMax(P);
	}
	const FVector2D Center2D((BoundsMin.X + BoundsMax.X) * 0.5, (BoundsMin.Y + BoundsMax.Y) * 0.5);
	const float SearchRadius = static_cast<float>(FVector2D::Distance(FVector2D(BoundsMin.X, BoundsMin.Y), FVector2D(BoundsMax.X, BoundsMax.Y)) * 0.5 + 1.0);

	const TArray<FFlexSegmentId> Candidates = SpatialGrid.QuerySegmentsNear(Center2D, SearchRadius);

	const FFlexArcLengthTable ProposedTable = FFlexBezierMath::BuildArcLengthTable(ProposedCurve);
	TArray<FVector> ProposedPolyline;
	ProposedPolyline.Reserve(ProposedTable.Samples.Num());
	for (const FFlexArcLengthSample& Sample : ProposedTable.Samples)
	{
		ProposedPolyline.Add(FFlexBezierMath::Evaluate(ProposedCurve, Sample.T));
	}

	for (FFlexSegmentId Id : Candidates)
	{
		const FFlexRoadSegment* Segment = Segments.Find(Id);
		if (!Segment || !Segment->ArcLengthTable.IsValid())
		{
			continue;
		}

		for (int32 i = 0; i + 1 < ProposedPolyline.Num(); ++i)
		{
			for (int32 j = 0; j + 1 < Segment->ArcLengthTable.Samples.Num(); ++j)
			{
				const FVector ExistingA = FFlexBezierMath::Evaluate(Segment->Curve, Segment->ArcLengthTable.Samples[j].T);
				const FVector ExistingB = FFlexBezierMath::Evaluate(Segment->Curve, Segment->ArcLengthTable.Samples[j + 1].T);

				FVector2D CrossPoint;
				float AlphaProposed = 0.f, AlphaExisting = 0.f;
				if (FlexGeometry2D::SegmentSegmentIntersection(
					FVector2D(ProposedPolyline[i].X, ProposedPolyline[i].Y), FVector2D(ProposedPolyline[i + 1].X, ProposedPolyline[i + 1].Y),
					FVector2D(ExistingA.X, ExistingA.Y), FVector2D(ExistingB.X, ExistingB.Y),
					CrossPoint, AlphaProposed, AlphaExisting))
				{
					FFlexSegmentCrossing Crossing;
					Crossing.ExistingSegmentId = Id;
					Crossing.ArcLengthOnExistingSegment = FMath::Lerp(Segment->ArcLengthTable.Samples[j].ArcLength, Segment->ArcLengthTable.Samples[j + 1].ArcLength, AlphaExisting);
					Crossing.ArcLengthOnProposedCurve = FMath::Lerp(ProposedTable.Samples[i].ArcLength, ProposedTable.Samples[i + 1].ArcLength, AlphaProposed);
					Crossing.WorldPoint = FVector(CrossPoint.X, CrossPoint.Y, FMath::Lerp(ExistingA.Z, ExistingB.Z, AlphaExisting));
					Result.Add(Crossing);
				}
			}
		}
	}

	// Two polyline sub-segments that share a vertex very close to the true crossing point can
	// each independently register a hit there (once per curve, so up to 4 near-identical entries
	// for what is geometrically one crossing) -- collapse those down to one per existing segment.
	TArray<FFlexSegmentCrossing> Deduped;
	Deduped.Reserve(Result.Num());
	for (const FFlexSegmentCrossing& Candidate : Result)
	{
		const bool bAlreadyHave = Deduped.ContainsByPredicate([&Candidate](const FFlexSegmentCrossing& Existing)
		{
			return Existing.ExistingSegmentId == Candidate.ExistingSegmentId
				&& FVector::DistSquared(Existing.WorldPoint, Candidate.WorldPoint) < FMath::Square(10.f);
		});
		if (!bAlreadyHave)
		{
			Deduped.Add(Candidate);
		}
	}

	return Deduped;
}

FVector UFlexNetworkSubsystem::SuggestOutgoingTangentDirection(FFlexNodeId NodeId) const
{
	const FFlexRoadNode* Node = Nodes.Find(NodeId);
	if (!Node || Node->ConnectedSegments.Num() == 0)
	{
		return FVector::ForwardVector;
	}

	FVector Accum = FVector::ZeroVector;
	for (FFlexSegmentId Id : Node->ConnectedSegments)
	{
		const FFlexRoadSegment* Segment = Segments.Find(Id);
		if (!Segment)
		{
			continue;
		}
		const bool bIsEnd = Segment->EndNodeId == NodeId;
		// Direction of travel arriving at this node, whichever end of the segment it is.
		const FVector InwardTangent = bIsEnd
			? FFlexBezierMath::EvaluateDerivative(Segment->Curve, 1.f)
			: -FFlexBezierMath::EvaluateDerivative(Segment->Curve, 0.f);
		Accum += InwardTangent.GetSafeNormal();
	}
	return Accum.GetSafeNormal(UE_SMALL_NUMBER, FVector::ForwardVector);
}

bool UFlexNetworkSubsystem::ValidateProposedSegment(const FFlexBezierCurve& Curve, const URoadTypeProfile* Profile, FText& OutReason) const
{
	const UFlexNetworkSettings* Settings = GetSettings();
	const float MinLength = (Profile && Profile->MinSegmentLengthOverride > 0.f) ? Profile->MinSegmentLengthOverride : Settings->MinSegmentLength;
	const float MinRadius = Profile ? Profile->MinTurnRadius : 0.f;

	const FFlexArcLengthTable Table = FFlexBezierMath::BuildArcLengthTable(Curve, Settings->ArcLengthChordTolerance, Settings->MaxArcLengthSubdivisionDepth);
	const float Length = Table.GetTotalLength();

	if (Length < MinLength)
	{
		OutReason = FText::Format(NSLOCTEXT("FlexNetwork", "TooShort", "Segment is too short ({0} cm, minimum {1} cm)."), FText::AsNumber(Length), FText::AsNumber(MinLength));
		return false;
	}

	if (MinRadius > 0.f)
	{
		for (const FFlexArcLengthSample& Sample : Table.Samples)
		{
			const float Curvature = FFlexBezierMath::EstimateCurvature(Curve, Sample.T);
			if (Curvature > KINDA_SMALL_NUMBER && (1.f / Curvature) < MinRadius)
			{
				OutReason = NSLOCTEXT("FlexNetwork", "TooSharp", "Curve is tighter than this road type's minimum turn radius.");
				return false;
			}
		}
	}

	TArray<FVector> Polyline;
	Polyline.Reserve(Table.Samples.Num());
	for (const FFlexArcLengthSample& Sample : Table.Samples)
	{
		Polyline.Add(FFlexBezierMath::Evaluate(Curve, Sample.T));
	}

	for (int32 i = 0; i + 1 < Polyline.Num(); ++i)
	{
		for (int32 j = i + 2; j + 1 < Polyline.Num(); ++j)
		{
			FVector2D CrossPoint;
			float AlphaA = 0.f, AlphaB = 0.f;
			if (FlexGeometry2D::SegmentSegmentIntersection(
				FVector2D(Polyline[i].X, Polyline[i].Y), FVector2D(Polyline[i + 1].X, Polyline[i + 1].Y),
				FVector2D(Polyline[j].X, Polyline[j].Y), FVector2D(Polyline[j + 1].X, Polyline[j + 1].Y),
				CrossPoint, AlphaA, AlphaB))
			{
				OutReason = NSLOCTEXT("FlexNetwork", "SelfIntersecting", "Curve self-intersects.");
				return false;
			}
		}
	}

	OutReason = FText::GetEmpty();
	return true;
}

// ---------------------------------------------------------------- Query API

TArray<FFlexSegmentId> UFlexNetworkSubsystem::GetConnectedSegments(FFlexNodeId NodeId) const
{
	if (const FFlexRoadNode* Node = Nodes.Find(NodeId))
	{
		return Node->ConnectedSegments;
	}
	return {};
}

TArray<FFlexLaneConnector> UFlexNetworkSubsystem::GetLaneConnectorsAtNode(FFlexNodeId NodeId) const
{
	if (const FFlexJunctionData* Data = JunctionDataByNode.Find(NodeId))
	{
		return Data->LaneConnectors;
	}
	return {};
}

FFlexCurveFrame UFlexNetworkSubsystem::SampleSegmentAtArcLength(FFlexSegmentId SegmentId, float ArcLength) const
{
	const FFlexRoadSegment* Segment = Segments.Find(SegmentId);
	if (!Segment)
	{
		return FFlexCurveFrame();
	}
	const FVector RefUp = Nodes.Contains(Segment->StartNodeId) ? Nodes.FindChecked(Segment->StartNodeId).UpVector : FVector::UpVector;
	return FFlexRoadMeshBuilder::SampleFrameAtArcLength(Segment->Curve, Segment->ArcLengthTable, ArcLength, RefUp);
}

// ---------------------------------------------------------------- Extension points

void UFlexNetworkSubsystem::RegisterExporter(TSharedPtr<IFlexNetworkExporter> Exporter)
{
	if (Exporter)
	{
		Exporters.AddUnique(Exporter);
		Exporter->ExportFullNetwork(*this);
	}
}

void UFlexNetworkSubsystem::UnregisterExporter(const TSharedPtr<IFlexNetworkExporter>& Exporter)
{
	Exporters.Remove(Exporter);
}

// ---------------------------------------------------------------- Rebuild pipeline

TArray<FFlexJunctionApproachInput> UFlexNetworkSubsystem::BuildApproachInputs(FFlexNodeId NodeId) const
{
	TArray<FFlexJunctionApproachInput> Result;
	const FFlexRoadNode* Node = Nodes.Find(NodeId);
	if (!Node)
	{
		return Result;
	}

	for (FFlexSegmentId SegId : Node->ConnectedSegments)
	{
		const FFlexRoadSegment* Segment = Segments.Find(SegId);
		if (!Segment)
		{
			continue;
		}
		FFlexJunctionApproachInput Input;
		Input.SegmentId = SegId;
		Input.bNodeIsSegmentEnd = (Segment->EndNodeId == NodeId);
		Input.Curve = Segment->Curve;
		Input.ArcLengthTable = Segment->ArcLengthTable;
		Input.Profile = Segment->Profile;
		Result.Add(Input);
	}
	return Result;
}

void UFlexNetworkSubsystem::BeginBatchUpdate()
{
	++BatchDepth;
}

void UFlexNetworkSubsystem::EndBatchUpdate()
{
	if (!ensure(BatchDepth > 0))
	{
		return;
	}
	if (--BatchDepth == 0)
	{
		RebuildDirty();
	}
}

void UFlexNetworkSubsystem::RebuildDirty()
{
	if (BatchDepth > 0)
	{
		return; // Accumulate only -- EndBatchUpdate will trigger the real (combined) rebuild.
	}

	if (DirtySegments.Num() == 0 && DirtyNodes.Num() == 0)
	{
		return;
	}

	const UFlexNetworkSettings* Settings = GetSettings();

	// 1. Affected nodes = explicitly dirty nodes + endpoints of every dirty segment.
	TSet<FFlexNodeId> AffectedNodes = DirtyNodes;
	for (FFlexSegmentId SegId : DirtySegments)
	{
		if (const FFlexRoadSegment* Segment = Segments.Find(SegId))
		{
			AffectedNodes.Add(Segment->StartNodeId);
			AffectedNodes.Add(Segment->EndNodeId);
		}
	}

	// 2. A junction rebuild at a shared node can change every connected segment's trim distance,
	// not just the one that was directly edited -- so the final rebuild set is every segment
	// touching an affected node, not just the explicitly dirty ones. This (rather than a full
	// network rebuild) is what makes the rebuild "incremental": only segments/junctions actually
	// adjacent to an edit are touched.
	TSet<FFlexSegmentId> FinalDirtySegments = DirtySegments;
	for (FFlexNodeId NodeId : AffectedNodes)
	{
		if (const FFlexRoadNode* Node = Nodes.Find(NodeId))
		{
			for (FFlexSegmentId SegId : Node->ConnectedSegments)
			{
				FinalDirtySegments.Add(SegId);
			}
		}
	}

	// 3. Arc-length tables first -- junction building below needs every approach's up-to-date table.
	for (FFlexSegmentId SegId : FinalDirtySegments)
	{
		if (FFlexRoadSegment* Segment = Segments.Find(SegId))
		{
			Segment->ArcLengthTable = FFlexBezierMath::BuildArcLengthTable(Segment->Curve, Settings->ArcLengthChordTolerance, Settings->MaxArcLengthSubdivisionDepth);
			Segment->bDirty = false;
		}
	}

	TMap<FFlexSegmentId, TPair<float, float>> TrimRangeBySegment;
	for (FFlexSegmentId SegId : FinalDirtySegments)
	{
		if (const FFlexRoadSegment* Segment = Segments.Find(SegId))
		{
			TrimRangeBySegment.Add(SegId, TPair<float, float>(0.f, Segment->GetLength()));
		}
	}

	// 4. Node roles + junction polygons/lane-connectors.
	for (FFlexNodeId NodeId : AffectedNodes)
	{
		FFlexRoadNode* Node = Nodes.Find(NodeId);
		if (!Node)
		{
			continue;
		}

		const TArray<FFlexJunctionApproachInput> Approaches = BuildApproachInputs(NodeId);

		uint8 NewRoleFlags = static_cast<uint8>(EFlexNodeRole::None);
		if (Approaches.Num() == 1)
		{
			NewRoleFlags |= static_cast<uint8>(EFlexNodeRole::Endpoint);
		}
		else if (Approaches.Num() >= 2)
		{
			NewRoleFlags |= static_cast<uint8>(EFlexNodeRole::Bend);
		}

		TSet<EFlexRoadElevationType> ElevationTypesAtNode;
		for (FFlexSegmentId SegId : Node->ConnectedSegments)
		{
			if (const FFlexRoadSegment* Seg = Segments.Find(SegId))
			{
				ElevationTypesAtNode.Add(Seg->ElevationType);
			}
		}
		if (ElevationTypesAtNode.Num() > 1)
		{
			NewRoleFlags |= static_cast<uint8>(EFlexNodeRole::ElevationTransition);
		}

		if (FFlexIntersectionBuilder::NeedsJunction(Approaches))
		{
			NewRoleFlags |= static_cast<uint8>(EFlexNodeRole::Junction);
			NewRoleFlags &= ~static_cast<uint8>(EFlexNodeRole::Bend);

			const float FilletRadius = Node->FilletRadiusOverride > 0.f ? Node->FilletRadiusOverride : Settings->DefaultFilletRadius;
			FFlexJunctionData JunctionData = FFlexIntersectionBuilder::BuildJunction(Node->Position, Node->UpVector, Approaches, FilletRadius, Settings->CrosswalkWidth, Settings->CrosswalkMinClearance, Settings->CurbReturnRadius, Settings->ParallelApproachAngleToleranceDegrees, 8, Settings->CurbReturnArcSegments);

			for (const TPair<FFlexSegmentId, float>& TrimPair : JunctionData.TrimArcLengthBySegment)
			{
				if (const FFlexRoadSegment* Segment = Segments.Find(TrimPair.Key))
				{
					TPair<float, float>& Range = TrimRangeBySegment.FindOrAdd(TrimPair.Key, TPair<float, float>(0.f, Segment->GetLength()));
					if (Segment->StartNodeId == NodeId)
					{
						Range.Key = TrimPair.Value;
					}
					if (Segment->EndNodeId == NodeId)
					{
						Range.Value = TrimPair.Value;
					}
				}
			}

			JunctionDataByNode.Add(NodeId, MoveTemp(JunctionData));
		}
		else
		{
			JunctionDataByNode.Remove(NodeId);
			if (MeshActor)
			{
				MeshActor->RemoveJunctionMesh(NodeId);
			}
		}

		Node->RoleFlags = NewRoleFlags;
	}

	// 5. Build + apply segment meshes. Building is pure per-segment work (no shared mutable
	// state), so it's safe to parallelize across worker threads once the batch is big enough
	// that ParallelFor's dispatch overhead is worth it; applying to components/terrain happens
	// back on the game thread afterward, same as UProceduralMeshComponent/Landscape require.
	TArray<FFlexSegmentId> SegmentIdsToRebuild = FinalDirtySegments.Array();
	TArray<FFlexSegmentMeshResult> MeshResults;
	MeshResults.SetNum(SegmentIdsToRebuild.Num());

	const bool bParallel = SegmentIdsToRebuild.Num() >= Settings->ParallelRebuildThreshold;
	ParallelFor(SegmentIdsToRebuild.Num(), [this, &SegmentIdsToRebuild, &MeshResults, &TrimRangeBySegment, Settings](int32 Index)
	{
		const FFlexSegmentId SegId = SegmentIdsToRebuild[Index];
		const FFlexRoadSegment* Segment = Segments.Find(SegId);
		if (!Segment || !Segment->Profile)
		{
			return;
		}

		const FVector RefUp = Nodes.Contains(Segment->StartNodeId) ? Nodes.FindChecked(Segment->StartNodeId).UpVector : FVector::UpVector;
		const TPair<float, float>* Range = TrimRangeBySegment.Find(SegId);
		const float TrimStart = Range ? Range->Key : 0.f;
		const float TrimEnd = Range ? Range->Value : Segment->GetLength();

		MeshResults[Index] = FFlexRoadMeshBuilder::BuildSegmentMesh(Segment->Curve, Segment->ArcLengthTable, Segment->Profile, RefUp, Settings->ArcLengthSampleStep, TrimStart, TrimEnd);
	}, bParallel ? EParallelForFlags::None : EParallelForFlags::ForceSingleThread);

	AFlexNetworkMeshActor* Actor = GetOrCreateMeshActor();
	UWorld* World = GetWorld();
	for (int32 Index = 0; Index < SegmentIdsToRebuild.Num(); ++Index)
	{
		const FFlexSegmentId SegId = SegmentIdsToRebuild[Index];
		const FFlexRoadSegment* Segment = Segments.Find(SegId);
		if (!Segment)
		{
			continue;
		}

		if (Actor)
		{
			Actor->ApplySegmentMesh(SegId, MeshResults[Index]);
		}

		// Only Ground segments get the terrain flattened to their height -- a Bridge/Elevated
		// segment sits above the terrain by design (flattening under it would visually merge the
		// deck into the ground, defeating the point), and Tunnel/Ramp don't have a sensible single
		// "flatten to this height" interpretation either.
		if (TerrainConformer && Segment->Profile && Segment->ElevationType == EFlexRoadElevationType::Ground)
		{
			const FVector RefUp = Nodes.Contains(Segment->StartNodeId) ? Nodes.FindChecked(Segment->StartNodeId).UpVector : FVector::UpVector;
			const TArray<FFlexCurveFrame> Frames = FFlexRoadMeshBuilder::BuildFramesForRange(Segment->Curve, Segment->ArcLengthTable, RefUp, Settings->ArcLengthSampleStep, MeshResults[Index].TrimStartArcLength, MeshResults[Index].TrimEndArcLength);
			TerrainConformer->ConformSegment(World, SegId, Frames, Segment->Profile->GetRoadwayHalfWidth(), Settings->TerrainConformMargin, Settings->TerrainFalloffDistance);
		}
	}

	// 6. Junction meshes.
	for (FFlexNodeId NodeId : AffectedNodes)
	{
		const FFlexRoadNode* Node = Nodes.Find(NodeId);
		if (!Node)
		{
			continue; // Node itself may have been removed as part of this batch.
		}

		if (const FFlexJunctionData* JunctionData = JunctionDataByNode.Find(NodeId))
		{
			UMaterialInterface* SurfaceMaterial = nullptr;
			UMaterialInterface* CrosswalkMaterial = nullptr;
			UMaterialInterface* SidewalkMaterial = nullptr;
			UMaterialInterface* MedianMaterial = nullptr;
			if (Node->ConnectedSegments.Num() > 0)
			{
				if (const FFlexRoadSegment* FirstSegment = Segments.Find(Node->ConnectedSegments[0]))
				{
					if (FirstSegment->Profile)
					{
						SurfaceMaterial = FirstSegment->Profile->JunctionMaterial;
						CrosswalkMaterial = FirstSegment->Profile->SidewalkMaterial;
						SidewalkMaterial = FirstSegment->Profile->SidewalkMaterial;
						MedianMaterial = FirstSegment->Profile->MedianMaterial;
					}
				}
			}
			const FFlexJunctionMeshResult JunctionMesh = FFlexIntersectionBuilder::BuildJunctionMesh(Node->UpVector, *JunctionData, SurfaceMaterial, CrosswalkMaterial, SidewalkMaterial, MedianMaterial);
			if (Actor)
			{
				Actor->ApplyJunctionMesh(NodeId, JunctionMesh);
			}
		}
	}

	// 7. Notify + clear dirty state.
	const TArray<FFlexNodeId> ChangedNodesArray = AffectedNodes.Array();
	const TArray<FFlexSegmentId> ChangedSegmentsArray = SegmentIdsToRebuild;

	DirtyNodes.Reset();
	DirtySegments.Reset();

	UE_LOG(LogTemp, Verbose, TEXT("FlexNetwork: incremental rebuild touched %d node(s), %d segment(s)."), ChangedNodesArray.Num(), ChangedSegmentsArray.Num());

	OnRoadNetworkChanged.Broadcast(ChangedNodesArray, ChangedSegmentsArray);
	OnRoadNetworkChangedBP.Broadcast(ChangedNodesArray, ChangedSegmentsArray);

	for (const TSharedPtr<IFlexNetworkExporter>& Exporter : Exporters)
	{
		if (Exporter)
		{
			Exporter->ExportChangedRegion(*this, ChangedNodesArray, ChangedSegmentsArray);
		}
	}
}
