#include "Synthetic/FlexSyntheticGraphBuilder.h"

#include "Arrangement2d.h"
#include "Algo/Reverse.h"
#include "HAL/PlatformTime.h"
#include "FlexNetworkSubsystem.h"
#include "RoadTypeProfile.h"

using namespace UE::Geometry;

namespace
{
	constexpr int32 kArterialGID = 0;
	constexpr int32 kLocalGID = 1;

	// See FlexSyntheticGraphBuilder::SampleFieldDirection's doc comment for why direction is
	// encoded this way instead of as a plain unit vector.
	FVector2d DoubledAngleTensor(double ThetaRadians)
	{
		return FVector2d(FMath::Cos(2.0 * ThetaRadians), FMath::Sin(2.0 * ThetaRadians));
	}

	/**
	 * Samples the field at Position, optionally rotates 90 degrees for the local (perpendicular)
	 * direction, then resolves the doubled-angle recovery's 180-degree sign ambiguity (Theta and
	 * Theta+180 encode the same undirected line) by keeping whichever sign continues roughly the
	 * same way as PrevDirection, so a traced path doesn't reverse on itself step to step.
	 */
	FVector2d SampleTraceDirection(const TArray<FFlexSyntheticFieldRegion>& Regions, const FVector2d& Position, bool bLocalTier, const FVector2d& PrevDirection)
	{
		FVector2d FieldDir = FlexSyntheticGraphBuilder::SampleFieldDirection(Regions, Position);
		if (bLocalTier)
		{
			FieldDir = FVector2d(-FieldDir.Y, FieldDir.X);
		}
		if (FieldDir.Dot(PrevDirection) < 0.0)
		{
			FieldDir = -FieldDir;
		}
		return FieldDir;
	}

	/**
	 * Simplified density control: a spatial hash of every point from streamlines already fully
	 * traced, used to stop a *new* streamline early once it gets within MinSeparation of a
	 * different, already-completed one. Without this, streamlines that converge toward a shared
	 * point (e.g. every major streamline's own path toward a radial field's center, or several
	 * minors seeded along nearby majors all tracing near-identical tangential arcs) keep going
	 * straight through each other -- confirmed in practice: it's what produced overlapping road
	 * mesh near a radial field's center once this generator started landing in the real graph. A
	 * simplified version of Jobard & Lefer's evenly-spaced streamline placement, checked only
	 * against *completed* streamlines (never a streamline's own in-progress points, since nothing
	 * is registered until AddCompletedStreamline runs), so order-of-tracing determines which
	 * streamline "wins" a contested area -- good enough for this generator's purposes, not a fully
	 * priority-ordered placement.
	 */
	class FStreamlineSeparationTracker
	{
	public:
		explicit FStreamlineSeparationTracker(double InMinSeparation)
			: Hash(FMath::Max(InMinSeparation, 1.0), -1)
			, MinSeparation(InMinSeparation)
		{
		}

		bool IsTooClose(const FVector2d& P) const
		{
			if (MinSeparation <= 0.0 || Points.IsEmpty())
			{
				return false;
			}
			auto FuncDistSq = [this, &P](int32 ID) { return FVector2d::DistSquared(P, Points[ID]); };
			const TPair<int32, double> Found = Hash.FindNearestInRadius(P, MinSeparation, FuncDistSq);
			return Found.Key != Hash.GetInvalidValue();
		}

		void AddCompletedStreamline(const TArray<FVector2d>& StreamlinePoints)
		{
			if (MinSeparation <= 0.0)
			{
				return;
			}
			for (const FVector2d& P : StreamlinePoints)
			{
				const int32 NewID = Points.Add(P);
				Hash.InsertPointUnsafe(NewID, P);
			}
		}

	private:
		TPointHashGrid2d<int32> Hash;
		TArray<FVector2d> Points;
		double MinSeparation;
	};

	/**
	 * Integrates one direction (forward or backward, chosen by the sign of PrevDirection) from
	 * Start through the field until leaving Bounds, getting too close to a different completed
	 * streamline (see FStreamlineSeparationTracker), or hitting MaxSteps, using RK2 (midpoint)
	 * integration: sample the direction at the current point, use it to estimate a midpoint, then
	 * resample there and use *that* direction for the actual step. Plain (forward Euler) stepping
	 * systematically drifts off a curved path, worse the tighter the curvature relative to
	 * StepSize -- validated against this exact failure mode in Phase 0 (a visible inward spiral
	 * near a radial field's center, where a fixed-length tangential step covers a large fraction of
	 * the local circle); RK2 is the standard, moderate-cost fix.
	 */
	TArray<FVector2d> TraceStreamlineOneWay(const TArray<FFlexSyntheticFieldRegion>& Regions, const FVector2d& Start,
		double StepSize, int32 MaxSteps, const FAxisAlignedBox2d& Bounds, bool bLocalTier, FVector2d PrevDirection,
		const FStreamlineSeparationTracker& SeparationTracker)
	{
		TArray<FVector2d> Points;
		Points.Add(Start);

		FVector2d Current = Start;
		for (int32 Step = 0; Step < MaxSteps; ++Step)
		{
			const FVector2d K1 = SampleTraceDirection(Regions, Current, bLocalTier, PrevDirection);
			const FVector2d Midpoint = Current + K1 * (StepSize * 0.5);
			const FVector2d K2 = SampleTraceDirection(Regions, Midpoint, bLocalTier, K1);

			const FVector2d Next = Current + K2 * StepSize;
			if (!Bounds.Contains(Next) || SeparationTracker.IsTooClose(Next))
			{
				break;
			}
			Points.Add(Next);
			PrevDirection = K2;
			Current = Next;
		}
		return Points;
	}

	/** Traces both ways from Seed and concatenates into one continuous polyline through Seed. */
	TArray<FVector2d> TraceStreamlineBothWays(const TArray<FFlexSyntheticFieldRegion>& Regions, const FVector2d& Seed,
		double StepSize, int32 MaxSteps, const FAxisAlignedBox2d& Bounds, bool bLocalTier, const FVector2d& InitialDirection,
		const FStreamlineSeparationTracker& SeparationTracker)
	{
		TArray<FVector2d> Forward = TraceStreamlineOneWay(Regions, Seed, StepSize, MaxSteps, Bounds, bLocalTier, InitialDirection, SeparationTracker);
		TArray<FVector2d> Backward = TraceStreamlineOneWay(Regions, Seed, StepSize, MaxSteps, Bounds, bLocalTier, -InitialDirection, SeparationTracker);
		Algo::Reverse(Backward);
		Backward.Pop(EAllowShrinking::No); // drop the duplicate seed point shared with Forward's first entry
		Backward.Append(MoveTemp(Forward));
		return Backward;
	}

	/** One road's raw shape between two real graph nodes -- see CollapseToRoadChains. */
	struct FRoadChain
	{
		TArray<FVector2d> Points;
		/** Arrangement graph vertex ID of each entry in Points, same indexing -- carried through directly from the walk so callers never need to re-find a vertex by position (including the closed-loop split point below, which is an ordinary interior vertex, not a chain endpoint the walk itself stopped at). */
		TArray<int32> VertexIDs;
		/** kArterialGID or kLocalGID -- see the mixed-tier tie-break note in CollapseToRoadChains. */
		int32 Group = kLocalGID;

		int32 StartVID() const { return VertexIDs[0]; }
		int32 EndVID() const { return VertexIDs.Last(); }
	};

	/**
	 * Walks Graph (a planarized FArrangement2d::Graph) and collapses it into one FRoadChain per
	 * real road: a vertex of degree != 2 is a genuine node (dead end / T / crossing); degree == 2 is
	 * just an interior sample point where a streamline continues straight through without anything
	 * crossing it. Each maximal degree-2 run between two real nodes becomes one chain. A closed loop
	 * with no real node anywhere on it (e.g. a local/tangential streamline that circled back on
	 * itself without crossing anything else) has no natural start/end, so it's cut in half at its
	 * own midpoint into two ordinary open chains sharing two distinct endpoints instead.
	 */
	TArray<FRoadChain> CollapseToRoadChains(const FDynamicGraph2d& Graph)
	{
		TArray<FRoadChain> Chains;
		TSet<int32> VisitedEdges;

		// Walks from (StartVID, StartEdgeID) through consecutive degree-2 vertices, marking every
		// traversed edge in VisitedEdges as it goes, until reaching the next real (degree != 2)
		// node or closing a loop back to its own start.
		auto WalkChainFrom = [&Graph, &VisitedEdges](int32 StartVID, int32 StartEdgeID) -> FRoadChain
		{
			FRoadChain Chain;
			Chain.Points.Add(Graph.GetVertex(StartVID));
			Chain.VertexIDs.Add(StartVID);
			Chain.Group = Graph.GetEdgeGroup(StartEdgeID) == kArterialGID ? kArterialGID : kLocalGID;

			int32 PrevVID = StartVID;
			int32 CurrentEdgeID = StartEdgeID;
			for (;;)
			{
				VisitedEdges.Add(CurrentEdgeID);
				const FIndex2i EdgeV = Graph.GetEdgeV(CurrentEdgeID);
				const int32 NextVID = (EdgeV.A == PrevVID) ? EdgeV.B : EdgeV.A;
				Chain.Points.Add(Graph.GetVertex(NextVID));
				Chain.VertexIDs.Add(NextVID);
				// Any arterial edge along the chain promotes the whole chain to arterial -- a
				// conservative tie-break for a chain that straddles a major/minor split after
				// intersection-splitting.
				if (Graph.GetEdgeGroup(CurrentEdgeID) == kArterialGID)
				{
					Chain.Group = kArterialGID;
				}

				if (NextVID == StartVID || Graph.GetVtxEdgeCount(NextVID) != 2)
				{
					break; // reached the next real node, or closed a loop back to its own start
				}

				int32 ContinuationEdgeID = -1;
				for (int32 CandidateEdgeID : Graph.VtxEdgesItr(NextVID))
				{
					if (CandidateEdgeID != CurrentEdgeID)
					{
						ContinuationEdgeID = CandidateEdgeID;
						break;
					}
				}
				if (ContinuationEdgeID == -1)
				{
					break; // shouldn't happen for a genuine degree-2 vertex, but guard anyway
				}
				CurrentEdgeID = ContinuationEdgeID;
				PrevVID = NextVID;
			}
			return Chain;
		};

		// Pass 1: every chain reachable from a real (degree != 2) node.
		for (int32 VID : Graph.VertexIndices())
		{
			if (Graph.GetVtxEdgeCount(VID) == 2)
			{
				continue; // not a real node -- it'll be walked through as part of some chain below
			}
			for (int32 StartEdgeID : Graph.VtxEdgesItr(VID))
			{
				if (VisitedEdges.Contains(StartEdgeID))
				{
					continue;
				}
				Chains.Add(WalkChainFrom(VID, StartEdgeID));
			}
		}

		// Pass 2: any edge not yet visited belongs entirely to an all-degree-2 closed loop (no real
		// node anywhere on it) -- cut each such loop in half into two open chains.
		for (int32 StartEdgeID : Graph.EdgeIndices())
		{
			if (VisitedEdges.Contains(StartEdgeID))
			{
				continue;
			}
			const FIndex2i StartEdgeV = Graph.GetEdgeV(StartEdgeID);
			const int32 LoopStartVID = StartEdgeV.A;

			FRoadChain Loop = WalkChainFrom(LoopStartVID, StartEdgeID);

			// Loop.Points starts and ends at LoopStartVID -- AddSegment needs two distinct
			// endpoints, so split at the loop's own halfway point (an ordinary interior vertex,
			// promoted here to a real node) into two ordinary open chains.
			if (Loop.Points.Num() >= 4)
			{
				const int32 MidIndex = Loop.Points.Num() / 2;
				FRoadChain SecondHalf;
				SecondHalf.Group = Loop.Group;
				SecondHalf.Points.Append(TArrayView<const FVector2d>(Loop.Points.GetData() + MidIndex, Loop.Points.Num() - MidIndex));
				SecondHalf.VertexIDs.Append(TArrayView<const int32>(Loop.VertexIDs.GetData() + MidIndex, Loop.VertexIDs.Num() - MidIndex));
				Loop.Points.SetNum(MidIndex + 1);
				Loop.VertexIDs.SetNum(MidIndex + 1);
				Chains.Add(MoveTemp(Loop));
				Chains.Add(MoveTemp(SecondHalf));
			}
			// Too few points to usefully split -- discard as degenerate.
		}

		return Chains;
	}

	/**
	 * Cheap, rough cubic Bezier tangent-handle fit for Chain -- P0/P3 are the chain's own fixed
	 * endpoints (not fitted), tangent direction at each end from a short local window of the
	 * chain's own points (more robust than a plain two-point difference), handle length via the
	 * standard chord-length/3 rule of thumb. Accuracy doesn't matter for a fictional network, so no
	 * least-squares fit is needed. BaseElevation lifts the flat 2D chain into 3D world space.
	 */
	void FitBezierHandles(const FRoadChain& Chain, double BaseElevation, FVector& OutStartTangentHandle, FVector& OutEndTangentHandle)
	{
		const int32 NumPoints = Chain.Points.Num();
		const FVector2d P0 = Chain.Points[0];
		const FVector2d P3 = Chain.Points[NumPoints - 1];

		const int32 WindowSize = FMath::Clamp(NumPoints / 5, 1, 5);

		FVector2d StartDir = (Chain.Points[FMath::Min(WindowSize, NumPoints - 1)] - P0).GetSafeNormal();
		if (StartDir.IsNearlyZero())
		{
			StartDir = (P3 - P0).GetSafeNormal();
		}
		FVector2d EndDir = (P3 - Chain.Points[FMath::Max(0, NumPoints - 1 - WindowSize)]).GetSafeNormal();
		if (EndDir.IsNearlyZero())
		{
			EndDir = (P3 - P0).GetSafeNormal();
		}

		const double HandleLength = FVector2d::Distance(P0, P3) / 3.0;
		const FVector2d P1 = P0 + StartDir * HandleLength;
		const FVector2d P2 = P3 - EndDir * HandleLength;

		OutStartTangentHandle = FVector(P1.X, P1.Y, BaseElevation);
		OutEndTangentHandle = FVector(P2.X, P2.Y, BaseElevation);
	}
}

FVector2d FlexSyntheticGraphBuilder::SampleFieldDirection(const TArray<FFlexSyntheticFieldRegion>& Regions, const FVector2d& P)
{
	FVector2d SummedTensor = FVector2d::Zero();
	for (const FFlexSyntheticFieldRegion& Region : Regions)
	{
		double Theta;
		if (Region.Kind == EFlexSyntheticFieldKind::Radial)
		{
			const FVector2d ToPoint = P - Region.Center;
			if (ToPoint.IsNearlyZero())
			{
				continue;
			}
			Theta = FMath::Atan2(ToPoint.Y, ToPoint.X);
		}
		else
		{
			Theta = FMath::DegreesToRadians(Region.GridAngleDegrees);
		}

		const double DistSq = FVector2d::DistSquared(P, Region.Center);
		const double DecaySq = FMath::Max(Region.DecayRadius * Region.DecayRadius, 1.0);
		const double Weight = FMath::Exp(-DistSq / DecaySq);
		SummedTensor += DoubledAngleTensor(Theta) * Weight;
	}

	if (SummedTensor.IsNearlyZero())
	{
		return FVector2d(1.0, 0.0); // arbitrary fallback in a region every field's influence has decayed away from
	}

	const double SummedAngle = 0.5 * FMath::Atan2(SummedTensor.Y, SummedTensor.X);
	return FVector2d(FMath::Cos(SummedAngle), FMath::Sin(SummedAngle));
}

FFlexSyntheticGenerationResult FlexSyntheticGraphBuilder::GenerateSyntheticNetwork(
	UFlexNetworkSubsystem& Subsystem, const FFlexSyntheticGenerationSettings& Settings, TFunctionRef<URoadTypeProfile*(EFlexSyntheticRoadTier)> ResolveProfile)
{
	const double StartSeconds = FPlatformTime::Seconds();
	FFlexSyntheticGenerationResult Result;

	if (Settings.FieldRegions.IsEmpty() || Settings.StepSize <= 0.0 || Settings.NumMajorSeeds <= 0)
	{
		return Result;
	}

	const FAxisAlignedBox2d Bounds(Settings.DomainMin, Settings.DomainMax);
	const FVector2d DomainSize = Settings.DomainMax - Settings.DomainMin;

	// Each major (arterial) streamline is immediately followed by its own minor (local)
	// streamlines in the same combined list, rather than tracing every major first and appending
	// every minor afterward -- if Settings.MaxRawSegmentsToInsert truncates insertion partway
	// through, this ordering keeps the truncated result spread across the domain with a mix of both
	// tiers instead of losing an entire tier (validated the hard way during Phase 0 testing).
	struct FTaggedStreamline { TArray<FVector2d> Points; int32 Group; };
	TArray<FTaggedStreamline> AllStreamlines;
	FStreamlineSeparationTracker SeparationTracker(Settings.MinStreamlineSeparation);
	for (int32 X = 0; X < Settings.NumMajorSeeds; ++X)
	{
		for (int32 Y = 0; Y < Settings.NumMajorSeeds; ++Y)
		{
			const FVector2d MajorSeed = Settings.DomainMin + DomainSize * FVector2d(
				(X + 0.5) / Settings.NumMajorSeeds, (Y + 0.5) / Settings.NumMajorSeeds);
			const FVector2d InitialDirection = SampleFieldDirection(Settings.FieldRegions, MajorSeed);
			FTaggedStreamline Major;
			Major.Points = TraceStreamlineBothWays(Settings.FieldRegions, MajorSeed, Settings.StepSize, Settings.MaxStepsPerStreamline, Bounds, false, InitialDirection, SeparationTracker);
			Major.Group = kArterialGID;
			SeparationTracker.AddCompletedStreamline(Major.Points);
			const TArray<FVector2d>& MajorPoints = Major.Points;
			AllStreamlines.Add(Major);

			double AccumulatedLength = 0.0;
			for (int32 i = 1; i < MajorPoints.Num(); ++i)
			{
				AccumulatedLength += FVector2d::Distance(MajorPoints[i - 1], MajorPoints[i]);
				if (AccumulatedLength < Settings.MinorSeedSpacing)
				{
					continue;
				}
				AccumulatedLength = 0.0;

				const FVector2d MinorSeed = MajorPoints[i];
				const FVector2d MajorDirection = SampleFieldDirection(Settings.FieldRegions, MinorSeed);
				const FVector2d MinorInitialDirection(-MajorDirection.Y, MajorDirection.X);
				FTaggedStreamline Minor;
				Minor.Points = TraceStreamlineBothWays(Settings.FieldRegions, MinorSeed, Settings.StepSize, Settings.MaxStepsPerStreamline, Bounds, true, MinorInitialDirection, SeparationTracker);
				Minor.Group = kLocalGID;
				SeparationTracker.AddCompletedStreamline(Minor.Points);
				AllStreamlines.Add(MoveTemp(Minor));
			}
		}
	}

	Result.NumStreamlinesTraced = AllStreamlines.Num();

	int32 TotalRawSegments = 0;
	for (const FTaggedStreamline& Line : AllStreamlines)
	{
		TotalRawSegments += FMath::Max(0, Line.Points.Num() - 1);
	}
	UE_LOG(LogTemp, Display, TEXT("FlexSyntheticGraphBuilder: %d streamlines, ~%d raw segments about to be planarized via FArrangement2d ")
		TEXT("(unaccelerated O(segment-count^2) edge-vs-edge search -- this is the potentially slow part, and nothing else logs until it's done)."),
		AllStreamlines.Num(), TotalRawSegments);
	if (TotalRawSegments > Settings.MaxRawSegmentsToInsert)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexSyntheticGraphBuilder: %d raw segments exceeds MaxRawSegmentsToInsert (%d) -- some streamlines will be skipped to stay within budget."),
			TotalRawSegments, Settings.MaxRawSegmentsToInsert);
	}

	FArrangement2d Arrangement(Bounds);
	for (const FTaggedStreamline& Line : AllStreamlines)
	{
		if (Result.NumRawSegmentsInserted >= Settings.MaxRawSegmentsToInsert)
		{
			Result.bSegmentBudgetExceeded = true;
			break;
		}
		for (int32 i = 0; i + 1 < Line.Points.Num(); ++i)
		{
			if (Result.NumRawSegmentsInserted >= Settings.MaxRawSegmentsToInsert)
			{
				Result.bSegmentBudgetExceeded = true;
				break;
			}
			const FVector2d& A = Line.Points[i];
			const FVector2d& B = Line.Points[i + 1];
			if (FVector2d::DistSquared(A, B) < KINDA_SMALL_NUMBER)
			{
				continue;
			}
			Arrangement.Insert(A, B, Line.Group);
			++Result.NumRawSegmentsInserted;
		}
	}

	Result.bHasSelfIntersectionsAfterPlanarization = Arrangement.HasSelfIntersections();
	if (Result.bHasSelfIntersectionsAfterPlanarization)
	{
		UE_LOG(LogTemp, Error, TEXT("FlexSyntheticGraphBuilder: planarization reported self-intersections -- aborting before touching UFlexNetworkSubsystem."));
		return Result;
	}

	TArray<FRoadChain> Chains = CollapseToRoadChains(Arrangement.Graph);

	Subsystem.BeginBatchUpdate();
	TMap<int32, FFlexNodeId> ArrangementVertexToNodeId;
	auto GetOrCreateNode = [&Subsystem, &Arrangement, &ArrangementVertexToNodeId, &Settings](int32 VID) -> FFlexNodeId
	{
		if (const FFlexNodeId* Existing = ArrangementVertexToNodeId.Find(VID))
		{
			return *Existing;
		}
		const FVector2d Pos = Arrangement.Graph.GetVertex(VID);
		const FFlexNodeId NewNodeId = Subsystem.AddNode(FVector(Pos.X, Pos.Y, Settings.BaseElevation));
		ArrangementVertexToNodeId.Add(VID, NewNodeId);
		return NewNodeId;
	};

	for (const FRoadChain& Chain : Chains)
	{
		if (Chain.Points.Num() < 2)
		{
			continue;
		}
		const double ChordLengthSq = FVector2d::DistSquared(Chain.Points[0], Chain.Points.Last());
		if (ChordLengthSq < Settings.MinSegmentLength * Settings.MinSegmentLength)
		{
			continue; // degenerate/near-zero-length chain
		}

		URoadTypeProfile* Profile = ResolveProfile(Chain.Group == kArterialGID ? EFlexSyntheticRoadTier::Arterial : EFlexSyntheticRoadTier::Local);
		if (!Profile)
		{
			continue; // caller declined to supply a profile for this tier -- skip, not a hard failure
		}

		const FFlexNodeId StartNodeId = GetOrCreateNode(Chain.StartVID());
		const FFlexNodeId EndNodeId = GetOrCreateNode(Chain.EndVID());

		FVector StartTangentHandle, EndTangentHandle;
		FitBezierHandles(Chain, Settings.BaseElevation, StartTangentHandle, EndTangentHandle);

		const FFlexSegmentId NewSegmentId = Subsystem.AddSegment(StartNodeId, EndNodeId, StartTangentHandle, EndTangentHandle, Profile);
		if (NewSegmentId.IsValid())
		{
			++Result.NumSegmentsCreated;
		}
	}
	Result.NumNodesCreated = ArrangementVertexToNodeId.Num();
	Subsystem.EndBatchUpdate();

	Result.GenerationSeconds = FPlatformTime::Seconds() - StartSeconds;
	UE_LOG(LogTemp, Display, TEXT("FlexSyntheticGraphBuilder: generated %d node(s), %d segment(s) from %d chain(s) in %.3f s."),
		Result.NumNodesCreated, Result.NumSegmentsCreated, Chains.Num(), Result.GenerationSeconds);
	return Result;
}

TArray<FFlexSyntheticFieldRegion> FlexSyntheticGraphBuilder::MakeDefaultFieldRegions(double HalfDomainSize)
{
	TArray<FFlexSyntheticFieldRegion> Regions;

	FFlexSyntheticFieldRegion RadialRegion;
	RadialRegion.Kind = EFlexSyntheticFieldKind::Radial;
	RadialRegion.Center = FVector2D(0.0, 0.0);
	RadialRegion.DecayRadius = HalfDomainSize * 0.6;
	Regions.Add(RadialRegion);

	FFlexSyntheticFieldRegion GridRegion;
	GridRegion.Kind = EFlexSyntheticFieldKind::Grid;
	GridRegion.Center = FVector2D(HalfDomainSize * 0.5, HalfDomainSize * 0.5);
	GridRegion.GridAngleDegrees = 30.0;
	GridRegion.DecayRadius = HalfDomainSize * 0.6;
	Regions.Add(GridRegion);

	return Regions;
}
