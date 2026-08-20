#include "Mesh/FlexRoadMarkingBuilder.h"

#include "Misc/Optional.h"
#include "RoadTypeProfile.h"
#include "Intersection/FlexLaneConnectorGraph.h"
#include "Math/FlexBezierMath.h"
#include "Mesh/FlexRoadMeshBuilder.h"

namespace
{
	/**
	 * Places a series of "simple 2D quad" dashes along Curve, each spanning exactly [DashStart,
	 * DashStart+DashLength] of arc length and laterally centered LateralOffset from the curve (a
	 * width of DashHalfWidth*2). Reuses FFlexRoadMeshBuilder::BuildFramesForRange with SampleStep
	 * equal to the dash length itself, which always yields exactly the two frames (start/end) a
	 * flat dash quad needs -- no separate quad-construction code is needed here.
	 */
	void AppendDashedStripAlongCurve(FFlexMeshSectionData& OutSection, const FFlexBezierCurve& Curve,
		const FFlexArcLengthTable& ArcLengthTable, const FVector& ReferenceUp, float LateralOffset,
		float DashHalfWidth, float DashLength, float DashGap, float TrimStart, float TrimEnd, float VerticalOffset)
	{
		const float Period = DashLength + FMath::Max(DashGap, 0.f);
		if (DashLength <= KINDA_SMALL_NUMBER || Period <= KINDA_SMALL_NUMBER)
		{
			return;
		}
		for (float DashStart = TrimStart; DashStart < TrimEnd - KINDA_SMALL_NUMBER; DashStart += Period)
		{
			const float DashEnd = FMath::Min(DashStart + DashLength, TrimEnd);
			if (DashEnd - DashStart <= KINDA_SMALL_NUMBER)
			{
				break;
			}
			const TArray<FFlexCurveFrame> Frames = FFlexRoadMeshBuilder::BuildFramesForRange(
				Curve, ArcLengthTable, ReferenceUp, DashLength, DashStart, DashEnd);
			FFlexRoadMeshBuilder::AppendExtrudedStrip(OutSection, Frames, LateralOffset - DashHalfWidth, LateralOffset + DashHalfWidth, VerticalOffset);
		}
	}
}

TArray<FVector2D> FFlexRoadMarkingBuilder::SubtractExcludedRanges(float Start, float End, TConstArrayView<FVector2D> ExcludedRanges)
{
	TArray<FVector2D> Remaining;
	if (End - Start > KINDA_SMALL_NUMBER)
	{
		Remaining.Add(FVector2D(Start, End));
	}
	for (const FVector2D& Excluded : ExcludedRanges)
	{
		TArray<FVector2D> Next;
		for (const FVector2D& Range : Remaining)
		{
			const float OverlapStart = FMath::Max(Range.X, Excluded.X);
			const float OverlapEnd = FMath::Min(Range.Y, Excluded.Y);
			if (OverlapEnd - OverlapStart <= KINDA_SMALL_NUMBER)
			{
				Next.Add(Range); // No meaningful overlap with this excluded range.
				continue;
			}
			if (OverlapStart - Range.X > KINDA_SMALL_NUMBER)
			{
				Next.Add(FVector2D(Range.X, OverlapStart));
			}
			if (Range.Y - OverlapEnd > KINDA_SMALL_NUMBER)
			{
				Next.Add(FVector2D(OverlapEnd, Range.Y));
			}
		}
		Remaining = MoveTemp(Next);
	}
	return Remaining;
}

void FFlexRoadMarkingBuilder::BuildSegmentLaneMarkings(
	const FFlexBezierCurve& Curve, const FFlexArcLengthTable& ArcLengthTable, const URoadTypeProfile* Profile,
	const FVector& ReferenceUp, float SampleStep, float TrimStartArcLength, float TrimEndArcLength,
	bool bStartAtJunction, bool bEndAtJunction, TConstArrayView<FVector2D> CrosswalkExclusionArcRanges,
	const FFlexRoadMarkingParams& Params, FFlexMeshSectionData* OutSolid, FFlexMeshSectionData* OutLaneDash)
{
	if ((!OutSolid && !OutLaneDash) || !Profile || !ArcLengthTable.IsValid()
		|| TrimEndArcLength - TrimStartArcLength <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	TArray<FRoadLaneDescriptor> CountedLanes;
	for (const FRoadLaneDescriptor& Lane : Profile->GetLanesSortedByOffset())
	{
		if (Lane.Type == EFlexLaneType::Vehicle || Lane.Type == EFlexLaneType::Bike)
		{
			CountedLanes.Add(Lane);
		}
	}
	if (CountedLanes.Num() < 2)
	{
		return;
	}

	const bool bTwoLaneRoad = CountedLanes.Num() == 2;
	const float TransitionDistance = FMath::Max(Params.SolidToDashedTransitionDistance, 0.f);

	// One piece per contiguous span of one marking style (dashed or solid) this boundary needs,
	// before crosswalk exclusion is applied. A solid boundary contributes up to three: a dashed
	// lead-in/lead-out near whichever end(s) actually border a junction, and a solid middle.
	struct FMarkingPiece { float Start; float End; bool bDashed; };
	TArray<FMarkingPiece> Pieces;

	for (int32 Index = 0; Index + 1 < CountedLanes.Num(); ++Index)
	{
		const FRoadLaneDescriptor& LaneA = CountedLanes[Index];
		const FRoadLaneDescriptor& LaneB = CountedLanes[Index + 1];
		const float BoundaryOffset = Profile->LateralOffset + (LaneA.GetOuterEdge() + LaneB.GetInnerEdge()) * 0.5f;
		// Exactly two lanes: always dashed, even though they're normally opposite directions --
		// this is the explicit exception to the direction-based rule below (an ordinary two-lane
		// road allows passing, unlike a multi-lane divided road).
		const bool bDashed = bTwoLaneRoad || LaneA.Direction == LaneB.Direction;

		Pieces.Reset();
		if (bDashed)
		{
			Pieces.Add({ TrimStartArcLength, TrimEndArcLength, true });
		}
		else
		{
			float SolidStart = TrimStartArcLength;
			float SolidEnd = TrimEndArcLength;
			if (bStartAtJunction)
			{
				SolidStart = FMath::Min(TrimStartArcLength + TransitionDistance, TrimEndArcLength);
				Pieces.Add({ TrimStartArcLength, SolidStart, true });
			}
			if (bEndAtJunction)
			{
				SolidEnd = FMath::Max(TrimEndArcLength - TransitionDistance, SolidStart);
			}
			if (SolidEnd - SolidStart > KINDA_SMALL_NUMBER)
			{
				Pieces.Add({ SolidStart, SolidEnd, false });
			}
			if (bEndAtJunction && TrimEndArcLength - SolidEnd > KINDA_SMALL_NUMBER)
			{
				Pieces.Add({ SolidEnd, TrimEndArcLength, true });
			}
		}

		TOptional<TArray<FFlexCurveFrame>> SolidFrames; // Built lazily, at most once per boundary.
		for (const FMarkingPiece& Piece : Pieces)
		{
			for (const FVector2D& Range : SubtractExcludedRanges(Piece.Start, Piece.End, CrosswalkExclusionArcRanges))
			{
				if (Piece.bDashed)
				{
					if (OutLaneDash && Params.LaneDashLength > KINDA_SMALL_NUMBER)
					{
						AppendDashedStripAlongCurve(*OutLaneDash, Curve, ArcLengthTable, ReferenceUp, BoundaryOffset,
							Params.LaneDashWidth * 0.5f, Params.LaneDashLength, Params.LaneDashGap,
							Range.X, Range.Y, Params.VerticalOffset);
					}
				}
				else if (OutSolid && Params.SolidLineWidth > KINDA_SMALL_NUMBER)
				{
					const TArray<FFlexCurveFrame> Frames = FFlexRoadMeshBuilder::BuildFramesForRange(
						Curve, ArcLengthTable, ReferenceUp, SampleStep, Range.X, Range.Y);
					FFlexRoadMeshBuilder::AppendExtrudedStrip(*OutSolid, Frames,
						BoundaryOffset - Params.SolidLineWidth * 0.5f, BoundaryOffset + Params.SolidLineWidth * 0.5f, Params.VerticalOffset);
				}
			}
		}
	}
}

void FFlexRoadMarkingBuilder::BuildCrosswalkMarkings(const FFlexCrosswalkPlacement& Crosswalk, const FVector& ReferenceUp,
	const FFlexRoadMarkingParams& Params, FFlexMeshSectionData* OutCrosswalkDash)
{
	if (!OutCrosswalkDash || Crosswalk.Width <= KINDA_SMALL_NUMBER || Crosswalk.Length <= KINDA_SMALL_NUMBER
		|| Params.CrosswalkDashLength <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	const FVector Up = ReferenceUp.GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
	// Along = the direction a pedestrian walks, crossing the road; Across = parallel to the road.
	// Matches the convention FFlexIntersectionBuilder/PCGFlexNetworkNodes already use for the same
	// FFlexCrosswalkPlacement: Length is measured along Along (how far you walk to cross -- the
	// road's own width), Width along Across (how wide the crossing corridor is along the curb).
	const FVector Along = FVector::VectorPlaneProject(Crosswalk.CrossingDirection, Up).GetSafeNormal();
	const FVector Across = FVector::CrossProduct(Up, Along).GetSafeNormal();
	if (Along.IsNearlyZero() || Across.IsNearlyZero())
	{
		return;
	}

	const float HalfLength = Crosswalk.Length * 0.5f;
	const float HalfWidth = Crosswalk.Width * 0.5f;

	// The crosswalk's two LONG edges -- running along the crossing direction (orthogonal to the
	// road) for the crosswalk's full Length, one on each side of the corridor -- not its short
	// near/far curb-line edges. Each edge is modeled as a straight (degenerate-control-point)
	// Bezier so it can reuse AppendDashedStripAlongCurve exactly like a lane boundary: a straight
	// curve's own RMF Right axis works out to be exactly Across, so a LateralOffset of 0 centers
	// each dash precisely on the edge line.
	for (const float Side : { -1.f, 1.f })
	{
		const FVector EdgeCenter = Crosswalk.Center + Across * (Side * HalfWidth);
		FFlexBezierCurve EdgeCurve;
		EdgeCurve.P0 = EdgeCenter - Along * HalfLength;
		EdgeCurve.P3 = EdgeCenter + Along * HalfLength;
		EdgeCurve.P1 = FMath::Lerp(EdgeCurve.P0, EdgeCurve.P3, 1.f / 3.f);
		EdgeCurve.P2 = FMath::Lerp(EdgeCurve.P0, EdgeCurve.P3, 2.f / 3.f);

		const FFlexArcLengthTable EdgeTable = FFlexBezierMath::BuildArcLengthTable(EdgeCurve);
		if (!EdgeTable.IsValid())
		{
			continue;
		}
		AppendDashedStripAlongCurve(*OutCrosswalkDash, EdgeCurve, EdgeTable, Up, 0.f,
			Params.CrosswalkDashWidth * 0.5f, Params.CrosswalkDashLength, Params.CrosswalkDashGap,
			0.f, EdgeTable.GetTotalLength(), Params.VerticalOffset);
	}
}

void FFlexRoadMarkingBuilder::BuildIntersectionLaneMarking(const FFlexLaneConnector& Connector, float LaneWidth,
	const FVector& ReferenceUp, const FFlexRoadMarkingParams& Params, FFlexMeshSectionData* OutIntersectionDash)
{
	if (!OutIntersectionDash || Params.IntersectionDashLength <= KINDA_SMALL_NUMBER)
	{
		return;
	}
	const FFlexArcLengthTable Table = FFlexBezierMath::BuildArcLengthTable(Connector.ConnectorCurve);
	if (!Table.IsValid())
	{
		return;
	}
	// Right = CrossProduct(Up, Tangent) is the codebase-wide convention (see Math/
	// FlexRotationMinimizingFrame.h); left is the negative of that, i.e. a negative lateral offset.
	const float LeftOffset = -LaneWidth * 0.5f;
	AppendDashedStripAlongCurve(*OutIntersectionDash, Connector.ConnectorCurve, Table, ReferenceUp, LeftOffset,
		Params.IntersectionDashWidth * 0.5f, Params.IntersectionDashLength, Params.IntersectionDashGap,
		0.f, Table.GetTotalLength(), Params.VerticalOffset);
}

void FFlexRoadMarkingBuilder::BuildParkingSpotMarkings(const FFlexBezierCurve& Curve, const FFlexArcLengthTable& ArcLengthTable,
	const FRoadLaneDescriptor& Lane, float LaneAbsoluteLateralOffset, const FVector& ReferenceUp,
	float TrimStartArcLength, float TrimEndArcLength, const FFlexRoadMarkingParams& Params, FFlexMeshSectionData* OutParkingMarking)
{
	if (!OutParkingMarking || Lane.Type != EFlexLaneType::Parking || !ArcLengthTable.IsValid()
		|| Params.ParkingSpotSpacing <= KINDA_SMALL_NUMBER || Params.ParkingLineWidth <= KINDA_SMALL_NUMBER
		|| TrimEndArcLength - TrimStartArcLength <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	const float AngleRad = FMath::DegreesToRadians(FMath::Clamp(Lane.ParkingAngleDegrees, 0.f, 90.f));
	const float CosAngle = FMath::Cos(AngleRad);
	const float SinAngle = FMath::Sin(AngleRad);
	const float HalfLine = Lane.Width * 0.5f;
	const float HalfThickness = Params.ParkingLineWidth * 0.5f;

	// Interior boundaries only -- one at each multiple of ParkingSpotSpacing strictly inside the
	// lane's own trimmed span, not at its very start/end (those are the bay row's own open ends,
	// not a boundary between two spots).
	for (float ArcLength = TrimStartArcLength + Params.ParkingSpotSpacing; ArcLength < TrimEndArcLength - KINDA_SMALL_NUMBER; ArcLength += Params.ParkingSpotSpacing)
	{
		const FFlexCurveFrame Frame = FFlexRoadMeshBuilder::SampleFrameAtArcLength(Curve, ArcLengthTable, ArcLength, ReferenceUp);

		// Rotates from Frame.Right (0 deg, perpendicular to the road -- parallel parking) toward
		// Frame.Tangent (90 deg, along the road -- perpendicular/orthogonal parking) about Frame.Up.
		const FVector LineDirection = (Frame.Right * CosAngle + Frame.Tangent * SinAngle).GetSafeNormal();
		const FVector ThicknessDirection = FVector::CrossProduct(Frame.Up, LineDirection).GetSafeNormal();
		if (LineDirection.IsNearlyZero() || ThicknessDirection.IsNearlyZero())
		{
			continue;
		}

		const FVector Center = Frame.Position + Frame.Right * LaneAbsoluteLateralOffset + Frame.Up * Params.VerticalOffset;
		const FVector HalfLineVec = LineDirection * HalfLine;
		const FVector HalfThicknessVec = ThicknessDirection * HalfThickness;

		const FVector InnerA = Center - HalfLineVec - HalfThicknessVec;
		const FVector OuterA = Center + HalfLineVec - HalfThicknessVec;
		const FVector OuterB = Center + HalfLineVec + HalfThicknessVec;
		const FVector InnerB = Center - HalfLineVec + HalfThicknessVec;
		OutParkingMarking->AppendQuad(InnerA, OuterA, OuterB, InnerB, Frame.Up, LineDirection,
			FVector2D(0.f, 0.f), FVector2D(1.f, 0.f), FVector2D(1.f, 1.f), FVector2D(0.f, 1.f));
	}
}

void FFlexRoadMarkingBuilder::BuildStopLineMarking(const FFlexCurveFrame& Frame, float SpanMinOffset, float SpanMaxOffset,
	const FFlexRoadMarkingParams& Params, FFlexMeshSectionData* OutStopLine)
{
	if (!OutStopLine || SpanMaxOffset - SpanMinOffset <= KINDA_SMALL_NUMBER || Params.StopLineThickness <= KINDA_SMALL_NUMBER)
	{
		return;
	}
	const FVector Base = Frame.Position + Frame.Up * Params.VerticalOffset;
	const FVector HalfThickness = Frame.Tangent.GetSafeNormal() * (Params.StopLineThickness * 0.5f);

	// Inner/outer naming mirrors FFlexRoadMeshBuilder::AppendExtrudedStrip's own quad corner order
	// (InnerA, OuterA, OuterB, InnerB) -- here "along the curve" is Frame.Right (the line spans
	// laterally across the lanes) and "lateral offset" is Frame.Tangent (its own thickness), the
	// two axes swapped relative to a normal strip, same as a crosswalk edge dash.
	const FVector InnerA = Base + Frame.Right * SpanMinOffset - HalfThickness;
	const FVector OuterA = Base + Frame.Right * SpanMaxOffset - HalfThickness;
	const FVector OuterB = Base + Frame.Right * SpanMaxOffset + HalfThickness;
	const FVector InnerB = Base + Frame.Right * SpanMinOffset + HalfThickness;
	OutStopLine->AppendQuad(InnerA, OuterA, OuterB, InnerB, Frame.Up, Frame.Right,
		FVector2D(0.f, 0.f), FVector2D(1.f, 0.f), FVector2D(1.f, 1.f), FVector2D(0.f, 1.f));
}
