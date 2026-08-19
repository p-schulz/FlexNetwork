#include "FlexNetworkSegmentActor.h"

#include "Components/SceneComponent.h"
#include "Components/SplineComponent.h"
#include "PCGComponent.h"
#include "PCGCommon.h"
#include "FlexRoadSegment.h"
#include "Mesh/FlexRoadMeshBuilder.h"

AFlexNetworkSegmentActor::AFlexNetworkSegmentActor()
{
	PrimaryActorTick.bCanEverTick = false;
	bReplicates = false;
	SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(SceneRoot);

	auto MakeSpline = [this](const TCHAR* Name)
	{
		USplineComponent* Spline = CreateDefaultSubobject<USplineComponent>(FName(Name));
		Spline->SetupAttachment(SceneRoot);
		Spline->SetMobility(EComponentMobility::Movable);
		Spline->SetDrawDebug(true);
		Spline->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		return Spline;
	};
	Centerline = MakeSpline(TEXT("Centerline"));
	RoadLeftEdge = MakeSpline(TEXT("RoadLeftEdge"));
	RoadRightEdge = MakeSpline(TEXT("RoadRightEdge"));
	SidewalkLeftEdge = MakeSpline(TEXT("SidewalkLeftEdge"));
	SidewalkRightEdge = MakeSpline(TEXT("SidewalkRightEdge"));
	PCGComponent = CreateDefaultSubobject<UPCGComponent>(TEXT("PCG"));
}

void AFlexNetworkSegmentActor::UpdateFromSegment(FFlexSegmentId InId, const FFlexRoadSegment& Segment, const FVector& ReferenceUp, float SampleStep, float TrimStartArcLength, float TrimEndArcLength)
{
	SegmentId = InId;
	Tags.AddUnique(TEXT("FlexNetworkSegment"));
	const float RoadMinOffset = Segment.Profile ? Segment.Profile->GetRoadwayMinOffset() : 0.f;
	const float RoadMaxOffset = Segment.Profile ? Segment.Profile->GetRoadwayMaxOffset() : 0.f;
	const float SidewalkWidth = Segment.Profile ? Segment.Profile->SidewalkWidth : 0.f;
	const TArray<FFlexCurveFrame> Frames = FFlexRoadMeshBuilder::BuildFramesForRange(
		Segment.Curve, Segment.ArcLengthTable, ReferenceUp, SampleStep, TrimStartArcLength, TrimEndArcLength);

	TArray<FVector> Center, RoadLeft, RoadRight, WalkLeft, WalkRight;
	Center.Reserve(Frames.Num()); RoadLeft.Reserve(Frames.Num()); RoadRight.Reserve(Frames.Num());
	WalkLeft.Reserve(Frames.Num()); WalkRight.Reserve(Frames.Num());
	for (const FFlexCurveFrame& Frame : Frames)
	{
		Center.Add(Frame.Position);
		RoadLeft.Add(Frame.Position + Frame.Right * RoadMinOffset);
		RoadRight.Add(Frame.Position + Frame.Right * RoadMaxOffset);
		WalkLeft.Add(Frame.Position + Frame.Right * (RoadMinOffset - SidewalkWidth));
		WalkRight.Add(Frame.Position + Frame.Right * (RoadMaxOffset + SidewalkWidth));
	}

	auto SetPoints = [](USplineComponent* Spline, const TArray<FVector>& Points, bool bVisible)
	{
		Spline->ClearSplinePoints(false);
		Spline->SetSplinePoints(Points, ESplineCoordinateSpace::World, true);
		Spline->SetVisibility(bVisible);
		Spline->SetHiddenInGame(!bVisible);
	};
	SetPoints(Centerline, Center, true);
	const bool bHasRoadway = RoadMaxOffset - RoadMinOffset > KINDA_SMALL_NUMBER;
	SetPoints(RoadLeftEdge, RoadLeft, bHasRoadway);
	SetPoints(RoadRightEdge, RoadRight, bHasRoadway);
	const bool bHasSidewalk = Segment.Profile && Segment.Profile->SidewalkWidth > KINDA_SMALL_NUMBER;
	SetPoints(SidewalkLeftEdge, WalkLeft, bHasSidewalk);
	SetPoints(SidewalkRightEdge, WalkRight, bHasSidewalk);
	if (PCGComponent)
	{
		PCGComponent->Refresh(EPCGChangeType::Input, true);
	}
}
