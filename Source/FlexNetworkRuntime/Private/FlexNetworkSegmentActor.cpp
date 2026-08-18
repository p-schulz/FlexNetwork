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
	const float RoadExtent = Segment.Profile ? Segment.Profile->GetRoadwayHalfWidth() : 0.f;
	const float SidewalkExtent = Segment.Profile ? RoadExtent + Segment.Profile->SidewalkWidth : RoadExtent;
	const TArray<FFlexCurveFrame> Frames = FFlexRoadMeshBuilder::BuildFramesForRange(
		Segment.Curve, Segment.ArcLengthTable, ReferenceUp, SampleStep, TrimStartArcLength, TrimEndArcLength);

	TArray<FVector> Center, RoadLeft, RoadRight, WalkLeft, WalkRight;
	Center.Reserve(Frames.Num()); RoadLeft.Reserve(Frames.Num()); RoadRight.Reserve(Frames.Num());
	WalkLeft.Reserve(Frames.Num()); WalkRight.Reserve(Frames.Num());
	for (const FFlexCurveFrame& Frame : Frames)
	{
		Center.Add(Frame.Position);
		RoadLeft.Add(Frame.Position - Frame.Right * RoadExtent);
		RoadRight.Add(Frame.Position + Frame.Right * RoadExtent);
		WalkLeft.Add(Frame.Position - Frame.Right * SidewalkExtent);
		WalkRight.Add(Frame.Position + Frame.Right * SidewalkExtent);
	}

	auto SetPoints = [](USplineComponent* Spline, const TArray<FVector>& Points, bool bVisible)
	{
		Spline->ClearSplinePoints(false);
		Spline->SetSplinePoints(Points, ESplineCoordinateSpace::World, true);
		Spline->SetVisibility(bVisible);
		Spline->SetHiddenInGame(!bVisible);
	};
	SetPoints(Centerline, Center, true);
	SetPoints(RoadLeftEdge, RoadLeft, RoadExtent > KINDA_SMALL_NUMBER);
	SetPoints(RoadRightEdge, RoadRight, RoadExtent > KINDA_SMALL_NUMBER);
	const bool bHasSidewalk = Segment.Profile && Segment.Profile->SidewalkWidth > KINDA_SMALL_NUMBER;
	SetPoints(SidewalkLeftEdge, WalkLeft, bHasSidewalk);
	SetPoints(SidewalkRightEdge, WalkRight, bHasSidewalk);
	if (PCGComponent)
	{
		PCGComponent->Refresh(EPCGChangeType::Input, true);
	}
}
