#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "FlexNetworkSubsystem.h"
#include "RoadTypeProfile.h"
#include "Rail/FlexTrackJunctionSolver.h"
#include "Math/FlexBezierMath.h"
#include "Editor.h"
#include "Engine/World.h"

namespace FlexTrackJunctionSolverTestHelpers
{
	URoadTypeProfile* MakeTramProfile(float MinTurnRadius)
	{
		URoadTypeProfile* Profile = NewObject<URoadTypeProfile>(GetTransientPackage());
		Profile->bIsRailProfile = true;
		Profile->RailGauge = 143.5f;
		Profile->RailWidth = 15.6f;
		Profile->MinTurnRadius = MinTurnRadius;
		Profile->SidewalkWidth = 0.f;
		Profile->CurbHeight = 0.f;

		FRoadLaneDescriptor Track;
		Track.Type = EFlexLaneType::Rail;
		Track.Direction = EFlexLaneDirection::Bidirectional;
		Track.Width = Profile->RailGauge + Profile->RailWidth;
		Profile->Lanes.Add(Track);
		return Profile;
	}
}

// A gently-diverging three-way turnout (well within tram minimum-radius reach) should resolve
// every port pair into a movement, and none of those movements should violate MinimumRadius.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexTrackJunctionSolverTest, "FlexNetwork.Rail.TrackJunctionSolverGentleTurnout", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FFlexTrackJunctionSolverTest::RunTest(const FString& Parameters)
{
	using namespace FlexTrackJunctionSolverTestHelpers;

	if (!GEditor)
	{
		AddError(TEXT("Test requires a running editor context (GEditor)."));
		return false;
	}
	UWorld* World = GEditor->GetEditorWorldContext().World();
	if (!TestNotNull(TEXT("Editor world available"), World))
	{
		return false;
	}
	UFlexNetworkSubsystem* Subsystem = World->GetSubsystem<UFlexNetworkSubsystem>();
	if (!TestNotNull(TEXT("UFlexNetworkSubsystem present"), Subsystem))
	{
		return false;
	}

	const float MinTurnRadius = 1800.f; // ~18m, a typical tram minimum radius.
	URoadTypeProfile* TramProfile = MakeTramProfile(MinTurnRadius);

	const FFlexNodeId Common = Subsystem->AddNode(FVector(0.f, 0.f, 0.f));
	const FFlexNodeId Hub = Subsystem->AddNode(FVector(4000.f, 0.f, 0.f));
	const FFlexNodeId BranchA = Subsystem->AddNode(FVector(9000.f, 2500.f, 0.f));
	const FFlexNodeId BranchB = Subsystem->AddNode(FVector(9000.f, -2500.f, 0.f));
	Subsystem->AddSegment(Common, Hub, FVector(1300.f, 0.f, 0.f), FVector(2700.f, 0.f, 0.f), TramProfile);
	Subsystem->AddSegment(Hub, BranchA, FVector(5700.f, 400.f, 0.f), FVector(7300.f, 1600.f, 0.f), TramProfile);
	Subsystem->AddSegment(Hub, BranchB, FVector(5700.f, -400.f, 0.f), FVector(7300.f, -1600.f, 0.f), TramProfile);

	const FFlexTrackJunction Junction = FFlexTrackJunctionSolver::Solve(Hub, *Subsystem, TramProfile,
		1000.f /*InitialTrimDistance*/, 2000.f /*MaxTrimDistance*/, 300.f /*TrimDistanceStep*/, 4 /*MaxIterations*/);

	TestEqual(TEXT("All three approaches produce a port"), Junction.Ports.Num(), 3);
	TestEqual(TEXT("Every port pair resolves into a movement for a gentle turnout"), Junction.Movements.Num(), 3);

	for (const FFlexTrackMovement& Movement : Junction.Movements)
	{
		bool bRadiusOk = true;
		for (int32 Step = 1; Step < 10; ++Step)
		{
			const float T = static_cast<float>(Step) / 10.f;
			const float Curvature = FFlexBezierMath::EstimateCurvature(Movement.Centerline, T);
			if (Curvature > 1.f / MinTurnRadius + KINDA_SMALL_NUMBER)
			{
				bRadiusOk = false;
				break;
			}
		}
		TestTrue(TEXT("Movement curve never tightens below MinimumRadius"), bRadiusOk);
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
