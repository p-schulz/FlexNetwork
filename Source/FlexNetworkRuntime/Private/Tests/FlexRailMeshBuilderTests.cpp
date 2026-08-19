#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "RoadTypeProfile.h"
#include "Rail/FlexRailGraph.h"
#include "Mesh/FlexRailMeshBuilder.h"

namespace FlexRailMeshBuilderTestHelpers
{
	URoadTypeProfile* MakeTramProfile(bool bGrooved)
	{
		URoadTypeProfile* Profile = NewObject<URoadTypeProfile>(GetTransientPackage());
		Profile->bIsRailProfile = true;
		Profile->RailGauge = 143.5f;
		Profile->RailWidth = 15.6f;
		Profile->RailTopWidth = 11.5f;
		Profile->RailHeight = 7.2f;
		Profile->bUseGroovedRailProfile = bGrooved;
		Profile->RailGrooveWidth = 4.f;
		Profile->RailGrooveDepth = 4.5f;
		Profile->RailGrooveInwardOffset = 1.5f;
		Profile->RailBooleanOverlap = 0.5f;
		return Profile;
	}

	/** A straight, flat edge running along +X, RailWidth to the +X extent so ArcLength matches X exactly. */
	FFlexRailEdge MakeStraightEdge(float Length, float SampleStep, bool bLeftRail, ERailEdgeType Type)
	{
		FFlexRailEdge Edge;
		Edge.bLeftRail = bLeftRail;
		Edge.Type = Type;
		for (float X = 0.f; X <= Length + KINDA_SMALL_NUMBER; X += SampleStep)
		{
			FFlexCurveFrame Frame;
			Frame.Position = FVector(X, bLeftRail ? -100.f : 100.f, 0.f);
			Frame.Tangent = FVector::ForwardVector;
			Frame.Right = FVector::RightVector;
			Frame.Up = FVector::UpVector;
			Frame.ArcLength = X;
			Edge.Frames.Add(Frame);
		}
		return Edge;
	}
}

// A simple ungrooved straight two-rail graph should sweep into a non-empty, plausible mesh.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexRailMeshBuilderStraightTest, "FlexNetwork.Rail.RailMeshBuilderStraightSolid", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexRailMeshBuilderStraightTest::RunTest(const FString& Parameters)
{
	using namespace FlexRailMeshBuilderTestHelpers;

	URoadTypeProfile* Profile = MakeTramProfile(false);
	FFlexRailGraph RailGraph;
	RailGraph.Edges.Add(MakeStraightEdge(2000.f, 100.f, true, ERailEdgeType::Normal));
	RailGraph.Edges.Add(MakeStraightEdge(2000.f, 100.f, false, ERailEdgeType::Normal));

	FFlexMeshSectionData Section;
	const bool bBuilt = FFlexRailMeshBuilder::BuildRailMesh(RailGraph, Profile, 4.f, Section);

	TestTrue(TEXT("Straight two-rail graph builds a mesh"), bBuilt);
	TestTrue(TEXT("Mesh has vertices"), Section.Vertices.Num() > 0);
	TestTrue(TEXT("Mesh has triangles"), Section.Triangles.Num() > 0);
	TestEqual(TEXT("Triangle index count is a multiple of 3"), Section.Triangles.Num() % 3, 0);

	return true;
}

// The same straight two-rail graph with the grooved profile enabled should still produce a
// non-empty mesh (the per-edge groove-cutter Difference boolean succeeds on a simple straight rail).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexRailMeshBuilderGroovedTest, "FlexNetwork.Rail.RailMeshBuilderGroovedSolid", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexRailMeshBuilderGroovedTest::RunTest(const FString& Parameters)
{
	using namespace FlexRailMeshBuilderTestHelpers;

	URoadTypeProfile* Profile = MakeTramProfile(true);
	FFlexRailGraph RailGraph;
	RailGraph.Edges.Add(MakeStraightEdge(2000.f, 100.f, true, ERailEdgeType::Normal));
	RailGraph.Edges.Add(MakeStraightEdge(2000.f, 100.f, false, ERailEdgeType::Normal));

	FFlexMeshSectionData Section;
	const bool bBuilt = FFlexRailMeshBuilder::BuildRailMesh(RailGraph, Profile, 4.f, Section);

	TestTrue(TEXT("Grooved straight two-rail graph builds a mesh"), bBuilt);
	TestTrue(TEXT("Grooved mesh has triangles"), Section.Triangles.Num() > 0);

	return true;
}

// A Crossing edge shorter than the configured gap has nothing left to sweep on that edge; it
// should be silently skipped rather than crash or produce degenerate geometry.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexRailMeshBuilderCrossingGapTest, "FlexNetwork.Rail.RailMeshBuilderCrossingGapDropsShortEdge", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexRailMeshBuilderCrossingGapTest::RunTest(const FString& Parameters)
{
	using namespace FlexRailMeshBuilderTestHelpers;

	URoadTypeProfile* Profile = MakeTramProfile(false);
	FFlexRailGraph RailGraph;
	// A 3cm-long Crossing edge with a 20cm total gap: entirely consumed by the trim.
	RailGraph.Edges.Add(MakeStraightEdge(3.f, 1.f, true, ERailEdgeType::Crossing));
	RailGraph.Edges.Add(MakeStraightEdge(3.f, 1.f, false, ERailEdgeType::Crossing));

	FFlexMeshSectionData Section;
	const bool bBuilt = FFlexRailMeshBuilder::BuildRailMesh(RailGraph, Profile, 20.f, Section);

	TestFalse(TEXT("A crossing edge fully consumed by its gap contributes no geometry"), bBuilt);
	TestTrue(TEXT("Section is left empty"), Section.IsEmpty());

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
