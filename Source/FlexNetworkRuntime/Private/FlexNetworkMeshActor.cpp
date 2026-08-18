#include "FlexNetworkMeshActor.h"
#include "ProceduralMeshComponent.h"
#include "FlexNetworkSettings.h"
#include "Components/SplineMeshComponent.h"

namespace
{
	constexpr int32 kRoadwaySection = 0;
	constexpr int32 kSidewalkSection = 1;
	constexpr int32 kJunctionSurfaceSection = 0;
	constexpr int32 kJunctionCrosswalkSection = 1;
	constexpr int32 kJunctionSidewalkCornerSection = 2;
	constexpr int32 kJunctionCornerIslandSection = 3;
}

AFlexNetworkMeshActor::AFlexNetworkMeshActor()
{
	PrimaryActorTick.bCanEverTick = false;
	bReplicates = false;

	RootSceneComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(RootSceneComponent);
}

UProceduralMeshComponent* AFlexNetworkMeshActor::GetOrCreateComponent(TMap<FFlexSegmentId, TObjectPtr<UProceduralMeshComponent>>& Map, FFlexSegmentId Id, const TCHAR* NamePrefix)
{
	if (TObjectPtr<UProceduralMeshComponent>* Found = Map.Find(Id))
	{
		return *Found;
	}

	UProceduralMeshComponent* Comp = NewObject<UProceduralMeshComponent>(this, *FString::Printf(TEXT("%s_%u_%u"), NamePrefix, Id.Index, Id.Generation));
	Comp->SetupAttachment(RootSceneComponent);
	Comp->RegisterComponent();
	Comp->SetMobility(EComponentMobility::Movable);
	// Raises the whole component (and so every section it hosts) a hair above the mesh's own
	// logical height, which is what terrain conforming flattens the landscape *to* -- without
	// this, road/junction meshes and the landscape underneath are exactly coplanar and z-fight.
	Comp->SetRelativeLocation(FVector(0.f, 0.f, GetDefault<UFlexNetworkSettings>()->MeshZFightOffset));
	Map.Add(Id, Comp);
	return Comp;
}

UProceduralMeshComponent* AFlexNetworkMeshActor::GetOrCreateComponent(TMap<FFlexNodeId, TObjectPtr<UProceduralMeshComponent>>& Map, FFlexNodeId Id, const TCHAR* NamePrefix)
{
	if (TObjectPtr<UProceduralMeshComponent>* Found = Map.Find(Id))
	{
		return *Found;
	}

	UProceduralMeshComponent* Comp = NewObject<UProceduralMeshComponent>(this, *FString::Printf(TEXT("%s_%u_%u"), NamePrefix, Id.Index, Id.Generation));
	Comp->SetupAttachment(RootSceneComponent);
	Comp->RegisterComponent();
	Comp->SetMobility(EComponentMobility::Movable);
	Comp->SetRelativeLocation(FVector(0.f, 0.f, GetDefault<UFlexNetworkSettings>()->MeshZFightOffset));
	Map.Add(Id, Comp);
	return Comp;
}

void AFlexNetworkMeshActor::ApplySectionData(UProceduralMeshComponent* Comp, int32 SectionIndex, const FFlexMeshSectionData& Data)
{
	if (!Comp)
	{
		return;
	}

	if (Data.IsEmpty())
	{
		Comp->ClearMeshSection(SectionIndex);
		return;
	}

	// FlexNetwork's procedural sections use the same convention as the existing junction builder:
	// a visible face has geometric winding opposite its supplied shading normal. Boolean/Delaunay
	// triangulators are free to return either orientation per triangle, so normalize here at the
	// renderer boundary. Without this, a strip can render exactly one triangle from each quad and
	// leave the alternating saw-tooth holes visible along its outer edge.
	TArray<int32> NormalizedTriangles = Data.Triangles;
	for (int32 TriangleIndex = 0; TriangleIndex + 2 < NormalizedTriangles.Num(); TriangleIndex += 3)
	{
		const int32 A = NormalizedTriangles[TriangleIndex];
		const int32 B = NormalizedTriangles[TriangleIndex + 1];
		const int32 C = NormalizedTriangles[TriangleIndex + 2];
		if (!Data.Vertices.IsValidIndex(A) || !Data.Vertices.IsValidIndex(B) || !Data.Vertices.IsValidIndex(C)
			|| !Data.Normals.IsValidIndex(A) || !Data.Normals.IsValidIndex(B) || !Data.Normals.IsValidIndex(C))
		{
			continue;
		}
		const FVector DesiredNormal = (Data.Normals[A] + Data.Normals[B] + Data.Normals[C]).GetSafeNormal();
		const FVector GeometricNormal = FVector::CrossProduct(Data.Vertices[B] - Data.Vertices[A], Data.Vertices[C] - Data.Vertices[A]);
		if (!DesiredNormal.IsNearlyZero() && FVector::DotProduct(GeometricNormal, DesiredNormal) > 0.f)
		{
			Swap(NormalizedTriangles[TriangleIndex + 1], NormalizedTriangles[TriangleIndex + 2]);
		}
	}

	Comp->CreateMeshSection(
		SectionIndex,
		Data.Vertices,
		NormalizedTriangles,
		Data.Normals,
		Data.UV0,
		TArray<FVector2D>(),
		TArray<FVector2D>(),
		TArray<FVector2D>(),
		Data.VertexColors,
		Data.Tangents,
		Data.bEnableCollision);

	// Set null as well: section indices are reassigned when material groups change, so retaining
	// an older slot material would incorrectly skin a newly-created unified section.
	Comp->SetMaterial(SectionIndex, Data.Material);
}

void AFlexNetworkMeshActor::ApplySegmentMesh(FFlexSegmentId SegmentId, const FFlexSegmentMeshResult& MeshResult)
{
	UProceduralMeshComponent* Comp = GetOrCreateComponent(SegmentComponents, SegmentId, TEXT("Segment"));
	ApplySectionData(Comp, kRoadwaySection, MeshResult.Roadway);
	ApplySectionData(Comp, kSidewalkSection, MeshResult.Sidewalks);
}

void AFlexNetworkMeshActor::RemoveSegmentMesh(FFlexSegmentId SegmentId)
{
	if (TObjectPtr<UProceduralMeshComponent> Comp; SegmentComponents.RemoveAndCopyValue(SegmentId, Comp) && Comp)
	{
		Comp->DestroyComponent();
	}
}

void AFlexNetworkMeshActor::ApplyJunctionMesh(FFlexNodeId NodeId, const FFlexJunctionMeshResult& MeshResult)
{
	UProceduralMeshComponent* Comp = GetOrCreateComponent(JunctionComponents, NodeId, TEXT("Junction"));
	ApplySectionData(Comp, kJunctionSurfaceSection, MeshResult.Surface);
	ApplySectionData(Comp, kJunctionCrosswalkSection, MeshResult.Crosswalks);
	ApplySectionData(Comp, kJunctionSidewalkCornerSection, MeshResult.SidewalkCorners);
	ApplySectionData(Comp, kJunctionCornerIslandSection, MeshResult.CornerIslands);
}

void AFlexNetworkMeshActor::RemoveJunctionMesh(FFlexNodeId NodeId)
{
	if (TObjectPtr<UProceduralMeshComponent> Comp; JunctionComponents.RemoveAndCopyValue(NodeId, Comp) && Comp)
	{
		Comp->DestroyComponent();
	}
}

void AFlexNetworkMeshActor::ApplyUnifiedNetworkMesh(const FFlexUnifiedNetworkMeshResult& MeshResult)
{
	UnifiedCurbLines = MeshResult.CurbLines;
	// The topology-first renderer supersedes the old per-segment road/sidewalk components.
	for (const TPair<FFlexSegmentId, TObjectPtr<UProceduralMeshComponent>>& Pair : SegmentComponents)
	{
		if (Pair.Value)
		{
			Pair.Value->DestroyComponent();
		}
	}
	SegmentComponents.Reset();
	for (USplineMeshComponent* Comp : CurbstoneComponents)
	{
		if (Comp)
		{
			Comp->DestroyComponent();
		}
	}
	CurbstoneComponents.Reset();

	// Junction components remain useful for crosswalk overlays, but their old surface and
	// independently-generated roadside layers must not overlap the unified result.
	for (const TPair<FFlexNodeId, TObjectPtr<UProceduralMeshComponent>>& Pair : JunctionComponents)
	{
		if (Pair.Value)
		{
			Pair.Value->ClearMeshSection(kJunctionSurfaceSection);
			Pair.Value->ClearMeshSection(kJunctionSidewalkCornerSection);
			Pair.Value->ClearMeshSection(kJunctionCornerIslandSection);
		}
	}

	if (!UnifiedNetworkComponent)
	{
		UnifiedNetworkComponent = NewObject<UProceduralMeshComponent>(this, TEXT("UnifiedNetwork"));
		UnifiedNetworkComponent->SetupAttachment(RootSceneComponent);
		UnifiedNetworkComponent->RegisterComponent();
		UnifiedNetworkComponent->SetMobility(EComponentMobility::Movable);
		UnifiedNetworkComponent->SetRelativeLocation(FVector(0.f, 0.f, GetDefault<UFlexNetworkSettings>()->MeshZFightOffset));
	}

	UnifiedNetworkComponent->ClearAllMeshSections();
	int32 SectionIndex = 0;
	for (const FFlexMeshSectionData& Section : MeshResult.Roadways)
	{
		ApplySectionData(UnifiedNetworkComponent, SectionIndex++, Section);
	}
	for (const FFlexMeshSectionData& Section : MeshResult.Sidewalks)
	{
		ApplySectionData(UnifiedNetworkComponent, SectionIndex++, Section);
	}
	for (const FFlexMeshSectionData& Section : MeshResult.Curbs)
	{
		ApplySectionData(UnifiedNetworkComponent, SectionIndex++, Section);
	}
}

void AFlexNetworkMeshActor::ApplyCurbstones(const TArray<TArray<FVector>>& CurbLines, UStaticMesh* Mesh)
{
	for (USplineMeshComponent* Comp : CurbstoneComponents)
	{
		if (Comp)
		{
			Comp->DestroyComponent();
		}
	}
	CurbstoneComponents.Reset();

	if (!Mesh)
	{
		return;
	}

	int32 NameCounter = 0;
	for (const TArray<FVector>& Line : CurbLines)
	{
		for (int32 i = 0; i + 1 < Line.Num(); ++i)
		{
			const FVector& Start = Line[i];
			const FVector& End = Line[i + 1];
			// A straight chord between consecutive samples rather than the curve's own tangent --
			// simple, and the curb line is already sampled finely enough (per-segment arc-length
			// step, or per-corner arc-segment count) that the chord and the true tangent are
			// visually indistinguishable at that spacing.
			const FVector Tangent = End - Start;
			if (Tangent.IsNearlyZero())
			{
				continue;
			}

			USplineMeshComponent* Comp = NewObject<USplineMeshComponent>(this, *FString::Printf(TEXT("Curbstone_%d"), NameCounter++));
			Comp->SetMobility(EComponentMobility::Movable);
			Comp->SetStaticMesh(Mesh);
			Comp->SetForwardAxis(ESplineMeshAxis::X);
			Comp->SetupAttachment(RootSceneComponent);
			Comp->RegisterComponent();
			Comp->SetStartAndEnd(Start, Tangent, End, Tangent, true);
			CurbstoneComponents.Add(Comp);
		}
	}
}

void AFlexNetworkMeshActor::ClearAll()
{
	if (UnifiedNetworkComponent)
	{
		UnifiedNetworkComponent->DestroyComponent();
		UnifiedNetworkComponent = nullptr;
	}
	UnifiedCurbLines.Reset();

	for (const TPair<FFlexSegmentId, TObjectPtr<UProceduralMeshComponent>>& Pair : SegmentComponents)
	{
		if (Pair.Value)
		{
			Pair.Value->DestroyComponent();
		}
	}
	SegmentComponents.Reset();

	for (const TPair<FFlexNodeId, TObjectPtr<UProceduralMeshComponent>>& Pair : JunctionComponents)
	{
		if (Pair.Value)
		{
			Pair.Value->DestroyComponent();
		}
	}
	JunctionComponents.Reset();

	for (USplineMeshComponent* Comp : CurbstoneComponents)
	{
		if (Comp)
		{
			Comp->DestroyComponent();
		}
	}
	CurbstoneComponents.Reset();
}
