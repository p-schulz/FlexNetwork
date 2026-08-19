#include "Osm/FlexOsmImportContextActor.h"

#include "Osm/FlexOsmGraphBuilder.h"
#include "Osm/OsmDataAsset.h"
#include "Components/SceneComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"

AFlexOsmImportContextActor::AFlexOsmImportContextActor()
{
	PrimaryActorTick.bCanEverTick = false;
	bReplicates = false;
	SetIsSpatiallyLoaded(false);
	SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(SceneRoot);
}

void AFlexOsmImportContextActor::SetContext(UOsmDataAsset* InOsmAsset, const FFlexOsmImportSettings& InImportSettings)
{
	Modify();
	OsmAsset = InOsmAsset;
	ImportSettings = InImportSettings;
	bHasResolvedOrigin = OsmAsset && FFlexOsmGraphBuilder::ResolveOrigin(
		*OsmAsset, ImportSettings, ResolvedOriginLatLon.X, ResolvedOriginLatLon.Y);
	if (!bHasResolvedOrigin)
	{
		ResolvedOriginLatLon = FVector2D::ZeroVector;
	}
	++ContextRevision;
	MarkPackageDirty();
}

AFlexOsmImportContextActor* AFlexOsmImportContextActor::Find(UWorld* World)
{
	if (!World)
	{
		return nullptr;
	}
	for (TActorIterator<AFlexOsmImportContextActor> It(World); It; ++It)
	{
		return *It;
	}
	return nullptr;
}

AFlexOsmImportContextActor* AFlexOsmImportContextActor::FindOrCreate(UWorld* World)
{
	if (!World)
	{
		return nullptr;
	}
	if (AFlexOsmImportContextActor* Existing = Find(World))
	{
		return Existing;
	}

	FActorSpawnParameters SpawnParameters;
	SpawnParameters.ObjectFlags |= RF_Transactional;
	AFlexOsmImportContextActor* Context = World->SpawnActor<AFlexOsmImportContextActor>(
		AFlexOsmImportContextActor::StaticClass(), FTransform::Identity, SpawnParameters);
#if WITH_EDITOR
	if (Context)
	{
		Context->SetActorLabel(TEXT("FlexNetwork_OSM_Context"));
	}
#endif
	return Context;
}
