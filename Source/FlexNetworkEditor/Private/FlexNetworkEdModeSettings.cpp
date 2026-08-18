#include "FlexNetworkEdModeSettings.h"
#include "Osm/OsmDataAsset.h"
#include "Osm/FlexOsmGraphBuilder.h"
#include "FlexNetworkSubsystem.h"
#include "FlexNetworkAssetUtils.h"
#include "FlexNetworkMeshActor.h"
#include "RoadTypeProfile.h"
#include "Satellite/FlexSatelliteImagerySettings.h"
#include "Satellite/FlexSatelliteImport.h"
#include "Satellite/FlexSatelliteTileActor.h"
#include "Satellite/FlexSatelliteImageBaker.h"
#include "ScopedTransaction.h"
#include "Misc/ScopeExit.h"
#include "Engine/World.h"
#include "Engine/Texture2D.h"
#include "Materials/MaterialInterface.h"

#if WITH_EDITOR
void UFlexNetworkEdModeSettings::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
	if (PropertyChangedEvent.GetPropertyName() == GET_MEMBER_NAME_CHECKED(UFlexNetworkEdModeSettings, VisualizationMode))
	{
		if (UWorld* World = TargetWorld.Get())
		{
			if (UFlexNetworkSubsystem* Subsystem = World->GetSubsystem<UFlexNetworkSubsystem>())
			{
				Subsystem->SetVisualizationMode(VisualizationMode);
			}
		}
	}
}
#endif

void UFlexNetworkEdModeSettings::GenerateRoadsFromOsm()
{
	UWorld* World = TargetWorld.Get();
	if (!World)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: no world available to import into."));
		return;
	}

	UFlexNetworkSubsystem* Subsystem = World->GetSubsystem<UFlexNetworkSubsystem>();
	if (!Subsystem)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: UFlexNetworkSubsystem not available on this world."));
		return;
	}
	if (!OsmAsset)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: set an OSM Asset before generating roads."));
		return;
	}

	FScopedTransaction Transaction(NSLOCTEXT("FlexNetwork", "ImportOsm", "Import OSM Roads"));

	// Keeps this call's created/reused profiles across ways -- FFlexOsmGraphBuilder calls
	// ResolveProfile once per distinct lane signature, but this cache is what actually makes that
	// true (without it every way would trigger a new asset creation, even for a lane layout
	// already generated earlier in the same import).
	TMap<FString, URoadTypeProfile*> ProfileCache;

	FFlexOsmGraphBuilder::FImportResult Result;
	{
		// SetVisualizationMode can itself dirty every existing graph item. Keep it in the same
		// outer batch as the OSM mutations so a mode switch cannot cause a pre-import rebuild.
		Subsystem->BeginBatchUpdate();
		ON_SCOPE_EXIT
		{
			Subsystem->EndBatchUpdate();
		};

		Subsystem->SetVisualizationMode(VisualizationMode);
		Result = FFlexOsmGraphBuilder::BuildFromOsm(*Subsystem, *OsmAsset, OsmImportSettings,
			[&ProfileCache](const FFlexOsmGraphBuilder::FLaneSignature& Signature) -> URoadTypeProfile*
			{
				const FString Key = Signature.ToKey();
				if (URoadTypeProfile** Existing = ProfileCache.Find(Key))
				{
					return *Existing;
				}

				URoadTypeProfile* NewProfile = FlexNetworkAssetUtils::CreateRoadTypeProfileAsset(
					TEXT("/FlexNetwork/Profiles/OSM"),
					TEXT("DA_OSM_") + Key,
					[&Signature](URoadTypeProfile& Profile) { FFlexOsmGraphBuilder::ConfigureProfileFromLaneSignature(Profile, Signature); });

				if (NewProfile)
				{
					ProfileCache.Add(Key, NewProfile);
				}
				return NewProfile;
			});
	}

	for (const FString& Warning : Result.Warnings)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork OSM import: %s"), *Warning);
	}

	UE_LOG(LogTemp, Display, TEXT("FlexNetwork: OSM import complete -- %d way(s) imported, %d segment(s)/%d node(s) created, %d junction(s) merged, %d distinct lane profile(s) generated under /FlexNetwork/Profiles/OSM/."),
		Result.NumWaysImported, Result.NumSegmentsCreated, Result.NumNodesCreated, Result.NumJunctionsMerged, Result.NumDistinctLaneSignatures);
}

void UFlexNetworkEdModeSettings::RebuildAllNetworkGeometry()
{
	UWorld* World = TargetWorld.Get();
	UFlexNetworkSubsystem* Subsystem = World ? World->GetSubsystem<UFlexNetworkSubsystem>() : nullptr;
	if (!Subsystem)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: no world subsystem available to rebuild."));
		return;
	}
	Subsystem->RebuildAllNetworkGeometry();
	UE_LOG(LogTemp, Display, TEXT("FlexNetwork: rebuilt all network geometry and segment sources."));
}

void UFlexNetworkEdModeSettings::ConformTerrainToRoads()
{
	UWorld* World = TargetWorld.Get();
	if (!World)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: no world available to conform terrain in."));
		return;
	}

	UFlexNetworkSubsystem* Subsystem = World->GetSubsystem<UFlexNetworkSubsystem>();
	if (!Subsystem)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: UFlexNetworkSubsystem not available on this world."));
		return;
	}

	FScopedTransaction Transaction(NSLOCTEXT("FlexNetwork", "ConformTerrain", "Conform Terrain To Roads"));
	Subsystem->ConformAllTerrainToRoads();
	UE_LOG(LogTemp, Display, TEXT("FlexNetwork: terrain conformed to Ground roads."));
}

void UFlexNetworkEdModeSettings::FitRoadsToTerrain()
{
	UWorld* World = TargetWorld.Get();
	if (!World)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: no world available to fit roads in."));
		return;
	}

	UFlexNetworkSubsystem* Subsystem = World->GetSubsystem<UFlexNetworkSubsystem>();
	if (!Subsystem)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: UFlexNetworkSubsystem not available on this world."));
		return;
	}

	FScopedTransaction Transaction(NSLOCTEXT("FlexNetwork", "FitToTerrain", "Fit Roads To Terrain"));
	Subsystem->FitNodesToTerrain();
	UE_LOG(LogTemp, Display, TEXT("FlexNetwork: Ground roads fitted to the landscape surface."));
}

void UFlexNetworkEdModeSettings::ApplyMaterialsToAllProfiles()
{
	if (!StandardRoadMaterial && !StandardSidewalkMaterial && !StandardCrosswalkMaterial && !StandardJunctionMaterial && !StandardMedianMaterial)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: set at least one Standard*Material before applying."));
		return;
	}

	const int32 NumModified = FlexNetworkAssetUtils::ApplyMaterialsToAllProfiles(StandardRoadMaterial, StandardSidewalkMaterial, StandardCrosswalkMaterial, StandardJunctionMaterial, StandardMedianMaterial);
	UE_LOG(LogTemp, Display, TEXT("FlexNetwork: applied materials to %d road profile asset(s)."), NumModified);
}

void UFlexNetworkEdModeSettings::GenerateCurbstones()
{
	UWorld* World = TargetWorld.Get();
	if (!World)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: no world available to generate curbstones in."));
		return;
	}

	UFlexNetworkSubsystem* Subsystem = World->GetSubsystem<UFlexNetworkSubsystem>();
	if (!Subsystem)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: UFlexNetworkSubsystem not available on this world."));
		return;
	}

	if (!CurbstoneMesh)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: set a Curbstone Mesh before generating curbstones."));
		return;
	}

	AFlexNetworkMeshActor* MeshActor = Subsystem->GetMeshActor();
	if (!MeshActor)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: no mesh actor to host curbstones on yet -- generate some roads first."));
		return;
	}

	FScopedTransaction Transaction(NSLOCTEXT("FlexNetwork", "GenerateCurbstones", "Generate Curbstones"));
	MeshActor->Modify();
	const TArray<TArray<FVector>>& CurbLines = MeshActor->GetUnifiedCurbLines();
	MeshActor->ApplyCurbstones(CurbLines, CurbstoneMesh);
	UE_LOG(LogTemp, Display, TEXT("FlexNetwork: generated curbstones along %d curb line(s)."), CurbLines.Num());
}

void UFlexNetworkEdModeSettings::ImportSatelliteImagery()
{
	UWorld* World = TargetWorld.Get();
	if (!World)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: no world available to import satellite imagery into."));
		return;
	}
	if (!OsmAsset)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: set an OSM Asset before importing satellite imagery."));
		return;
	}

	// Clear out any previously-spawned tiles so re-running this doesn't pile up duplicates.
	for (const TWeakObjectPtr<AFlexSatelliteTileActor>& Tile : SpawnedSatelliteTiles)
	{
		if (AFlexSatelliteTileActor* TileActor = Tile.Get())
		{
			TileActor->Destroy();
		}
	}
	SpawnedSatelliteTiles.Reset();

	const UFlexSatelliteImagerySettings* ImagerySettings = GetDefault<UFlexSatelliteImagerySettings>();

	// Fetches are asynchronous HTTP requests that resolve after this function returns, so unlike
	// GenerateRoadsFromOsm above there's no single synchronous edit to wrap in a transaction here --
	// each tile actor's own SpawnActor call is undoable on its own once it happens.
	FlexNetwork::Satellite::ImportSatelliteImagery(World, *OsmAsset, OsmImportSettings, *ImagerySettings, SpawnedSatelliteTiles,
		[](FlexNetwork::Satellite::FImportResult Result)
		{
			for (const FString& Warning : Result.Warnings)
			{
				UE_LOG(LogTemp, Warning, TEXT("FlexNetwork satellite import: %s"), *Warning);
			}
			UE_LOG(LogTemp, Display, TEXT("FlexNetwork: satellite import complete -- %d/%d tile(s) spawned. (c) LGL Baden-Wuerttemberg (www.lgl-bw.de), Datenlizenz Deutschland - Namensnennung 2.0."), Result.NumTilesSpawned, Result.NumTilesRequested);
		});

	UE_LOG(LogTemp, Display, TEXT("FlexNetwork: satellite import started -- fetching tiles asynchronously, see the Output Log for completion."));
}

void UFlexNetworkEdModeSettings::BakeSatelliteImageryToContent()
{
	if (SpawnedSatelliteTiles.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: no satellite tiles to bake -- run 'Import Satellite/Landuse Imagery' first (and wait for it to finish -- check the Output Log)."));
		return;
	}

	const UFlexSatelliteImagerySettings* ImagerySettings = GetDefault<UFlexSatelliteImagerySettings>();
	UMaterialInterface* BaseLandscapeMaterial = ImagerySettings->BaseLandscapeMaterial.LoadSynchronous();

	const FString AssetFolderName = OsmAsset ? OsmAsset->GetName() : TEXT("Untitled");
	const FString PackagePath = ImagerySettings->BakePackagePath.Path / AssetFolderName;
	const int32 ResizeToSquareSize = ImagerySettings->bResizeToPowerOfTwo ? ImagerySettings->BakeTextureSize : 0;

	int32 NumBaked = 0;
	int32 TileIndex = 0;
	for (const TWeakObjectPtr<AFlexSatelliteTileActor>& TileWeak : SpawnedSatelliteTiles)
	{
		AFlexSatelliteTileActor* Tile = TileWeak.Get();
		if (!Tile)
		{
			continue;
		}
		++TileIndex;

		UTexture2D* BakedAerial = Tile->AerialTexture
			? FlexNetwork::Satellite::BakeTextureToContent(Tile->AerialTexture, PackagePath, FString::Printf(TEXT("T_Aerial_%03d"), TileIndex), ImagerySettings->bAerialTextureSRGB, ResizeToSquareSize)
			: nullptr;
		if (!BakedAerial)
		{
			continue;
		}
		++NumBaked;

		UTexture2D* BakedLandUse = Tile->LandUseTexture
			? FlexNetwork::Satellite::BakeTextureToContent(Tile->LandUseTexture, PackagePath, FString::Printf(TEXT("T_LandUse_%03d"), TileIndex), ImagerySettings->bLandUseTextureSRGB, ResizeToSquareSize)
			: nullptr;

		if (BaseLandscapeMaterial)
		{
			TMap<FName, UTexture*> TextureParams;
			TextureParams.Add(ImagerySettings->AerialLayer.TextureParameterName, BakedAerial);
			TMap<FName, float> ScalarParams;
			if (BakedLandUse)
			{
				TextureParams.Add(ImagerySettings->LandUseLayer.TextureParameterName, BakedLandUse);
				ScalarParams.Add(ImagerySettings->LandUseOpacityParameterName, static_cast<float>(ImagerySettings->LandUseOpacity));
			}
			FlexNetwork::Satellite::CreateLandscapeMaterialInstance(BaseLandscapeMaterial, PackagePath, FString::Printf(TEXT("MI_SatelliteTile_%03d"), TileIndex), TextureParams, ScalarParams);
		}
	}

	UE_LOG(LogTemp, Display, TEXT("FlexNetwork: baked %d satellite tile texture set(s) to %s."), NumBaked, *PackagePath);
}
