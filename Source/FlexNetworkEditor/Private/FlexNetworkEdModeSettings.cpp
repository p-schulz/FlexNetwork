#include "FlexNetworkEdModeSettings.h"
#include "Osm/OsmDataAsset.h"
#include "Osm/FlexOsmGraphBuilder.h"
#include "Osm/FlexOsmImportContextActor.h"
#include "FlexNetworkSubsystem.h"
#include "FlexNetworkAssetUtils.h"
#include "FlexNetworkMeshActor.h"
#include "FlexNetworkBakeActor.h"
#include "FlexNetworkZoneGraphGenerator.h"
#include "FlexNetworkMassSignalExporter.h"
#include "MassTrafficLights.h"
#include "ZoneGraphData.h"
#include "RoadTypeProfile.h"
#include "Satellite/FlexSatelliteImagerySettings.h"
#include "Satellite/FlexSatelliteImport.h"
#include "Satellite/FlexSatelliteTileActor.h"
#include "Satellite/FlexSatelliteImageBaker.h"
#include "ScopedTransaction.h"
#include "Misc/ScopeExit.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Engine/Texture2D.h"
#include "Materials/MaterialInterface.h"
#include "Editor.h"
#include "EditorModeManager.h"

#if WITH_EDITOR
void UFlexNetworkEdModeSettings::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
	const FName PropertyName = PropertyChangedEvent.GetPropertyName();
	if (PropertyName == GET_MEMBER_NAME_CHECKED(UFlexNetworkEdModeSettings, VisualizationMode))
	{
		if (UWorld* World = TargetWorld.Get())
		{
			if (UFlexNetworkSubsystem* Subsystem = World->GetSubsystem<UFlexNetworkSubsystem>())
			{
				Subsystem->SetVisualizationMode(VisualizationMode);
			}
		}
	}

	if (PropertyName == GET_MEMBER_NAME_CHECKED(UFlexNetworkEdModeSettings, NodeEditTool)
		|| (PropertyName == GET_MEMBER_NAME_CHECKED(UFlexNetworkEdModeSettings, bDrawModeActive) && !bDrawModeActive))
	{
		GLevelEditorModeTools().SetWidgetMode(NodeEditTool == EFlexNetworkNodeEditTool::Rotate
			? UE::Widget::WM_Rotate
			: UE::Widget::WM_Translate);
		if (GEditor)
		{
			GEditor->RedrawLevelEditingViewports();
		}
	}
}
#endif

void UFlexNetworkEdModeSettings::PublishOsmContextToLevel()
{
	UWorld* World = TargetWorld.Get();
	if (!World || !OsmAsset)
	{
		OsmContextStatus = TEXT("Select an OSM asset and make sure an editor world is open.");
		return;
	}

	FScopedTransaction Transaction(NSLOCTEXT("FlexNetwork", "PublishOsmContext", "Publish Shared OSM Context"));
	AFlexOsmImportContextActor* Context = AFlexOsmImportContextActor::FindOrCreate(World);
	if (!Context)
	{
		OsmContextStatus = TEXT("Failed to create the level's shared OSM context actor.");
		return;
	}
	Context->SetContext(OsmAsset.Get(), OsmImportSettings);
	OsmContextStatus = Context->bHasResolvedOrigin
		? FString::Printf(TEXT("Published %s at origin %.8f, %.8f. Save the level to persist it."),
			*OsmAsset->GetName(), Context->ResolvedOriginLatLon.X, Context->ResolvedOriginLatLon.Y)
		: FString::Printf(TEXT("Published %s, but no projection origin could be resolved."), *OsmAsset->GetName());
}

void UFlexNetworkEdModeSettings::LoadOsmContextFromLevel()
{
	AFlexOsmImportContextActor* Context = AFlexOsmImportContextActor::Find(TargetWorld.Get());
	if (!Context || !Context->OsmAsset)
	{
		OsmContextStatus = TEXT("This level has no shared OSM context.");
		return;
	}
	OsmAsset = Context->OsmAsset;
	OsmImportSettings = Context->ImportSettings;
	OsmContextStatus = Context->bHasResolvedOrigin
		? FString::Printf(TEXT("Loaded %s at origin %.8f, %.8f."), *OsmAsset->GetName(),
			Context->ResolvedOriginLatLon.X, Context->ResolvedOriginLatLon.Y)
		: FString::Printf(TEXT("Loaded %s; its origin is unresolved."), *OsmAsset->GetName());
}

void UFlexNetworkEdModeSettings::GenerateRoadsFromOsm()
{
	GenerateTransportFromOsm(true, false);
}

void UFlexNetworkEdModeSettings::GenerateRailsFromOsm()
{
	GenerateTransportFromOsm(false, true);
}

void UFlexNetworkEdModeSettings::GenerateTransportFromOsm(const bool bIncludeRoads, const bool bIncludeRailways)
{
	if (!ensureMsgf(bIncludeRoads != bIncludeRailways,
		TEXT("The split OSM command must select exactly one transport type.")))
	{
		return;
	}

	UWorld* World = TargetWorld.Get();
	const TCHAR* ImportKind = bIncludeRoads ? TEXT("roads") : TEXT("rails");
	if (!World)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: no world available to import %s into."), ImportKind);
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
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: set an OSM Asset before generating %s."), ImportKind);
		return;
	}

	const FText TransactionText = bIncludeRoads
		? NSLOCTEXT("FlexNetwork", "ImportOsmRoads", "Import OSM Roads")
		: NSLOCTEXT("FlexNetwork", "ImportOsmRails", "Import OSM Rails");
	FScopedTransaction Transaction(TransactionText);
	if (bPublishSharedOsmContext)
	{
		if (AFlexOsmImportContextActor* Context = AFlexOsmImportContextActor::FindOrCreate(World))
		{
			Context->SetContext(OsmAsset.Get(), OsmImportSettings);
			OsmContextStatus = Context->bHasResolvedOrigin
				? FString::Printf(TEXT("Roads, rails, and buildings share origin %.8f, %.8f from %s."),
					Context->ResolvedOriginLatLon.X, Context->ResolvedOriginLatLon.Y, *OsmAsset->GetName())
				: TEXT("The OSM context was published, but its origin is unresolved.");
		}
	}

	FFlexOsmImportSettings FilteredSettings = OsmImportSettings;
	// Resolve from the complete configured transport set before disabling one filter. Files without
	// <bounds> otherwise choose the first matching way as their fallback origin, which would place
	// separately generated roads and rails in different local coordinate frames.
	double SharedOriginLat = 0.0;
	double SharedOriginLon = 0.0;
	if (FFlexOsmGraphBuilder::ResolveOrigin(*OsmAsset, OsmImportSettings, SharedOriginLat, SharedOriginLon))
	{
		FilteredSettings.bUseOriginOverride = true;
		FilteredSettings.OriginLatLon = FVector2D(SharedOriginLat, SharedOriginLon);
	}
	if (!bIncludeRoads)
	{
		FilteredSettings.HighwayTags.Reset();
	}
	if (!bIncludeRailways)
	{
		FilteredSettings.RailwayTags.Reset();
	}

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
		Result = FFlexOsmGraphBuilder::BuildFromOsm(*Subsystem, *OsmAsset, FilteredSettings,
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
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork OSM %s import: %s"), ImportKind, *Warning);
	}

	UE_LOG(LogTemp, Display, TEXT("FlexNetwork: OSM %s import complete -- %d way(s) imported (%d railway), %d segment(s)/%d node(s) created, %d traffic control(s), %d proximity junction(s) merged, %d complex intersection surface region(s) detected, %d distinct profile(s) generated under /FlexNetwork/Profiles/OSM/."),
		ImportKind,
		Result.NumWaysImported, Result.NumRailwayWaysImported, Result.NumSegmentsCreated, Result.NumNodesCreated, Result.NumTrafficControlsImported, Result.NumJunctionsMerged,
		Result.NumComplexIntersectionsCollapsed, Result.NumDistinctLaneSignatures);
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

void UFlexNetworkEdModeSettings::BakeNetworkToLevel()
{
	UWorld* World = TargetWorld.Get();
	UFlexNetworkSubsystem* Subsystem = World ? World->GetSubsystem<UFlexNetworkSubsystem>() : nullptr;
	if (!World || !Subsystem || Subsystem->GetAllNodes().IsEmpty())
	{
		BakeStatus = TEXT("Nothing to bake: the current editor world has no FlexNetwork graph.");
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: %s"), *BakeStatus);
		return;
	}

	FScopedTransaction Transaction(NSLOCTEXT("FlexNetwork", "BakeNetwork", "Bake FlexNetwork To Level"));
	AFlexNetworkBakeActor* BakeActor = nullptr;
	for (TActorIterator<AFlexNetworkBakeActor> It(World); It; ++It)
	{
		BakeActor = *It;
		break;
	}
	if (!BakeActor)
	{
		BakeActor = World->SpawnActor<AFlexNetworkBakeActor>();
	}
	if (!BakeActor)
	{
		BakeStatus = TEXT("Failed to spawn the persistent FlexNetwork bake actor.");
		UE_LOG(LogTemp, Error, TEXT("FlexNetwork: %s"), *BakeStatus);
		return;
	}

#if WITH_EDITOR
	BakeActor->SetActorLabel(TEXT("FlexNetwork_BakedGraph"));
#endif
	const int32 SegmentCount = BakeActor->CaptureFromSubsystem(*Subsystem);
	// CaptureFromSubsystem prefers the mesh that was actually generated. The toolkit selection is
	// a useful fallback when the user configured a curbstone mesh but has not run the manual pass
	// since the most recent network rebuild.
	if (!BakeActor->CurbstoneMesh && CurbstoneMesh)
	{
		BakeActor->CurbstoneMesh = CurbstoneMesh;
		BakeActor->bRestoreCurbstones = true;
		BakeActor->MarkPackageDirty();
	}
	const TCHAR* CurbSummary = BakeActor->bRestoreCurbstones && BakeActor->CurbstoneMesh
		? TEXT(" plus the configured curbstone mesh")
		: TEXT("");
	BakeStatus = FString::Printf(TEXT("Baked %d node(s), %d segment(s), and %d traffic control(s)%s into %s. Save the level to persist it."),
		Subsystem->GetAllNodes().Num(), SegmentCount, Subsystem->GetAllTrafficSignals().Num(), CurbSummary, *BakeActor->GetActorLabel());
	UE_LOG(LogTemp, Display, TEXT("FlexNetwork: %s"), *BakeStatus);
}

void UFlexNetworkEdModeSettings::RestoreBakedNetwork()
{
	UWorld* World = TargetWorld.Get();
	UFlexNetworkSubsystem* Subsystem = World ? World->GetSubsystem<UFlexNetworkSubsystem>() : nullptr;
	if (!World || !Subsystem)
	{
		BakeStatus = TEXT("No editor world or FlexNetwork subsystem is available.");
		return;
	}

	AFlexNetworkBakeActor* BakeActor = nullptr;
	for (TActorIterator<AFlexNetworkBakeActor> It(World); It; ++It)
	{
		BakeActor = *It;
		break;
	}
	if (!BakeActor || BakeActor->BakedNodes.IsEmpty())
	{
		BakeStatus = TEXT("No baked FlexNetwork snapshot exists in this level.");
		return;
	}

	FScopedTransaction Transaction(NSLOCTEXT("FlexNetwork", "RestoreBakedNetwork", "Restore Baked FlexNetwork"));
	const int32 SegmentCount = Subsystem->LoadBakedNetwork(
		BakeActor->BakedNodes, BakeActor->BakedSegments, BakeActor->BakedTrafficSignals, BakeActor->VisualizationMode);
	BakeActor->RestoreCurbstones(*Subsystem);
	const TCHAR* CurbSummary = BakeActor->bRestoreCurbstones && BakeActor->CurbstoneMesh
		? TEXT(" and curbstones")
		: TEXT("");
	BakeStatus = FString::Printf(TEXT("Restored %d node(s), %d segment(s), %d traffic control(s)%s from %s."),
		BakeActor->BakedNodes.Num(), SegmentCount, BakeActor->BakedTrafficSignals.Num(), CurbSummary, *BakeActor->GetActorLabel());
}

void UFlexNetworkEdModeSettings::RemoveNetworkBake()
{
	UWorld* World = TargetWorld.Get();
	if (!World)
	{
		BakeStatus = TEXT("No editor world is available.");
		return;
	}

	TArray<AFlexNetworkBakeActor*> BakeActors;
	for (TActorIterator<AFlexNetworkBakeActor> It(World); It; ++It)
	{
		BakeActors.Add(*It);
	}
	FScopedTransaction Transaction(NSLOCTEXT("FlexNetwork", "RemoveNetworkBake", "Remove FlexNetwork Bake"));
	int32 RemovedCount = 0;
	for (AFlexNetworkBakeActor* BakeActor : BakeActors)
	{
		BakeActor->Modify();
		if (BakeActor->Destroy())
		{
			++RemovedCount;
		}
	}
	BakeStatus = FString::Printf(TEXT("Removed %d baked snapshot actor(s). The currently loaded network was not cleared."), RemovedCount);
}

void UFlexNetworkEdModeSettings::GenerateMassAIZoneGraph()
{
	UWorld* World = TargetWorld.Get();
	UFlexNetworkSubsystem* Subsystem = World ? World->GetSubsystem<UFlexNetworkSubsystem>() : nullptr;
	if (!World || !Subsystem || Subsystem->GetAllSegments().IsEmpty())
	{
		ZoneGraphStatus = TEXT("Nothing to export: the current editor world has no FlexNetwork road segments.");
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: %s"), *ZoneGraphStatus);
		return;
	}

	FScopedTransaction Transaction(NSLOCTEXT("FlexNetwork", "GenerateMassAIZoneGraph", "Generate FlexNetwork Mass AI ZoneGraph"));
	FFlexZoneGraphGenerationOptions Options;
	Options.SampleSpacing = FMath::Max(ZoneGraphSampleSpacing, 10.f);
	Options.bIncludePedestrianLanes = bZoneGraphIncludePedestrians;
	Options.bReplaceExisting = bZoneGraphReplaceExisting;
	Options.bConfigureMassAI = bZoneGraphConfigureMassAI;

	const FFlexZoneGraphGenerationResult Result = FFlexNetworkZoneGraphGenerator::Generate(*Subsystem, *World, Options);
	if (bGenerateMassTrafficSignalsWithZoneGraph)
	{
		GenerateMassTrafficSignals();
	}
	if (!Result.DataActor)
	{
		ZoneGraphStatus = FString::Printf(TEXT("ZoneGraph generation produced no baked data (%d source shape(s)); %d previous actor(s) removed."),
			Result.GetShapeCount(), Result.RemovedActors);
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: %s"), *ZoneGraphStatus);
		return;
	}

	ZoneGraphStatus = FString::Printf(
		TEXT("Generated %d road, %d vehicle-connector, %d crosswalk, and %d sidewalk-corner shape(s); baked to %s. Removed %d previous actor(s). Save the level to persist it."),
		Result.SegmentShapes, Result.IntersectionShapes, Result.CrosswalkShapes, Result.PedestrianCornerShapes,
		*Result.DataActor->GetActorLabel(), Result.RemovedActors);
	UE_LOG(LogTemp, Display, TEXT("FlexNetwork: %s"), *ZoneGraphStatus);
}

void UFlexNetworkEdModeSettings::GenerateMassTrafficSignals()
{
	UWorld* World = TargetWorld.Get();
	UFlexNetworkSubsystem* Subsystem = World ? World->GetSubsystem<UFlexNetworkSubsystem>() : nullptr;
	if (!Subsystem)
	{
		MassTrafficSignalStatus = TEXT("No editor world or FlexNetwork subsystem is available.");
		return;
	}

	FFlexMassSignalExportOptions Options;
	Options.AssetName = MassTrafficLightInstancesAssetName;
	Options.TrafficLightTypes = MassTrafficLightTypes;
	const FFlexMassSignalExportResult Result = FFlexNetworkMassSignalExporter::GenerateOrUpdate(*Subsystem, Options);
	GeneratedMassTrafficLightTypes = Result.Types;
	GeneratedMassTrafficLightInstances = Result.Instances;
	if (!Result.Instances)
	{
		MassTrafficSignalStatus = TEXT("Failed to create or save the MassTraffic light instances asset.");
		UE_LOG(LogTemp, Error, TEXT("FlexNetwork: %s"), *MassTrafficSignalStatus);
		return;
	}

	MassTrafficSignalStatus = FString::Printf(
		TEXT("Generated %d MassTraffic light instance(s) in %s. Assign this Instances asset and %s as Types on the MassTraffic intersection spawn-data generator."),
		Result.NumTrafficLights, *Result.Instances->GetPathName(), Result.Types ? *Result.Types->GetPathName() : TEXT("the generated Types asset"));
	UE_LOG(LogTemp, Display, TEXT("FlexNetwork: %s"), *MassTrafficSignalStatus);
}

void UFlexNetworkEdModeSettings::RemoveMassAIZoneGraph()
{
	UWorld* World = TargetWorld.Get();
	if (!World)
	{
		ZoneGraphStatus = TEXT("No editor world is available.");
		return;
	}

	FScopedTransaction Transaction(NSLOCTEXT("FlexNetwork", "RemoveMassAIZoneGraph", "Remove FlexNetwork Mass AI ZoneGraph"));
	const int32 RemovedCount = FFlexNetworkZoneGraphGenerator::RemoveGeneratedActors(*World);
	ZoneGraphStatus = FString::Printf(TEXT("Removed %d FlexNetwork-generated ZoneGraph actor(s)."), RemovedCount);
	UE_LOG(LogTemp, Display, TEXT("FlexNetwork: %s"), *ZoneGraphStatus);
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
	if (!StandardRoadMaterial && !StandardSidewalkMaterial && !StandardCrosswalkMaterial && !StandardJunctionMaterial && !StandardMedianMaterial
		&& !StandardSolidMarkingMaterial && !StandardLaneDashMarkingMaterial && !StandardIntersectionDashMarkingMaterial && !StandardCrosswalkDashMarkingMaterial && !StandardBikeLaneMaterial && !StandardParkingMarkingMaterial && !StandardParkingLaneMaterial)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: set at least one Standard*Material before applying."));
		return;
	}

	const int32 NumModified = FlexNetworkAssetUtils::ApplyMaterialsToAllProfiles(StandardRoadMaterial, StandardSidewalkMaterial, StandardCrosswalkMaterial, StandardJunctionMaterial, StandardMedianMaterial,
		StandardSolidMarkingMaterial, StandardLaneDashMarkingMaterial, StandardIntersectionDashMarkingMaterial, StandardCrosswalkDashMarkingMaterial, StandardBikeLaneMaterial, StandardParkingMarkingMaterial, StandardParkingLaneMaterial);
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

	// The OSM road/rail generation commands above mutate synchronously and fit one transaction.
	// These HTTP fetches resolve after this function returns, so
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
