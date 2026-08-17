#include "FlexNetworkEdModeSettings.h"
#include "Osm/OsmDataAsset.h"
#include "Osm/FlexOsmGraphBuilder.h"
#include "FlexNetworkSubsystem.h"
#include "FlexNetworkAssetUtils.h"
#include "FlexNetworkSettings.h"
#include "FlexNetworkMeshActor.h"
#include "RoadTypeProfile.h"
#include "Mesh/FlexRoadMeshBuilder.h"
#include "Satellite/FlexSatelliteImagerySettings.h"
#include "Satellite/FlexSatelliteImport.h"
#include "Satellite/FlexSatelliteTileActor.h"
#include "Satellite/FlexSatelliteImageBaker.h"
#include "ScopedTransaction.h"
#include "Engine/World.h"
#include "Engine/Texture2D.h"
#include "Materials/MaterialInterface.h"

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

	const FFlexOsmGraphBuilder::FImportResult Result = FFlexOsmGraphBuilder::BuildFromOsm(*Subsystem, *OsmAsset, OsmImportSettings,
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

	for (const FString& Warning : Result.Warnings)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork OSM import: %s"), *Warning);
	}

	UE_LOG(LogTemp, Display, TEXT("FlexNetwork: OSM import complete -- %d way(s) imported, %d segment(s)/%d node(s) created, %d junction(s) merged, %d distinct lane profile(s) generated under /FlexNetwork/Profiles/OSM/."),
		Result.NumWaysImported, Result.NumSegmentsCreated, Result.NumNodesCreated, Result.NumJunctionsMerged, Result.NumDistinctLaneSignatures);
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
	if (!StandardRoadMaterial && !StandardSidewalkMaterial && !StandardJunctionMaterial && !StandardMedianMaterial)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: set at least one Standard*Material before applying."));
		return;
	}

	const int32 NumModified = FlexNetworkAssetUtils::ApplyMaterialsToAllProfiles(StandardRoadMaterial, StandardSidewalkMaterial, StandardJunctionMaterial, StandardMedianMaterial);
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

	const UFlexNetworkSettings* Settings = GetDefault<UFlexNetworkSettings>();

	TArray<TArray<FVector>> CurbLines;

	// One curb line per side of every road that has a sidewalk -- clipped to the same trimmed
	// range the road's own mesh uses at each end (JunctionData::TrimArcLengthBySegment, resolved
	// the same way UFlexNetworkSubsystem::RebuildDirty does for a segment's roadway trim), so the
	// curbstones stop at the junction boundary instead of continuing along the pre-trim curve
	// straight across the intersection surface.
	for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Subsystem->GetAllSegments())
	{
		const FFlexSegmentId SegId = Pair.Key;
		const FFlexRoadSegment& Segment = Pair.Value;
		if (!Segment.Profile || Segment.Profile->SidewalkWidth <= KINDA_SMALL_NUMBER || !Segment.ArcLengthTable.IsValid())
		{
			continue;
		}

		const FFlexRoadNode* StartNode = Subsystem->GetNode(Segment.StartNodeId);
		const FVector ReferenceUp = StartNode ? StartNode->UpVector : FVector::UpVector;
		const float RoadwayHalfWidth = Segment.Profile->GetRoadwayHalfWidth();
		const float SegmentLength = Segment.ArcLengthTable.GetTotalLength();

		float TrimStart = 0.f;
		float TrimEnd = SegmentLength;
		if (const FFlexJunctionData* StartJunction = Subsystem->GetJunctionData(Segment.StartNodeId))
		{
			if (const float* Trim = StartJunction->TrimArcLengthBySegment.Find(SegId))
			{
				TrimStart = *Trim;
			}
		}
		if (const FFlexJunctionData* EndJunction = Subsystem->GetJunctionData(Segment.EndNodeId))
		{
			if (const float* Trim = EndJunction->TrimArcLengthBySegment.Find(SegId))
			{
				TrimEnd = *Trim;
			}
		}

		const TArray<FFlexCurveFrame> Frames = FFlexRoadMeshBuilder::BuildFramesForRange(Segment.Curve, Segment.ArcLengthTable, ReferenceUp, Settings->ArcLengthSampleStep, TrimStart, TrimEnd);
		if (Frames.Num() < 2)
		{
			continue;
		}

		TArray<FVector> LeftLine, RightLine;
		LeftLine.Reserve(Frames.Num());
		RightLine.Reserve(Frames.Num());
		for (const FFlexCurveFrame& Frame : Frames)
		{
			LeftLine.Add(Frame.Position - Frame.Right * RoadwayHalfWidth);
			RightLine.Add(Frame.Position + Frame.Right * RoadwayHalfWidth);
		}
		CurbLines.Add(MoveTemp(LeftLine));
		CurbLines.Add(MoveTemp(RightLine));
	}

	// One curb line per contiguous run of genuine curb-line edges around each junction's drivable
	// polygon boundary -- NOT one single closed loop around the whole thing, since the boundary
	// also has a short "closing" edge at each approach's own near-node end (connecting that
	// approach's left curb point to its right one, running straight across the road rather than
	// along it -- see FFlexJunctionData::PolygonEdgeIsCurbLine). Placing a curbstone along a
	// closing edge would stretch a curb-profile mesh across the full road width instead of along
	// the curb, so those edges are skipped, splitting the ring into one open run per corner.
	for (const TPair<FFlexNodeId, FFlexRoadNode>& Pair : Subsystem->GetAllNodes())
	{
		const FFlexJunctionData* Junction = Subsystem->GetJunctionData(Pair.Key);
		const int32 NumVerts = Junction ? Junction->PolygonBoundary.Num() : 0;
		if (NumVerts < 3)
		{
			continue;
		}

		auto IsCurbEdge = [Junction](int32 EdgeIdx) { return !Junction->PolygonEdgeIsCurbLine.IsValidIndex(EdgeIdx) || Junction->PolygonEdgeIsCurbLine[EdgeIdx]; };

		// Start right after a closing edge (guaranteed to exist -- one per approach) so a single
		// linear pass around the ring, with no wraparound stitching, captures every run intact.
		int32 StartIdx = 0;
		for (int32 i = 0; i < NumVerts; ++i)
		{
			if (!IsCurbEdge((i - 1 + NumVerts) % NumVerts))
			{
				StartIdx = i;
				break;
			}
		}

		TArray<FVector> Current;
		Current.Add(Junction->PolygonBoundary[StartIdx]);
		for (int32 Step = 0; Step < NumVerts; ++Step)
		{
			const int32 Idx = (StartIdx + Step) % NumVerts;
			const int32 NextIdx = (Idx + 1) % NumVerts;
			if (!IsCurbEdge(Idx))
			{
				if (Current.Num() >= 2)
				{
					CurbLines.Add(Current);
				}
				Current.Reset();
			}
			Current.Add(Junction->PolygonBoundary[NextIdx]);
		}
		if (Current.Num() >= 2)
		{
			CurbLines.Add(MoveTemp(Current));
		}
	}

	AFlexNetworkMeshActor* MeshActor = Subsystem->GetMeshActor();
	if (!MeshActor)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: no mesh actor to host curbstones on yet -- generate some roads first."));
		return;
	}

	FScopedTransaction Transaction(NSLOCTEXT("FlexNetwork", "GenerateCurbstones", "Generate Curbstones"));
	MeshActor->Modify();
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
