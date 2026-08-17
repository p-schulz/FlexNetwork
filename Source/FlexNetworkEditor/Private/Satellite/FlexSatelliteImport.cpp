#include "Satellite/FlexSatelliteImport.h"
#include "Satellite/FlexSatelliteImagerySettings.h"
#include "Satellite/FlexSatelliteTileFetcher.h"
#include "Satellite/FlexSatelliteTileActor.h"
#include "Osm/FlexOsmGraphBuilder.h"
#include "Osm/OsmDataAsset.h"

#include "Engine/World.h"
#include "Engine/Texture2D.h"

namespace FlexNetwork::Satellite
{
	namespace
	{
		// Per-tile join state: two independent WMS fetches (aerial always, land-use optionally)
		// have to both resolve before the tile actor can be spawned with both textures at once.
		struct FTileJoinState
		{
			bool bWantsLandUse = false;
			bool bAerialDone = false;
			bool bLandUseDone = false;
			UTexture2D* AerialTexture = nullptr;
			UTexture2D* LandUseTexture = nullptr;
			double SouthCm = 0.0, NorthCm = 0.0, WestCm = 0.0, EastCm = 0.0;

			bool IsComplete() const { return bAerialDone && (bLandUseDone || !bWantsLandUse); }
		};
	}

	void ImportSatelliteImagery(
		UWorld* World,
		const UOsmDataAsset& OsmAsset,
		const FFlexOsmImportSettings& OsmSettings,
		const UFlexSatelliteImagerySettings& ImagerySettings,
		TArray<TWeakObjectPtr<AFlexSatelliteTileActor>>& OutSpawnedTiles,
		TFunction<void(FImportResult)> OnComplete)
	{
		const TSharedRef<FImportResult> Result = MakeShared<FImportResult>();

		if (!World)
		{
			Result->Warnings.Add(TEXT("No world to spawn satellite tiles into."));
			OnComplete(*Result);
			return;
		}

		// Extent + origin: prefer the actual bounding box of the nodes BuildFromOsm would place
		// into the graph (ComputeMatchingRoadExtent) over the file's own <bounds> declaration --
		// a raw Overpass/download bbox is routinely much larger than the specific roads
		// OsmSettings.HighwayTags ends up keeping, and laying the tile grid out over that whole
		// declared area (rather than just where the roads actually are) is what made tiles look
		// wrong/misplaced relative to the imported network. Falls back to <bounds>/all-nodes only
		// for an asset with no matching ways at all (e.g. imagery wanted with no road import).
		double OriginLat = 0.0, OriginLon = 0.0;
		FVector2D LocalMin, LocalMax;
		if (!FFlexOsmGraphBuilder::ComputeMatchingRoadExtent(OsmAsset, OsmSettings, OriginLat, OriginLon, LocalMin, LocalMax))
		{
			if (!FFlexOsmGraphBuilder::ResolveOrigin(OsmAsset, OsmSettings, OriginLat, OriginLon))
			{
				Result->Warnings.Add(TEXT("Could not resolve a projection origin from the OSM asset (it has no <bounds> element and no nodes)."));
				OnComplete(*Result);
				return;
			}

			double MinLat, MinLon, MaxLat, MaxLon;
			if (OsmAsset.Bounds.bIsValid)
			{
				MinLat = OsmAsset.Bounds.MinLat;
				MinLon = OsmAsset.Bounds.MinLon;
				MaxLat = OsmAsset.Bounds.MaxLat;
				MaxLon = OsmAsset.Bounds.MaxLon;
			}
			else
			{
				if (OsmAsset.Nodes.Num() == 0)
				{
					Result->Warnings.Add(TEXT("OSM asset has no <bounds> element and no nodes to derive an extent from."));
					OnComplete(*Result);
					return;
				}
				MinLat = MinLon = TNumericLimits<double>::Max();
				MaxLat = MaxLon = TNumericLimits<double>::Lowest();
				for (const TPair<int64, FOsmNode>& Pair : OsmAsset.Nodes)
				{
					MinLat = FMath::Min(MinLat, Pair.Value.Latitude);
					MaxLat = FMath::Max(MaxLat, Pair.Value.Latitude);
					MinLon = FMath::Min(MinLon, Pair.Value.Longitude);
					MaxLon = FMath::Max(MaxLon, Pair.Value.Longitude);
				}
			}

			// The projection is separable (X depends only on Lon, Y depends only on Lat, both
			// monotonic), so the extent's own two opposite corners project straight to the
			// local-cm bounding box -- no need to project all four corners.
			LocalMin = FFlexOsmGraphBuilder::ProjectLatLonToLocalCm(MinLat, MinLon, OriginLat, OriginLon);
			LocalMax = FFlexOsmGraphBuilder::ProjectLatLonToLocalCm(MaxLat, MaxLon, OriginLat, OriginLon);
		}

		const double TileSideCm = 2.0 * ImagerySettings.TileRadiusM * 100.0;
		if (TileSideCm <= KINDA_SMALL_NUMBER)
		{
			Result->Warnings.Add(TEXT("TileRadiusM is zero or negative."));
			OnComplete(*Result);
			return;
		}

		// Snapped to the origin (not the extent) so a re-import of a slightly different extent
		// around the same origin lands on identical tile boundaries.
		const double GridMinX = FMath::FloorToDouble(LocalMin.X / TileSideCm) * TileSideCm;
		const double GridMinY = FMath::FloorToDouble(LocalMin.Y / TileSideCm) * TileSideCm;
		const int32 NumTilesX = FMath::Max(1, FMath::CeilToInt32((LocalMax.X - GridMinX) / TileSideCm));
		const int32 NumTilesY = FMath::Max(1, FMath::CeilToInt32((LocalMax.Y - GridMinY) / TileSideCm));

		Result->NumTilesRequested = NumTilesX * NumTilesY;
		if (Result->NumTilesRequested == 0)
		{
			OnComplete(*Result);
			return;
		}

		const TSharedRef<int32> RemainingTiles = MakeShared<int32>(Result->NumTilesRequested);
		const TWeakObjectPtr<UWorld> WeakWorld = World;

		auto SpawnTile = [WeakWorld, &OutSpawnedTiles, &ImagerySettings](const FTileJoinState& Join)
		{
			UWorld* SpawnWorld = WeakWorld.Get();
			if (!SpawnWorld)
			{
				return;
			}
			AFlexSatelliteTileActor* Tile = SpawnWorld->SpawnActor<AFlexSatelliteTileActor>();
			if (!Tile)
			{
				return;
			}
#if WITH_EDITOR
			Tile->SetActorLabel(FString::Printf(TEXT("SatelliteTile_%d_%d"), FMath::RoundToInt(Join.WestCm / 100.0), FMath::RoundToInt(Join.SouthCm / 100.0)));
#endif
			Tile->Initialize(
				Join.SouthCm, Join.NorthCm, Join.WestCm, Join.EastCm,
				Join.AerialTexture, ImagerySettings.AerialLayer,
				Join.LandUseTexture, ImagerySettings.LandUseLayer,
				ImagerySettings.LandUseOpacity, ImagerySettings.LandUseOpacityParameterName,
				ImagerySettings.PreviewTileMaterial.LoadSynchronous());
			OutSpawnedTiles.Add(Tile);
		};

		for (int32 Ty = 0; Ty < NumTilesY; ++Ty)
		{
			for (int32 Tx = 0; Tx < NumTilesX; ++Tx)
			{
				const double CenterX = GridMinX + (static_cast<double>(Tx) + 0.5) * TileSideCm;
				const double CenterY = GridMinY + (static_cast<double>(Ty) + 0.5) * TileSideCm;
				double CenterLat = 0.0, CenterLon = 0.0;
				FFlexOsmGraphBuilder::UnprojectLocalCmToLatLon(CenterX, CenterY, OriginLat, OriginLon, CenterLat, CenterLon);

				const TSharedRef<FTileJoinState> Join = MakeShared<FTileJoinState>();
				Join->bWantsLandUse = ImagerySettings.bFetchLandUseOverlay;

				auto MaybeFinishTile = [Join, Result, RemainingTiles, SpawnTile, OnComplete]()
				{
					if (!Join->IsComplete())
					{
						return;
					}
					if (Join->AerialTexture)
					{
						SpawnTile(*Join);
						++Result->NumTilesSpawned;
					}
					if (--(*RemainingTiles) == 0)
					{
						OnComplete(*Result);
					}
				};

				FetchLGLImageryTileAsync(
					ImagerySettings.AerialLayer, ImagerySettings.TileRadiusM, ImagerySettings.ResolutionM, ImagerySettings.MaxPixelsPerSide,
					CenterLat, CenterLon, OriginLat, OriginLon,
					[Join, Result, MaybeFinishTile](UTexture2D* Texture, double SouthCm, double NorthCm, double WestCm, double EastCm, const FString& Error)
					{
						Join->bAerialDone = true;
						Join->AerialTexture = Texture;
						Join->SouthCm = SouthCm; Join->NorthCm = NorthCm; Join->WestCm = WestCm; Join->EastCm = EastCm;
						if (!Texture)
						{
							Result->Warnings.Add(FString::Printf(TEXT("Aerial tile fetch failed: %s"), *Error));
						}
						MaybeFinishTile();
					});

				if (Join->bWantsLandUse)
				{
					FetchLGLImageryTileAsync(
						ImagerySettings.LandUseLayer, ImagerySettings.TileRadiusM, ImagerySettings.ResolutionM, ImagerySettings.MaxPixelsPerSide,
						CenterLat, CenterLon, OriginLat, OriginLon,
						[Join, Result, MaybeFinishTile](UTexture2D* Texture, double SouthCm, double NorthCm, double WestCm, double EastCm, const FString& Error)
						{
							Join->bLandUseDone = true;
							Join->LandUseTexture = Texture;
							if (!Join->bAerialDone)
							{
								Join->SouthCm = SouthCm; Join->NorthCm = NorthCm; Join->WestCm = WestCm; Join->EastCm = EastCm;
							}
							if (!Texture)
							{
								Result->Warnings.Add(FString::Printf(TEXT("Land-use tile fetch failed: %s"), *Error));
							}
							MaybeFinishTile();
						});
				}
			}
		}
	}
}
