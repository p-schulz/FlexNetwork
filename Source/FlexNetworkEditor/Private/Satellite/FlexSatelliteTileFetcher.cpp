#include "Satellite/FlexSatelliteTileFetcher.h"
#include "Satellite/FlexSatelliteImagerySettings.h"
#include "Osm/FlexOsmGraphBuilder.h"

#include "HttpModule.h"
#include "Interfaces/IHttpRequest.h"
#include "Interfaces/IHttpResponse.h"
#include "IImageWrapperModule.h"
#include "IImageWrapper.h"
#include "Modules/ModuleManager.h"
#include "Engine/Texture2D.h"

namespace FlexNetwork::Satellite
{
	namespace
	{
		// Web Mercator <-> WGS84 conversions -- the LGL-BW WMS only returns imagery for
		// CRS=EPSG:3857 requests (EPSG:4326 GetMap responses come back blank against the live
		// service), so every tile fetch goes through Web Mercator as a wire-format detail; the
		// result is converted straight back to lat/lon below and from there into the same local
		// centimeter projection every other FlexNetwork import uses, so Web Mercator never leaks
		// into how a tile is actually positioned.
		constexpr double kEarthRadiusM = 6378137.0; // WGS84 semi-major axis, used as a sphere radius.
		constexpr double kPi = 3.14159265358979323846;

		double DegToRad(double Deg) { return Deg * (kPi / 180.0); }
		double RadToDeg(double Rad) { return Rad * (180.0 / kPi); }

		void LatLonToWebMercator(double Lat, double Lon, double& OutX, double& OutY)
		{
			OutX = DegToRad(Lon) * kEarthRadiusM;
			OutY = FMath::Loge(FMath::Tan(kPi / 4.0 + DegToRad(Lat) / 2.0)) * kEarthRadiusM;
		}

		void WebMercatorToLatLon(double X, double Y, double& OutLat, double& OutLon)
		{
			OutLat = RadToDeg(2.0 * FMath::Atan(FMath::Exp(Y / kEarthRadiusM)) - kPi / 2.0);
			OutLon = RadToDeg(X / kEarthRadiusM);
		}

		// Same WMS GetMap URL shape LGL-BW's own reference tooling uses -- CRS=EPSG:3857
		// specifically (see the header comment on why: EPSG:4326 requests come back blank against
		// the live service).
		FString BuildGetMapUrl(const FFlexLGLImageryLayerPreset& Layer, double MinX, double MinY, double MaxX, double MaxY, int32 SidePx)
		{
			FString Url = FString::Printf(
				TEXT("%s?SERVICE=WMS&VERSION=1.3.0&REQUEST=GetMap&LAYERS=%s&STYLES=%s&CRS=EPSG:3857&BBOX=%.3f,%.3f,%.3f,%.3f&WIDTH=%d&HEIGHT=%d&FORMAT=image/png"),
				*Layer.ServiceUrl, *Layer.LayerName, *Layer.Style, MinX, MinY, MaxX, MaxY, SidePx, SidePx);
			if (Layer.bTransparent)
			{
				Url += TEXT("&TRANSPARENT=true");
			}
			return Url;
		}
	}

	void FetchLGLImageryTileAsync(
		const FFlexLGLImageryLayerPreset& Layer,
		const double TileRadiusM, const double ResolutionM, const int32 MaxPixelsPerSide,
		const double CenterLat, const double CenterLon,
		const double OriginLat, const double OriginLon,
		TFunction<void(UTexture2D*, double, double, double, double, const FString&)> OnComplete)
	{
		double CenterX = 0.0, CenterY = 0.0;
		LatLonToWebMercator(CenterLat, CenterLon, CenterX, CenterY);
		// Web Mercator's "meters" are ground meters scaled by 1/cos(lat); inflate the requested
		// half-extent so the fetched tile actually covers a TileRadiusM ground-meter square.
		const double MercRadius = TileRadiusM / FMath::Cos(DegToRad(CenterLat));
		const double MinX = CenterX - MercRadius, MaxX = CenterX + MercRadius;
		const double MinY = CenterY - MercRadius, MaxY = CenterY + MercRadius;
		const int32 SidePx = FMath::Clamp(FMath::RoundToInt32(2.0 * TileRadiusM / ResolutionM), 64, MaxPixelsPerSide);

		const FString Url = BuildGetMapUrl(Layer, MinX, MinY, MaxX, MaxY, SidePx);

		// Corner bounds in local (origin-relative) centimeters, computed up front from the
		// REQUESTED bbox (not the response) using the exact same projection FlexNetwork's own OSM
		// road import uses -- the plane this later spawns must match exactly what was asked for,
		// and must line up with any OSM-imported roads sharing the same origin, regardless of how
		// the image itself is used.
		double SwLat = 0.0, SwLon = 0.0, NeLat = 0.0, NeLon = 0.0;
		WebMercatorToLatLon(MinX, MinY, SwLat, SwLon);
		WebMercatorToLatLon(MaxX, MaxY, NeLat, NeLon);
		const FVector2D LocalSw = FFlexOsmGraphBuilder::ProjectLatLonToLocalCm(SwLat, SwLon, OriginLat, OriginLon);
		const FVector2D LocalNe = FFlexOsmGraphBuilder::ProjectLatLonToLocalCm(NeLat, NeLon, OriginLat, OriginLon);
		// Shared world convention is X=north, Y=east; keep the callback arguments geographic so
		// AFlexSatelliteTileActor can map them onto the correct world axes explicitly.
		const double SouthCm = LocalSw.X, WestCm = LocalSw.Y, NorthCm = LocalNe.X, EastCm = LocalNe.Y;

		const FHttpRequestRef Request = FHttpModule::Get().CreateRequest();
		Request->SetURL(Url);
		Request->SetVerb(TEXT("GET"));
		Request->SetTimeout(30.0f);

		Request->OnProcessRequestComplete().BindLambda(
			[SouthCm, NorthCm, WestCm, EastCm, OnComplete](FHttpRequestPtr, const FHttpResponsePtr Response, const bool bSucceeded)
			{
				if (!bSucceeded || !Response.IsValid())
				{
					OnComplete(nullptr, SouthCm, NorthCm, WestCm, EastCm, TEXT("WMS request failed (network error)."));
					return;
				}
				if (Response->GetResponseCode() != 200)
				{
					OnComplete(nullptr, SouthCm, NorthCm, WestCm, EastCm, FString::Printf(TEXT("WMS request failed: HTTP %d."), Response->GetResponseCode()));
					return;
				}
				if (!Response->GetContentType().Contains(TEXT("image")))
				{
					OnComplete(nullptr, SouthCm, NorthCm, WestCm, EastCm, FString::Printf(TEXT("WMS did not return an image (Content-Type: %s) -- the requested point is likely outside LGL-BW's Baden-Wuerttemberg coverage."), *Response->GetContentType()));
					return;
				}

				const TArray<uint8>& Bytes = Response->GetContent();
				IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
				const TSharedPtr<IImageWrapper> ImageWrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::PNG);
				if (!ImageWrapper.IsValid() || !ImageWrapper->SetCompressed(Bytes.GetData(), Bytes.Num()))
				{
					OnComplete(nullptr, SouthCm, NorthCm, WestCm, EastCm, TEXT("Failed to parse the WMS response as a PNG image."));
					return;
				}

				TArray64<uint8> RawData;
				if (!ImageWrapper->GetRaw(ERGBFormat::BGRA, 8, RawData))
				{
					OnComplete(nullptr, SouthCm, NorthCm, WestCm, EastCm, TEXT("Failed to decode the WMS response PNG image."));
					return;
				}

				UTexture2D* Texture = UTexture2D::CreateTransient(ImageWrapper->GetWidth(), ImageWrapper->GetHeight(), PF_B8G8R8A8, NAME_None, RawData);
				if (!Texture)
				{
					OnComplete(nullptr, SouthCm, NorthCm, WestCm, EastCm, TEXT("Failed to create a texture from the decoded image."));
					return;
				}
				Texture->UpdateResource();

				OnComplete(Texture, SouthCm, NorthCm, WestCm, EastCm, FString());
			});

		Request->ProcessRequest();
	}
}
