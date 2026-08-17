#include "Satellite/FlexSatelliteImageBaker.h"

#include "Engine/Texture2D.h"
#include "Materials/MaterialInstanceConstant.h"
#include "Materials/MaterialInterface.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "UObject/Package.h"
#include "UObject/SavePackage.h"
#include "Misc/PackageName.h"

namespace FlexNetwork::Satellite
{
	namespace
	{
		UPackage* CreateAssetPackage(const FString& FullPackagePath)
		{
			UPackage* Package = CreatePackage(*FullPackagePath);
			if (Package)
			{
				Package->FullyLoad();
			}
			return Package;
		}

		bool SaveAssetPackage(UPackage* Package, UObject* Asset, const FString& FullPackagePath)
		{
			Asset->PostEditChange();
			FAssetRegistryModule::AssetCreated(Asset);
			Package->MarkPackageDirty();

			FSavePackageArgs SaveArgs;
			SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
			SaveArgs.SaveFlags = SAVE_NoError;
			const FString FileName = FPackageName::LongPackageNameToFilename(FullPackagePath, FPackageName::GetAssetPackageExtension());
			return UPackage::SavePackage(Package, Asset, *FileName, SaveArgs);
		}

		// Bilinear resize of a tightly-packed BGRA8 buffer -- written locally (rather than relying on
		// an engine image-resize utility of uncertain exact signature/availability here) since the
		// only requirement is "look reasonable at a modest up/downscale factor for a photographic
		// aerial/land-use tile," not perfect gamma-correct filtering. Edge-clamped sampling, half-
		// pixel-center-correct mapping from destination to source space.
		TArray64<uint8> ResizeBGRA8(const TArray64<uint8>& Src, int32 SrcW, int32 SrcH, int32 DstW, int32 DstH)
		{
			TArray64<uint8> Dst;
			Dst.SetNumUninitialized(static_cast<int64>(DstW) * DstH * 4);

			const float ScaleX = static_cast<float>(SrcW) / static_cast<float>(DstW);
			const float ScaleY = static_cast<float>(SrcH) / static_cast<float>(DstH);

			for (int32 DstY = 0; DstY < DstH; ++DstY)
			{
				const float SrcYf = (static_cast<float>(DstY) + 0.5f) * ScaleY - 0.5f;
				const int32 Y0 = FMath::Clamp(FMath::FloorToInt32(SrcYf), 0, SrcH - 1);
				const int32 Y1 = FMath::Clamp(Y0 + 1, 0, SrcH - 1);
				const float FracY = FMath::Clamp(SrcYf - static_cast<float>(Y0), 0.f, 1.f);

				for (int32 DstX = 0; DstX < DstW; ++DstX)
				{
					const float SrcXf = (static_cast<float>(DstX) + 0.5f) * ScaleX - 0.5f;
					const int32 X0 = FMath::Clamp(FMath::FloorToInt32(SrcXf), 0, SrcW - 1);
					const int32 X1 = FMath::Clamp(X0 + 1, 0, SrcW - 1);
					const float FracX = FMath::Clamp(SrcXf - static_cast<float>(X0), 0.f, 1.f);

					const int64 Idx00 = (static_cast<int64>(Y0) * SrcW + X0) * 4;
					const int64 Idx10 = (static_cast<int64>(Y0) * SrcW + X1) * 4;
					const int64 Idx01 = (static_cast<int64>(Y1) * SrcW + X0) * 4;
					const int64 Idx11 = (static_cast<int64>(Y1) * SrcW + X1) * 4;
					const int64 DstIdx = (static_cast<int64>(DstY) * DstW + DstX) * 4;

					for (int32 Channel = 0; Channel < 4; ++Channel)
					{
						const float C00 = Src[Idx00 + Channel];
						const float C10 = Src[Idx10 + Channel];
						const float C01 = Src[Idx01 + Channel];
						const float C11 = Src[Idx11 + Channel];
						const float Top = FMath::Lerp(C00, C10, FracX);
						const float Bottom = FMath::Lerp(C01, C11, FracX);
						const float Value = FMath::Lerp(Top, Bottom, FracY);
						Dst[DstIdx + Channel] = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(Value), 0, 255));
					}
				}
			}

			return Dst;
		}
	}

	UTexture2D* BakeTextureToContent(UTexture2D* SourceTexture, const FString& PackagePath, const FString& AssetName, const bool bSRGB, const int32 ResizeToSquareSize)
	{
		if (!SourceTexture)
		{
			return nullptr;
		}

		FTexturePlatformData* PlatformData = SourceTexture->GetPlatformData();
		if (!PlatformData || PlatformData->Mips.Num() == 0)
		{
			return nullptr;
		}
		FTexture2DMipMap& Mip0 = PlatformData->Mips[0];
		int32 Width = Mip0.SizeX;
		int32 Height = Mip0.SizeY;
		const int64 ExpectedBytes = static_cast<int64>(Width) * Height * 4;

		const void* LockedData = Mip0.BulkData.LockReadOnly();
		if (!LockedData || Mip0.BulkData.GetBulkDataSize() < ExpectedBytes)
		{
			Mip0.BulkData.Unlock();
			return nullptr;
		}
		TArray64<uint8> PixelData;
		PixelData.SetNumUninitialized(ExpectedBytes);
		FMemory::Memcpy(PixelData.GetData(), LockedData, ExpectedBytes);
		Mip0.BulkData.Unlock();

		if (ResizeToSquareSize > 0)
		{
			const int32 TargetSize = FMath::RoundUpToPowerOfTwo(ResizeToSquareSize);
			if (TargetSize != Width || TargetSize != Height)
			{
				PixelData = ResizeBGRA8(PixelData, Width, Height, TargetSize, TargetSize);
				Width = TargetSize;
				Height = TargetSize;
			}
		}

		const FString FullPackagePath = PackagePath / AssetName;
		UPackage* Package = CreateAssetPackage(FullPackagePath);
		if (!Package)
		{
			return nullptr;
		}

		UTexture2D* NewTexture = NewObject<UTexture2D>(Package, *AssetName, RF_Public | RF_Standalone);
		// Source (not just platform data) is what makes this a real, re-compressible/re-mippable
		// asset -- CreateTransient (how SourceTexture itself was built) never populates Source, only
		// a single already-uncompressed mip, so the pixel bytes have to be copied across explicitly.
		NewTexture->Source.Init(Width, Height, 1, 1, TSF_BGRA8, PixelData.GetData());
		NewTexture->SRGB = bSRGB;
		NewTexture->CompressionSettings = TC_Default;
		NewTexture->MipGenSettings = TMGS_FromTextureGroup;
		NewTexture->LODGroup = TEXTUREGROUP_World;
		NewTexture->UpdateResource();

		SaveAssetPackage(Package, NewTexture, FullPackagePath);
		return NewTexture;
	}

	UMaterialInstanceConstant* CreateLandscapeMaterialInstance(
		UMaterialInterface* BaseMaterial, const FString& PackagePath, const FString& AssetName,
		const TMap<FName, UTexture*>& TextureParameters, const TMap<FName, float>& ScalarParameters)
	{
		if (!BaseMaterial)
		{
			return nullptr;
		}

		const FString FullPackagePath = PackagePath / AssetName;
		UPackage* Package = CreateAssetPackage(FullPackagePath);
		if (!Package)
		{
			return nullptr;
		}

		UMaterialInstanceConstant* Instance = NewObject<UMaterialInstanceConstant>(Package, *AssetName, RF_Public | RF_Standalone);
		Instance->SetParentEditorOnly(BaseMaterial);
		for (const TPair<FName, UTexture*>& Pair : TextureParameters)
		{
			Instance->SetTextureParameterValueEditorOnly(FMaterialParameterInfo(Pair.Key), Pair.Value);
		}
		for (const TPair<FName, float>& Pair : ScalarParameters)
		{
			Instance->SetScalarParameterValueEditorOnly(FMaterialParameterInfo(Pair.Key), Pair.Value);
		}

		SaveAssetPackage(Package, Instance, FullPackagePath);
		return Instance;
	}
}
