#pragma once

#include "CoreMinimal.h"
#include "PCGElement.h"
#include "PCGSettings.h"
#include "PCGFlexNetworkNodes.generated.h"

UCLASS(BlueprintType, ClassGroup=(Procedural))
class FLEXNETWORKPCG_API UPCGFlexRoadMeshesSettings : public UPCGSettings
{
	GENERATED_BODY()
public:
#if WITH_EDITOR
	virtual FName GetDefaultNodeName() const override { return TEXT("FlexRoadMeshes"); }
	virtual FText GetDefaultNodeTitle() const override { return NSLOCTEXT("FlexNetworkPCG", "RoadTitle", "Flex Network: Road / Rail Meshes"); }
	virtual FText GetNodeTooltipText() const override { return NSLOCTEXT("FlexNetworkPCG", "RoadTip", "Generates FlexNetwork road and railway dynamic meshes. World-scoped rail profiles are unified before grooved tram-rail boolean subtraction so switches remain continuous."); }
	virtual EPCGSettingsType GetType() const override { return EPCGSettingsType::Spatial; }
#endif
	virtual TArray<FPCGPinProperties> InputPinProperties() const override { return {}; }
	virtual TArray<FPCGPinProperties> OutputPinProperties() const override;
protected:
	virtual FPCGElementPtr CreateElement() const override;
};

UCLASS(BlueprintType, ClassGroup=(Procedural))
class FLEXNETWORKPCG_API UPCGFlexSidewalkMeshesSettings : public UPCGSettings
{
	GENERATED_BODY()
public:
#if WITH_EDITOR
	virtual FName GetDefaultNodeName() const override { return TEXT("FlexSidewalkMeshes"); }
	virtual FText GetDefaultNodeTitle() const override { return NSLOCTEXT("FlexNetworkPCG", "SidewalkTitle", "Flex Network: Sidewalk Meshes"); }
	virtual FText GetNodeTooltipText() const override { return NSLOCTEXT("FlexNetworkPCG", "SidewalkTip", "Generates profile-driven sidewalk strips on exposed FlexNetwork road segments; routing-only links inside grouped intersections are omitted."); }
	virtual EPCGSettingsType GetType() const override { return EPCGSettingsType::Spatial; }
#endif
	virtual TArray<FPCGPinProperties> InputPinProperties() const override { return {}; }
	virtual TArray<FPCGPinProperties> OutputPinProperties() const override;
protected:
	virtual FPCGElementPtr CreateElement() const override;
};

UCLASS(BlueprintType, ClassGroup=(Procedural))
class FLEXNETWORKPCG_API UPCGFlexIntersectionMeshesSettings : public UPCGSettings
{
	GENERATED_BODY()
public:
#if WITH_EDITOR
	virtual FName GetDefaultNodeName() const override { return TEXT("FlexIntersectionMeshes"); }
	virtual FText GetDefaultNodeTitle() const override { return NSLOCTEXT("FlexNetworkPCG", "IntersectionTitle", "Flex Network: Intersection Meshes"); }
	virtual FText GetNodeTooltipText() const override { return NSLOCTEXT("FlexNetworkPCG", "IntersectionTip", "Generates validated junction surfaces, including one filled surface for grouped multi-port OSM intersections, plus separately composable outer crosswalk layers."); }
	virtual EPCGSettingsType GetType() const override { return EPCGSettingsType::Spatial; }
#endif
	virtual TArray<FPCGPinProperties> InputPinProperties() const override { return {}; }
	virtual TArray<FPCGPinProperties> OutputPinProperties() const override;
protected:
	virtual FPCGElementPtr CreateElement() const override;
};

UCLASS(BlueprintType, ClassGroup=(Procedural))
class FLEXNETWORKPCG_API UPCGFlexCurbMeshesSettings : public UPCGSettings
{
	GENERATED_BODY()
public:
#if WITH_EDITOR
	virtual FName GetDefaultNodeName() const override { return TEXT("FlexCurbMeshes"); }
	virtual FText GetDefaultNodeTitle() const override { return NSLOCTEXT("FlexNetworkPCG", "CurbTitle", "Flex Network: Curb Meshes"); }
	virtual FText GetNodeTooltipText() const override { return NSLOCTEXT("FlexNetworkPCG", "CurbTip", "Extrudes outward-facing chamfered curb prisms along exposed road/junction boundaries, leaving curb-cut gaps at crosswalks."); }
	virtual EPCGSettingsType GetType() const override { return EPCGSettingsType::Spatial; }
#endif
	virtual TArray<FPCGPinProperties> InputPinProperties() const override { return {}; }
	virtual TArray<FPCGPinProperties> OutputPinProperties() const override;

	/** Horizontal curb depth measured outward from the roadway edge. */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category=Settings, meta=(PCG_Overridable, ClampMin="1.0", Units="cm"))
	float CurbWidth = 18.f;

	/** Bevel applied to both upper longitudinal edges. */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category=Settings, meta=(PCG_Overridable, ClampMin="0.0", Units="cm"))
	float ChamferSize = 2.f;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category=Settings, meta=(PCG_Overridable))
	bool bGenerateRoadCurbs = true;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category=Settings, meta=(PCG_Overridable))
	bool bGenerateJunctionCurbs = true;

protected:
	virtual FPCGElementPtr CreateElement() const override;
};

class FPCGFlexRoadMeshesElement : public IPCGElement { protected: virtual bool ExecuteInternal(FPCGContext* Context) const override; };
class FPCGFlexSidewalkMeshesElement : public IPCGElement { protected: virtual bool ExecuteInternal(FPCGContext* Context) const override; };
class FPCGFlexIntersectionMeshesElement : public IPCGElement { protected: virtual bool ExecuteInternal(FPCGContext* Context) const override; };
class FPCGFlexCurbMeshesElement : public IPCGElement { protected: virtual bool ExecuteInternal(FPCGContext* Context) const override; };
