#include "FlexNetworkZoneGraphGenerator.h"

#include "FlexNetworkSubsystem.h"
#include "FlexRoadSegment.h"
#include "RoadTypeProfile.h"
#include "Intersection/FlexLaneConnectorGraph.h"
#include "Math/FlexBezierMath.h"
#include "Mesh/FlexRoadMeshBuilder.h"

#include "ZoneGraphBuilder.h"
#include "ZoneGraphData.h"
#include "ZoneGraphSettings.h"
#include "ZoneShapeActor.h"
#include "ZoneShapeComponent.h"

#include "MassCrowdSettings.h"
#include "MassCrowdSubsystem.h"
#include "MassTrafficSettings.h"
#include "MassTrafficSubsystem.h"

#include "Engine/World.h"
#include "EngineUtils.h"
#include "UObject/UnrealType.h"

namespace
{
	const FName GeneratedActorTag(TEXT("FlexNetwork.ZoneGraph.Generated"));

	namespace Tags
	{
		const FName Vehicle(TEXT("Vehicle"));
		const FName Pedestrian(TEXT("Pedestrian"));
		const FName Crosswalk(TEXT("Crosswalk"));
		const FName Intersection(TEXT("Intersection"));
		const FName Trunk(TEXT("Trunk"));
		const FName LaneChanging(TEXT("LaneChanging"));
		const FName SpeedLimit30(TEXT("SpeedLimit_30"));
		const FName SpeedLimit50(TEXT("SpeedLimit_50"));
		const FName SpeedLimit70(TEXT("SpeedLimit_70"));
		const FName SpeedLimit100(TEXT("SpeedLimit_100"));
	}

	struct FTagSet
	{
		FZoneGraphTag Vehicle;
		FZoneGraphTag Pedestrian;
		FZoneGraphTag Crosswalk;
		FZoneGraphTag Intersection;
		FZoneGraphTag Trunk;
		FZoneGraphTag LaneChanging;
		FZoneGraphTag Speed30;
		FZoneGraphTag Speed50;
		FZoneGraphTag Speed70;
		FZoneGraphTag Speed100;
	};

	FZoneGraphTag FindOrAddTag(const FName Name, const FColor Color)
	{
		UZoneGraphSettings* Settings = GetMutableDefault<UZoneGraphSettings>();
		if (!Settings)
		{
			return FZoneGraphTag::None;
		}

		FStructProperty* TagsProperty = FindFProperty<FStructProperty>(UZoneGraphSettings::StaticClass(), TEXT("Tags"));
		if (!ensure(TagsProperty))
		{
			return FZoneGraphTag::None;
		}

		int32 FirstFreeIndex = INDEX_NONE;
		for (int32 Index = 0; Index < TagsProperty->ArrayDim; ++Index)
		{
			FZoneGraphTagInfo* Info = TagsProperty->ContainerPtrToValuePtr<FZoneGraphTagInfo>(Settings, Index);
			if (Info->IsValid())
			{
				if (Info->Name == Name)
				{
					return Info->Tag;
				}
			}
			else if (FirstFreeIndex == INDEX_NONE)
			{
				FirstFreeIndex = Index;
			}
		}

		if (FirstFreeIndex == INDEX_NONE)
		{
			UE_LOG(LogTemp, Error, TEXT("FlexNetwork: all ZoneGraph tag slots are occupied; cannot register '%s'."), *Name.ToString());
			return FZoneGraphTag::None;
		}

		FZoneGraphTagInfo* NewInfo = TagsProperty->ContainerPtrToValuePtr<FZoneGraphTagInfo>(Settings, FirstFreeIndex);
		NewInfo->Name = Name;
		NewInfo->Color = Color;
		NewInfo->Tag = FZoneGraphTag(static_cast<uint8>(FirstFreeIndex));
		return NewInfo->Tag;
	}

	FZoneLaneProfileRef FindOrAddLaneProfile(const FName Name, const FGuid& StableId, const TArray<FZoneLaneDesc>& Lanes)
	{
		UZoneGraphSettings* Settings = GetMutableDefault<UZoneGraphSettings>();
		if (!Settings)
		{
			return FZoneLaneProfileRef();
		}

		FArrayProperty* LaneProfilesProperty = FindFProperty<FArrayProperty>(UZoneGraphSettings::StaticClass(), TEXT("LaneProfiles"));
		if (!ensure(LaneProfilesProperty))
		{
			return FZoneLaneProfileRef();
		}

		FScriptArrayHelper Helper(LaneProfilesProperty, LaneProfilesProperty->ContainerPtrToValuePtr<void>(Settings));
		for (int32 Index = 0; Index < Helper.Num(); ++Index)
		{
			FZoneLaneProfile* Existing = reinterpret_cast<FZoneLaneProfile*>(Helper.GetRawPtr(Index));
			if (Existing->ID == StableId)
			{
				Existing->Name = Name;
				Existing->Lanes = Lanes;
				return FZoneLaneProfileRef(*Existing);
			}
		}

		const int32 NewIndex = Helper.AddValue();
		FZoneLaneProfile* NewProfile = reinterpret_cast<FZoneLaneProfile*>(Helper.GetRawPtr(NewIndex));
		NewProfile->Name = Name;
		NewProfile->ID = StableId;
		NewProfile->Lanes = Lanes;
		return FZoneLaneProfileRef(*NewProfile);
	}

	FTagSet RegisterTagPalette()
	{
		FTagSet Result;
		Result.Vehicle = FindOrAddTag(Tags::Vehicle, FColor(255, 140, 0));
		Result.Pedestrian = FindOrAddTag(Tags::Pedestrian, FColor(0, 200, 200));
		Result.Crosswalk = FindOrAddTag(Tags::Crosswalk, FColor(255, 220, 0));
		Result.Intersection = FindOrAddTag(Tags::Intersection, FColor(200, 0, 200));
		Result.Trunk = FindOrAddTag(Tags::Trunk, FColor(220, 0, 0));
		Result.LaneChanging = FindOrAddTag(Tags::LaneChanging, FColor(0, 180, 0));
		Result.Speed30 = FindOrAddTag(Tags::SpeedLimit30, FColor(180, 200, 255));
		Result.Speed50 = FindOrAddTag(Tags::SpeedLimit50, FColor(120, 160, 255));
		Result.Speed70 = FindOrAddTag(Tags::SpeedLimit70, FColor(60, 110, 255));
		Result.Speed100 = FindOrAddTag(Tags::SpeedLimit100, FColor(0, 60, 220));
		return Result;
	}

	FZoneGraphTag SpeedTagForCmPerSecond(const float Speed, const FTagSet& TagsSet)
	{
		const float Kph = Speed * 0.036f;
		if (Kph <= 30.f) return TagsSet.Speed30;
		if (Kph <= 50.f) return TagsSet.Speed50;
		if (Kph <= 70.f) return TagsSet.Speed70;
		return TagsSet.Speed100;
	}

	uint32 HashLaneDescs(const TArray<FZoneLaneDesc>& Lanes)
	{
		uint32 Hash = 2166136261u;
		for (const FZoneLaneDesc& Lane : Lanes)
		{
			Hash = HashCombine(Hash, GetTypeHash(Lane.Width));
			Hash = HashCombine(Hash, GetTypeHash(static_cast<uint8>(Lane.Direction)));
			Hash = HashCombine(Hash, GetTypeHash(Lane.Tags.GetValue()));
		}
		return Hash;
	}

	FZoneLaneProfileRef RegisterLaneProfile(const TArray<FZoneLaneDesc>& Lanes)
	{
		const uint32 Hash = HashLaneDescs(Lanes);
		const FName Name(*FString::Printf(TEXT("FlexNetwork_%08X"), Hash));
		const FGuid StableId(0x464C4558u /* FLEX */, Hash, Hash ^ 0x9E3779B9u, ~Hash);
		return FindOrAddLaneProfile(Name, StableId, Lanes);
	}

	FZoneGraphTagFilter RequireTag(const FZoneGraphTag Tag)
	{
		FZoneGraphTagFilter Filter;
		Filter.AllTags = FZoneGraphTagMask(Tag);
		return Filter;
	}

	void ConfigureMassAI(const FTagSet& TagsSet)
	{
		if (UMassCrowdSettings* Crowd = GetMutableDefault<UMassCrowdSettings>())
		{
			Crowd->CrowdTag = TagsSet.Pedestrian;
			Crowd->CrossingTag = TagsSet.Crosswalk;
			Crowd->TryUpdateDefaultConfigFile();
		}

		if (UMassTrafficSettings* Traffic = GetMutableDefault<UMassTrafficSettings>())
		{
			Traffic->TrafficLaneFilter = RequireTag(TagsSet.Vehicle);
			Traffic->IntersectionLaneFilter = RequireTag(TagsSet.Intersection);
			Traffic->TrunkLaneFilter = RequireTag(TagsSet.Trunk);
			Traffic->LaneChangingLaneFilter = RequireTag(TagsSet.LaneChanging);
			Traffic->CrosswalkLaneFilter = RequireTag(TagsSet.Crosswalk);
			Traffic->SpeedLimits.Reset();

			const TPair<FZoneGraphTag, float> SpeedLimits[] = {
				{TagsSet.Speed30, 30.f * 0.621371f},
				{TagsSet.Speed50, 50.f * 0.621371f},
				{TagsSet.Speed70, 70.f * 0.621371f},
				{TagsSet.Speed100, 100.f * 0.621371f}
			};
			for (const TPair<FZoneGraphTag, float>& Speed : SpeedLimits)
			{
				FMassTrafficLaneSpeedLimit& Entry = Traffic->SpeedLimits.Emplace_GetRef();
				Entry.LaneFilter = RequireTag(Speed.Key);
				Entry.SpeedLimitMPH = Speed.Value;
			}
			Traffic->TryUpdateDefaultConfigFile();
		}
	}

	void RebuildMassLaneData(UWorld& World)
	{
#if WITH_EDITOR
		if (UMassTrafficSubsystem* Traffic = World.GetSubsystem<UMassTrafficSubsystem>())
		{
			Traffic->RebuildLaneData();
		}
		if (UMassCrowdSubsystem* Crowd = World.GetSubsystem<UMassCrowdSubsystem>())
		{
			Crowd->RebuildLaneData();
		}
#endif
	}

	struct FPhysicalLane
	{
		float Center = 0.f;
		float Width = 0.f;
		EZoneLaneDirection Direction = EZoneLaneDirection::None;
		FZoneGraphTagMask Tags;
	};

	struct FProfileBuild
	{
		FZoneLaneProfileRef Profile;
		FZoneGraphTagMask ShapeTags;
		float CenterOffset = 0.f;
		bool bValid = false;
	};

	void AddBidirectionalPhysicalLane(TArray<FPhysicalLane>& Out, const float Center, const float Width,
		const FZoneGraphTagMask& LaneTags)
	{
		if (Width <= KINDA_SMALL_NUMBER)
		{
			return;
		}
		const float HalfWidth = Width * 0.5f;
		Out.Add({Center - Width * 0.25f, HalfWidth, EZoneLaneDirection::Backward, LaneTags});
		Out.Add({Center + Width * 0.25f, HalfWidth, EZoneLaneDirection::Forward, LaneTags});
	}

	bool IsTrunkProfile(const URoadTypeProfile& Profile, const float MaximumVehicleSpeed)
	{
		// OSM-created profile asset names retain the source highway value in their stable key.
		// Hand-authored profiles do not have that metadata, so high design speed is the fallback.
		const FString Name = Profile.GetName().ToLower();
		return Name.Contains(TEXT("motorway")) || Name.Contains(TEXT("trunk"))
			|| MaximumVehicleSpeed >= 2222.f; // 80 km/h
	}

	FProfileBuild BuildProfile(const URoadTypeProfile& Profile, const FTagSet& TagsSet, const bool bIncludePedestrians)
	{
		FProfileBuild Result;
		if (Profile.bIsRailProfile)
		{
			return Result;
		}

		int32 ForwardVehicleLaneCount = 0;
		int32 BackwardVehicleLaneCount = 0;
		int32 BidirectionalVehicleLaneCount = 0;
		float MaximumVehicleSpeed = 0.f;
		for (const FRoadLaneDescriptor& Lane : Profile.Lanes)
		{
			if (Lane.Type == EFlexLaneType::Vehicle)
			{
				MaximumVehicleSpeed = FMath::Max(MaximumVehicleSpeed, Lane.SpeedLimit);
				switch (Lane.Direction)
				{
				case EFlexLaneDirection::Forward:
					++ForwardVehicleLaneCount;
					break;
				case EFlexLaneDirection::Backward:
					++BackwardVehicleLaneCount;
					break;
				case EFlexLaneDirection::Bidirectional:
					++BidirectionalVehicleLaneCount;
					break;
				default:
					break;
				}
			}
		}
		const bool bTrunk = IsTrunkProfile(Profile, MaximumVehicleSpeed);

		TArray<FPhysicalLane> PhysicalLanes;
		for (const FRoadLaneDescriptor& Lane : Profile.GetLanesSortedByOffset())
		{
			if (Lane.Width <= KINDA_SMALL_NUMBER)
			{
				continue;
			}

			const float Center = Profile.GetLaneLateralOffset(Lane);
			FZoneGraphTagMask LaneTags;
			if (Lane.Type == EFlexLaneType::Vehicle)
			{
				LaneTags.Add(TagsSet.Vehicle);
				LaneTags.Add(SpeedTagForCmPerSecond(Lane.SpeedLimit, TagsSet));
				if (bTrunk) LaneTags.Add(TagsSet.Trunk);
				const bool bHasParallelSameDirectionLane =
					(Lane.Direction == EFlexLaneDirection::Forward && ForwardVehicleLaneCount > 1)
					|| (Lane.Direction == EFlexLaneDirection::Backward && BackwardVehicleLaneCount > 1)
					|| (Lane.Direction == EFlexLaneDirection::Bidirectional && BidirectionalVehicleLaneCount > 1);
				if (bHasParallelSameDirectionLane) LaneTags.Add(TagsSet.LaneChanging);
			}
			else if (Lane.Type == EFlexLaneType::Sidewalk && bIncludePedestrians)
			{
				LaneTags.Add(TagsSet.Pedestrian);
			}

			if (Lane.Type == EFlexLaneType::Vehicle || (Lane.Type == EFlexLaneType::Sidewalk && bIncludePedestrians))
			{
				if (Lane.Direction == EFlexLaneDirection::Bidirectional)
				{
					AddBidirectionalPhysicalLane(PhysicalLanes, Center, Lane.Width, LaneTags);
				}
				else
				{
					const EZoneLaneDirection Direction = Lane.Direction == EFlexLaneDirection::Backward
						? EZoneLaneDirection::Backward
						: Lane.Direction == EFlexLaneDirection::None ? EZoneLaneDirection::None : EZoneLaneDirection::Forward;
					PhysicalLanes.Add({Center, Lane.Width, Direction, LaneTags});
				}
			}
			else
			{
				// Preserve medians, parking, bike, and rail-position gaps as untagged ZoneGraph spacers.
				PhysicalLanes.Add({Center, Lane.Width, EZoneLaneDirection::None, FZoneGraphTagMask()});
			}
		}

		if (bIncludePedestrians && Profile.SidewalkWidth > KINDA_SMALL_NUMBER)
		{
			FZoneGraphTagMask PedestrianTags(TagsSet.Pedestrian);
			AddBidirectionalPhysicalLane(PhysicalLanes,
				Profile.GetRoadwayMinOffset() - Profile.SidewalkWidth * 0.5f,
				Profile.SidewalkWidth, PedestrianTags);
			AddBidirectionalPhysicalLane(PhysicalLanes,
				Profile.GetRoadwayMaxOffset() + Profile.SidewalkWidth * 0.5f,
				Profile.SidewalkWidth, PedestrianTags);
		}

		PhysicalLanes.Sort([](const FPhysicalLane& A, const FPhysicalLane& B) { return A.Center < B.Center; });
		const bool bHasNavigableLane = PhysicalLanes.ContainsByPredicate([](const FPhysicalLane& Lane)
		{
			return Lane.Direction != EZoneLaneDirection::None && Lane.Tags.GetValue() != 0;
		});
		if (PhysicalLanes.IsEmpty() || !bHasNavigableLane)
		{
			return Result;
		}

		const float MinEdge = PhysicalLanes[0].Center - PhysicalLanes[0].Width * 0.5f;
		float Cursor = MinEdge;
		float MaxEdge = MinEdge;
		TArray<FZoneLaneDesc> LaneDescs;
		for (const FPhysicalLane& Lane : PhysicalLanes)
		{
			const float LaneMin = Lane.Center - Lane.Width * 0.5f;
			const float LaneMax = Lane.Center + Lane.Width * 0.5f;
			if (LaneMin > Cursor + KINDA_SMALL_NUMBER)
			{
				FZoneLaneDesc& Spacer = LaneDescs.Emplace_GetRef();
				Spacer.Width = LaneMin - Cursor;
				Spacer.Direction = EZoneLaneDirection::None;
				Spacer.Tags = FZoneGraphTagMask();
			}

			FZoneLaneDesc& Desc = LaneDescs.Emplace_GetRef();
			Desc.Width = Lane.Width;
			Desc.Direction = Lane.Direction;
			Desc.Tags = Lane.Tags;
			Cursor = FMath::Max(Cursor, LaneMax);
			MaxEdge = FMath::Max(MaxEdge, LaneMax);
		}

		Result.CenterOffset = (MinEdge + MaxEdge) * 0.5f;
		Result.Profile = RegisterLaneProfile(LaneDescs);
		Result.bValid = true;
		return Result;
	}

	FProfileBuild BuildSingleLaneProfile(const float Width, const EZoneLaneDirection Direction,
		const FZoneGraphTagMask& LaneTags, const FZoneGraphTagMask& ShapeTags)
	{
		FProfileBuild Result;
		if (Width <= KINDA_SMALL_NUMBER)
		{
			return Result;
		}
		FZoneLaneDesc Lane;
		Lane.Width = Width;
		Lane.Direction = Direction;
		Lane.Tags = LaneTags;
		TArray<FZoneLaneDesc> Lanes;
		Lanes.Add(Lane);
		Result.Profile = RegisterLaneProfile(Lanes);
		Result.ShapeTags = ShapeTags;
		Result.bValid = true;
		return Result;
	}

	FProfileBuild BuildBidirectionalPedestrianProfile(const float Width, const FTagSet& TagsSet,
		const bool bIntersection, const bool bCrosswalk = false)
	{
		FProfileBuild Result;
		if (Width <= KINDA_SMALL_NUMBER)
		{
			return Result;
		}
		FZoneGraphTagMask LaneTags(TagsSet.Pedestrian);
		FZoneGraphTagMask ShapeTags(TagsSet.Pedestrian);
		if (bIntersection)
		{
			LaneTags.Add(TagsSet.Intersection);
			ShapeTags.Add(TagsSet.Intersection);
		}
		if (bCrosswalk)
		{
			LaneTags.Add(TagsSet.Crosswalk);
			ShapeTags.Add(TagsSet.Crosswalk);
		}
		TArray<FZoneLaneDesc> Lanes;
		FZoneLaneDesc& Backward = Lanes.Emplace_GetRef();
		Backward.Width = Width * 0.5f;
		Backward.Direction = EZoneLaneDirection::Backward;
		Backward.Tags = LaneTags;
		FZoneLaneDesc& Forward = Lanes.Emplace_GetRef();
		Forward.Width = Width * 0.5f;
		Forward.Direction = EZoneLaneDirection::Forward;
		Forward.Tags = LaneTags;
		Result.Profile = RegisterLaneProfile(Lanes);
		Result.ShapeTags = ShapeTags;
		Result.bValid = true;
		return Result;
	}

	void ClampAutoBezierOvershoot(TArray<FZoneShapePoint>& Points)
	{
		constexpr double MaxTangentToSegmentRatio = 0.5;
		for (int32 Index = 1; Index + 1 < Points.Num(); ++Index)
		{
			FZoneShapePoint& Point = Points[Index];
			const double PreviousLength = FVector::Distance(Points[Index - 1].Position, Point.Position);
			const double NextLength = FVector::Distance(Point.Position, Points[Index + 1].Position);
			const double MinimumLength = FMath::Min(PreviousLength, NextLength);
			if (MinimumLength <= KINDA_SMALL_NUMBER)
			{
				continue;
			}
			const FVector NaturalTangent = (Points[Index + 1].Position - Points[Index - 1].Position) * 0.5 / 3.0;
			const double MaximumSafeLength = MaxTangentToSegmentRatio * MinimumLength;
			if (NaturalTangent.Size() > MaximumSafeLength)
			{
				Point.Type = FZoneShapePointType::Bezier;
				Point.Rotation = NaturalTangent.Rotation();
				Point.TangentLength = static_cast<float>(MaximumSafeLength);
			}
		}
	}

	UZoneShapeComponent* SpawnSplineShape(UWorld& World, const FProfileBuild& Profile,
		TConstArrayView<FVector> Positions, const FString& Label)
	{
		if (!Profile.bValid || Positions.Num() < 2)
		{
			return nullptr;
		}
		AZoneShape* Actor = World.SpawnActor<AZoneShape>();
		if (!Actor)
		{
			return nullptr;
		}
		Actor->Tags.AddUnique(GeneratedActorTag);
#if WITH_EDITOR
		Actor->SetActorLabel(Label);
#endif

		UZoneShapeComponent* Shape = const_cast<UZoneShapeComponent*>(Actor->GetShape());
		if (!Shape)
		{
			Actor->Destroy();
			return nullptr;
		}
		Shape->SetShapeType(FZoneShapeType::Spline);
		Shape->SetCommonLaneProfile(Profile.Profile);
		Shape->SetTags(Profile.ShapeTags);
		TArray<FZoneShapePoint>& Points = Shape->GetMutablePoints();
		Points.Reset(Positions.Num());
		for (const FVector& Position : Positions)
		{
			FZoneShapePoint& Point = Points.Emplace_GetRef();
			Point.Position = Position;
			Point.Type = FZoneShapePointType::AutoBezier;
		}
		ClampAutoBezierOvershoot(Points);
		Shape->UpdateShape();
		return Shape;
	}

	const FRoadLaneDescriptor* FindLane(const UFlexNetworkSubsystem& Network, const FFlexSegmentId SegmentId, const int32 LaneIndex)
	{
		const FFlexRoadSegment* Segment = Network.GetSegment(SegmentId);
		return Segment && Segment->Profile && Segment->Profile->Lanes.IsValidIndex(LaneIndex)
			? &Segment->Profile->Lanes[LaneIndex]
			: nullptr;
	}
}

int32 FFlexNetworkZoneGraphGenerator::RemoveGeneratedActors(UWorld& World)
{
	TArray<AActor*> ActorsToRemove;
	for (TActorIterator<AActor> It(&World); It; ++It)
	{
		if (It->Tags.Contains(GeneratedActorTag))
		{
			ActorsToRemove.Add(*It);
		}
	}
	// Remove the baked actor first so it unregisters its storage before its source shapes vanish.
	ActorsToRemove.StableSort([](const AActor& A, const AActor& B)
	{
		return A.IsA<AZoneGraphData>() && !B.IsA<AZoneGraphData>();
	});
	for (AActor* Actor : ActorsToRemove)
	{
		if (Actor)
		{
			Actor->Modify();
			Actor->Destroy();
		}
	}
	RebuildMassLaneData(World);
	return ActorsToRemove.Num();
}

FFlexZoneGraphGenerationResult FFlexNetworkZoneGraphGenerator::Generate(
	const UFlexNetworkSubsystem& Network,
	UWorld& World,
	const FFlexZoneGraphGenerationOptions& Options)
{
	FFlexZoneGraphGenerationResult Result;
	if (Options.bReplaceExisting)
	{
		Result.RemovedActors = RemoveGeneratedActors(World);
	}

	const FTagSet TagsSet = RegisterTagPalette();
	if (Options.bConfigureMassAI)
	{
		ConfigureMassAI(TagsSet);
	}

	TArray<UZoneShapeComponent*> Shapes;
	TMap<const URoadTypeProfile*, FProfileBuild> SegmentProfiles;
	const float SampleSpacing = FMath::Max(Options.SampleSpacing, 10.f);

	for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Network.GetAllSegments())
	{
		const FFlexRoadSegment& Segment = Pair.Value;
		if (!Segment.Profile || Segment.Profile->bIsRailProfile || !Segment.ArcLengthTable.IsValid())
		{
			continue;
		}

		FProfileBuild& Profile = SegmentProfiles.FindOrAdd(Segment.Profile.Get());
		if (!Profile.bValid)
		{
			Profile = BuildProfile(*Segment.Profile, TagsSet, Options.bIncludePedestrianLanes);
		}
		if (!Profile.bValid)
		{
			continue;
		}

		float TrimStart = 0.f;
		float TrimEnd = Segment.GetLength();
		Network.GetSegmentTrimRange(Pair.Key, TrimStart, TrimEnd);
		if (TrimEnd - TrimStart <= KINDA_SMALL_NUMBER)
		{
			continue;
		}
		const FFlexRoadNode* StartNode = Network.GetNode(Segment.StartNodeId);
		const FVector ReferenceUp = StartNode ? StartNode->UpVector : FVector::UpVector;
		const TArray<FFlexCurveFrame> Frames = FFlexRoadMeshBuilder::BuildFramesForRange(
			Segment.Curve, Segment.ArcLengthTable, ReferenceUp, SampleSpacing, TrimStart, TrimEnd);
		TArray<FVector> Positions;
		Positions.Reserve(Frames.Num());
		for (const FFlexCurveFrame& Frame : Frames)
		{
			Positions.Add(Frame.Position + Frame.Right * Profile.CenterOffset);
		}
		if (UZoneShapeComponent* Shape = SpawnSplineShape(World, Profile, Positions,
			FString::Printf(TEXT("Flex Zone Road %u"), Pair.Key.Index)))
		{
			Shapes.Add(Shape);
			++Result.SegmentShapes;
		}
	}

	for (const TPair<FFlexNodeId, FFlexRoadNode>& NodePair : Network.GetAllNodes())
	{
		const FFlexJunctionData* Junction = Network.GetJunctionData(NodePair.Key);
		if (!Junction)
		{
			continue;
		}

		for (int32 ConnectorIndex = 0; ConnectorIndex < Junction->LaneConnectors.Num(); ++ConnectorIndex)
		{
			const FFlexLaneConnector& Connector = Junction->LaneConnectors[ConnectorIndex];
			const FRoadLaneDescriptor* FromLane = FindLane(Network, Connector.FromSegment, Connector.FromLaneIndex);
			const FRoadLaneDescriptor* ToLane = FindLane(Network, Connector.ToSegment, Connector.ToLaneIndex);
			if (!FromLane || !ToLane || FromLane->Type != EFlexLaneType::Vehicle || ToLane->Type != EFlexLaneType::Vehicle)
			{
				continue;
			}

			FZoneGraphTagMask LaneTags(TagsSet.Vehicle);
			LaneTags.Add(TagsSet.Intersection);
			LaneTags.Add(SpeedTagForCmPerSecond(Connector.SpeedLimit, TagsSet));
			FZoneGraphTagMask ShapeTags(TagsSet.Vehicle);
			ShapeTags.Add(TagsSet.Intersection);
			const FProfileBuild ConnectorProfile = BuildSingleLaneProfile(
				FMath::Min(FromLane->Width, ToLane->Width), EZoneLaneDirection::Forward, LaneTags, ShapeTags);

			const FFlexArcLengthTable Table = FFlexBezierMath::BuildArcLengthTable(Connector.ConnectorCurve);
			if (!Table.IsValid())
			{
				continue;
			}
			// Junction curves are often much shorter/tighter than road segments. Keep at least four
			// intervals where possible so ZoneGraph endpoint headings follow the authoritative
			// Flex connector instead of collapsing a whole turn to one straight chord.
			const float ConnectorSampleSpacing = FMath::Min(SampleSpacing,
				FMath::Max(Table.GetTotalLength() * 0.25f, 10.f));
			const TArray<FFlexCurveFrame> Frames = FFlexRoadMeshBuilder::BuildFramesForRange(
				Connector.ConnectorCurve, Table, NodePair.Value.UpVector, ConnectorSampleSpacing, 0.f, Table.GetTotalLength());
			TArray<FVector> Positions;
			Positions.Reserve(Frames.Num());
			for (const FFlexCurveFrame& Frame : Frames)
			{
				Positions.Add(Frame.Position);
			}
			if (UZoneShapeComponent* Shape = SpawnSplineShape(World, ConnectorProfile, Positions,
				FString::Printf(TEXT("Flex Zone Junction %u Connector %d"), NodePair.Key.Index, ConnectorIndex)))
			{
				Shapes.Add(Shape);
				++Result.IntersectionShapes;
			}
		}

		if (Options.bIncludePedestrianLanes)
		{
			for (int32 CrosswalkIndex = 0; CrosswalkIndex < Junction->Crosswalks.Num(); ++CrosswalkIndex)
			{
				const FFlexCrosswalkPlacement& Crosswalk = Junction->Crosswalks[CrosswalkIndex];
				const FVector CrossingDirection = Crosswalk.CrossingDirection.GetSafeNormal();
				if (Crosswalk.Width <= KINDA_SMALL_NUMBER || Crosswalk.Length <= KINDA_SMALL_NUMBER
					|| CrossingDirection.IsNearlyZero())
				{
					continue;
				}

				const FProfileBuild CrosswalkProfile = BuildBidirectionalPedestrianProfile(
					Crosswalk.Width, TagsSet, true, true);
				TArray<FVector> Positions;
				Positions.Add(Crosswalk.Center - CrossingDirection * Crosswalk.Length * 0.5f);
				Positions.Add(Crosswalk.Center + CrossingDirection * Crosswalk.Length * 0.5f);
				if (UZoneShapeComponent* Shape = SpawnSplineShape(World, CrosswalkProfile, Positions,
					FString::Printf(TEXT("Flex Zone Junction %u Crosswalk %d"), NodePair.Key.Index, CrosswalkIndex)))
				{
					Shapes.Add(Shape);
					++Result.CrosswalkShapes;
				}
			}
		}

		if (Options.bIncludePedestrianLanes)
		{
			for (int32 CornerIndex = 0; CornerIndex < Junction->CornerIslands.Num(); ++CornerIndex)
			{
				const FFlexJunctionCornerIsland& Corner = Junction->CornerIslands[CornerIndex];
				const int32 PointCount = FMath::Min(Corner.BandInnerArc.Num(), Corner.BandOuterArc.Num());
				if (PointCount < 2)
				{
					continue;
				}
				float WidthSum = 0.f;
				TArray<FVector> Positions;
				Positions.Reserve(PointCount);
				for (int32 PointIndex = 0; PointIndex < PointCount; ++PointIndex)
				{
					Positions.Add((Corner.BandInnerArc[PointIndex] + Corner.BandOuterArc[PointIndex]) * 0.5f);
					WidthSum += FVector::Distance(Corner.BandInnerArc[PointIndex], Corner.BandOuterArc[PointIndex]);
				}
				const FProfileBuild PedestrianProfile = BuildBidirectionalPedestrianProfile(WidthSum / PointCount, TagsSet, true);
				if (UZoneShapeComponent* Shape = SpawnSplineShape(World, PedestrianProfile, Positions,
					FString::Printf(TEXT("Flex Zone Junction %u Sidewalk %d"), NodePair.Key.Index, CornerIndex)))
				{
					Shapes.Add(Shape);
					++Result.PedestrianCornerShapes;
				}
			}
		}
	}

	if (UZoneGraphSettings* ZoneSettings = GetMutableDefault<UZoneGraphSettings>())
	{
		ZoneSettings->TryUpdateDefaultConfigFile();
	}

	if (!Shapes.IsEmpty())
	{
		AZoneGraphData* DataActor = World.SpawnActor<AZoneGraphData>();
		if (DataActor)
		{
			DataActor->Tags.AddUnique(GeneratedActorTag);
#if WITH_EDITOR
			DataActor->SetActorLabel(TEXT("FlexNetwork ZoneGraph Data"));
#endif
			FZoneGraphBuilder Builder;
			for (UZoneShapeComponent* Shape : Shapes)
			{
				if (Shape)
				{
					Builder.RegisterZoneShapeComponent(*Shape);
				}
			}
			Builder.BuildAll({DataActor}, true);
			Result.DataActor = DataActor;
		}
	}

	RebuildMassLaneData(World);
	return Result;
}
