// Copyright notice placeholder

#include "Generator/LocationRoom.h"

#include "Generator/RoadSegment.h"
#include "Generator/WorldFinishMarker.h"
#include "Generator/WorldGenSettings.h"
#include "Generator/WorldStartMarker.h"

#include "Algo/Shuffle.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"

namespace
{
    struct FPortalInfo
    {
        int32 WallIndex = 0;
        float AxisOffset = 0.f;
        FVector WorldLocation = FVector::ZeroVector;
        FVector WorldNormal = FVector::ForwardVector;
        bool bIsExit = false;
    };

    struct FRoadPortalAnchor
    {
        FPortalInfo Portal;
        FVector Anchor = FVector::ZeroVector;
    };

    static float DistancePointToSegment2D(const FVector& P, const FVector& A, const FVector& B)
    {
        const FVector FlatP(P.X, P.Y, 0.f);
        const FVector FlatA(A.X, A.Y, 0.f);
        const FVector FlatB(B.X, B.Y, 0.f);
        return FMath::PointDistToSegment(FlatP, FlatA, FlatB);
    }

    static int32 SampleSpawnCount(const FSpawnStruct& Entry, FRandomStream& Rnd)
    {
        if (!Entry.Class)
        {
            return 0;
        }

        const int32 MinCount = FMath::Max(0, Entry.MinCount);
        const int32 MaxCount = FMath::Max(MinCount, Entry.MaxCount);
        const float Chance = FMath::Clamp(Entry.Weight, 0.f, 1.f);

        int32 Count = MinCount;
        for (int32 Index = MinCount; Index < MaxCount; ++Index)
        {
            if (Rnd.FRand() <= Chance)
            {
                ++Count;
            }
        }
        return Count;
    }
}

ALocationRoom::ALocationRoom()
{
    PrimaryActorTick.bCanEverTick = false;

    Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(Root);

    WallISM = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("WallISM"));
    WallISM->SetupAttachment(RootComponent);
}

void ALocationRoom::Generate(const UWorldGenSettings* Settings, FRandomStream InRndStream, AWorldStartMarker* StartMarker)
{
    GenSettings = Settings;
    RndStream = InRndStream;

    if (!GenSettings || !StartMarker)
    {
        UE_LOG(LogTemp, Warning, TEXT("LocationRoom::Generate received invalid settings or start marker."));
        return;
    }

    UWorld* World = GetWorld();
    if (!World)
    {
        return;
    }

    // Align the room so that the start marker sits on the back wall.
    const FVector StartLocation = StartMarker->GetActorLocation();
    const FRotator EntranceRotation = StartMarker->GetActorRotation();
    const FVector Forward = EntranceRotation.Vector();
    const FVector Right = EntranceRotation.RotateVector(FVector::RightVector);

    const FVector2f HalfSize = GenSettings->RoomSize * 0.5f;
    const FVector RoomCenter = StartLocation + Forward * HalfSize.X;
    SetActorLocationAndRotation(RoomCenter, EntranceRotation);

    if (GenSettings->WallSegmentMesh)
    {
        WallISM->SetStaticMesh(GenSettings->WallSegmentMesh);
    }
    WallISM->ClearInstances();

    const FTransform RoomTransform = GetActorTransform();

    const auto PortalNormalFromWall = [&](int32 WallIndex) -> FVector
    {
        switch (WallIndex)
        {
        case 0: return -Forward;
        case 1: return Forward;
        case 2: return -Right;
        case 3: return Right;
        default: return Forward;
        }
    };

    const auto PortalLocalFromWall = [&](int32 WallIndex, float Axis) -> FVector
    {
        FVector Local(0.f, 0.f, 0.f);
        switch (WallIndex)
        {
        case 0:
            Local.X = -HalfSize.X;
            Local.Y = Axis;
            break;
        case 1:
            Local.X = HalfSize.X;
            Local.Y = Axis;
            break;
        case 2:
            Local.Y = -HalfSize.Y;
            Local.X = Axis;
            break;
        case 3:
            Local.Y = HalfSize.Y;
            Local.X = Axis;
            break;
        default:
            break;
        }
        return Local;
    };

    const auto PortalWorldLocation = [&](int32 WallIndex, float Axis) -> FVector
    {
        return RoomTransform.TransformPosition(PortalLocalFromWall(WallIndex, Axis));
    };

    const auto PortalAxisFromWorld = [&](int32 WallIndex, const FVector& WorldPoint) -> float
    {
        const FVector LocalPoint = RoomTransform.InverseTransformPosition(WorldPoint);
        switch (WallIndex)
        {
        case 0:
        case 1:
            return LocalPoint.Y;
        case 2:
        case 3:
            return LocalPoint.X;
        default:
            return 0.f;
        }
    };

    TMap<int32, TArray<float>> PortalOffsets;
    TArray<FPortalInfo> Portals;
    TArray<FVector> PortalLocations;

    FPortalInfo Entrance;
    Entrance.WallIndex = 0;
    Entrance.AxisOffset = 0.f;
    Entrance.WorldLocation = StartLocation;
    Entrance.WorldNormal = PortalNormalFromWall(0);
    Entrance.bIsExit = false;
    Entrance.AxisOffset = PortalAxisFromWorld(Entrance.WallIndex, Entrance.WorldLocation);

    Portals.Add(Entrance);
    PortalOffsets.FindOrAdd(0).Add(Entrance.AxisOffset);
    PortalLocations.Add(Entrance.WorldLocation);

    // Generate exits and spawn finish markers.
    const int32 ExitCount = GenSettings->MaxExitCount > 0 ? RndStream.RandRange(0, GenSettings->MaxExitCount) : 0;
    const TArray<int32> CandidateWalls = {1, 2, 3};

    for (int32 ExitIndex = 0; ExitIndex < ExitCount; ++ExitIndex)
    {
        const int32 WallIndex = CandidateWalls.IsEmpty() ? 1 : CandidateWalls[RndStream.RandRange(0, CandidateWalls.Num() - 1)];
        const float WallHalfLength = (WallIndex == 0 || WallIndex == 1) ? HalfSize.Y : HalfSize.X;
        const float HalfPortalWidth = GenSettings->PortalWidth * 0.5f;
        float MinAxis = -WallHalfLength + HalfPortalWidth + GenSettings->PortalEdgePadding;
        float MaxAxis = WallHalfLength - HalfPortalWidth - GenSettings->PortalEdgePadding;
        if (MinAxis > MaxAxis)
        {
            Swap(MinAxis, MaxAxis);
        }

        bool bPlaced = false;
        for (int32 Attempt = 0; Attempt < 32 && !bPlaced; ++Attempt)
        {
            const float AxisOffset = RndStream.FRandRange(MinAxis, MaxAxis);
            bool bOverlaps = false;
            if (const TArray<float>* Existing = PortalOffsets.Find(WallIndex))
            {
                for (float ExistingOffset : *Existing)
                {
                    const float AllowedSpacing = GenSettings->PortalWidth + GenSettings->PortalMinSeparation;
                    if (FMath::Abs(ExistingOffset - AxisOffset) < AllowedSpacing)
                    {
                        bOverlaps = true;
                        break;
                    }
                }
            }

            if (bOverlaps)
            {
                continue;
            }

            FPortalInfo ExitPortal;
            ExitPortal.WallIndex = WallIndex;
            ExitPortal.AxisOffset = AxisOffset;
            ExitPortal.WorldLocation = PortalWorldLocation(WallIndex, AxisOffset);
            ExitPortal.WorldNormal = PortalNormalFromWall(WallIndex);
            ExitPortal.bIsExit = true;
            ExitPortal.AxisOffset = PortalAxisFromWorld(WallIndex, ExitPortal.WorldLocation);

            Portals.Add(ExitPortal);
            PortalOffsets.FindOrAdd(WallIndex).Add(ExitPortal.AxisOffset);
            PortalLocations.Add(ExitPortal.WorldLocation);

            const FVector MarkerLocation = ExitPortal.WorldLocation + ExitPortal.WorldNormal * GenSettings->PortalSurfaceOffset;
            const FRotator MarkerRotation = ExitPortal.WorldNormal.Rotation();

            FActorSpawnParameters SpawnParams;
            SpawnParams.Owner = this;
            SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
            World->SpawnActor<AWorldFinishMarker>(AWorldFinishMarker::StaticClass(), MarkerLocation, MarkerRotation, SpawnParams);

            bPlaced = true;
        }
    }

    // Fallback point if there are no exits.
    FVector FallbackPoint = FVector::ZeroVector;
    bool bHasFallbackPoint = false;
    const auto SampleInterior = [&](const FVector2f& Margin) -> FVector
    {
        float MinX = -HalfSize.X + Margin.X;
        float MaxX = HalfSize.X - Margin.X;
        float MinY = -HalfSize.Y + Margin.Y;
        float MaxY = HalfSize.Y - Margin.Y;

        if (MinX > MaxX)
        {
            const float MidX = 0.f;
            MinX = MidX - 10.f;
            MaxX = MidX + 10.f;
        }
        if (MinY > MaxY)
        {
            const float MidY = 0.f;
            MinY = MidY - 10.f;
            MaxY = MidY + 10.f;
        }

        const float LocalX = RndStream.FRandRange(MinX, MaxX);
        const float LocalY = RndStream.FRandRange(MinY, MaxY);
        const FVector Local(LocalX, LocalY, 0.f);
        return RoomTransform.TransformPosition(Local);
    };

    if (Portals.Num() == 1)
    {
        const FVector EntranceLocation = Portals[0].WorldLocation;
        for (int32 Attempt = 0; Attempt < 32; ++Attempt)
        {
            const FVector Candidate = SampleInterior(GenSettings->InteriorMargin);
            if (FVector::Dist2D(Candidate, EntranceLocation) >= GenSettings->FallbackMinDistanceFromEntrance)
            {
                FallbackPoint = Candidate;
                bHasFallbackPoint = true;
                PortalLocations.Add(Candidate);
                break;
            }
        }
    }

    // Build the perimeter walls leaving openings for portals.
    FVector MeshSize(100.f, 100.f, 100.f);
    if (GenSettings->WallSegmentMesh)
    {
        const FBox Bounds = GenSettings->WallSegmentMesh->GetBoundingBox();
        const FVector BoundSize = Bounds.GetSize();
        MeshSize.X = BoundSize.X > KINDA_SMALL_NUMBER ? BoundSize.X : 100.f;
        MeshSize.Y = BoundSize.Y > KINDA_SMALL_NUMBER ? BoundSize.Y : 100.f;
        MeshSize.Z = BoundSize.Z > KINDA_SMALL_NUMBER ? BoundSize.Z : 100.f;
    }

    const auto BuildWall = [&](int32 WallIndex)
    {
        FVector WallCenterLocal(0.f, 0.f, 0.f);
        FVector Tangent = FVector::ZeroVector;

        const bool bHorizontal = (WallIndex == 0 || WallIndex == 1);
        const float WallLength = bHorizontal ? HalfSize.Y * 2.f : HalfSize.X * 2.f;

        switch (WallIndex)
        {
        case 0:
            WallCenterLocal.X = -HalfSize.X;
            Tangent = Right;
            break;
        case 1:
            WallCenterLocal.X = HalfSize.X;
            Tangent = Right;
            break;
        case 2:
            WallCenterLocal.Y = -HalfSize.Y;
            Tangent = Forward;
            break;
        case 3:
            WallCenterLocal.Y = HalfSize.Y;
            Tangent = Forward;
            break;
        default:
            return;
        }

        const int32 SegmentCount = FMath::Max(1, FMath::CeilToInt(WallLength / FMath::Max(10.f, GenSettings->WallSegmentLength)));
        const float ActualSegmentLength = WallLength / SegmentCount;

        const TArray<float>* DoorOffsets = PortalOffsets.Find(WallIndex);

        for (int32 SegmentIndex = 0; SegmentIndex < SegmentCount; ++SegmentIndex)
        {
            const float CenterOffset = -WallLength * 0.5f + ActualSegmentLength * (SegmentIndex + 0.5f);
            bool bBlockedByDoor = false;
            if (DoorOffsets)
            {
                for (float DoorOffset : *DoorOffsets)
                {
                    if (FMath::Abs(DoorOffset - CenterOffset) <= GenSettings->PortalWidth * 0.5f)
                    {
                        bBlockedByDoor = true;
                        break;
                    }
                }
            }

            if (bBlockedByDoor)
            {
                continue;
            }

            FVector LocalPosition = WallCenterLocal;
            if (bHorizontal)
            {
                LocalPosition.Y += CenterOffset;
            }
            else
            {
                LocalPosition.X += CenterOffset;
            }
            LocalPosition.Z = GenSettings->WallHeight * 0.5f;

            const FVector WorldPosition = RoomTransform.TransformPosition(LocalPosition);
            const FRotator SegmentRotation = FRotationMatrix::MakeFromXZ(Tangent, FVector::UpVector).Rotator();

            FVector Scale(1.f, 1.f, 1.f);
            Scale.X = ActualSegmentLength / MeshSize.X;
            Scale.Y = GenSettings->WallThickness / MeshSize.Y;
            Scale.Z = GenSettings->WallHeight / MeshSize.Z;

            const FTransform SegmentTransform(SegmentRotation, WorldPosition, Scale);
            WallISM->AddInstance(SegmentTransform);
        }
    };

    BuildWall(0);
    BuildWall(1);
    BuildWall(2);
    BuildWall(3);

    // Helper to create natural looking road paths.
    const auto ClampLocalToRoadBounds = [&](FVector& LocalPoint)
    {
        float MinX = -HalfSize.X + GenSettings->RoadClampMargin.X;
        float MaxX = HalfSize.X - GenSettings->RoadClampMargin.X;
        if (MinX > MaxX)
        {
            MinX = MaxX = 0.f;
        }

        float MinY = -HalfSize.Y + GenSettings->RoadClampMargin.Y;
        float MaxY = HalfSize.Y - GenSettings->RoadClampMargin.Y;
        if (MinY > MaxY)
        {
            MinY = MaxY = 0.f;
        }

        LocalPoint.X = FMath::Clamp(LocalPoint.X, MinX, MaxX);
        LocalPoint.Y = FMath::Clamp(LocalPoint.Y, MinY, MaxY);
        LocalPoint.Z = 0.f;
    };

    const auto BuildRoadPath = [&](const FVector& From, const FVector& To) -> TArray<FVector>
    {
        TArray<FVector> WorldPoints;

        const FTransform InverseTransform = RoomTransform.Inverse();
        const FVector LocalFrom = InverseTransform.TransformPosition(From);
        const FVector LocalTo = InverseTransform.TransformPosition(To);

        int32 MinControl = FMath::Max(0, GenSettings->RoadControlPointRange.X);
        int32 MaxControl = FMath::Max(MinControl, GenSettings->RoadControlPointRange.Y);
        const int32 ControlCount = RndStream.RandRange(MinControl, MaxControl);

        TArray<FVector> LocalPoints;
        LocalPoints.Add(LocalFrom);
        for (int32 ControlIndex = 0; ControlIndex < ControlCount; ++ControlIndex)
        {
            const float Alpha = (ControlIndex + 1.f) / (ControlCount + 1.f);
            FVector BasePoint = FMath::Lerp(LocalFrom, LocalTo, Alpha);
            BasePoint.X += RndStream.FRandRange(-GenSettings->RoadDeviation.X, GenSettings->RoadDeviation.X);
            BasePoint.Y += RndStream.FRandRange(-GenSettings->RoadDeviation.Y, GenSettings->RoadDeviation.Y);
            ClampLocalToRoadBounds(BasePoint);
            LocalPoints.Add(BasePoint);
        }
        LocalPoints.Add(LocalTo);

        for (int32 PointIndex = 0; PointIndex < LocalPoints.Num(); ++PointIndex)
        {
            FVector LocalPoint = LocalPoints[PointIndex];
            if (PointIndex != 0 && PointIndex != LocalPoints.Num() - 1)
            {
                ClampLocalToRoadBounds(LocalPoint);
            }
            else
            {
                LocalPoint.Z = 0.f;
            }

            FVector WorldPoint = RoomTransform.TransformPosition(LocalPoint);
            WorldPoint.Z += GenSettings->RoadSplineZOffset;
            WorldPoints.Add(WorldPoint);
        }
        return WorldPoints;
    };

    TArray<FVector> RoadNodes;
    TArray<TArray<FVector>> RoadPaths;
    const auto SpawnRoadBetween = [&](const FVector& From, const FVector& To)
    {
        if (!GenSettings->RoadSplineMesh)
        {
            return;
        }

        const TArray<FVector> Points = BuildRoadPath(From, To);
        if (Points.Num() < 2)
        {
            return;
        }

        TSubclassOf<ARoadSegment> RoadClass = GenSettings->RoadSegmentClass;
        if (!RoadClass)
        {
            RoadClass = ARoadSegment::StaticClass();
        }

        FActorSpawnParameters SpawnParams;
        SpawnParams.Owner = this;
        SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        ARoadSegment* Road = World->SpawnActor<ARoadSegment>(RoadClass, FTransform::Identity, SpawnParams);
        if (!Road)
        {
            return;
        }

        Road->AttachToActor(this, FAttachmentTransformRules::KeepWorldTransform);
        Road->BuildFromPoints(Points, GenSettings->RoadSplineMesh, GenSettings->RoadSplineScale);
        RoadPaths.Add(Points);
    };

    TArray<FRoadPortalAnchor> PortalAnchors;
    PortalAnchors.Reserve(Portals.Num());

    const auto BuildAnchorForPortal = [&](const FPortalInfo& Portal) -> FVector
    {
        FVector LocalAnchor = RoomTransform.InverseTransformPosition(Portal.WorldLocation);
        const float Inset = FMath::Max(GenSettings->PortalWidth * 0.5f, 300.f);
        switch (Portal.WallIndex)
        {
        case 0:
            LocalAnchor.X += Inset;
            LocalAnchor.Y += RndStream.FRandRange(-GenSettings->RoadDeviation.Y, GenSettings->RoadDeviation.Y);
            break;
        case 1:
            LocalAnchor.X -= Inset;
            LocalAnchor.Y += RndStream.FRandRange(-GenSettings->RoadDeviation.Y, GenSettings->RoadDeviation.Y);
            break;
        case 2:
            LocalAnchor.Y += Inset;
            LocalAnchor.X += RndStream.FRandRange(-GenSettings->RoadDeviation.X, GenSettings->RoadDeviation.X);
            break;
        case 3:
            LocalAnchor.Y -= Inset;
            LocalAnchor.X += RndStream.FRandRange(-GenSettings->RoadDeviation.X, GenSettings->RoadDeviation.X);
            break;
        default:
            LocalAnchor.X += Inset;
            break;
        }

        ClampLocalToRoadBounds(LocalAnchor);
        return RoomTransform.TransformPosition(LocalAnchor);
    };

    for (const FPortalInfo& Portal : Portals)
    {
        FRoadPortalAnchor AnchorInfo;
        AnchorInfo.Portal = Portal;
        AnchorInfo.Anchor = BuildAnchorForPortal(Portal);
        PortalAnchors.Add(AnchorInfo);
        RoadNodes.Add(AnchorInfo.Anchor);
    }

    if (bHasFallbackPoint)
    {
        RoadNodes.Add(FallbackPoint);
    }

    for (const FRoadPortalAnchor& AnchorInfo : PortalAnchors)
    {
        if (!AnchorInfo.Anchor.Equals(AnchorInfo.Portal.WorldLocation, 1.f))
        {
            SpawnRoadBetween(AnchorInfo.Portal.WorldLocation, AnchorInfo.Anchor);
        }
    }

    if (RoadNodes.Num() > 1)
    {
        TArray<int32> ConnectedIndices;
        ConnectedIndices.Add(0);
        for (int32 NodeIndex = 1; NodeIndex < RoadNodes.Num(); ++NodeIndex)
        {
            float BestDistance = TNumericLimits<float>::Max();
            int32 BestConnectedIndex = 0;
            for (int32 ConnectedIndex : ConnectedIndices)
            {
                const float DistSq = FVector::DistSquared2D(RoadNodes[ConnectedIndex], RoadNodes[NodeIndex]);
                if (DistSq < BestDistance)
                {
                    BestDistance = DistSq;
                    BestConnectedIndex = ConnectedIndex;
                }
            }

            if (BestDistance < KINDA_SMALL_NUMBER)
            {
                continue;
            }

            SpawnRoadBetween(RoadNodes[BestConnectedIndex], RoadNodes[NodeIndex]);
            ConnectedIndices.Add(NodeIndex);
        }
    }

    // --- POI spawning ---
    TArray<FVector> PoiLocations;
    if (!GenSettings->POITable.IsEmpty() && RndStream.FRand() <= GenSettings->POISpawnChance)
    {
        TArray<TSubclassOf<AActor>> PoiClasses;
        for (const FSpawnStruct& Entry : GenSettings->POITable)
        {
            const int32 Count = SampleSpawnCount(Entry, RndStream);
            for (int32 SpawnIndex = 0; SpawnIndex < Count; ++SpawnIndex)
            {
                PoiClasses.Add(Entry.Class);
            }
        }

        Algo::Shuffle(PoiClasses, RndStream);
        for (const TSubclassOf<AActor>& PoiClass : PoiClasses)
        {
            if (!PoiClass)
            {
                continue;
            }

            bool bPlaced = false;
            for (int32 Attempt = 0; Attempt < 32 && !bPlaced; ++Attempt)
            {
                const FVector Candidate = SampleInterior(GenSettings->POIMargin);
                bool bTooClose = false;
                for (const FVector& Portal : PortalLocations)
                {
                    if (FVector::Dist2D(Candidate, Portal) < GenSettings->POIMinDistanceFromPortals)
                    {
                        bTooClose = true;
                        break;
                    }
                }
                if (bTooClose)
                {
                    continue;
                }
                for (const FVector& Poi : PoiLocations)
                {
                    if (FVector::Dist2D(Candidate, Poi) < GenSettings->POIMinDistanceBetween)
                    {
                        bTooClose = true;
                        break;
                    }
                }
                if (bTooClose)
                {
                    continue;
                }

                const FRotator SpawnRotation(0.f, RndStream.FRandRange(-180.f, 180.f), 0.f);
                FActorSpawnParameters SpawnParams;
                SpawnParams.Owner = this;
                SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
                if (AActor* Spawned = World->SpawnActor<AActor>(PoiClass, Candidate, SpawnRotation, SpawnParams))
                {
                    PoiLocations.Add(Candidate);
                    bPlaced = true;
                }
            }
        }
    }

    // Optional roads that lead to POIs.
    if (!PoiLocations.IsEmpty() && GenSettings->RoadSplineMesh)
    {
        for (const FVector& PoiLocation : PoiLocations)
        {
            if (RndStream.FRand() > GenSettings->ProbabilityOfRoadToPOI)
            {
                continue;
            }

            float BestDistance = TNumericLimits<float>::Max();
            FVector BestAnchor = RoadNodes.IsEmpty() ? Portals[0].WorldLocation : RoadNodes[0];
            for (const FVector& Anchor : RoadNodes)
            {
                const float DistSq = FVector::DistSquared2D(Anchor, PoiLocation);
                if (DistSq < BestDistance)
                {
                    BestDistance = DistSq;
                    BestAnchor = Anchor;
                }
            }

            SpawnRoadBetween(BestAnchor, PoiLocation);
            RoadNodes.Add(PoiLocation);
        }
    }

    // --- Monster spawning ---
    TArray<FVector> MonsterLocations;
    if (!GenSettings->MonsterTable.IsEmpty() && RndStream.FRand() <= GenSettings->MonsterSpawnChance)
    {
        TArray<TSubclassOf<AActor>> MonsterClasses;
        for (const FSpawnStruct& Entry : GenSettings->MonsterTable)
        {
            const int32 Count = SampleSpawnCount(Entry, RndStream);
            for (int32 SpawnIndex = 0; SpawnIndex < Count; ++SpawnIndex)
            {
                MonsterClasses.Add(Entry.Class);
            }
        }

        Algo::Shuffle(MonsterClasses, RndStream);
        for (const TSubclassOf<AActor>& MonsterClass : MonsterClasses)
        {
            if (!MonsterClass)
            {
                continue;
            }

            bool bPlaced = false;
            for (int32 Attempt = 0; Attempt < 48 && !bPlaced; ++Attempt)
            {
                const FVector Candidate = SampleInterior(GenSettings->MonsterMargin);
                bool bBlocked = false;

                for (const FVector& Portal : PortalLocations)
                {
                    if (FVector::Dist2D(Candidate, Portal) < GenSettings->MonsterMinDistanceFromPortals)
                    {
                        bBlocked = true;
                        break;
                    }
                }
                if (bBlocked)
                {
                    continue;
                }

                for (const FVector& Poi : PoiLocations)
                {
                    if (FVector::Dist2D(Candidate, Poi) < GenSettings->MonsterMinDistanceFromPOI)
                    {
                        bBlocked = true;
                        break;
                    }
                }
                if (bBlocked)
                {
                    continue;
                }

                for (const FVector& Monster : MonsterLocations)
                {
                    if (FVector::Dist2D(Candidate, Monster) < GenSettings->MonsterMinDistanceBetween)
                    {
                        bBlocked = true;
                        break;
                    }
                }
                if (bBlocked)
                {
                    continue;
                }

                float RoadDistance = TNumericLimits<float>::Max();
                for (const TArray<FVector>& Road : RoadPaths)
                {
                    for (int32 PointIndex = 0; PointIndex + 1 < Road.Num(); ++PointIndex)
                    {
                        const float Dist = DistancePointToSegment2D(Candidate, Road[PointIndex], Road[PointIndex + 1]);
                        RoadDistance = FMath::Min(RoadDistance, Dist);
                    }
                }

                if (RoadDistance < GenSettings->MonsterMinDistanceFromRoads)
                {
                    continue;
                }

                const FRotator SpawnRotation(0.f, RndStream.FRandRange(-180.f, 180.f), 0.f);
                FActorSpawnParameters SpawnParams;
                SpawnParams.Owner = this;
                SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
                if (AActor* Spawned = World->SpawnActor<AActor>(MonsterClass, Candidate, SpawnRotation, SpawnParams))
                {
                    MonsterLocations.Add(Candidate);
                    bPlaced = true;
                }
            }
        }
    }
}
