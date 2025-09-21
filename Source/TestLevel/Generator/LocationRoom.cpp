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
    FRotator EntranceRotation = StartMarker->GetActorRotation();
    EntranceRotation.Pitch = 0.f;
    EntranceRotation.Roll = 0.f;
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

    // Ensure the entrance opening lines up exactly with the provided start marker
    // even if the marker was placed away from world origin.
    const FVector ExpectedEntranceWorld = PortalWorldLocation(0, 0.f);
    const FVector EntranceCorrection = StartLocation - ExpectedEntranceWorld;
    if (!EntranceCorrection.IsNearlyZero(1.f))
    {
        AddActorWorldOffset(EntranceCorrection);
        RoomTransform = GetActorTransform();
    }

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

    // --- Road generation ---
    TArray<TArray<FVector>> RoadPaths;
    if (GenSettings->RoadSplineMesh)
    {
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

        struct FRoadNode
        {
            FVector Anchor = FVector::ZeroVector;
            FVector EntrancePoint = FVector::ZeroVector;
            bool bHasEntrance = false;
        };

        const float BaseInset = FMath::Max(GenSettings->PortalWidth * 0.5f, 320.f);
        auto BuildAnchorTowardsInterior = [&](const FVector& WorldPoint, float PullStrength) -> FVector
        {
            FVector LocalPoint = RoomTransform.InverseTransformPosition(WorldPoint);
            FVector LocalDirection = (-LocalPoint);
            if (!LocalDirection.IsNearlyZero())
            {
                LocalDirection.Normalize();
                LocalPoint += LocalDirection * BaseInset * PullStrength;
            }

            LocalPoint.X += RndStream.FRandRange(-GenSettings->RoadDeviation.X * PullStrength, GenSettings->RoadDeviation.X * PullStrength);
            LocalPoint.Y += RndStream.FRandRange(-GenSettings->RoadDeviation.Y * PullStrength, GenSettings->RoadDeviation.Y * PullStrength);
            ClampLocalToRoadBounds(LocalPoint);
            return RoomTransform.TransformPosition(LocalPoint);
        };

        auto MakePortalNode = [&](const FPortalInfo& Portal) -> FRoadNode
        {
            FRoadNode Node;
            Node.bHasEntrance = true;
            Node.EntrancePoint = Portal.WorldLocation;

            FVector LocalAnchor = RoomTransform.InverseTransformPosition(Portal.WorldLocation);
            switch (Portal.WallIndex)
            {
            case 0:
                LocalAnchor.X += BaseInset;
                LocalAnchor.Y += RndStream.FRandRange(-GenSettings->RoadDeviation.Y * 0.5f, GenSettings->RoadDeviation.Y * 0.5f);
                break;
            case 1:
                LocalAnchor.X -= BaseInset;
                LocalAnchor.Y += RndStream.FRandRange(-GenSettings->RoadDeviation.Y * 0.5f, GenSettings->RoadDeviation.Y * 0.5f);
                break;
            case 2:
                LocalAnchor.Y += BaseInset;
                LocalAnchor.X += RndStream.FRandRange(-GenSettings->RoadDeviation.X * 0.5f, GenSettings->RoadDeviation.X * 0.5f);
                break;
            case 3:
                LocalAnchor.Y -= BaseInset;
                LocalAnchor.X += RndStream.FRandRange(-GenSettings->RoadDeviation.X * 0.5f, GenSettings->RoadDeviation.X * 0.5f);
                break;
            default:
                LocalAnchor.X += BaseInset;
                break;
            }

            ClampLocalToRoadBounds(LocalAnchor);
            Node.Anchor = RoomTransform.TransformPosition(LocalAnchor);
            return Node;
        };

        TArray<FRoadNode> RoadNodesData;
        RoadNodesData.Reserve(Portals.Num() + PoiLocations.Num() + 4);
        for (const FPortalInfo& Portal : Portals)
        {
            RoadNodesData.Add(MakePortalNode(Portal));
        }

        if (bHasFallbackPoint)
        {
            FRoadNode FallbackNode;
            FallbackNode.bHasEntrance = true;
            FallbackNode.EntrancePoint = FallbackPoint;
            FallbackNode.Anchor = BuildAnchorTowardsInterior(FallbackPoint, 0.5f);
            RoadNodesData.Add(FallbackNode);
        }

        for (const FVector& PoiLocation : PoiLocations)
        {
            if (RndStream.FRand() > GenSettings->ProbabilityOfRoadToPOI)
            {
                continue;
            }

            FRoadNode PoiNode;
            PoiNode.bHasEntrance = true;
            PoiNode.EntrancePoint = PoiLocation;
            PoiNode.Anchor = BuildAnchorTowardsInterior(PoiLocation, 0.35f);
            RoadNodesData.Add(PoiNode);
        }

        if (RoadNodesData.Num() > 2)
        {
            const int32 GuideCount = 2;
            for (int32 GuideIndex = 0; GuideIndex < GuideCount; ++GuideIndex)
            {
                FRoadNode GuideNode;
                FVector LocalGuide(
                    RndStream.FRandRange(-HalfSize.X * 0.35f, HalfSize.X * 0.35f),
                    RndStream.FRandRange(-HalfSize.Y * 0.35f, HalfSize.Y * 0.35f),
                    0.f);
                ClampLocalToRoadBounds(LocalGuide);
                GuideNode.Anchor = RoomTransform.TransformPosition(LocalGuide);
                RoadNodesData.Add(GuideNode);
            }
        }

        auto AppendRoadPoint = [&](TArray<FVector>& Points, const FVector& WorldPoint)
        {
            FVector Adjusted = WorldPoint;
            Adjusted.Z += GenSettings->RoadSplineZOffset;
            if (Points.IsEmpty() || !Points.Last().Equals(Adjusted, 1.f))
            {
                Points.Add(Adjusted);
            }
        };

        auto SpawnRoad = [&](const TArray<FVector>& Points)
        {
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
            if (ARoadSegment* Road = World->SpawnActor<ARoadSegment>(RoadClass, FTransform::Identity, SpawnParams))
            {
                Road->AttachToActor(this, FAttachmentTransformRules::KeepWorldTransform);
                Road->BuildFromPoints(Points, GenSettings->RoadSplineMesh, GenSettings->RoadSplineScale);
                RoadPaths.Add(Points);
            }
        };

        TArray<FIntPoint> RoadEdges;
        if (RoadNodesData.Num() > 1)
        {
            TArray<int32> Connected;
            TArray<int32> Remaining;
            Connected.Add(0);
            for (int32 Index = 1; Index < RoadNodesData.Num(); ++Index)
            {
                Remaining.Add(Index);
            }

            while (!Remaining.IsEmpty())
            {
                float BestDist = TNumericLimits<float>::Max();
                int32 BestConnected = INDEX_NONE;
                int32 BestRemaining = INDEX_NONE;

                for (int32 ConnectedIndex : Connected)
                {
                    for (int32 RemainingIndex = 0; RemainingIndex < Remaining.Num(); ++RemainingIndex)
                    {
                        const int32 Candidate = Remaining[RemainingIndex];
                        const float DistSq = FVector::DistSquared2D(RoadNodesData[ConnectedIndex].Anchor, RoadNodesData[Candidate].Anchor);
                        if (DistSq < BestDist)
                        {
                            BestDist = DistSq;
                            BestConnected = ConnectedIndex;
                            BestRemaining = RemainingIndex;
                        }
                    }
                }

                if (BestRemaining == INDEX_NONE)
                {
                    break;
                }

                const int32 NodeIndex = Remaining[BestRemaining];
                Remaining.RemoveAtSwap(BestRemaining);
                RoadEdges.Add(FIntPoint(BestConnected, NodeIndex));
                Connected.Add(NodeIndex);
            }
        }

        TArray<bool> bEntranceLinked;
        bEntranceLinked.Init(false, RoadNodesData.Num());

        for (const FIntPoint& Edge : RoadEdges)
        {
            const FRoadNode& NodeA = RoadNodesData[Edge.X];
            const FRoadNode& NodeB = RoadNodesData[Edge.Y];

            TArray<FVector> Points;
            Points.Reserve(6);

            if (NodeA.bHasEntrance && !bEntranceLinked[Edge.X])
            {
                AppendRoadPoint(Points, NodeA.EntrancePoint);
                bEntranceLinked[Edge.X] = true;
            }

            AppendRoadPoint(Points, NodeA.Anchor);

            const int32 ControlCount = FMath::Max(0, RndStream.RandRange(GenSettings->RoadControlPointRange.X, GenSettings->RoadControlPointRange.Y));
            for (int32 ControlIndex = 0; ControlIndex < ControlCount; ++ControlIndex)
            {
                const float Alpha = (ControlIndex + 1.f) / (ControlCount + 1.f);
                FVector LocalPoint = RoomTransform.InverseTransformPosition(FMath::Lerp(NodeA.Anchor, NodeB.Anchor, Alpha));
                LocalPoint.X += RndStream.FRandRange(-GenSettings->RoadDeviation.X, GenSettings->RoadDeviation.X);
                LocalPoint.Y += RndStream.FRandRange(-GenSettings->RoadDeviation.Y, GenSettings->RoadDeviation.Y);
                ClampLocalToRoadBounds(LocalPoint);
                AppendRoadPoint(Points, RoomTransform.TransformPosition(LocalPoint));
            }

            AppendRoadPoint(Points, NodeB.Anchor);

            if (NodeB.bHasEntrance && !bEntranceLinked[Edge.Y])
            {
                AppendRoadPoint(Points, NodeB.EntrancePoint);
                bEntranceLinked[Edge.Y] = true;
            }

            SpawnRoad(Points);
        }

        for (int32 NodeIndex = 0; NodeIndex < RoadNodesData.Num(); ++NodeIndex)
        {
            if (!RoadNodesData[NodeIndex].bHasEntrance || bEntranceLinked[NodeIndex])
            {
                continue;
            }

            TArray<FVector> Points;
            AppendRoadPoint(Points, RoadNodesData[NodeIndex].EntrancePoint);
            AppendRoadPoint(Points, RoadNodesData[NodeIndex].Anchor);
            SpawnRoad(Points);
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
