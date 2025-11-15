// Copyright notice placeholder.

#include "Generator/RoadSegment.h"
#include "Components/SplineComponent.h"
#include "Components/SplineMeshComponent.h"
#include "Engine/World.h"
#include "Templates/NumericLimits.h"

namespace
{
        /** Distance from a 2D point to a segment defined by A/B. */
        float DistancePointToSegment2D(const FVector2D& P, const FVector2D& A, const FVector2D& B)
        {
                const FVector2D AB = B - A;
                const float Den = AB.SizeSquared();
                if (Den <= KINDA_SMALL_NUMBER)
                {
                        return FVector2D::Distance(P, A);
                }
                const float T = FMath::Clamp(FVector2D::DotProduct(P - A, AB) / Den, 0.f, 1.f);
                const FVector2D Closest = A + AB * T;
                return FVector2D::Distance(P, Closest);
        }
}

ARoadSegment::ARoadSegment()
{
        PrimaryActorTick.bCanEverTick = false;
        Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
        SetRootComponent(Root);
}

float ARoadSegment::DistanceToRoads(const FVector& Point) const
{
        float Best = FLT_MAX;
        const FVector2D P(Point.X, Point.Y);
        for (const TArray<FVector>& Poly : RoadPolylines)
        {
                for (int32 idx = 0; idx + 1 < Poly.Num(); ++idx)
                {
                        const FVector2D A(Poly[idx].X, Poly[idx].Y);
                        const FVector2D B(Poly[idx + 1].X, Poly[idx + 1].Y);
                        Best = FMath::Min(Best, DistancePointToSegment2D(P, A, B));
                }
        }
        return Best;
}

void ARoadSegment::ClearNetwork()
{
        for (USplineMeshComponent* Mesh : SplineMeshes)
        {
                if (Mesh)
                {
                        Mesh->DestroyComponent();
                }
        }
        SplineMeshes.Reset();

        for (USplineComponent* Spline : Splines)
        {
                if (Spline)
                {
                        Spline->DestroyComponent();
                }
        }
        Splines.Reset();

        RoadPolylines.Reset();
}

void ARoadSegment::BuildNetwork(const TArray<FVector>& NodesWS, int32 ExitCount, const FVector2f& RoomHalfSize, const UWorldGenSettings* Settings, FRandomStream& Rng, const TArray<FEnvironmentObstacle>& Obstacles)
{
        ClearNetwork();
        static_cast<void>(ExitCount);

        if (!Settings || !Settings->RoadSplineMesh)
        {
                return;
        }
        if (NodesWS.Num() <= 1)
        {
                return;
        }

        TArray<FVector> Nodes = NodesWS;
        const FVector2f Margin = Settings->RoadMargin;

        TArray<int32> Remaining;
        for (int32 idx = 1; idx < Nodes.Num(); ++idx)
        {
                Remaining.Add(idx);
        }
        TArray<int32> Connected;
        Connected.Add(0);

        while (!Remaining.IsEmpty())
        {
                float BestDist = TNumericLimits<float>::Max();
                int32 BestRemainingIdx = 0;
                int32 BestAnchor = 0;

                for (int32 i = 0; i < Remaining.Num(); ++i)
                {
                        const int32 CandidateIdx = Remaining[i];
                        for (int32 AnchorIdx : Connected)
                        {
                                const float Dist = FVector::Dist2D(Nodes[CandidateIdx], Nodes[AnchorIdx]);
                                if (Dist < BestDist)
                                {
                                        BestDist = Dist;
                                        BestRemainingIdx = i;
                                        BestAnchor = AnchorIdx;
                                }
                        }
                }

                const int32 TargetIndex = Remaining[BestRemainingIdx];
                Remaining.RemoveAtSwap(BestRemainingIdx);

                if (Connected.Num() > 1 && Rng.FRand() < 0.3f)
                {
                        BestAnchor = Connected[Rng.RandRange(0, Connected.Num() - 1)];
                }

                const FVector Start = Nodes[BestAnchor];
                const FVector End = Nodes[TargetIndex];
                const float Length2D = FVector::Dist2D(Start, End);
                if (Length2D <= KINDA_SMALL_NUMBER)
                {
                        TArray<FVector> Polyline;
                        Polyline.Add(Start);
                        Polyline.Add(End);
                        RoadPolylines.Add(Polyline);
                        BuildSingleRoad(Polyline, Settings);
                        Connected.Add(TargetIndex);
                        continue;
                }

                const FVector Forward = FVector(End.X - Start.X, End.Y - Start.Y, 0.f).GetSafeNormal();
                FVector Right = FVector::CrossProduct(FVector::UpVector, Forward).GetSafeNormal();
                if (Right.IsNearlyZero())
                {
                        Right = FVector::RightVector;
                }

                const int32 MidCount = FMath::Max(1, Settings->RoadMidpointCount);
                TArray<FVector> Polyline;
                Polyline.Reserve(MidCount + 2);
                Polyline.Add(Start);

                for (int32 Mid = 0; Mid < MidCount; ++Mid)
                {
                        const float T = static_cast<float>(Mid + 1) / static_cast<float>(MidCount + 1);
                        FVector Candidate = FMath::Lerp(Start, End, T);
                        const float Curvature = Settings->RoadBaselineCurvature * Length2D;
                        const float Sign = (Rng.FRand() < 0.5f) ? -1.f : 1.f;
                        const float Offset = Sign * Rng.FRandRange(Settings->RoadMaxPerpOffset * 0.3f, Settings->RoadMaxPerpOffset);
                        Candidate += Right * (Offset + Curvature * FMath::Sin(PI * T));
                        Candidate += Forward * Rng.FRandRange(-Settings->RoadNoiseJitter, Settings->RoadNoiseJitter);

                        for (const FEnvironmentObstacle& Ob : Obstacles)
                        {
                                FVector Delta = Candidate - Ob.Location;
                                Delta.Z = 0.f;
                                const float Dist = Delta.Size();
                                const float Required = Ob.Radius + Settings->RoadExtraClearanceUU;
                                if (Dist < Required && Dist > KINDA_SMALL_NUMBER)
                                {
                                        Candidate += Delta.GetSafeNormal() * (Required - Dist);
                                }
                                else if (Dist <= KINDA_SMALL_NUMBER)
                                {
                                        const FVector RandDir = FVector(Rng.FRandRange(-1.f, 1.f), Rng.FRandRange(-1.f, 1.f), 0.f).GetSafeNormal();
                                        Candidate += RandDir * Required;
                                }
                        }

                        Candidate = ClampToRoom(Candidate, RoomHalfSize, Margin);
                        Candidate.Z = Start.Z;
                        Polyline.Add(Candidate);
                }

                Polyline.Add(End);
                RoadPolylines.Add(Polyline);
                BuildSingleRoad(Polyline, Settings);
                Connected.Add(TargetIndex);
        }
}

FVector ARoadSegment::ClampToRoom(const FVector& WorldPoint, const FVector2f& RoomHalfSize, const FVector2f& Margin) const
{
        const FTransform RoomTransform = GetActorTransform();
        FVector Local = RoomTransform.InverseTransformPosition(WorldPoint);

        const float AllowX = FMath::Max(0.f, RoomHalfSize.X - FMath::Max(0.f, Margin.X));
        const float AllowY = FMath::Max(0.f, RoomHalfSize.Y - FMath::Max(0.f, Margin.Y));

        Local.X = FMath::Clamp(Local.X, -AllowX, AllowX);
        Local.Y = FMath::Clamp(Local.Y, -AllowY, AllowY);
        return RoomTransform.TransformPosition(Local);
}

void ARoadSegment::BuildSingleRoad(const TArray<FVector>& Polyline, const UWorldGenSettings* Settings)
{
        if (Polyline.Num() < 2 || !Settings || !Settings->RoadSplineMesh)
        {
                return;
        }

        USplineComponent* Spline = NewObject<USplineComponent>(this);
        if (!Spline)
        {
                return;
        }
        Spline->SetupAttachment(RootComponent);
        Spline->SetMobility(EComponentMobility::Movable);
        Spline->RegisterComponent();
        Spline->ClearSplinePoints(false);
        for (const FVector& P : Polyline)
        {
                Spline->AddSplinePoint(P, ESplineCoordinateSpace::World, false);
        }
        Spline->SetClosedLoop(false);
        Spline->UpdateSpline();
        Splines.Add(Spline);

        for (int32 i = 0; i + 1 < Spline->GetNumberOfSplinePoints(); ++i)
        {
                USplineMeshComponent* SegmentMesh = NewObject<USplineMeshComponent>(this);
                if (!SegmentMesh)
                {
                        continue;
                }
                SegmentMesh->SetMobility(EComponentMobility::Movable);
                SegmentMesh->SetStaticMesh(Settings->RoadSplineMesh);
                SegmentMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
                SegmentMesh->SetupAttachment(Spline);
                SegmentMesh->RegisterComponent();

                const FVector StartPos = Spline->GetLocationAtSplinePoint(i, ESplineCoordinateSpace::Local);
                const FVector EndPos = Spline->GetLocationAtSplinePoint(i + 1, ESplineCoordinateSpace::Local);
                const FVector StartTan = Spline->GetTangentAtSplinePoint(i, ESplineCoordinateSpace::Local).GetClampedToMaxSize(Settings->RoadTangentStrength);
                const FVector EndTan = Spline->GetTangentAtSplinePoint(i + 1, ESplineCoordinateSpace::Local).GetClampedToMaxSize(Settings->RoadTangentStrength);

                SegmentMesh->SetStartAndEnd(StartPos, StartTan, EndPos, EndTan);
                SplineMeshes.Add(SegmentMesh);
        }
}
