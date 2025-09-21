// Copyright notice placeholder

#include "Generator/RoadSegment.h"

#include "Components/SplineComponent.h"
#include "Components/SplineMeshComponent.h"
#include "Engine/StaticMesh.h"

ARoadSegment::ARoadSegment()
{

        PrimaryActorTick.bCanEverTick = false;

        Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
        SetRootComponent(Root);

        Spline = CreateDefaultSubobject<USplineComponent>(TEXT("Spline"));
        Spline->SetupAttachment(RootComponent);
        Spline->SetMobility(EComponentMobility::Movable);
        Spline->bDrawDebug = false;

}

void ARoadSegment::ResetSplineMeshes()
{

        for (USplineMeshComponent* MeshComp : SplineMeshes)
        {
                if (MeshComp)
                {
                        MeshComp->DestroyComponent();
                }
        }
        SplineMeshes.Reset();

}

void ARoadSegment::BuildFromPoints(const TArray<FVector>& Points, UStaticMesh* Mesh, const FVector2f& Scale)
{

        if (Points.Num() < 2 || !Mesh)
        {
                return;
        }

        ResetSplineMeshes();

        SetActorLocation(Points[0]);
        TArray<FVector> LocalPoints;
        LocalPoints.Reserve(Points.Num());
        LocalPoints.Add(FVector::ZeroVector);
        for (int32 Index = 1; Index < Points.Num(); ++Index)
        {
                LocalPoints.Add(Points[Index] - Points[0]);
        }

        Spline->ClearSplinePoints(false);
        for (const FVector& Point : LocalPoints)
        {
                Spline->AddSplinePoint(Point, ESplineCoordinateSpace::Local, false);
        }
        Spline->UpdateSpline();

        for (int32 PointIndex = 0; PointIndex + 1 < Spline->GetNumberOfSplinePoints(); ++PointIndex)
        {
                USplineMeshComponent* MeshComp = NewObject<USplineMeshComponent>(this);
                MeshComp->SetMobility(EComponentMobility::Movable);
                MeshComp->SetStaticMesh(Mesh);
                MeshComp->SetForwardAxis(ESplineMeshAxis::X, true);
                MeshComp->SetCollisionEnabled(ECollisionEnabled::NoCollision);

                MeshComp->AttachToComponent(Spline, FAttachmentTransformRules::KeepRelativeTransform);
                MeshComp->RegisterComponent();
                SplineMeshes.Add(MeshComp);

                FVector StartPos, StartTangent;
                FVector EndPos, EndTangent;
                Spline->GetLocationAndTangentAtSplinePoint(PointIndex, StartPos, StartTangent, ESplineCoordinateSpace::Local);
                Spline->GetLocationAndTangentAtSplinePoint(PointIndex + 1, EndPos, EndTangent, ESplineCoordinateSpace::Local);

                MeshComp->SetStartAndEnd(StartPos, StartTangent, EndPos, EndTangent);
                const FVector2D MeshScale(Scale.X, Scale.Y);
                MeshComp->SetStartScale(MeshScale);
                MeshComp->SetEndScale(MeshScale);
        }

}
