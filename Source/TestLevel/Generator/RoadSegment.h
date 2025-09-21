// Copyright notice placeholder

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "RoadSegment.generated.h"

class USplineComponent;
class UStaticMesh;

UCLASS()
class TESTLEVEL_API ARoadSegment : public AActor
{
	GENERATED_BODY()


public:
        ARoadSegment();

        /** Build spline and spline-mesh representation from given world-space points. */
        void BuildFromPoints(const TArray<FVector>& Points, UStaticMesh* Mesh, const FVector2f& Scale);

	/** Build spline and spline-mesh representation from given world-space points. */
	void BuildFromPoints(const TArray<FVector>& Points, UStaticMesh* Mesh, const FVector2f& Scale);

protected:
        UPROPERTY(VisibleAnywhere)
        USceneComponent* Root;

        UPROPERTY(VisibleAnywhere)
        USplineComponent* Spline;

private:
        void ResetSplineMeshes();


        UPROPERTY()
        TArray<class USplineMeshComponent*> SplineMeshes;

	UPROPERTY(VisibleAnywhere)
	USplineComponent* Spline;

private:
	void ResetSplineMeshes();


};
