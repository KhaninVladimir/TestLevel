// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "WorldGenTypes.h"
#include "Components/SplineMeshComponent.h"
#include "WorldGenSettings.h"
#include "RoadSegment.generated.h"

class USplineComponent;

UCLASS()
class TESTLEVEL_API ARoadSegment : public AActor
{
	GENERATED_BODY()
	
public:
	ARoadSegment();

	void BuildNetwork(const TArray<FVector>& NodesWS, int32 ExitCount, const FVector2f& RoomHalfSize, const UWorldGenSettings* Settings, FRandomStream& Rng, const TArray<FEnvironmentObstacle>& Obstacles);

	void ClearNetwork();

	// Returns minimal planar distance (XY) from the cached road polylines.
	float DistanceToRoads(const FVector& Point) const;

protected:
	UPROPERTY(VisibleAnywhere)
	USceneComponent* Root;


};
