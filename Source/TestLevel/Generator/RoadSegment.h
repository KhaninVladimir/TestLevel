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

protected:
	UPROPERTY(VisibleAnywhere)
	USceneComponent* Root;

	void BuildOnePath(const TArray<FVector>& PathPointsWS);

	int32 FindNearestPointOnPath(const FVector& Point, const TArray<FVector>& Path);

	// Utility used by BuildNetwork
	void ComputeMST_Prim(const TArray<FVector2f>& PtsLocal, TArray<FIntPoint>& OutEdges);
	void MaybeAddShortcuts(const TArray<FVector2f>& PtsLocal, const FVector2f& H, TArray<FIntPoint>& InOutEdges);

	// Generate naturally-curved path points for the edge (A,B)
        void MakeCurvedPath(const FVector& A, const FVector& B,
                int32 MidCount,
                float MaxPerp,
                float NoiseJitter,
                float TangentStrength,
                float BaselineCurvature,
                FRandomStream& Rng,
                TArray<FVector>& OutPoints);

        // Compute per-edge offset scale so paths near walls wiggle less
        float EdgeOffsetScale(const FVector2f& A_L, const FVector2f& B_L, const FVector2f& H);

        FVector AdjustForObstacles(const FVector& Point) const;
        FVector ClampToRoomBounds(const FVector& Point) const;

        UPROPERTY()
        TArray<class USplineMeshComponent*> MeshSegments;

        void ClearMeshes();

public:
        // Returns minimal planar distance (XY) from the cached road polylines.
        float DistanceToRoads(const FVector& Point) const;

private:
        UPROPERTY()
        const UWorldGenSettings* GenSettings = nullptr;

        TArray<TArray<FVector>> BuiltPaths;
        TArray<FEnvironmentObstacle> CachedObstacles;
        FVector2f CachedRoomHalfSize = FVector2f::ZeroVector;

        float ClearanceToRect(const FVector2f& P, const FVector2f& H);

};
