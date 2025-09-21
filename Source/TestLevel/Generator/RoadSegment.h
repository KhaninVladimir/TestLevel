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

private:
	// Runtime components used to render spline roads.
	UPROPERTY(Transient)
	TArray<USplineComponent*> RoadSplines;

	UPROPERTY(Transient)
	TArray<USplineMeshComponent*> RoadMeshes;

	// Cached polylines describing the generated roads (world-space points).
	UPROPERTY(Transient)
	TArray<TArray<FVector>> CachedPolylines;

	void RegisterRoadSpline(const TArray<FVector>& Polyline, const UWorldGenSettings* Settings);
	TArray<FVector> MakeNaturalPath(const FVector& Start, const FVector& End, const FVector2f& RoomHalfSize, const UWorldGenSettings* Settings, FRandomStream& Rng, const TArray<FEnvironmentObstacle>& Obstacles) const;
	FVector ClampToRoomBounds(const FVector& Point, const FVector2f& RoomHalfSize, const UWorldGenSettings* Settings) const;
	FVector PushOutOfObstacles(const FVector& Point, const TArray<FEnvironmentObstacle>& Obstacles, float Clearance, FRandomStream& Rng) const;
	static FVector ClosestPointOnSegment2D(const FVector& Point, const FVector& A, const FVector& B);
	static float DistancePointToSegment2D(const FVector2D& Point, const FVector2D& A, const FVector2D& B);
};
