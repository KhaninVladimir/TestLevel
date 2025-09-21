// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "WorldGenTypes.h"
#include "LocationRoom.generated.h"

class UWorldGenSettings;
class AWorldStartMarker;
class AWorldFinishMarker;
class ARoadSegment;
class UInstancedStaticMeshComponent;

/**
 * Rectangular top-down room generator:
 * - Cuts wall openings for entrance/exits, spawns finish markers.
 * - Spawns POIs and monsters with distance rules.
 * - Connects entrance to exits (or a random interior point) via natural roads with spline meshes.
 * - Optionally connects some POIs with roads.
 */
UCLASS(Blueprintable)
class TESTLEVEL_API ALocationRoom : public AActor
{
    GENERATED_BODY()
public:
	ALocationRoom();

	// Entry point to generate everything.
	UFUNCTION(BlueprintCallable, Category = "WorldGen")
	void Generate(const UWorldGenSettings* Settings, AWorldStartMarker* EntranceMarker, FRandomStream RndStream);

protected:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	USceneComponent* Root;

	// Visual walls as segments; could be HISM for efficiency if desired.
	UPROPERTY(VisibleAnywhere) 
	UInstancedStaticMeshComponent* WallISM;

	// Cached for convenience
	UPROPERTY() 
	const UWorldGenSettings* GenSettings = nullptr;
	UPROPERTY() 
	FRandomStream Rng;

	// Geometry helpers
        FVector2f GetHalfSize() const;
        FVector SideOriginWorld(ERoomSide Side) const;
        FVector SideDirection(ERoomSide Side) const;
        FVector SideDirectionWorld(ERoomSide Side) const;
        FVector LocalOut(ERoomSide) const;
	bool IsInsideRoom(const FVector& P) const;
	bool SatisfiesMinDist(const FVector& P, const TArray<FVector>& Points, float MinDist) const;


	// Steps
	// Choose the wall side using marker's forward vector (designer controls the entrance side).
	ERoomSide GuessEntranceSideFromMarker(const class AWorldStartMarker* Marker) const;

	void BuildWallsWithOpenings(const FDoorwaySpec& Entrance);
	void SpawnFinishMarkers(TArray<AWorldFinishMarker*>& OutMarkers);
	void SpawnPOIs(const FVector& EntranceWorld, TArray<AActor*>& OutPOIs);
	void SpawnMonsters(TArray<AActor*>& OutMonsters);
	void SpawnRoads();

	FORCEINLINE FVector GetRoomCenter() const;
	FORCEINLINE FTransform GetRoomTransform() const;
	FORCEINLINE FVector WorldToRoomLocal(const FVector& P) const;
	FORCEINLINE FVector RoomLocalToWorld(const FVector& L) const;

private:
	UPROPERTY(Transient)
	TArray<FDoorwaySpec> Exits;

	UPROPERTY(Transient)
	TArray<FVector> RoadPoint;

	UPROPERTY(Transient)
	TArray<AActor*> POIs;

	UPROPERTY(Transient)
	FVector RoomCenter = FVector::ZeroVector;
};