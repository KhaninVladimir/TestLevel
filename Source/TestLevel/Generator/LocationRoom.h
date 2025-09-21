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
        void Generate(const UWorldGenSettings* Settings, FRandomStream RndStream, AWorldStartMarker* StartMarker);


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
	FRandomStream RndStream;

};