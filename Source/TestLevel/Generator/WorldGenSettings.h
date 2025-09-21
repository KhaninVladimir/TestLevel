// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "WorldGenTypes.h"
#include "Components/SplineMeshComponent.h"
#include "WorldGenSettings.generated.h"

/**
 * Data-driven parameters for room generation.
 * Place an instance in Content and assign in your GameMode or Subsystem.
 */
UCLASS(BlueprintType)
class TESTLEVEL_API UWorldGenSettings : public UDataAsset
{
    GENERATED_BODY()

public:
	
	// Meshes for walls and roads
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Room|Wall")
	UStaticMesh* WallSegmentMesh = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Meshes")
	UStaticMesh* RoadSplineMesh = nullptr;

	// --- POI ---
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "POI")
	float POISpawnChance = 0.35f;

	//Distance from walls to safety spawn
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Monsters")
	FVector2f POIMargin = FVector2f(100.f, 100.f);

	//Distance from exits and entarance
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "POI")
	float POIMinDistanceFromPortals = 500.f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "POI")
	float POIMinDistanceBetween = 800.f;

	//Chanse to make road to POI
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "POI")
	float ProbabilityOfRoadToPOI = 0.6f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "POI")
	TArray<FSpawnStruct> POITable;

	// --- Monsters ---
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Monsters")
	float MonsterSpawnChance = 0.7f;

	//Distance from walls to safety spawn
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Monsters")
	FVector2f MonsterMargin = FVector2f(100.f, 100.f);

	//Distance from exits and entarance
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Monsters")
	float MonsterMinDistanceFromPortals = 700.f;

	//Distance from POI and roads
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Monsters")
	float MonsterMinDistanceFromPOI = 500.f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Monsters")
	TArray<FSpawnStruct> MonsterTable;

	// Randomness
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Random")
	int32 Seed = 1337;
};