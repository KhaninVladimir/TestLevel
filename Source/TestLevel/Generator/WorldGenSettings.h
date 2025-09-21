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
	// --- Room ---
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Room")
	FRoomRect RoomSize;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Room")
	float WallThickness = 100.f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Room")
	float DoorwayHalfWidth = 150.f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Room|Exits")
	FIntRangeInclusive ExitCountRange{ 0, 3 };

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Room|Exits")
	float MinDistanceFromEntranceIfNoExits = 1500.f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Room|Wall")
	float WallSegmentStep = 300.f;

	// Meshes for walls and roads
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Room|Wall")
	UStaticMesh* WallSegmentMesh = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Meshes")
	UStaticMesh* RoadSplineMesh = nullptr;

	// --- Roads ---
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road")
	int32 RoadMidpointCount = 2;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road")
	float RoadMaxPerpOffset = 600.f;

        UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road")
        float RoadNoiseJitter = 150.f;

        UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road")
        float RoadTangentStrength = 800.f;

        // Adds a gentle baseline curvature to every road (scaled by path length).
        UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road")
        float RoadBaselineCurvature = 0.05f;

        UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road")
        FVector2f RoadMargin = FVector2f(100.f, 100.f);
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road")
	float RoadExitApproachOffset = 100.f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road")
	float RoadExtraClearanceUU = 100.f;

	// --- POI ---
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "POI")
	float POISpawnChance = 0.35f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "POI")
	FIntRangeInclusive POICountRange{ 0, 3 };

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "POI")
	float POIMinDistanceFromPortals = 500.f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "POI")
	float POIMinDistanceBetween = 800.f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "POI")
	float ProbabilityOfRoadToPOI = 0.6f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "POI")
	TArray<FPOISpawn> POITable;

	// --- Monsters ---
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Monsters")
	float MonsterSpawnChance = 0.7f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Monsters")
	FVector2f MonsterMargin = FVector2f(100.f, 100.f);

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Monsters")
	FIntRangeInclusive MonsterPackCountRange{ 1, 3 };

        UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Monsters")
        float MonsterMinDistanceFromPortals = 700.f;

        UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Monsters")
        float MonsterMinDistanceFromPOI = 500.f;

        UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Monsters")
        float MonsterMinDistanceFromRoad = 400.f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Monsters")
	TArray<FMonsterSpawn> MonsterTable;

	// Randomness
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Random")
	int32 Seed = 1337;
};