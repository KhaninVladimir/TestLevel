// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/EngineTypes.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"
#include "WorldGenTypes.generated.h"

/** Which side of the rectangle a portal (entrance/exit) belongs to. */
UENUM(BlueprintType)
enum class ERoomSide : uint8
{
	North, // +Y
	South, // -Y
	East,  // +X
	West   // -X
};

/** Simple inclusive integer range. */
USTRUCT(BlueprintType)
struct FIntRangeInclusive
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite) int32 Min = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) int32 Max = 0;

	int32 ClampRand(FRandomStream& Rng) const
	{
		const int32 Lo = FMath::Min(Min, Max);
		const int32 Hi = FMath::Max(Min, Max);
		return Rng.RandRange(Lo, Hi);
	}
};

/** Room size in Unreal units (top-down rectangle). */
USTRUCT(BlueprintType)
struct FRoomRect
{
	GENERATED_BODY()

	// Full extents in world units (width along X, height along Y).
	UPROPERTY(EditAnywhere, BlueprintReadWrite) 
	float Width = 6000.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) 
	float Height = 4000.f;

	FVector2f Half() const 
	{ 
		return FVector2f(Width * 0.5f, Height * 0.5f); 
	}
};

/** A weighted spawn entry for monsters. */
USTRUCT(BlueprintType)
struct FMonsterSpawn
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite) 
	TSubclassOf<AActor> Class;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) 
	int32 MinCount = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) 
	int32 MaxCount = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) 
	float Weight = 1.f;
};

/** POI archetype; you can expand with extra data if needed. */
USTRUCT(BlueprintType)
struct FPOISpawn
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite) 
	TSubclassOf<AActor> Class;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) 
	float Weight = 1.f;
};

/** Static environment mesh spawn entry. */
USTRUCT(BlueprintType)
struct FEnvironmentSpawnEntry
{
	GENERATED_BODY();

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	UStaticMesh* Mesh = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0"))
	int32 MinCount = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0"))
	int32 MaxCount = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float Radius = 200.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float MinSpacing = 150.f;

	// Keep some breathing room around entrance/exits/POIs so they stay reachable.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float ClearanceFromKeyPoints = 400.f;

	// Corridor half-width that must remain unobstructed between entrance and every key point.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float CorridorHalfWidth = 250.f;

	int32 ResolveCount(FRandomStream& Rng) const
	{
		const int32 Lo = FMath::Max(0, FMath::Min(MinCount, MaxCount));
		const int32 Hi = FMath::Max(Lo, FMath::Max(MinCount, MaxCount));
		return (Lo == Hi) ? Lo : Rng.RandRange(Lo, Hi);
	}
};

/** Runtime obstacle description used by road/path planning. */
USTRUCT(BlueprintType)
struct FEnvironmentObstacle
{
	GENERATED_BODY();

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector Location = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float Radius = 0.f;
};

/** One doorway opening spec along a given side. */
USTRUCT(BlueprintType)
struct FDoorwaySpec
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite) 
	ERoomSide Side = ERoomSide::North;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FTransform WorldTransform = FTransform::Identity;

	// Position along the side in range [-Half, +Half] (0 = centered).
	UPROPERTY(EditAnywhere, BlueprintReadWrite) 
	float OffsetAlongSide = 0.f;

	// Half-width of the opening (so full opening is 2 * HalfWidth).
	UPROPERTY(EditAnywhere, BlueprintReadWrite) 
	float HalfWidth = 150.f;
};