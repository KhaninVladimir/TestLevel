// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/EngineTypes.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"
#include "WorldGenTypes.generated.h"

USTRUCT(BlueprintType)
struct FSpawnStruct
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