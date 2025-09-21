// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "WorldGenSettings.h"
#include "WorldStartMarker.h"
#include "WorldFinishMarker.h"
#include "WorldGenSubsystem.generated.h"

class UWorldGenSettings;
class AWorldStartMarker;
class ALocationRoom;

/**
 * 
 */
UCLASS()
class TESTLEVEL_API UWorldGenSubsystem : public UWorldSubsystem
{
    GENERATED_BODY()

public:
	UFUNCTION(BlueprintCallable, Category = "WorldGen")
	ALocationRoom* CreateLocation(const UWorldGenSettings* Settings);

private:
	UPROPERTY()
	FRandomStream RndStream;
};
