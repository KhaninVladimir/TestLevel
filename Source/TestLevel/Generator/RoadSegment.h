// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "WorldGenTypes.h"
#include "Components/SplineMeshComponent.h"
#include "WorldGenSettings.h"
#include "RoadSegment.generated.h"

UCLASS()
class TESTLEVEL_API ARoadSegment : public AActor
{
	GENERATED_BODY()
	
public:
	ARoadSegment();

protected:
	UPROPERTY(VisibleAnywhere)
	USceneComponent* Root;

};
