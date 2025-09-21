// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "WorldFinishMarker.generated.h"

class UBillboardComponent;

UCLASS(Blueprintable)
class TESTLEVEL_API AWorldFinishMarker : public AActor
{
	GENERATED_BODY()
	
public:
	AWorldFinishMarker();

protected:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	USceneComponent* Root;

#if WITH_EDITORONLY_DATA
	UPROPERTY(VisibleAnywhere) 
	class UBillboardComponent* SpriteComp;
#endif
};
