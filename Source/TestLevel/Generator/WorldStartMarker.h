// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "WorldStartMarker.generated.h"

class UBillboardComponent;
class UArrowComponent;

UCLASS(Blueprintable)
class TESTLEVEL_API AWorldStartMarker : public AActor
{
	GENERATED_BODY()
	
public:
	AWorldStartMarker();

protected:

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	USceneComponent* Root;

#if WITH_EDITORONLY_DATA
	UPROPERTY(VisibleAnywhere) 
	UBillboardComponent* SpriteComp;
#endif
	
	UPROPERTY(VisibleAnywhere) 
	UArrowComponent* ArrowComp;
};