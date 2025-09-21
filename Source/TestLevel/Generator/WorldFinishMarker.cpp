// Fill out your copyright notice in the Description page of Project Settings.


#include "Generator/WorldFinishMarker.h"
#include "Components/BillboardComponent.h"

AWorldFinishMarker::AWorldFinishMarker()
{
	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);

#if WITH_EDITORONLY_DATA
	SpriteComp = CreateDefaultSubobject<UBillboardComponent>(TEXT("Billboard"));
	SpriteComp->SetupAttachment(RootComponent);
#endif
}