// Fill out your copyright notice in the Description page of Project Settings.


#include "Generator/WorldStartMarker.h"
#include "Components/BillboardComponent.h"
#include "Components/ArrowComponent.h"

AWorldStartMarker::AWorldStartMarker()
{
	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);

#if WITH_EDITORONLY_DATA
	SpriteComp = CreateDefaultSubobject<UBillboardComponent>(TEXT("Billboard"));
	SpriteComp->SetupAttachment(RootComponent);
#endif

	ArrowComp = CreateDefaultSubobject<UArrowComponent>(TEXT("Arrow"));
	ArrowComp->SetupAttachment(RootComponent);
}