// Fill out your copyright notice in the Description page of Project Settings.

#include "Generator/LocationRoom.h"
#include "WorldGenSettings.h"
#include "WorldStartMarker.h"
#include "WorldFinishMarker.h"
#include "RoadSegment.h"
#include "Algo/MaxElement.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Kismet/KismetMathLibrary.h"
#include "Engine/World.h"

// ===== Helpers: coordinate system =====

ALocationRoom::ALocationRoom()
{
	PrimaryActorTick.bCanEverTick = false;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);

	WallISM = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("WallISM"));
	WallISM->SetupAttachment(RootComponent);
}

void ALocationRoom::Generate(const UWorldGenSettings* Settings, FRandomStream InRndStream)
{
	GenSettings = Settings;
	RndStream = InRndStream;
}