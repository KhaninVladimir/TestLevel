// Fill out your copyright notice in the Description page of Project Settings.

#include "Generator/WorldGenSubsystem.h"
#include "WorldGenSettings.h"
#include "WorldStartMarker.h"
#include "LocationRoom.h"
#include "EngineUtils.h"
#include "Engine/World.h"
#include "Kismet/GameplayStatics.h"

ALocationRoom* UWorldGenSubsystem::CreateLocation(const UWorldGenSettings* Settings)
{
	if (!Settings) return nullptr;
	UWorld* W = GetWorld();
	if (!W) return nullptr;

	int32 Seed = Settings->Seed == 0 ? FMath::RandRange(0, 65000) : Settings->Seed;
	RndStream.Initialize(Seed);

	if (Settings->Seed == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("Current Seed: %d"), Seed);
	}

	// Find the first start marker in the world (or choose your own criteria)
	AWorldStartMarker* Start = nullptr;
	for (TActorIterator<AWorldStartMarker> It(W); It; ++It)
	{
		Start = *It;
		break;
	}
	if (!Start) 
	{ 
		UE_LOG(LogTemp, Warning, TEXT("WorldGenSubsystem: No AWorldStartMarker found.")); 
		return nullptr; 
	}

	// Spawn the room actor centered around Start marker (you can offset as needed)
	FTransform RoomXform = Start->GetActorTransform();
	ALocationRoom* Room = W->SpawnActorDeferred<ALocationRoom>(ALocationRoom::StaticClass(), RoomXform);
	if (!Room) return nullptr;

	UGameplayStatics::FinishSpawningActor(Room, RoomXform);

	// Generate content
	Room->Generate(Settings, RndStream);
	return Room;
}