// Fill out your copyright notice in the Description page of Project Settings.

#include "Generator/LocationRoom.h"
#include "WorldGenSettings.h"
#include "WorldStartMarker.h"
#include "WorldFinishMarker.h"
#include "RoadSegment.h"
#include "Algo/MaxElement.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/PrimitiveComponent.h"
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

FVector2f ALocationRoom::GetHalfSize() const
{
	check(GenSettings);
	return GenSettings->RoomSize.Half();
}

FTransform ALocationRoom::GetRoomTransform() const
{
	return FTransform(GetActorQuat(), GetRoomCenter());
}

FVector ALocationRoom::GetRoomCenter() const
{
	return RoomCenter;
}

FVector ALocationRoom::WorldToRoomLocal(const FVector& P) const
{
	return GetActorQuat().UnrotateVector(P - GetRoomCenter());
}

FVector ALocationRoom::RoomLocalToWorld(const FVector& L) const
{
	return GetRoomCenter() + GetActorQuat().RotateVector(L);
}

FVector ALocationRoom::SideOriginWorld(ERoomSide Side) const
{
        const FVector2f H = GetHalfSize();
        const FVector Center = GetRoomCenter();
        const FQuat RoomQ = GetActorQuat();
        const FVector LocalOffset = LocalOut(Side) * FVector(H.X, H.Y, 0.f);

        return Center + RoomQ.RotateVector(LocalOffset);
}

FVector ALocationRoom::SideDirection(ERoomSide Side) const
{
        if (Side == ERoomSide::North || Side == ERoomSide::South)
        {
                return FVector(1, 0, 0); // along X
        }
        return FVector(0, 1, 0); // along Y
}

FVector ALocationRoom::SideDirectionWorld(ERoomSide Side) const
{
        const FQuat RoomQ = GetActorQuat();
        return RoomQ.RotateVector(SideDirection(Side));
}

FVector ALocationRoom::LocalOut(ERoomSide Side) const
{
	switch (Side)
	{
	case ERoomSide::North: return FVector(0, +1, 0);
	case ERoomSide::South: return FVector(0, -1, 0);
	case ERoomSide::East:  return FVector(+1, 0, 0);
	case ERoomSide::West:  return FVector(-1, 0, 0);
	}
	return FVector::ZeroVector;
}

bool ALocationRoom::IsInsideRoom(const FVector& P) const
{
	const FVector2f H = GetHalfSize();
	const FVector L = WorldToRoomLocal(P); // rotate & shift into room-local
	return FMath::Abs(L.X) <= H.X && FMath::Abs(L.Y) <= H.Y;
}

bool ALocationRoom::SatisfiesMinDist(const FVector& P, const TArray<FVector>& Points, float MinDist) const
{
        for (const FVector& Q : Points)
        {
                if (FVector::Dist2D(P, Q) < MinDist) return false;
        }
        return true;
}

float ALocationRoom::DistanceToRoads(const FVector& P) const
{
        if (RoadNetwork)
        {
                return RoadNetwork->DistanceToRoads(P);
        }

		return FLT_MAX;
}

// ===== Generation entry =====

void ALocationRoom::Generate(const UWorldGenSettings* Settings, AWorldStartMarker* EntranceMarker, FRandomStream RndStream)
{
	check(Settings && EntranceMarker);
	GenSettings = Settings;
	Rng = RndStream;

	// Align room transform with the provided marker so that the actor's transform becomes
	// the authoritative entry transform for the whole location (designers rotate the actor).
	SetActorTransform(EntranceMarker->GetActorTransform());

	// Room transform (rotation-aware math)
	const FTransform EntryTransform = GetActorTransform();
	const FQuat RoomQ = EntryTransform.GetRotation();

	// --- 0) Inputs ---
	EntranceLocation = EntryTransform.GetLocation();
	const FVector EntranceWorld = EntranceLocation;
	const FVector EntryForwardWorld = EntryTransform.GetRotation().GetForwardVector();
	const FVector EntryForwardLocal = RoomQ.UnrotateVector(EntryForwardWorld);

	// If actor's forward points INSIDE the room, flip to get OUTWARD normal direction in local space.
	FVector2D OutwardLocal2D(-EntryForwardLocal.X, -EntryForwardLocal.Y);

	if (OutwardLocal2D.IsNearlyZero())
	{
		// Degenerate case — default to pointing north.
		OutwardLocal2D = FVector2D(0.f, 1.f);
	}

	// --- 1) Pick entrance wall strictly by actor direction (dominant axis in room-local) ---
	ERoomSide EntranceSide;
	if (FMath::Abs(OutwardLocal2D.X) >= FMath::Abs(OutwardLocal2D.Y))
	{
		EntranceSide = (OutwardLocal2D.X >= 0.f) ? ERoomSide::East : ERoomSide::West;
	}
	else
	{
		EntranceSide = (OutwardLocal2D.Y >= 0.f) ? ERoomSide::North : ERoomSide::South;
	}

	// --- 2) Entrance spec aligned with final geometry ---
	FDoorwaySpec Entrance;
	Entrance.Side = EntranceSide;
	Entrance.HalfWidth = GenSettings->DoorwayHalfWidth;
	Entrance.WorldTransform = EntryTransform;

        const FVector OutWS = RoomQ.RotateVector(LocalOut(Entrance.Side)).GetSafeNormal();
        const FVector AlongWS = SideDirectionWorld(Entrance.Side).GetSafeNormal();
	const FVector2f H = GetHalfSize();
	const float HalfSpanToSide = (Entrance.Side == ERoomSide::North || Entrance.Side == ERoomSide::South) ? H.Y : H.X;
	RoomCenter = EntranceWorld - OutWS * HalfSpanToSide - AlongWS * Entrance.OffsetAlongSide;

	// --- 3) Compute along-offset ON THAT SIDE (rotation-aware) ---
        const FVector SideOriginNow = SideOriginWorld(EntranceSide);
        const FVector AlongDir = SideDirectionWorld(EntranceSide);

	const float HalfSpan = (EntranceSide == ERoomSide::North || EntranceSide == ERoomSide::South)
		? GenSettings->RoomSize.Width * 0.5f    // span along local X
		: GenSettings->RoomSize.Height * 0.5f;  // span along local Y

	float EntranceOffsetAlongSide = FVector::DotProduct(EntranceWorld - SideOriginNow, AlongDir);
	EntranceOffsetAlongSide = FMath::Clamp(
		EntranceOffsetAlongSide,
		-HalfSpan + GenSettings->DoorwayHalfWidth,
		+HalfSpan - GenSettings->DoorwayHalfWidth
	);

	Entrance.OffsetAlongSide = EntranceOffsetAlongSide;

	// --- 4) Exits (same logic; rotation-aware later when converting to world) ---
	const int32 ExitCount = GenSettings->ExitCountRange.ClampRand(Rng);
	
	Exits.Empty();
	Exits.Reserve(ExitCount);

	TArray<ERoomSide> Sides = { ERoomSide::North, ERoomSide::South, ERoomSide::East, ERoomSide::West };
	Sides.Remove(Entrance.Side);
	for (int32 i = 0; i < ExitCount; ++i)
	{
		if (Sides.IsEmpty())
		{
			Sides = { ERoomSide::North, ERoomSide::South, ERoomSide::East, ERoomSide::West };
		}
		
		FDoorwaySpec D;
		D.Side = Sides[Rng.RandRange(0, Sides.Num() - 1)];

		Sides.Remove(D.Side);

		const float HalfSpanSide = (D.Side == ERoomSide::North || D.Side == ERoomSide::South)
			? GenSettings->RoomSize.Width * 0.5f
			: GenSettings->RoomSize.Height * 0.5f;

		const int32 TryMax = 20;
		for (int32 t = 0; t < TryMax; ++t)
		{
			const float Off = Rng.FRandRange(-HalfSpanSide * 0.85f, HalfSpanSide * 0.85f);
			bool bOk = true;

			if (D.Side == Entrance.Side &&
				FMath::Abs(Off - Entrance.OffsetAlongSide) < (GenSettings->DoorwayHalfWidth * 3.f))
			{
				bOk = false;
			}

			for (const FDoorwaySpec& E : Exits)
			{
				if (E.Side == D.Side &&
					FMath::Abs(Off - E.OffsetAlongSide) < (GenSettings->DoorwayHalfWidth * 3.f))
				{
					bOk = false; 
					break;
				}
			}
			if (bOk) 
			{ 
				D.OffsetAlongSide = Off; 
				break; 
			}
		}

		D.OffsetAlongSide = FMath::Clamp(
			D.OffsetAlongSide,
			-HalfSpanSide + GenSettings->DoorwayHalfWidth,
			+HalfSpanSide - GenSettings->DoorwayHalfWidth
		);

		D.HalfWidth = GenSettings->DoorwayHalfWidth;

                const FVector DOriginWS = SideOriginWorld(D.Side);
                const FVector DAlongWS = SideDirectionWorld(D.Side).GetSafeNormal();
                const FVector CenterWS = DOriginWS + DAlongWS * D.OffsetAlongSide;

                const FVector DOutWS = RoomQ.RotateVector(LocalOut(D.Side)).GetSafeNormal();
		const FRotator RotWS = UKismetMathLibrary::MakeRotFromXZ(DOutWS, FVector::UpVector);
		
		D.WorldTransform = FTransform(RotWS, CenterWS);
		Exits.Add(D);
	}

	// --- 5) Build walls (rotation-aware inside the function) ---
	BuildWallsWithOpenings(Entrance);

	// --- 6) Exit markers ---
	TArray<AWorldFinishMarker*> FinishMarkers;
	FinishMarkers.Empty();
	FinishMarkers.Reserve(Exits.Num());
	SpawnFinishMarkers(FinishMarkers);

	// --- 7) Targets (rotation-aware world positions) ---
	RoadPoint.Empty();
	RoadPoint.Add(EntranceWorld);

	for (const FDoorwaySpec& D : Exits)
	{
		RoadPoint.Add(D.WorldTransform.GetLocation());
	}

	// --- 8) POIs / Monsters ---
	POIs.Empty();
	POIs.Reserve(GenSettings->POICountRange.Max); // верхняя граница — разумная эвристика
	SpawnPOIs(EntranceLocation, POIs);

	SpawnEnvironment();
	SpawnRoads();

	TArray<AActor*> Monsters;
	Monsters.Reserve(GenSettings->MonsterPackCountRange.Max * 4); // грубая эвристика
	SpawnMonsters(Monsters);
}


// ===== Walls & openings =====

void ALocationRoom::BuildWallsWithOpenings(const FDoorwaySpec& Entrance)
{
	check(GenSettings);
	WallISM->ClearInstances();
	WallISM->SetStaticMesh(GenSettings->WallSegmentMesh);

	const float Step = GenSettings->WallSegmentStep;
	const float HalfStep = Step * 0.5f;

	// For each side, we iterate along the span and place segments except where openings are.
	auto PlaceSide = [&](ERoomSide Side)
		{
			const FVector2f H = GetHalfSize();
			const bool bNS = (Side == ERoomSide::North || Side == ERoomSide::South);
			const float Span = bNS ? GenSettings->RoomSize.Width : GenSettings->RoomSize.Height;
			const int32 Segments = FMath::CeilToInt(Span / Step);

			// Gather openings for this side once
			TArray<FDoorwaySpec> Openings;
			Openings.Reserve(1 + Exits.Num());
			if (Entrance.Side == Side) 
				Openings.Add(Entrance);
			for (const FDoorwaySpec& E : Exits)
			{
				if (E.Side == Side)
					Openings.Add(E);
			}

                        const FVector SideOrigin = SideOriginWorld(Side);
                        const FVector Along = SideDirectionWorld(Side).GetSafeNormal();

                        // Rotation for all segments on this side is constant
                        const FQuat SegmentRot = Along.Rotation().Quaternion();

			// Fast path: no openings → add all segments
			if (Openings.Num() == 0)
			{
				for (int32 i = 0; i < Segments; ++i)
				{
					const float CenterOff = -Span * 0.5f + (i + 0.5f) * Step;

                                        FTransform T;
                                        T.SetLocation(SideOrigin + Along * CenterOff);
                                        T.SetRotation(SegmentRot);
                                        T.SetScale3D(FVector(1, 1, 1));
                                        WallISM->AddInstance(T, true);
				}
				return;
			}

			for (int32 i = 0; i < Segments; ++i)
			{
				// Segment center offset along the side
				const float CenterOff = -Span * 0.5f + (i + 0.5f) * Step;

				// Skip if overlapping an opening
				bool bCovered = false;
				for (const FDoorwaySpec& O : Openings)
				{
					if (FMath::Abs(CenterOff - O.OffsetAlongSide) < (O.HalfWidth + HalfStep))
					{
						bCovered = true; break;
					}
				}
				if (bCovered) continue;

                                FTransform T;
                                T.SetLocation(SideOrigin + Along * CenterOff);
                                T.SetRotation(SegmentRot);
                                T.SetScale3D(FVector(1, 1, 1));
                                WallISM->AddInstance(T, true);
			}
		};

	PlaceSide(ERoomSide::North);
	PlaceSide(ERoomSide::South);
	PlaceSide(ERoomSide::East);
	PlaceSide(ERoomSide::West);
}

ERoomSide ALocationRoom::GuessEntranceSideFromMarker(const AWorldStartMarker* Marker) const
{
	const FVector Fwd3D = Marker ? Marker->GetActorForwardVector() : FVector::ForwardVector;
	const FVector2D F = FVector2D(Fwd3D.X, Fwd3D.Y); // top-down
	if (FMath::Abs(F.X) >= FMath::Abs(F.Y))
	{
		return (F.X >= 0.f) ? ERoomSide::East : ERoomSide::West;   // X dominates
	}
	else
	{
		return (F.Y >= 0.f) ? ERoomSide::North : ERoomSide::South; // Y dominates
	}
}

void ALocationRoom::SpawnFinishMarkers(TArray<AWorldFinishMarker*>& OutMarkers)
{
	UWorld* W = GetWorld();
	if (!W) return;

	for (const FDoorwaySpec& D : Exits)
	{
                const FVector Origin = SideOriginWorld(D.Side);
                const FVector Along = SideDirectionWorld(D.Side);
		const FVector P = Origin + Along * D.OffsetAlongSide;

		if (AWorldFinishMarker* Marker = W->SpawnActor<AWorldFinishMarker>(AWorldFinishMarker::StaticClass(), D.WorldTransform))
		{
			OutMarkers.Add(Marker);
		}
	}
}

// ===== Targets / POIs / Monsters =====
void ALocationRoom::SpawnPOIs(const FVector& EntranceWorld, TArray<AActor*>& OutPOIs)
{
	UWorld* W = GetWorld();
	if (!W || !GenSettings) return;

	if (Rng.FRand() > GenSettings->POISpawnChance) 
		return;

	const int32 Count = GenSettings->POICountRange.ClampRand(Rng);
	if (Count <= 0 || GenSettings->POITable.Num() == 0) 
		return;

	// Build weights & precompute index of max weight (exactly the same selection policy as before)
	TArray<float> Weights;
	Weights.Reserve(GenSettings->POITable.Num());
	for (const FPOISpawn& E : GenSettings->POITable) 
		Weights.Add(FMath::Max(0.f, E.Weight));

	int32 MaxIdx = 0;
	if (!Weights.IsEmpty())
	{
		const float* MaxPtr = Algo::MaxElement(Weights);
		MaxIdx = MaxPtr ? Weights.IndexOfByKey(*MaxPtr) : 0;
	}

	const FVector2f H = GetHalfSize();
	TArray<FVector> Placed;
	Placed.Reserve(Count);

	auto RandPOIClass = [&]() -> TSubclassOf<AActor>
		{
			// previous logic: pick the element with the maximum weight (not roulette)
			return GenSettings->POITable[MaxIdx].Class;
		};


	const int32 TriesPer = 24;
	for (int32 i = 0; i < Count; ++i)
	{
		for (int32 t = 0; t < TriesPer; ++t)
		{
			const FVector L(
				Rng.FRandRange(-H.X * 0.75f, +H.X * 0.75f),
				Rng.FRandRange(-H.Y * 0.75f, +H.Y * 0.75f),
				0.f);
			const FVector P = RoomLocalToWorld(L);

			if (!IsInsideRoom(P)) 
				continue;
			if (FVector::Dist2D(P, EntranceWorld) < GenSettings->POIMinDistanceFromPortals) 
				continue;
			if (!SatisfiesMinDist(P, Placed, GenSettings->POIMinDistanceBetween)) 
				continue;

			if (TSubclassOf<AActor> C = RandPOIClass())
			{
				if (AActor* A = W->SpawnActor<AActor>(C, P, FRotator::ZeroRotator))
				{
					Placed.Add(P);
					OutPOIs.Add(A);
					
					if (Rng.FRand() <= GenSettings->ProbabilityOfRoadToPOI)
					{
						RoadPoint.Add(P);
					}
						

					break;
				}
			}
		}
	}
}

void ALocationRoom::SpawnEnvironment()
{
	if (!GenSettings)
	{
		return;
	}

	for (UInstancedStaticMeshComponent* Comp : EnvironmentComponents)
	{
		if (Comp)
		{
			Comp->DestroyComponent();
		}
	}
	EnvironmentComponents.Reset();
	EnvironmentObstacles.Reset();

	const TArray<FEnvironmentSpawnEntry>& Entries = GenSettings->EnvironmentMeshes;
	if (Entries.Num() == 0)
	{
		return;
	}

	const FVector2f H = GetHalfSize();

	TArray<FVector> CorridorTargets;
	CorridorTargets.Reserve(1 + Exits.Num() + POIs.Num());
	FVector EntranceRef = EntranceLocation;
	if (EntranceRef.IsNearlyZero())
	{
		EntranceRef = GetActorLocation();
	}
	CorridorTargets.Add(EntranceRef);
	for (const FDoorwaySpec& Exit : Exits)
	{
		CorridorTargets.AddUnique(Exit.WorldTransform.GetLocation());
	}
	for (AActor* POI : POIs)
	{
		if (POI)
		{
			CorridorTargets.AddUnique(POI->GetActorLocation());
		}
	}

	for (const FEnvironmentSpawnEntry& Entry : Entries)
	{
		if (!Entry.Mesh)
		{
			continue;
		}

		const int32 DesiredCount = Entry.ResolveCount(Rng);
		if (DesiredCount <= 0)
		{
			continue;
		}

		UInstancedStaticMeshComponent* Comp = NewObject<UInstancedStaticMeshComponent>(this);
		if (!Comp)
		{
			continue;
		}

		Comp->SetMobility(EComponentMobility::Static);
		Comp->SetFlags(RF_Transactional);
		Comp->SetupAttachment(RootComponent);
		Comp->RegisterComponent();
		Comp->SetStaticMesh(Entry.Mesh);
		Comp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		EnvironmentComponents.Add(Comp);

		const float MarginX = FMath::Max(Entry.Radius, GenSettings->RoadMargin.X);
		const float MarginY = FMath::Max(Entry.Radius, GenSettings->RoadMargin.Y);
		if (MarginX >= H.X || MarginY >= H.Y)
		{
			continue;
		}

		int32 Placed = 0;
		int32 Attempts = 0;
		const int32 MaxAttempts = 48 * FMath::Max(1, DesiredCount);

		while (Placed < DesiredCount && Attempts < MaxAttempts)
		{
			++Attempts;

			const FVector CandidateLocal(
				Rng.FRandRange(-H.X + MarginX, H.X - MarginX),
				Rng.FRandRange(-H.Y + MarginY, H.Y - MarginY),
				0.f);
			const FVector Candidate = RoomLocalToWorld(CandidateLocal);

			if (!CanPlaceEnvironmentAt(Candidate, Entry, CorridorTargets))
			{
				continue;
			}

			FTransform InstanceXf;
			InstanceXf.SetLocation(Candidate);
			InstanceXf.SetRotation(FQuat(FVector::UpVector, FMath::DegreesToRadians(Rng.FRandRange(0.f, 360.f))));
			InstanceXf.SetScale3D(FVector(1.f));
			Comp->AddInstance(InstanceXf);

			FEnvironmentObstacle Obstacle;
			Obstacle.Location = Candidate;
			Obstacle.Radius = Entry.Radius;
			EnvironmentObstacles.Add(Obstacle);

			++Placed;
		}
	}
}

void ALocationRoom::SpawnMonsters(TArray<AActor*>& OutMonsters)
{
	UWorld* W = GetWorld();
	if (!W || !GenSettings) return;
	if (Rng.FRand() > GenSettings->MonsterSpawnChance) return;
	if (GenSettings->MonsterTable.Num() == 0) return;

	// --- Room geometry (actor-local half extents; works for any rotation in world) ---
	const FVector2f H = GetHalfSize();              // local half-size
	const FTransform RoomXf = GetRoomTransform();  // full transform (yaw/roll/pitch safe)

	// Collect POI world positions
	TArray<FVector> POIPoints;
	POIPoints.Reserve(POIs.Num());
	for (AActor* A : POIs) 
		if (A) POIPoints.Add(A->GetActorLocation());

	auto PickMonsterClass = [&]() -> TSubclassOf<AActor>
		{
			float Total = 0.f;
			for (const auto& E : GenSettings->MonsterTable) 
				Total += FMath::Max(0.f, E.Weight);
			float R = Rng.FRandRange(0.f, FMath::Max(0.001f, Total));
			for (const auto& E : GenSettings->MonsterTable)
			{
				const float Wt = FMath::Max(0.f, E.Weight);
				if ((R -= Wt) <= 0.f) 
					return E.Class;
			}
			return GenSettings->MonsterTable.Last().Class;
		};

	// --- Decide packs and total count (your original policy) ---
	const int32 Packs = GenSettings->MonsterPackCountRange.ClampRand(Rng);
	struct FPack { TSubclassOf<AActor> Class; int32 Count = 0; };
	TArray<FPack> PackList; PackList.Reserve(Packs);

	int32 TotalNeed = 0;
	for (int32 p = 0; p < Packs; ++p)
	{
		FPack Pk;
		Pk.Class = PickMonsterClass();
		Pk.Count = FMath::Clamp(
			Rng.RandRange(GenSettings->MonsterTable[0].MinCount, GenSettings->MonsterTable[0].MaxCount),
			1, 25);
		TotalNeed += Pk.Count;
		PackList.Add(Pk);
	}
	if (TotalNeed <= 0) 
		return;

	// Clamp margins to not exceed half-size
	const float SafeMarginX = FMath::Clamp(GenSettings->MonsterMargin.X, 0.f, H.X - 1.f);
	const float SafeMarginY = FMath::Clamp(GenSettings->MonsterMargin.Y, 0.f, H.Y - 1.f);

	// --- Jittered grid over the FULL local rectangle (uses all area) ---
	const float Width = 2.f * (H.X - SafeMarginX);
	const float Height = 2.f * (H.Y - SafeMarginY);
	const float Aspect = (Height > KINDA_SMALL_NUMBER) ? (Width / Height) : 1.f;

	// Number of cells chosen to roughly match total needed samples
	int32 CellsY = FMath::CeilToInt(FMath::Sqrt(TotalNeed / FMath::Max(0.001f, Aspect)));
	int32 CellsX = FMath::CeilToInt(static_cast<float>(TotalNeed) / FMath::Max(1, CellsY));
	CellsX = FMath::Max(1, CellsX);
	CellsY = FMath::Max(1, CellsY);

	const float CellW = Width / CellsX;
	const float CellH = Height / CellsY;

	// Jitter inside cell (keeps samples away from borders a bit)
	const float Jx = CellW * 0.35f;
        const float Jy = CellH * 0.35f;

        // --- Generate candidate points in world, covering the WHOLE room ---
        TArray<FVector> Chosen; Chosen.Reserve(TotalNeed);
        const float MinRoadDistance = FMath::Max(0.f, GenSettings->MonsterMinDistanceFromRoad);

        const int32 MaxPasses = 3;
        for (int32 pass = 0; pass < MaxPasses && Chosen.Num() < TotalNeed; ++pass)
        {
                for (int32 iy = 0; iy < CellsY && Chosen.Num() < TotalNeed; ++iy)
		{
			for (int32 ix = 0; ix < CellsX && Chosen.Num() < TotalNeed; ++ix)
			{
				// Cell center in actor-local, spanning the entire rectangle
				const float Cx = -H.X + SafeMarginX + (ix + 0.5f) * CellW;
				const float Cy = -H.Y + SafeMarginY + (iy + 0.5f) * CellH;

				// Jittered local sample
				const float Lx = FMath::Clamp(Cx + Rng.FRandRange(-Jx, +Jx), -H.X + SafeMarginX, H.X - SafeMarginX);
				const float Ly = FMath::Clamp(Cy + Rng.FRandRange(-Jy, +Jy), -H.Y + SafeMarginY, H.Y - SafeMarginY);

                                const FVector P = RoomLocalToWorld(FVector(Lx, Ly, 0.f));
                                if (!IsInsideRoom(P)) continue;
                                if (!SatisfiesMinDist(P, POIPoints, GenSettings->MonsterMinDistanceFromPOI)) continue;
                                if (MinRoadDistance > 0.f && DistanceToRoads(P) < MinRoadDistance) continue;

                                Chosen.Add(P);
                        }
                }
        }

	// Fallback: relaxed sampling if constraints are tight
        const int32 TriesPer = 64;
        const int32 MaxTries = TriesPer * FMath::Max(1, TotalNeed);
        int32 Attempt = 0;
        while (Chosen.Num() < TotalNeed && Attempt < MaxTries)
        {
                ++Attempt;
                const float Lx = Rng.FRandRange(-H.X + SafeMarginX, H.X - SafeMarginX);
                const float Ly = Rng.FRandRange(-H.Y + SafeMarginY, H.Y - SafeMarginY);
                const FVector P = RoomLocalToWorld(FVector(Lx, Ly, 0.f));

                if (!IsInsideRoom(P)) continue;
                if (!SatisfiesMinDist(P, POIPoints, GenSettings->MonsterMinDistanceFromPOI)) continue;
                if (MinRoadDistance > 0.f && DistanceToRoads(P) < MinRoadDistance) continue;

                Chosen.Add(P);
        }

	if (Chosen.IsEmpty()) 
		return;

	// Shuffle a bit so packs distribute across the whole room
	for (int32 i = 0; i < Chosen.Num(); ++i)
		Chosen.Swap(i, Rng.RandRange(0, Chosen.Num() - 1));

	// --- Spawn by packs over the distributed positions ---
	int32 idx = 0;
	for (const FPack& Pk : PackList)
	{
		if (!Pk.Class) 
			continue;
		for (int32 i = 0; i < Pk.Count && idx < Chosen.Num(); ++i, ++idx)
		{
			if (AActor* M = W->SpawnActor<AActor>(Pk.Class, Chosen[idx], FRotator::ZeroRotator))
			{
				OutMonsters.Add(M);
			}
		}
	}
}

void ALocationRoom::SpawnRoads()
{
        UWorld* W = GetWorld();
        if (!W || !GenSettings)
        {
                RoadNetwork = nullptr;
                return;
        }

        if (RoadNetwork && !RoadNetwork->IsPendingKillPending())
        {
                RoadNetwork->Destroy();
        }
        RoadNetwork = nullptr;

        if (GenSettings->RoadSplineMesh && RoadPoint.Num() > 1)
        {
                if (ARoadSegment* RoadNet = W->SpawnActor<ARoadSegment>(ARoadSegment::StaticClass(), GetActorTransform()))
                {
                        RoadNetwork = RoadNet;
                        RoadNet->BuildNetwork(RoadPoint, Exits.Num(), GetHalfSize(), GenSettings, Rng, EnvironmentObstacles);
                }
        }
}

bool ALocationRoom::CanPlaceEnvironmentAt(const FVector& Candidate, const FEnvironmentSpawnEntry& Entry, const TArray<FVector>& CorridorTargets) const
{
	const FVector2f H = GetHalfSize();
	const FVector Local = WorldToRoomLocal(Candidate);
	const float Radius = FMath::Max(0.f, Entry.Radius);

	if (FMath::Abs(Local.X) > H.X - Radius)
	{
		return false;
	}
	if (FMath::Abs(Local.Y) > H.Y - Radius)
	{
		return false;
	}

	const FVector2D Candidate2D(Candidate.X, Candidate.Y);

	for (const FEnvironmentObstacle& Ob : EnvironmentObstacles)
	{
		const FVector2D Ob2(Ob.Location.X, Ob.Location.Y);
		const float Dist = FVector2D::Distance(Candidate2D, Ob2);
		if (Dist < (Radius + Ob.Radius + Entry.MinSpacing))
		{
			return false;
		}
	}

	const float Clearance = FMath::Max(0.f, Entry.ClearanceFromKeyPoints);
	if (Clearance > 0.f)
	{
		for (const FVector& Key : CorridorTargets)
		{
			const FVector2D Key2(Key.X, Key.Y);
			const float Dist = FVector2D::Distance(Candidate2D, Key2);
			if (Dist < Radius + Clearance)
			{
				return false;
			}
		}
	}

	return KeepsCorridorsOpen(Candidate2D, Radius, Entry, CorridorTargets);
}

bool ALocationRoom::KeepsCorridorsOpen(const FVector2D& Candidate, float Radius, const FEnvironmentSpawnEntry& Entry, const TArray<FVector>& CorridorTargets) const
{
	const float CorridorHalf = FMath::Max(0.f, Entry.CorridorHalfWidth);
	if (CorridorHalf <= 0.f || CorridorTargets.Num() <= 1)
	{
		return true;
	}

	const FVector2D Entrance2D(CorridorTargets[0].X, CorridorTargets[0].Y);

	for (int32 idx = 1; idx < CorridorTargets.Num(); ++idx)
	{
		const FVector2D Target2D(CorridorTargets[idx].X, CorridorTargets[idx].Y);
		if (FVector2D::Distance(Entrance2D, Target2D) <= KINDA_SMALL_NUMBER)
		{
			continue;
		}

		const float DistToSegment = DistancePointToSegment2D(Candidate, Entrance2D, Target2D);
		if (DistToSegment < Radius + CorridorHalf)
		{
			return false;
		}
	}

	return true;
}

float ALocationRoom::DistancePointToSegment2D(const FVector2D& P, const FVector2D& A, const FVector2D& B)
{
	const FVector2D AB = B - A;
	const float LenSq = AB.SizeSquared();
	if (LenSq <= KINDA_SMALL_NUMBER)
	{
		return FVector2D::Distance(P, A);
	}

	const float T = FMath::Clamp(FVector2D::DotProduct(P - A, AB) / LenSq, 0.f, 1.f);
	const FVector2D Closest = A + AB * T;
	return FVector2D::Distance(P, Closest);
}
