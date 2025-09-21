#include "Generator/RoadSegment.h"
#include "Components/SplineComponent.h"
#include "Components/SplineMeshComponent.h"

namespace
{
	constexpr int32 MaxObstacleRelaxIterations = 4;
}

ARoadSegment::ARoadSegment()
{
	PrimaryActorTick.bCanEverTick = false;
	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);
}

float ARoadSegment::DistanceToRoads(const FVector& Point) const
{
	if (CachedPolylines.IsEmpty())
	{
		return FLT_MAX;
	}

	const FVector2D Target(Point.X, Point.Y);
	float BestDistance = FLT_MAX;

	for (const TArray<FVector>& Polyline : CachedPolylines)
	{
		if (Polyline.Num() < 2)
		{
			continue;
		}

		for (int32 i = 0; i < Polyline.Num() - 1; ++i)
		{
			const FVector2D A(Polyline[i].X, Polyline[i].Y);
			const FVector2D B(Polyline[i + 1].X, Polyline[i + 1].Y);
			const float Distance = DistancePointToSegment2D(Target, A, B);
			BestDistance = FMath::Min(BestDistance, Distance);
		}
	}

	return BestDistance;
}

void ARoadSegment::ClearNetwork()
{
	for (USplineMeshComponent* Mesh : RoadMeshes)
	{
		if (Mesh)
		{
			Mesh->DestroyComponent();
		}
	}
	RoadMeshes.Reset();

	for (USplineComponent* Spline : RoadSplines)
	{
		if (Spline)
		{
			Spline->DestroyComponent();
		}
	}
	RoadSplines.Reset();

	CachedPolylines.Reset();
}

void ARoadSegment::BuildNetwork(const TArray<FVector>& NodesWS, int32 ExitCount, const FVector2f& RoomHalfSize, const UWorldGenSettings* Settings, FRandomStream& Rng, const TArray<FEnvironmentObstacle>& Obstacles)
{
	ClearNetwork();

	if (!Settings || NodesWS.Num() < 2)
	{
		return;
	}

	TArray<FVector> AnchorPoints;
	AnchorPoints.Reserve(NodesWS.Num() * FMath::Max(2, Settings->RoadMidpointCount + 2));
	AnchorPoints.Add(NodesWS[0]);

	const int32 NodeCount = NodesWS.Num();
	const int32 ActualExitCount = FMath::Clamp(ExitCount, 0, NodeCount - 1);

	TArray<int32> ExitIndices;
	ExitIndices.Reserve(ActualExitCount);
	for (int32 i = 0; i < ActualExitCount; ++i)
	{
		ExitIndices.Add(i + 1);
	}

	auto AppendRoad = [&](const TArray<FVector>& Polyline)
	{
		if (Polyline.Num() < 2)
		{
			return;
		}

		CachedPolylines.Add(Polyline);
		for (const FVector& P : Polyline)
		{
			AnchorPoints.Add(P);
		}
		RegisterRoadSpline(Polyline, Settings);
	};

	// Step 1: main road from entrance to the farthest exit (or another target if no exits).
	const FVector Entrance = NodesWS[0];
	int32 PrimaryTargetIndex = INDEX_NONE;

	if (!ExitIndices.IsEmpty())
	{
		float BestDistSq = -1.f;
		for (int32 NodeIdx : ExitIndices)
		{
			const float DistSq = FVector::DistSquared2D(Entrance, NodesWS[NodeIdx]);
			if (DistSq > BestDistSq)
			{
				BestDistSq = DistSq;
				PrimaryTargetIndex = NodeIdx;
			}
		}
	}
	else
	{
		float BestDistSq = -1.f;
		for (int32 idx = 1; idx < NodeCount; ++idx)
		{
			const float DistSq = FVector::DistSquared2D(Entrance, NodesWS[idx]);
			if (DistSq > BestDistSq)
			{
				BestDistSq = DistSq;
				PrimaryTargetIndex = idx;
			}
		}
	}

	if (PrimaryTargetIndex != INDEX_NONE)
	{
		const FVector& Target = NodesWS[PrimaryTargetIndex];
		if (FVector::DistSquared(Entrance, Target) > KINDA_SMALL_NUMBER)
		{
			AppendRoad(MakeNaturalPath(Entrance, Target, RoomHalfSize, Settings, Rng, Obstacles));
		}
	}

	// Step 2: branch remaining exits to their closest point on the first road.
	if (!ExitIndices.IsEmpty() && !CachedPolylines.IsEmpty())
	{
		const TArray<FVector>& PrimaryRoad = CachedPolylines[0];
		for (int32 NodeIdx : ExitIndices)
		{
			if (NodeIdx == PrimaryTargetIndex)
			{
				continue;
			}

			const FVector& ExitPos = NodesWS[NodeIdx];
			FVector AttachPoint = PrimaryRoad[0];
			float BestDistSq = FLT_MAX;

			for (int32 i = 0; i < PrimaryRoad.Num() - 1; ++i)
			{
				const FVector Candidate = ClosestPointOnSegment2D(ExitPos, PrimaryRoad[i], PrimaryRoad[i + 1]);
				const float DistSq = FVector::DistSquared2D(ExitPos, Candidate);
				if (DistSq < BestDistSq)
				{
					BestDistSq = DistSq;
					AttachPoint = Candidate;
				}
			}

			if (FVector::DistSquared(AttachPoint, ExitPos) > KINDA_SMALL_NUMBER)
			{
				AppendRoad(MakeNaturalPath(AttachPoint, ExitPos, RoomHalfSize, Settings, Rng, Obstacles));
			}
		}
	}

	// Step 3: connect POIs to their nearest available anchor point.
	const int32 PoiStart = 1 + ActualExitCount;
	if (PoiStart < NodeCount)
	{
		for (int32 idx = PoiStart; idx < NodeCount; ++idx)
		{
			const FVector& Poi = NodesWS[idx];
			FVector BestAnchor = AnchorPoints.IsEmpty() ? Entrance : AnchorPoints[0];
			float BestDistSq = FLT_MAX;

			for (const FVector& Anchor : AnchorPoints)
			{
				const float DistSq = FVector::DistSquared2D(Poi, Anchor);
				if (DistSq < BestDistSq)
				{
					BestDistSq = DistSq;
					BestAnchor = Anchor;
				}
			}

			if (FVector::DistSquared(BestAnchor, Poi) > KINDA_SMALL_NUMBER)
			{
				AppendRoad(MakeNaturalPath(BestAnchor, Poi, RoomHalfSize, Settings, Rng, Obstacles));
			}
		}
	}
}

void ARoadSegment::RegisterRoadSpline(const TArray<FVector>& Polyline, const UWorldGenSettings* Settings)
{
	if (Polyline.Num() < 2)
	{
		return;
	}

	USplineComponent* Spline = NewObject<USplineComponent>(this, USplineComponent::StaticClass());
	if (!Spline)
	{
		return;
	}

	Spline->SetMobility(EComponentMobility::Movable);
	Spline->AttachToComponent(Root, FAttachmentTransformRules::KeepRelativeTransform);
	Spline->RegisterComponent();
	Spline->ClearSplinePoints(false);
	Spline->SetClosedLoop(false, false);

	for (int32 i = 0; i < Polyline.Num(); ++i)
	{
		Spline->AddSplinePoint(Polyline[i], ESplineCoordinateSpace::World, false);
		Spline->SetSplinePointType(i, ESplinePointType::Curve);
	}

	if (Settings)
	{
		const float TangentStrength = Settings->RoadTangentStrength;
		for (int32 i = 0; i < Polyline.Num(); ++i)
		{
			const FVector Prev = (i > 0) ? Polyline[i - 1] : Polyline[i];
			const FVector Next = (i + 1 < Polyline.Num()) ? Polyline[i + 1] : Polyline[i];
			FVector TangentDir = (Next - Prev);
			if (!TangentDir.IsNearlyZero())
			{
				TangentDir = TangentDir.GetSafeNormal() * TangentStrength;
				Spline->SetTangentAtSplinePoint(i, TangentDir, ESplineCoordinateSpace::World);
			}
		}
	}

	Spline->UpdateSpline();
	RoadSplines.Add(Spline);

	if (!(Settings && Settings->RoadSplineMesh))
	{
		return;
	}

	for (int32 i = 0; i < Spline->GetNumberOfSplinePoints() - 1; ++i)
	{
		USplineMeshComponent* Mesh = NewObject<USplineMeshComponent>(this, USplineMeshComponent::StaticClass());
		if (!Mesh)
		{
			continue;
		}

		Mesh->SetMobility(EComponentMobility::Movable);
		Mesh->SetStaticMesh(Settings->RoadSplineMesh);
		Mesh->AttachToComponent(Spline, FAttachmentTransformRules::KeepRelativeTransform);
		Mesh->RegisterComponent();

		FVector StartPos, StartTangent;
		Spline->GetLocationAndTangentAtSplinePoint(i, StartPos, StartTangent, ESplineCoordinateSpace::Local);
		FVector EndPos, EndTangent;
		Spline->GetLocationAndTangentAtSplinePoint(i + 1, EndPos, EndTangent, ESplineCoordinateSpace::Local);
		Mesh->SetStartAndEnd(StartPos, StartTangent, EndPos, EndTangent, true);
		RoadMeshes.Add(Mesh);
	}
}

TArray<FVector> ARoadSegment::MakeNaturalPath(const FVector& Start, const FVector& End, const FVector2f& RoomHalfSize, const UWorldGenSettings* Settings, FRandomStream& Rng, const TArray<FEnvironmentObstacle>& Obstacles) const
{
	TArray<FVector> Result;
	Result.Reserve(FMath::Max(2, Settings ? Settings->RoadMidpointCount + 2 : 2));
	Result.Add(Start);

	if (FVector::DistSquared(Start, End) <= KINDA_SMALL_NUMBER)
	{
		Result.Add(End);
		return Result;
	}

	const int32 MidCount = Settings ? FMath::Max(0, Settings->RoadMidpointCount) : 0;
	const float NoiseAmplitude = Settings ? Settings->RoadNoiseJitter : 0.f;
	const float BaselineCurvature = Settings ? Settings->RoadBaselineCurvature : 0.f;
	const float MaxPerpOffset = Settings ? Settings->RoadMaxPerpOffset : 0.f;
	const float Clearance = Settings ? Settings->RoadExtraClearanceUU : 0.f;

	const FVector Delta = End - Start;
	const float PathLength = FVector::Dist2D(Start, End);
	FVector2D Right2D(0.f, 1.f);
	const FVector2D Dir2D(Delta.X, Delta.Y);
	if (!Dir2D.IsNearlyZero())
	{
		Right2D = Dir2D.GetRotated(90.f).GetSafeNormal();
	}

	for (int32 i = 0; i < MidCount; ++i)
	{
		const float T = (i + 1.f) / (MidCount + 1.f);
		FVector Candidate = FMath::Lerp(Start, End, T);
		Candidate.Z = FMath::Lerp(Start.Z, End.Z, T);

		if (Settings)
		{
			const float CurvatureOffset = BaselineCurvature * PathLength * FMath::Sin(PI * T);
			const float RandomPerp = Rng.FRandRange(-MaxPerpOffset, MaxPerpOffset);
			const float JitterX = Rng.FRandRange(-NoiseAmplitude, NoiseAmplitude);
			const float JitterY = Rng.FRandRange(-NoiseAmplitude, NoiseAmplitude);

			Candidate.X += Right2D.X * (CurvatureOffset + RandomPerp) + JitterX;
			Candidate.Y += Right2D.Y * (CurvatureOffset + RandomPerp) + JitterY;
		}

		Candidate = ClampToRoomBounds(Candidate, RoomHalfSize, Settings);
		if (Settings && Clearance > 0.f && !Obstacles.IsEmpty())
		{
			Candidate = PushOutOfObstacles(Candidate, Obstacles, Clearance, Rng);
			Candidate = ClampToRoomBounds(Candidate, RoomHalfSize, Settings);
		}

		Result.Add(Candidate);
	}

	Result.Add(End);
	return Result;
}

FVector ARoadSegment::ClampToRoomBounds(const FVector& Point, const FVector2f& RoomHalfSize, const UWorldGenSettings* Settings) const
{
	if (!Settings)
	{
		return Point;
	}

	const FVector2f Margin = Settings->RoadMargin;
	const FTransform RoomTransform = GetActorTransform();
	FVector Local = RoomTransform.InverseTransformPosition(Point);

	const float MinX = -RoomHalfSize.X + Margin.X;
	const float MaxX = RoomHalfSize.X - Margin.X;
	const float MinY = -RoomHalfSize.Y + Margin.Y;
	const float MaxY = RoomHalfSize.Y - Margin.Y;

	Local.X = FMath::Clamp(Local.X, FMath::Min(MinX, MaxX), FMath::Max(MinX, MaxX));
	Local.Y = FMath::Clamp(Local.Y, FMath::Min(MinY, MaxY), FMath::Max(MinY, MaxY));

	return RoomTransform.TransformPosition(Local);
}

FVector ARoadSegment::PushOutOfObstacles(const FVector& Point, const TArray<FEnvironmentObstacle>& Obstacles, float Clearance, FRandomStream& Rng) const
{
	FVector Result = Point;
	if (Clearance <= 0.f || Obstacles.IsEmpty())
	{
		return Result;
	}

	for (int32 Iter = 0; Iter < MaxObstacleRelaxIterations; ++Iter)
	{
		bool bAdjusted = false;
		for (const FEnvironmentObstacle& Obstacle : Obstacles)
		{
			const FVector2D Center(Obstacle.Location.X, Obstacle.Location.Y);
			FVector2D ToPoint(Result.X - Center.X, Result.Y - Center.Y);
			const float Distance = ToPoint.Size();
			const float Required = Obstacle.Radius + Clearance;

			if (Required <= 0.f)
			{
				continue;
			}

			if (Distance < Required)
			{
				FVector2D Direction;
				if (Distance < KINDA_SMALL_NUMBER)
				{
					const float Angle = Rng.FRandRange(0.f, 2.f * PI);
					Direction = FVector2D(FMath::Cos(Angle), FMath::Sin(Angle));
				}
				else
				{
					Direction = ToPoint / Distance;
				}

				const FVector2D Adjusted = Center + Direction * Required;
				Result.X = Adjusted.X;
				Result.Y = Adjusted.Y;
				bAdjusted = true;
			}
		}

		if (!bAdjusted)
		{
			break;
		}
	}

	return Result;
}

FVector ARoadSegment::ClosestPointOnSegment2D(const FVector& Point, const FVector& A, const FVector& B)
{
	const FVector2D AP(Point.X - A.X, Point.Y - A.Y);
	const FVector2D AB(B.X - A.X, B.Y - A.Y);
	const float Denom = AB.SizeSquared();
	float Alpha = 0.f;
	if (Denom > KINDA_SMALL_NUMBER)
	{
		Alpha = FMath::Clamp(FVector2D::DotProduct(AP, AB) / Denom, 0.f, 1.f);
	}

	return FMath::Lerp(A, B, Alpha);
}

float ARoadSegment::DistancePointToSegment2D(const FVector2D& Point, const FVector2D& A, const FVector2D& B)
{
	const FVector2D AP = Point - A;
	const FVector2D AB = B - A;
	const float Denom = AB.SizeSquared();
	if (Denom <= KINDA_SMALL_NUMBER)
	{
		return FVector2D::Distance(Point, A);
	}

	float Alpha = FVector2D::DotProduct(AP, AB) / Denom;
	Alpha = FMath::Clamp(Alpha, 0.f, 1.f);
	const FVector2D Closest = A + AB * Alpha;
	return FVector2D::Distance(Point, Closest);
}
