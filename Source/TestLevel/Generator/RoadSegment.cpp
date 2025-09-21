// Fill out your copyright notice in the Description page of Project Settings.


#include "Generator/RoadSegment.h"
#include "Components/SplineComponent.h"
#include "Components/SplineMeshComponent.h"
#include "Kismet/KismetMathLibrary.h"

ARoadSegment::ARoadSegment()
{
	PrimaryActorTick.bCanEverTick = false;
	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);
}

void ARoadSegment::ClearNetwork()
{
        BuiltPaths.Reset();
        CachedObstacles.Reset();
        CachedRoomHalfSize = FVector2f::ZeroVector;
        CachedSplineSamples.Reset();

        // Destroy all spline/spline mesh components created earlier
        TArray<UActorComponent*> Comps = GetComponents().Array();
        for (UActorComponent* C : Comps)
        {
                if (C && (C->IsA<USplineComponent>() || C->IsA<USplineMeshComponent>()))
                {
                        C->DestroyComponent();
                }
        }
}

float ARoadSegment::ClearanceToRect(const FVector2f& P, const FVector2f& H)
{
	// minimal distance to inner rectangle border in local space
	const float dx = H.X - FMath::Abs(P.X);
	const float dy = H.Y - FMath::Abs(P.Y);
	return FMath::Min(dx, dy);
}

float ARoadSegment::EdgeOffsetScale(const FVector2f& A_L, const FVector2f& B_L, const FVector2f& H)
{
	const float m = FMath::Min(ClearanceToRect(A_L, H), ClearanceToRect(B_L, H));
	const float mRef = 0.5f * FMath::Min(H.X, H.Y);
	float Scale = FMath::Clamp(m / FMath::Max(1.f, mRef), 0.25f, 1.0f);
	return Scale;
}

void ARoadSegment::ComputeMST_Prim(const TArray<FVector2f>& PtsLocal, TArray<FIntPoint>& OutEdges)
{
	OutEdges.Reset();
	const int32 N = PtsLocal.Num();
	if (N <= 1) return;

	TArray<float> Best;   Best.Init(FLT_MAX, N);
	TArray<int32> Par;    Par.Init(-1, N);
	TArray<uint8> In;     In.Init(0, N);

	auto Dist2 = [&](int32 i, int32 j)->float
		{
			const FVector2f d = PtsLocal[i] - PtsLocal[j];
			return d.X * d.X + d.Y * d.Y;
		};

	Best[0] = 0.f; // entrance root
	for (int32 it = 0; it < N; ++it)
	{
		int32 u = -1; float bu = FLT_MAX;
		for (int32 i = 0; i < N; ++i)
			if (!In[i] && Best[i] < bu) { bu = Best[i]; u = i; }
		if (u < 0) break;
		In[u] = 1;

		for (int32 v = 0; v < N; ++v)
		{
			if (In[v] || v == u) continue;
			const float w = Dist2(u, v);
			if (w < Best[v]) { Best[v] = w; Par[v] = u; }
		}
	}

	for (int32 v = 1; v < N; ++v)
		if (Par[v] >= 0) OutEdges.Add(FIntPoint(Par[v], v));
}

void ARoadSegment::MaybeAddShortcuts(const TArray<FVector2f>& PtsLocal, const FVector2f& H, TArray<FIntPoint>& InOutEdges)
{
	const float ShortCutMax = 0.6f * FMath::Max(H.X, H.Y); // uu

	auto HasEdge = [&](int32 a, int32 b)->bool
		{
			for (const FIntPoint& e : InOutEdges)
				if ((e.X == a && e.Y == b) || (e.X == b && e.Y == a)) return true;
			return false;
		};

	const int32 N = PtsLocal.Num();
	for (int32 i = 1; i < N; ++i)
	{
		// nearest j
		int32 jBest = -1; float dBest = FLT_MAX;
		for (int32 j = 0; j < N; ++j)
		{
			if (j == i) continue;
			const float d = FVector2D::Distance(FVector2D(PtsLocal[i]), FVector2D(PtsLocal[j]));
			if (d < dBest) { dBest = d; jBest = j; }
		}
		if (jBest >= 0 && dBest < ShortCutMax && !HasEdge(i, jBest))
		{
			InOutEdges.Add(FIntPoint(i, jBest));
		}
	}
}

void ARoadSegment::BuildMainPath(const FVector& Start, const FVector& End, FRandomStream& Rng, TArray<FVector>& OutPoints)
{
        const float Distance = FVector::Dist2D(Start, End);
        const float Ratio = (Distance > 2000.f) ? 0.25f : 0.2f;
        const int32 MinAnchors = (Distance > 1800.f) ? 3 : 1;
        BuildOrganicPath(Start, End, Rng, Ratio, MinAnchors, OutPoints);
}

void ARoadSegment::BuildBranchPath(const FVector& StartPoint, const FVector& Target, FRandomStream& Rng, TArray<FVector>& OutPoints)
{
        const float Distance = FVector::Dist2D(StartPoint, Target);
        const float Ratio = (Distance > 1200.f) ? 0.22f : 0.16f;
        const int32 MinAnchors = (Distance > 900.f) ? 2 : (Distance > 480.f ? 1 : 0);
        BuildOrganicPath(StartPoint, Target, Rng, Ratio, MinAnchors, OutPoints);
}

void ARoadSegment::BuildOrganicPath(const FVector& Start, const FVector& End, FRandomStream& Rng, float DeviationRatio, int32 MinAnchorCount, TArray<FVector>& OutPoints)
{
        OutPoints.Reset();
        OutPoints.Add(Start);

        const FVector2D Delta(End.X - Start.X, End.Y - Start.Y);
        const float Distance = Delta.Size();
        if (Distance <= KINDA_SMALL_NUMBER)
        {
                OutPoints.Add(End);
                return;
        }

        FVector2D Dir = Delta / Distance;
        FVector2D Perp(-Dir.Y, Dir.X);

        const float MaxDeviation = FMath::Clamp(Distance * 0.65f, 120.f, 620.f);
        const float BaseDeviation = FMath::Clamp(Distance * DeviationRatio, 24.f, MaxDeviation);
        const float ForwardDeviation = BaseDeviation * 0.35f;

        int32 AnchorCount = 0;
        if (Distance > 320.f)
        {
                AnchorCount = FMath::FloorToInt(Distance / 900.f);
                AnchorCount = FMath::Max(AnchorCount, MinAnchorCount);
        }
        else if (MinAnchorCount > 0 && Distance > 220.f)
        {
                AnchorCount = MinAnchorCount;
        }

        const float NoiseSeed = Rng.FRandRange(-5000.f, 5000.f);
        for (int32 AnchorIdx = 1; AnchorIdx <= AnchorCount; ++AnchorIdx)
        {
                const float T = static_cast<float>(AnchorIdx) / static_cast<float>(AnchorCount + 1);
                FVector Anchor = FMath::Lerp(Start, End, T);
                Anchor.Z = FMath::Lerp(Start.Z, End.Z, T);

                const float NoiseLateral = FMath::PerlinNoise1D(NoiseSeed + T * 2.173f);
                const float NoiseForward = FMath::PerlinNoise1D(NoiseSeed * 0.37f + T * 3.114f);
                const float RandomLateral = (Rng.FRand() - 0.5f) * 0.6f;
                const float RandomForward = (Rng.FRand() - 0.5f) * 0.6f;

                const float LateralOffset = (NoiseLateral + RandomLateral) * BaseDeviation;
                const float ForwardOffset = (NoiseForward + RandomForward) * ForwardDeviation;

                Anchor.X += Dir.X * ForwardOffset + Perp.X * LateralOffset;
                Anchor.Y += Dir.Y * ForwardOffset + Perp.Y * LateralOffset;

                Anchor = AdjustForObstacles(Anchor);
                OutPoints.Add(Anchor);
        }

        OutPoints.Add(End);

        if (OutPoints.Num() > 2)
        {
                RelaxPolyline(OutPoints);
        }

        ResolveDetours(OutPoints, Rng);

        if (OutPoints.Num() >= 2)
        {
                OutPoints[0] = Start;
                OutPoints.Last() = End;
        }
}

bool ARoadSegment::SegmentBlockedByAny(const FVector& A, const FVector& B, const FEnvironmentObstacle*& OutObstacle, float& OutHitParam) const
{
        if (CachedObstacles.IsEmpty())
        {
                        return false;
        }

        bool bBlocked = false;
        float BestParam = 1.f;
        const FEnvironmentObstacle* BestObstacle = nullptr;

        const float ExtraClearance = GenSettings ? FMath::Max(0.f, GenSettings->RoadExtraClearanceUU) : 0.f;
        const float Padding = 120.f;

        const FVector2D A2(A.X, A.Y);
        const FVector2D B2(B.X, B.Y);
        const FVector2D AB = B2 - A2;
        const float LenSq = AB.SizeSquared();

        for (const FEnvironmentObstacle& Ob : CachedObstacles)
        {
                const FVector2D Center(Ob.Location.X, Ob.Location.Y);
                float Param = 0.f;
                FVector2D Closest = A2;
                if (LenSq > KINDA_SMALL_NUMBER)
                {
                        Param = FMath::Clamp(FVector2D::DotProduct(Center - A2, AB) / LenSq, 0.f, 1.f);
                        Closest = A2 + AB * Param;
                }

                const float Radius = Ob.Radius + ExtraClearance + Padding;
                const float DistSq = FVector2D::DistSquared(Center, Closest);
                if (DistSq <= Radius * Radius)
                {
                        if (!bBlocked || Param < BestParam)
                        {
                                bBlocked = true;
                                BestParam = Param;
                                BestObstacle = &Ob;
                        }
                }
        }

        if (bBlocked)
        {
                OutObstacle = BestObstacle;
                OutHitParam = BestParam;
        }

        return bBlocked;
}

FVector ARoadSegment::MakeDetourPoint(const FVector& A, const FVector& B, const FEnvironmentObstacle& Obstacle, float SegmentParam, FRandomStream& Rng) const
{
        const float ExtraClearance = GenSettings ? FMath::Max(0.f, GenSettings->RoadExtraClearanceUU) : 0.f;
        const float GuardBand = 140.f;
        const float SideClear = Obstacle.Radius + ExtraClearance + GuardBand;
        const float ForwardPush = FMath::Max(220.f, Obstacle.Radius * 0.5f + ExtraClearance);

        FVector2D A2(A.X, A.Y);
        FVector2D B2(B.X, B.Y);
        FVector2D Center(Obstacle.Location.X, Obstacle.Location.Y);

        FVector2D AB = B2 - A2;
        float Len = AB.Size();
        FVector2D Dir = (Len > KINDA_SMALL_NUMBER) ? (AB / Len) : FVector2D(1.f, 0.f);
        FVector2D Perp(-Dir.Y, Dir.X);

        const float Along = SegmentParam * Len;
        FVector2D Closest = A2 + Dir * Along;
        const float SideIndicator = Dir.X * (Center.Y - Closest.Y) - Dir.Y * (Center.X - Closest.X);
        const float PreferredSign = (SideIndicator >= 0.f) ? -1.f : 1.f;

        auto CandidateFor = [&](float Sign)->FVector
        {
                const float SideScale = SideClear * (0.85f + 0.3f * Rng.FRand());
                const float ForwardScale = ForwardPush * (0.8f + 0.4f * Rng.FRand());
                FVector2D Candidate2 = Center + Dir * (ForwardScale + Along * 0.15f) + Perp * Sign * SideScale;
                FVector Candidate(Candidate2.X, Candidate2.Y, FMath::Lerp(A.Z, B.Z, SegmentParam));
                Candidate = AdjustForObstacles(Candidate);
                Candidate.Z = FMath::Lerp(A.Z, B.Z, SegmentParam);
                return Candidate;
        };

        const FVector PreferredCandidate = CandidateFor(PreferredSign);
        const FVector OppositeCandidate = CandidateFor(-PreferredSign);

        if (CachedObstacles.Num() <= 1)
        {
                return (Rng.FRand() < 0.5f) ? PreferredCandidate : OppositeCandidate;
        }

        auto ScoreCandidate = [&](const FVector& Candidate)->float
        {
                float MinClearance = FLT_MAX;
                for (const FEnvironmentObstacle& Other : CachedObstacles)
                {
                        const float Clearance = Other.Radius + ExtraClearance;
                        const float Dist = FVector::Dist2D(Candidate, Other.Location) - Clearance;
                        MinClearance = FMath::Min(MinClearance, Dist);
                }

                const float TravelPenalty = FVector::DistSquared(Candidate, (A + B) * 0.5f);
                return MinClearance - 0.0025f * TravelPenalty;
        };

        const float ScorePreferred = ScoreCandidate(PreferredCandidate);
        const float ScoreOpposite = ScoreCandidate(OppositeCandidate);

        if (ScorePreferred > ScoreOpposite + 5.f)
        {
                return PreferredCandidate;
        }
        if (ScoreOpposite > ScorePreferred + 5.f)
        {
                return OppositeCandidate;
        }

        return (Rng.FRand() < 0.5f) ? PreferredCandidate : OppositeCandidate;
}

bool ARoadSegment::InsertDetours(TArray<FVector>& Points, FRandomStream& Rng) const
{
        if (Points.Num() < 2 || CachedObstacles.IsEmpty())
        {
                return false;
        }

        bool bAddedAny = false;
        const int32 MaxIterations = 64;
        for (int32 Iter = 0; Iter < MaxIterations; ++Iter)
        {
                bool bInserted = false;
                for (int32 SegmentIdx = 0; SegmentIdx < Points.Num() - 1; ++SegmentIdx)
                {
                        const FEnvironmentObstacle* Blocking = nullptr;
                        float HitParam = 0.f;
                        if (SegmentBlockedByAny(Points[SegmentIdx], Points[SegmentIdx + 1], Blocking, HitParam))
                        {
                                if (!Blocking)
                                {
                                        continue;
                                }

                                FVector Candidate = MakeDetourPoint(Points[SegmentIdx], Points[SegmentIdx + 1], *Blocking, HitParam, Rng);
                                if (Candidate.Equals(Points[SegmentIdx], 1.f) || Candidate.Equals(Points[SegmentIdx + 1], 1.f))
                                {
                                        FVector Dir = Points[SegmentIdx + 1] - Points[SegmentIdx];
                                        if (!Dir.IsNearlyZero())
                                        {
                                                Dir.Normalize();
                                        }
                                        const float ExtraClearance = GenSettings ? FMath::Max(0.f, GenSettings->RoadExtraClearanceUU) : 0.f;
                                        Candidate = Points[SegmentIdx] + Dir * (Blocking->Radius + ExtraClearance + 200.f);
                                        Candidate.Z = FMath::Lerp(Points[SegmentIdx].Z, Points[SegmentIdx + 1].Z, HitParam);
                                        Candidate = AdjustForObstacles(Candidate);
                                }

                                Points.Insert(Candidate, SegmentIdx + 1);
                                bInserted = true;
                                bAddedAny = true;
                                break;
                        }
                }

                if (!bInserted)
                {
                        break;
                }
        }

        return bAddedAny;
}

void ARoadSegment::RelaxPolyline(TArray<FVector>& Points) const
{
        if (Points.Num() < 3)
        {
                return;
        }

        const int32 Count = Points.Num();
        const int32 Iterations = 2;
        for (int32 Iter = 0; Iter < Iterations; ++Iter)
        {
                for (int32 Idx = 1; Idx < Count - 1; ++Idx)
                {
                        const FVector Prev = Points[Idx - 1];
                        const FVector Curr = Points[Idx];
                        const FVector Next = Points[Idx + 1];

                        FVector Smoothed = Curr * 0.4f + Prev * 0.3f + Next * 0.3f;
                        Smoothed.Z = FMath::Lerp(Prev.Z, Next.Z, 0.5f);
                        Smoothed = AdjustForObstacles(Smoothed);
                        Points[Idx] = Smoothed;
                }
        }
}

void ARoadSegment::RemoveRedundantPoints(TArray<FVector>& Points) const
{
        if (Points.Num() < 3)
        {
                return;
        }

        for (int32 Idx = Points.Num() - 2; Idx > 0; --Idx)
        {
                const FVector2D Prev(Points[Idx - 1].X, Points[Idx - 1].Y);
                const FVector2D Curr(Points[Idx].X, Points[Idx].Y);
                const FVector2D Next(Points[Idx + 1].X, Points[Idx + 1].Y);

                const float DistPrev = FVector2D::DistSquared(Prev, Curr);
                const float DistNext = FVector2D::DistSquared(Curr, Next);
                if (DistPrev < 25.f && DistNext < 25.f)
                {
                        Points.RemoveAt(Idx);
                        continue;
                }

                FVector2D V0 = Curr - Prev;
                FVector2D V1 = Next - Curr;
                const float V0Sq = V0.SizeSquared();
                const float V1Sq = V1.SizeSquared();
                if (V0Sq < 1.f || V1Sq < 1.f)
                {
                        continue;
                }

                V0 /= FMath::Sqrt(V0Sq);
                V1 /= FMath::Sqrt(V1Sq);
                const float Cross = V0.X * V1.Y - V0.Y * V1.X;
                const float Dot = FVector2D::DotProduct(V0, V1);

                if (FMath::Abs(Cross) < 0.02f && Dot > 0.f)
                {
                        Points.RemoveAt(Idx);
                }
        }
}

void ARoadSegment::ResolveDetours(TArray<FVector>& Points, FRandomStream& Rng) const
{
        if (Points.Num() < 2)
        {
                return;
        }

        InsertDetours(Points, Rng);

        const int32 Passes = CachedObstacles.IsEmpty() ? 1 : 3;
        for (int32 Pass = 0; Pass < Passes; ++Pass)
        {
                        RelaxPolyline(Points);
                        RemoveRedundantPoints(Points);
                        if (!InsertDetours(Points, Rng))
                        {
                                break;
                        }
        }

        RemoveRedundantPoints(Points);

        for (int32 Idx = 1; Idx < Points.Num() - 1; ++Idx)
        {
                Points[Idx] = AdjustForObstacles(Points[Idx]);
        }
}

FVector ARoadSegment::AdjustForObstacles(const FVector& Point) const
{
        FVector Result = Point;

        if (CachedObstacles.IsEmpty())
        {
                return ClampToRoomBounds(Result);
        }

        const float ExtraClearance = GenSettings ? FMath::Max(0.f, GenSettings->RoadExtraClearanceUU) : 0.f;

        for (int32 Iter = 0; Iter < 4; ++Iter)
        {
                bool bAdjusted = false;
                for (const FEnvironmentObstacle& Ob : CachedObstacles)
                {
                        const FVector2D P2(Result.X, Result.Y);
                        const FVector2D O2(Ob.Location.X, Ob.Location.Y);
                        const float Desired = Ob.Radius + ExtraClearance;
                        const float Dist = FVector2D::Distance(P2, O2);

                        if (Desired > KINDA_SMALL_NUMBER && Dist < Desired)
                        {
                                FVector2D Dir = P2 - O2;
                                if (Dir.IsNearlyZero())
                                {
                                        Dir = FVector2D(1.f, 0.f);
                                }
                                Dir.Normalize();
                                const float Push = Desired - Dist;
                                Result.X += Dir.X * Push;
                                Result.Y += Dir.Y * Push;
                                bAdjusted = true;
                        }
                }

                Result = ClampToRoomBounds(Result);

                if (!bAdjusted)
                {
                        break;
                }
        }

        return Result;
}

FVector ARoadSegment::ClampToRoomBounds(const FVector& Point) const
{
        if (CachedRoomHalfSize.X <= KINDA_SMALL_NUMBER || CachedRoomHalfSize.Y <= KINDA_SMALL_NUMBER)
        {
                return Point;
        }

        const FTransform ActorXf = GetActorTransform();
        FVector Local = ActorXf.InverseTransformPosition(Point);

        const float MarginX = GenSettings ? FMath::Clamp(GenSettings->RoadMargin.X, 0.f, CachedRoomHalfSize.X) : 0.f;
        const float MarginY = GenSettings ? FMath::Clamp(GenSettings->RoadMargin.Y, 0.f, CachedRoomHalfSize.Y) : 0.f;

        const float LimitX = FMath::Max(0.f, CachedRoomHalfSize.X - MarginX);
        const float LimitY = FMath::Max(0.f, CachedRoomHalfSize.Y - MarginY);

        Local.X = FMath::Clamp(Local.X, -LimitX, LimitX);
        Local.Y = FMath::Clamp(Local.Y, -LimitY, LimitY);

        return ActorXf.TransformPosition(Local);
}

void ARoadSegment::BuildOnePath(const TArray<FVector>& PathPointsWS)
{
	if (!GenSettings || !GenSettings->RoadSplineMesh || PathPointsWS.Num() < 2) return;

	// --- 1) Densify the input polyline so the spline has enough control points ---
	TArray<FVector> DensePts;
	DensePts.Reserve(PathPointsWS.Num() * 4);
	DensePts.Add(PathPointsWS[0]);

	// Approx mesh length along X; use it to choose target spacing between spline points
	const FBoxSphereBounds MeshBounds = GenSettings->RoadSplineMesh->GetBounds();
	const float MeshLenApprox = FMath::Max(150.f, 2.f * MeshBounds.BoxExtent.X);
	const float TargetSpacing = FMath::Clamp(MeshLenApprox * 0.5f, 120.f, 600.f); // more points → smoother
	const int32 MaxSubdivPerSeg = 32;

	for (int32 i = 0; i < PathPointsWS.Num() - 1; ++i)
	{
		const FVector A = PathPointsWS[i];
		const FVector B = PathPointsWS[i + 1];

		const float segLen = FVector::Dist2D(A, B);
		const int32 steps = FMath::Clamp(FMath::CeilToInt(segLen / TargetSpacing), 1, MaxSubdivPerSeg);

		for (int32 s = 1; s <= steps; ++s)
		{
			const float t = static_cast<float>(s) / static_cast<float>(steps);
			DensePts.Add(FMath::Lerp(A, B, t));
		}
	}

        CachedSplineSamples.Append(DensePts);

        // --- 2) Create the spline and feed dense points as WORLD points, make them curved ---
	USplineComponent* Spline = NewObject<USplineComponent>(this, USplineComponent::StaticClass(), NAME_None, RF_Transactional);
	Spline->SetMobility(EComponentMobility::Movable);
	Spline->SetupAttachment(Root);
	Spline->RegisterComponent();

	Spline->ClearSplinePoints(false);
	for (int32 i = 0; i < DensePts.Num(); ++i)
	{
		Spline->AddSplinePoint(DensePts[i], ESplineCoordinateSpace::World, false);
		Spline->SetSplinePointType(i, ESplinePointType::Curve, false); // smooth interpolation
	}

	// --- 3) Provide Catmull-Rom style tangents so neighbor segments join smoothly ---
	auto TangentFor = [&](int32 idx)->FVector
		{
			if (idx <= 0)              return (DensePts[1] - DensePts[0]);
			if (idx >= DensePts.Num() - 1) return (DensePts.Last() - DensePts[DensePts.Num() - 2]);
			return 0.5f * (DensePts[idx + 1] - DensePts[idx - 1]);
		};

	const float TangentScale = FMath::Max(1.f, GenSettings->RoadTangentStrength);
	for (int32 i = 0; i < DensePts.Num(); ++i)
	{
		const FVector T = TangentFor(i).GetSafeNormal() * TangentScale;
		// Same arrive/leave keeps it smooth enough for a "trail"; feel free to bias if needed.
		Spline->SetTangentsAtSplinePoint(i, T, T, ESplineCoordinateSpace::World);
	}

	Spline->SetClosedLoop(false, false);
	Spline->UpdateSpline();

	// --- 4) Lay spline meshes uniformly along the spline arc-length (continuous, no gaps) ---
	const float PathLen = Spline->GetSplineLength();
	const int32 NumSegments = FMath::Max(1, FMath::RoundToInt(PathLen / MeshLenApprox));
	const float Step = PathLen / static_cast<float>(NumSegments);

	for (int32 s = 0; s < NumSegments; ++s)
	{
		const float D0 = s * Step;
		const float D1 = (s + 1) * Step;

		// Use WORLD space here because we sampled the spline in world and will keep world transforms on SMC
		const FVector StartPos = Spline->GetLocationAtDistanceAlongSpline(D0, ESplineCoordinateSpace::World);
		const FVector EndPos = Spline->GetLocationAtDistanceAlongSpline(D1, ESplineCoordinateSpace::World);
		FVector StartTangent = Spline->GetTangentAtDistanceAlongSpline(D0, ESplineCoordinateSpace::World);
		FVector EndTangent = Spline->GetTangentAtDistanceAlongSpline(D1, ESplineCoordinateSpace::World);

		// Normalize tangent magnitude to local segment size, then scale by user strength
		const float LocalLen = FVector::Dist2D(StartPos, EndPos);
		const float SafeLen = FMath::Max(LocalLen, 1.f);
		StartTangent = StartTangent.GetSafeNormal() * SafeLen * 0.5f;
		EndTangent = EndTangent.GetSafeNormal() * SafeLen * 0.5f;

		StartTangent *= TangentScale;
		EndTangent *= TangentScale;

		USplineMeshComponent* SMC = NewObject<USplineMeshComponent>(this, USplineMeshComponent::StaticClass(), NAME_None, RF_Transactional);
		SMC->SetMobility(EComponentMobility::Movable);
		SMC->SetStaticMesh(GenSettings->RoadSplineMesh);
		SMC->SetForwardAxis(ESplineMeshAxis::X, false);

		// Keep WORLD so we can pass world positions/tangents directly
		SMC->AttachToComponent(Spline, FAttachmentTransformRules::KeepWorldTransform);
		SMC->RegisterComponent();

		SMC->SetStartAndEnd(StartPos, StartTangent, EndPos, EndTangent, true);
		SMC->SetStartScale(FVector2D(1.f, 1.f));
		SMC->SetEndScale(FVector2D(1.f, 1.f));
		SMC->SetStartRoll(0.f);
		SMC->SetEndRoll(0.f);
	}
}

bool ARoadSegment::FindNearestSplineSample(const FVector& Point, FVector& OutPoint, float* OutDistSq) const
{
        if (CachedSplineSamples.IsEmpty())
        {
                if (OutDistSq)
                {
                        *OutDistSq = 0.f;
                }
                return false;
        }

        bool bFound = false;
        float BestDistSq = FLT_MAX;
        FVector BestPoint = FVector::ZeroVector;

        for (const FVector& Sample : CachedSplineSamples)
        {
                const float Dx = Point.X - Sample.X;
                const float Dy = Point.Y - Sample.Y;
                const float DistSq = Dx * Dx + Dy * Dy;

                if (DistSq < BestDistSq)
                {
                        BestDistSq = DistSq;
                        BestPoint = Sample;
                        bFound = true;
                }
        }

        if (bFound)
        {
                OutPoint = BestPoint;
                if (OutDistSq)
                {
                        *OutDistSq = BestDistSq;
                }
        }

        return bFound;
}

bool ARoadSegment::FindNearestPointOnPathDetailed(const FVector& Point, const TArray<FVector>& Path, FVector& OutPoint, FVector& OutTangent,
        float& OutDistSq, int32* OutSegmentIdx, float* OutSegmentT) const
{
        OutTangent = FVector::ZeroVector;
        if (OutSegmentIdx)
        {
                *OutSegmentIdx = INDEX_NONE;
        }
        if (OutSegmentT)
        {
                *OutSegmentT = 0.f;
        }

        if (Path.Num() < 2)
        {
                return false;
        }

        bool bFound = false;
        float BestDistSq = FLT_MAX;
        FVector BestPoint = FVector::ZeroVector;
        FVector BestTangent = FVector::ZeroVector;
        int32 BestSegment = INDEX_NONE;
        float BestT = 0.f;

        for (int32 i = 0; i < Path.Num() - 1; ++i)
        {
                const FVector A = Path[i];
                const FVector B = Path[i + 1];
                const FVector AB = B - A;
                const float LenSq = AB.SizeSquared();

                FVector Candidate = A;
                float SegmentT = 0.f;
                if (LenSq > KINDA_SMALL_NUMBER)
                {
                        SegmentT = FMath::Clamp(FVector::DotProduct(Point - A, AB) / LenSq, 0.0f, 1.0f);
                        Candidate = A + AB * SegmentT;
                }

                const float DistSq = FVector::DistSquared(Point, Candidate);
                if (DistSq < BestDistSq)
                {
                        BestDistSq = DistSq;
                        BestPoint = Candidate;
                        BestTangent = AB;
                        BestSegment = i;
                        BestT = SegmentT;
                        bFound = true;
                }
        }

        if (bFound)
        {
                OutPoint = BestPoint;
                OutDistSq = BestDistSq;
                OutTangent = BestTangent.GetSafeNormal();
                if (OutSegmentIdx)
                {
                        *OutSegmentIdx = BestSegment;
                }
                if (OutSegmentT)
                {
                        *OutSegmentT = BestT;
                }
        }

        return bFound;
}

bool ARoadSegment::FindNearestPointOnPath(const FVector& Point, const TArray<FVector>& Path, FVector& OutPoint, float& OutDistSq) const
{
FVector DummyTangent = FVector::ZeroVector;
return FindNearestPointOnPathDetailed(Point, Path, OutPoint, DummyTangent, OutDistSq);
}

void ARoadSegment::BuildNetwork(const TArray<FVector>& NodesWS, int32 ExitCount, const FVector2f& RoomHalfSize, const UWorldGenSettings* Settings, FRandomStream& Rng, const TArray<FEnvironmentObstacle>& Obstacles)
{
        BuiltPaths.Reset();

        if (NodesWS.Num() <= 1 || !Settings || ExitCount < 1)
        {
                return;
        }

        GenSettings = Settings;

        ClearNetwork();

        CachedObstacles = Obstacles;
        CachedRoomHalfSize = RoomHalfSize;

        const int32 EntryIdx = 0;
        const int32 FirstExitIdx = 1;
        const int32 LastExitIdx = ExitCount;
        if (LastExitIdx >= NodesWS.Num())
        {
                return;
        }

        // Determine which exit will be the primary road target.
        int32 FarthestExitIdx = FirstExitIdx;
        float MaxDistSq = 0.f;
        for (int32 ExitIdx = FirstExitIdx; ExitIdx <= LastExitIdx; ++ExitIdx)
        {
                const float DistSq = FVector::DistSquared(NodesWS[EntryIdx], NodesWS[ExitIdx]);
                if (DistSq > MaxDistSq)
                {
                        MaxDistSq = DistSq;
                        FarthestExitIdx = ExitIdx;
                }
        }

        TArray<TArray<FVector>> AllPaths;

        // Main road from entry to selected exit.
        TArray<FVector> MainPath;
        BuildMainPath(NodesWS[EntryIdx], NodesWS[FarthestExitIdx], Rng, MainPath);
        if (MainPath.Num() < 2)
        {
                MainPath.Reset();
                MainPath.Add(NodesWS[EntryIdx]);
                MainPath.Add(NodesWS[FarthestExitIdx]);
        }

        AllPaths.Add(MainPath);
        BuildOnePath(MainPath);

        // Secondary roads to remaining exits.
        for (int32 ExitIdx = FirstExitIdx; ExitIdx <= LastExitIdx; ++ExitIdx)
        {
                if (ExitIdx == FarthestExitIdx)
                {
                        continue;
                }

                FVector ConnectionPoint = FVector::ZeroVector;
                if (!FindNearestSplineSample(NodesWS[ExitIdx], ConnectionPoint))
                {
                        continue;
                }

                TArray<FVector> ExitPath;
                BuildBranchPath(ConnectionPoint, NodesWS[ExitIdx], Rng, ExitPath);
                if (ExitPath.Num() < 2)
                {
                        ExitPath.Reset();
                        ExitPath.Add(ConnectionPoint);
                        ExitPath.Add(NodesWS[ExitIdx]);
                }

                AllPaths.Add(ExitPath);
                BuildOnePath(ExitPath);
        }

        // Attach POIs to the nearest existing road (main or exit branches).
        for (int32 PoiIdx = LastExitIdx + 1; PoiIdx < NodesWS.Num(); ++PoiIdx)
        {
                const FVector PoiLocation = NodesWS[PoiIdx];

                FVector ConnectionPoint = FVector::ZeroVector;
                if (!FindNearestSplineSample(PoiLocation, ConnectionPoint))
                {
                        continue;
                }

                TArray<FVector> PoiPath;
                BuildBranchPath(ConnectionPoint, PoiLocation, Rng, PoiPath);
                if (PoiPath.Num() < 2)
                {
                        PoiPath.Reset();
                        PoiPath.Add(ConnectionPoint);
                        PoiPath.Add(PoiLocation);
                }

                AllPaths.Add(PoiPath);
                BuildOnePath(PoiPath);
        }

        BuiltPaths = AllPaths;
}

float ARoadSegment::DistanceToRoads(const FVector& Point) const
{
        if (BuiltPaths.IsEmpty())
        {
			return FLT_MAX;
        }

        const FVector2D P2(Point.X, Point.Y);
        float Best = FLT_MAX;

        for (const TArray<FVector>& Path : BuiltPaths)
        {
                for (int32 idx = 0; idx < Path.Num() - 1; ++idx)
                {
                        const FVector2D A(Path[idx].X, Path[idx].Y);
                        const FVector2D B(Path[idx + 1].X, Path[idx + 1].Y);
                        const FVector2D AB = B - A;
                        const float LenSq = AB.SizeSquared();

                        float Dist = 0.f;
                        if (LenSq <= KINDA_SMALL_NUMBER)
                        {
                                Dist = FVector2D::Distance(P2, A);
                        }
                        else
                        {
                                const float t = FMath::Clamp(FVector2D::DotProduct(P2 - A, AB) / LenSq, 0.f, 1.f);
                                const FVector2D Closest = A + AB * t;
                                Dist = FVector2D::Distance(P2, Closest);
                        }

                        Best = FMath::Min(Best, Dist);
                }
        }

        return Best;
}
