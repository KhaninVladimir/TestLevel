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

void ARoadSegment::MakeCurvedPath(const FVector& A, const FVector& B,
        int32 MidCount,
        float MaxPerp,
        float NoiseJitter,
        float TangentStrength,
        float BaselineCurvature,
        FRandomStream& Rng,
        TArray<FVector>& OutPoints)
{
        OutPoints.Reset();
        OutPoints.Add(A);

        const FVector AB = B - A;
        const float   Len = FMath::Max(1.f, AB.Size());
        const FVector Dir = AB / Len;
        // 2D perp in XY plane (top-down feel). If you want full 3D, build an orthonormal basis.
        const FVector Perp = FVector(-Dir.Y, Dir.X, 0.f).GetSafeNormal();

        const float BaseSign = (BaselineCurvature != 0.f) ? (Rng.FRand() < 0.5f ? -1.f : 1.f) : 0.f;
        const float BaseAmplitude = FMath::Max(0.f, BaselineCurvature) * Len;
        const float WavePhase = Rng.FRandRange(0.f, 2.f * PI);
        const float WaveFrequency = Rng.FRandRange(0.65f, 1.35f);

        // Remember the previous lateral offset so the curve changes smoothly instead of
        // rapidly zig-zagging left/right between midpoints.
        float PrevOffset = 0.f;

        for (int32 i = 1; i <= MidCount; ++i)
        {
                const float tRaw = static_cast<float>(i) / (MidCount + 1);
                const FVector Base = A + Dir * (tRaw * Len);

                const float SmoothT = FMath::InterpEaseInOut(0.f, 1.f, tRaw, 2.f);
                const float EdgeFalloff = FMath::Sin(PI * SmoothT); // 0..1..0

                const float BaselineOffset = BaseSign * BaseAmplitude * EdgeFalloff;
                const float WaveOffset = BaseAmplitude * 0.5f * FMath::Sin((SmoothT * WaveFrequency + WavePhase) * PI) * EdgeFalloff;

                // Blend towards a new random target offset so the road bends smoothly.
                const float TargetOffset = Rng.FRandRange(-MaxPerp, MaxPerp) * EdgeFalloff;
                const float BlendAlpha = 0.35f + 0.35f * SmoothT;
                PrevOffset = FMath::Lerp(PrevOffset, BaselineOffset + WaveOffset + TargetOffset, BlendAlpha);

                const float jitter = Rng.FRandRange(-NoiseJitter, +NoiseJitter) * EdgeFalloff;
                const FVector Offset = Perp * PrevOffset + Dir * jitter * 0.1f;
                FVector Candidate = Base + Offset;
                Candidate = AdjustForObstacles(Candidate);
                OutPoints.Add(Candidate);
        }

        OutPoints.Add(B);

        // (Tangents are set when we build spline meshes from these points)
}

FVector ARoadSegment::EvaluateHermite(const FVector& P0, const FVector& T0, const FVector& P1, const FVector& T1, float T) const
{
        const float T2 = T * T;
        const float T3 = T2 * T;

        const float H00 = 2.f * T3 - 3.f * T2 + 1.f;
        const float H10 = T3 - 2.f * T2 + T;
        const float H01 = -2.f * T3 + 3.f * T2;
        const float H11 = T3 - T2;

        return H00 * P0 + H10 * T0 + H01 * P1 + H11 * T1;
}

FVector ARoadSegment::ClampBranchDeviation(const FVector& ConnectionPoint, const FVector& Forward, float MaxDistance, const FVector& Candidate) const
{
        FVector Dir2D = FVector(Forward.X, Forward.Y, 0.f);
        if (!Dir2D.Normalize())
        {
                return ClampToRoomBounds(Candidate);
        }

        FVector Rel = Candidate - ConnectionPoint;
        const float ForwardDist = FVector::DotProduct(Rel, Dir2D);
        const float MinForward = 0.f;
        const float MaxForward = FMath::Max(10.f, MaxDistance * 0.98f);
        const float ClampedForward = FMath::Clamp(ForwardDist, MinForward, MaxForward);

        FVector Side = Rel - Dir2D * ClampedForward;
        const float MaxSide = FMath::Clamp(MaxDistance * 0.4f, 120.f, 900.f);
        const float SideLenSq = Side.SizeSquared();
        if (SideLenSq > MaxSide * MaxSide && SideLenSq > KINDA_SMALL_NUMBER)
        {
                const float Scale = MaxSide / FMath::Sqrt(SideLenSq);
                Side *= Scale;
        }

        FVector Result = ConnectionPoint + Dir2D * ClampedForward + Side;
        Result.Z = Candidate.Z;
        return ClampToRoomBounds(Result);
}

void ARoadSegment::BuildPOIBranchPath(const FVector& ConnectionPoint,
        const FVector& ConnectionTangent,
        const FVector& POILocation,
        float ScalePOI,
        FRandomStream& Rng,
        TArray<FVector>& OutPoints)
{
        OutPoints.Reset();
        OutPoints.Add(ConnectionPoint);

        FVector ToPOI = POILocation - ConnectionPoint;
        ToPOI.Z = 0.f;

        const float DistanceToPOI = FVector2D(ToPOI.X, ToPOI.Y).Size();
        if (DistanceToPOI <= KINDA_SMALL_NUMBER)
        {
                OutPoints.Add(POILocation);
                return;
        }

        FVector Forward = FVector(ToPOI.X, ToPOI.Y, 0.f);
        Forward.Normalize();

        FVector RoadDir = FVector(ConnectionTangent.X, ConnectionTangent.Y, 0.f);
        if (!RoadDir.Normalize())
        {
                RoadDir = Forward;
        }
        else
        {
                const float Alignment = FMath::Clamp(FVector::DotProduct(RoadDir, Forward), 0.f, 1.f);
                RoadDir = (Forward + RoadDir * Alignment * 0.5f).GetSafeNormal();
                if (RoadDir.IsNearlyZero())
                {
                        RoadDir = Forward;
                }
        }

        const float BranchScale = FMath::Clamp(ScalePOI, 0.25f, 1.25f);
        const float ExtraClearance = GenSettings ? FMath::Max(0.f, GenSettings->RoadExtraClearanceUU) : 0.f;
        const float ClearancePadding = ExtraClearance + FMath::Lerp(90.f, 180.f, FMath::Clamp(BranchScale, 0.f, 1.f));

        auto SegmentHitsObstacle = [&](const FVector& A, const FVector& B, const FEnvironmentObstacle& Ob, float& OutParam)->bool
        {
                const FVector2D A2(A.X, A.Y);
                const FVector2D B2(B.X, B.Y);
                const FVector2D AB = B2 - A2;
                const float LenSq = AB.SizeSquared();

                FVector2D Closest = A2;
                float Param = 0.f;

                if (LenSq > KINDA_SMALL_NUMBER)
                {
                        Param = FMath::Clamp(FVector2D::DotProduct(FVector2D(Ob.Location.X, Ob.Location.Y) - A2, AB) / LenSq, 0.f, 1.f);
                        Closest = A2 + AB * Param;
                }

                const float DistSq = FVector2D::DistSquared(Closest, FVector2D(Ob.Location.X, Ob.Location.Y));
                const float Radius = Ob.Radius + ClearancePadding;
                if (DistSq <= Radius * Radius)
                {
                        OutParam = Param;
                        return true;
                }

                return false;
        };

        auto SegmentBlocked = [&](const FVector& A, const FVector& B, const FEnvironmentObstacle*& OutObstacle, float& OutParam)->bool
        {
                bool bHit = false;
                float BestParam = 1.f;
                const FEnvironmentObstacle* BestObstacle = nullptr;

                for (const FEnvironmentObstacle& Ob : CachedObstacles)
                {
                        float Param = 0.f;
                        if (SegmentHitsObstacle(A, B, Ob, Param))
                        {
                                if (!bHit || Param < BestParam)
                                {
                                        bHit = true;
                                        BestParam = Param;
                                        BestObstacle = &Ob;
                                }
                        }
                }

                if (bHit)
                {
                        OutObstacle = BestObstacle;
                        OutParam = BestParam;
                }

                return bHit;
        };

        auto ProgressToZ = [&](const FVector& Point)->float
        {
                const float DistFromConnection = FVector::Dist2D(ConnectionPoint, Point);
                return (DistanceToPOI > KINDA_SMALL_NUMBER)
                        ? FMath::Clamp(DistFromConnection / DistanceToPOI, 0.f, 1.f)
                        : 0.f;
        };

        // Initial guide along the road tangent to avoid immediate sharp turns.
        const float StartAdvance = FMath::Clamp(DistanceToPOI * (0.18f + 0.22f * BranchScale), 150.f, 420.f);
        FVector StartAnchor = ConnectionPoint + RoadDir * StartAdvance;
        StartAnchor.Z = FMath::Lerp(ConnectionPoint.Z, POILocation.Z, FMath::Clamp(StartAdvance / DistanceToPOI, 0.f, 1.f));
        StartAnchor = AdjustForObstacles(StartAnchor);

        const FEnvironmentObstacle* AnchorBlocker = nullptr;
        float AnchorBlockParam = 0.f;
        if (SegmentBlocked(ConnectionPoint, StartAnchor, AnchorBlocker, AnchorBlockParam))
        {
                StartAnchor = ConnectionPoint;
        }

        if (!StartAnchor.Equals(ConnectionPoint, 1.f))
        {
                StartAnchor = ClampBranchDeviation(ConnectionPoint, Forward, DistanceToPOI, StartAnchor);
                OutPoints.Add(StartAnchor);
        }

        auto ChooseDetourPoint = [&](const FVector& CurrentPoint, const FEnvironmentObstacle& Ob, float HitParam)->FVector
        {
                FVector2D Start2(CurrentPoint.X, CurrentPoint.Y);
                FVector2D End2(POILocation.X, POILocation.Y);
                FVector2D Dir2 = End2 - Start2;
                if (!Dir2.Normalize())
                {
                        Dir2 = FVector2D(Forward.X, Forward.Y);
                        Dir2.Normalize();
                }

                const FVector2D Perp2(-Dir2.Y, Dir2.X);
                const FVector2D Center2(Ob.Location.X, Ob.Location.Y);
                const float Radius = Ob.Radius + ClearancePadding;

                const float SideCross = Dir2.X * (Center2.Y - Start2.Y) - Dir2.Y * (Center2.X - Start2.X);
                const float PreferredSign = (SideCross >= 0.f) ? -1.f : 1.f;

                auto MakeCandidate = [&](float SideSign)->FVector
                {
                        const float ForwardPush = Radius + 120.f;
                        const float SidePush = Radius + 140.f;
                        const FVector2D Candidate2 = Center2 + Dir2 * ForwardPush + Perp2 * SideSign * SidePush;
                        FVector Candidate(Candidate2.X, Candidate2.Y, FMath::Lerp(ConnectionPoint.Z, POILocation.Z, FMath::Clamp(HitParam + 0.2f, 0.f, 1.f)));
                        Candidate = AdjustForObstacles(Candidate);
                        return Candidate;
                };

                auto EvaluateCandidate = [&](const FVector& Candidate)->float
                {
                        const float LargePenalty = 1e12f;
                        if (Candidate.Equals(CurrentPoint, 1.f))
                        {
                                return LargePenalty;
                        }

                        const FEnvironmentObstacle* Blocker = nullptr;
                        float BlockParam = 0.f;
                        if (SegmentBlocked(CurrentPoint, Candidate, Blocker, BlockParam))
                        {
                                return LargePenalty;
                        }

                        float Score = FVector::DistSquared(CurrentPoint, Candidate);
                        Score += FVector::DistSquared(Candidate, POILocation) * 0.1f;
                        return Score;
                };

                const FVector CandidatePreferred = MakeCandidate(PreferredSign);
                const float ScorePreferred = EvaluateCandidate(CandidatePreferred);

                const FVector CandidateOpposite = MakeCandidate(-PreferredSign);
                const float ScoreOpposite = EvaluateCandidate(CandidateOpposite);

                const bool bPreferredValid = ScorePreferred < 1e12f;
                const bool bOppositeValid = ScoreOpposite < 1e12f;

                if (!bPreferredValid && !bOppositeValid)
                {
                        // Fall back to a perpendicular push if both candidates collide.
                        const FVector2D Fallback2 = Start2 + Perp2 * PreferredSign * (Radius + 200.f);
                        FVector Fallback(Fallback2.X, Fallback2.Y, FMath::Lerp(ConnectionPoint.Z, POILocation.Z, FMath::Clamp(HitParam + 0.2f, 0.f, 1.f)));
                        return AdjustForObstacles(Fallback);
                }

                if (bPreferredValid && bOppositeValid && FMath::IsNearlyEqual(ScorePreferred, ScoreOpposite))
                {
                        return (Rng.FRand() < 0.5f) ? CandidatePreferred : CandidateOpposite;
                }

                if (!bOppositeValid || ScorePreferred <= ScoreOpposite)
                {
                        return CandidatePreferred;
                }

                return CandidateOpposite;
        };

        FVector CurrentPoint = OutPoints.Last();
        const int32 MaxDetours = 12;
        for (int32 Iter = 0; Iter < MaxDetours; ++Iter)
        {
                const FEnvironmentObstacle* BlockingObstacle = nullptr;
                float HitParam = 1.f;
                if (!SegmentBlocked(CurrentPoint, POILocation, BlockingObstacle, HitParam))
                {
                        break; // Direct path to POI is clear.
                }

                FVector NextPoint = ChooseDetourPoint(CurrentPoint, *BlockingObstacle, HitParam);
                const float Progress = ProgressToZ(NextPoint);
                NextPoint.Z = FMath::Lerp(ConnectionPoint.Z, POILocation.Z, Progress);
                NextPoint = AdjustForObstacles(NextPoint);
                NextPoint = ClampBranchDeviation(ConnectionPoint, Forward, DistanceToPOI, NextPoint);

                if (FVector::DistSquared(NextPoint, CurrentPoint) < 4.f)
                {
                        // If we failed to move, push forward slightly to avoid stalling.
                        FVector Nudge = CurrentPoint + Forward * (BlockingObstacle->Radius + ClearancePadding + 200.f);
                        Nudge.Z = FMath::Lerp(ConnectionPoint.Z, POILocation.Z, HitParam);
                        NextPoint = AdjustForObstacles(Nudge);
                        NextPoint = ClampBranchDeviation(ConnectionPoint, Forward, DistanceToPOI, NextPoint);
                }

                if (FVector::DistSquared(NextPoint, CurrentPoint) < 4.f)
                {
                        // No progress possible, stop detouring to avoid infinite loops.
                        break;
                }

                OutPoints.Add(NextPoint);
                CurrentPoint = NextPoint;
        }

        if (!OutPoints.Last().Equals(POILocation, 1.f))
        {
                OutPoints.Add(POILocation);
        }

        for (int32 i = OutPoints.Num() - 1; i > 0; --i)
        {
                if (FVector::DistSquared(OutPoints[i], OutPoints[i - 1]) <= 4.f)
                {
                        OutPoints.RemoveAt(i);
                }
        }

        if (OutPoints.Num() < 2)
        {
                OutPoints.Reset();
                OutPoints.Add(ConnectionPoint);
                OutPoints.Add(POILocation);
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

        if (NodesWS.Num() <= 1 || !Settings || ExitCount < 1) return;

	GenSettings = Settings;

	// Clean old paths
	ClearNetwork();

	CachedObstacles = Obstacles;
	CachedRoomHalfSize = RoomHalfSize;

	// Validate indices
	const int32 EntryIdx = 0;
	const int32 FirstExitIdx = 1;
	const int32 LastExitIdx = ExitCount; // Inclusive
	const int32 FirstPOIIdx = ExitCount + 1;

	if (LastExitIdx >= NodesWS.Num()) return; // Not enough nodes

	// Convert nodes to actor-local 2D (room space) for topology
	TArray<FVector2f> PtsLocal;
	PtsLocal.Reserve(NodesWS.Num());
	for (const FVector& P : NodesWS)
	{
		const FVector L = GetActorTransform().InverseTransformPosition(P);
		PtsLocal.Add(FVector2f(L.X, L.Y));
	}

	// Storage for all paths
	TArray<TArray<FVector>> AllPaths;

	// 1. Find the farthest exit from entry
	int32 FarthestExitIdx = FirstExitIdx;
	float MaxDist = 0.0f;
	for (int32 i = FirstExitIdx; i <= LastExitIdx; ++i)
	{
		float Dist = FVector::Dist(NodesWS[EntryIdx], NodesWS[i]);
		if (Dist > MaxDist)
		{
			MaxDist = Dist;
			FarthestExitIdx = i;
		}
	}

        // 2. Build main road from entry to farthest exit
        const float ExitOffset = 400.0f; // Offset distance before snapping to the exit portal

        auto MakeExitApproachPoint = [&](const FVector& ExitLocation, const FVector& ReferencePoint)
                {
                        FVector Dir = ExitLocation - ReferencePoint;
                        Dir.Z = 0.f;

                        const float Distance = Dir.Size();
                        if (Distance <= KINDA_SMALL_NUMBER)
                        {
                                return AdjustForObstacles(ExitLocation);
                        }

                        Dir /= Distance;
                        float DesiredOffset = ExitOffset;
                        if (Distance <= ExitOffset)
                        {
                                DesiredOffset = Distance * 0.5f;
                        }

                        const FVector Candidate = ExitLocation - Dir * DesiredOffset;
                        return AdjustForObstacles(Candidate);
                };

        const FVector MainOffsetPoint = MakeExitApproachPoint(NodesWS[FarthestExitIdx], NodesWS[EntryIdx]);

        // Build curved path to offset point
        TArray<FVector> MainPath;
        const float Scale = EdgeOffsetScale(PtsLocal[EntryIdx], PtsLocal[FarthestExitIdx], RoomHalfSize);
        MakeCurvedPath(NodesWS[EntryIdx], MainOffsetPoint,
                static_cast<int32>(GenSettings->RoadMidpointCount),
                GenSettings->RoadMaxPerpOffset * Scale,
                GenSettings->RoadNoiseJitter * Scale,
                GenSettings->RoadTangentStrength,
                GenSettings->RoadBaselineCurvature * Scale,
                Rng,
                MainPath);

        // Add final segment to actual exit
        MainPath.Add(NodesWS[FarthestExitIdx]);

	AllPaths.Add(MainPath);
	BuildOnePath(MainPath);

        // 3. Connect other exits to the main road
        for (int32 i = FirstExitIdx; i <= LastExitIdx; ++i)
        {
                if (i == FarthestExitIdx) continue; // Skip the one we already connected

                FVector ConnectionPoint = FVector::ZeroVector;
                FVector ConnectionTangent = FVector::ZeroVector;
                float DummyDistSq = 0.f;
                if (!FindNearestPointOnPathDetailed(NodesWS[i], MainPath, ConnectionPoint, ConnectionTangent, DummyDistSq))
                {
                        continue;
                }

                // Calculate offset point for this exit
                const FVector OffsetPointLocal = MakeExitApproachPoint(NodesWS[i], ConnectionPoint);

                // Build path from connection point to offset point
                TArray<FVector> ExitPath;
                const FVector ConnectionPointLocal = GetActorTransform().InverseTransformPosition(ConnectionPoint);
                const float ScaleLocal = EdgeOffsetScale(
                        FVector2f(ConnectionPointLocal.X, ConnectionPointLocal.Y),
                        PtsLocal[i],
                        RoomHalfSize);

                MakeCurvedPath(ConnectionPoint, OffsetPointLocal,
                        FMath::Max(1, static_cast<int32>(GenSettings->RoadMidpointCount) / 2), // Fewer midpoints for branches
                        GenSettings->RoadMaxPerpOffset * ScaleLocal * 0.7f, // Less deviation for branches
                        GenSettings->RoadNoiseJitter * ScaleLocal * 0.7f,
                        GenSettings->RoadTangentStrength,
                        GenSettings->RoadBaselineCurvature * ScaleLocal * 0.7f,
                        Rng,
                        ExitPath);

		// Add final segment to actual exit
		ExitPath.Add(NodesWS[i]);

		AllPaths.Add(ExitPath);
		BuildOnePath(ExitPath);
	}

        // 4. Connect POIs to the nearest road
        for (int32 i = FirstPOIIdx; i < NodesWS.Num(); ++i)
        {
                const FVector PoiLocation = NodesWS[i];

                // Find nearest point on any existing path
                int32 BestPathIdx = -1;
                float BestDistSq = FLT_MAX;
                FVector BestConnectionPoint = FVector::ZeroVector;
                FVector BestConnectionTangent = FVector::ZeroVector;
                int32 BestConnectionSegment = INDEX_NONE;
                float BestConnectionParam = 0.f;

                for (int32 PathIdx = 0; PathIdx < AllPaths.Num(); ++PathIdx)
                {
                        FVector CandidatePoint = FVector::ZeroVector;
                        FVector CandidateTangent = FVector::ZeroVector;
                        float DistSq = 0.f;
                        int32 SegmentIdx = INDEX_NONE;
                        float SegmentParam = 0.f;
                        if (FindNearestPointOnPathDetailed(PoiLocation,
                                AllPaths[PathIdx],
                                CandidatePoint,
                                CandidateTangent,
                                DistSq,
                                &SegmentIdx,
                                &SegmentParam)
                                && DistSq < BestDistSq)
                        {
                                BestDistSq = DistSq;
                                BestPathIdx = PathIdx;
                                BestConnectionPoint = CandidatePoint;
                                BestConnectionTangent = CandidateTangent;
                                BestConnectionSegment = SegmentIdx;
                                BestConnectionParam = SegmentParam;
                        }
                }

                if (BestPathIdx >= 0)
                {
                        const float EndpointBias = 0.12f;
                        bool bConnectionNearEndpoint = (BestConnectionSegment == INDEX_NONE)
                                || (BestConnectionParam <= EndpointBias)
                                || (BestConnectionParam >= 1.f - EndpointBias);

                        if (bConnectionNearEndpoint)
                        {
                                const TArray<FVector>& PathPts = AllPaths[BestPathIdx];
                                float BestInteriorDistSq = FLT_MAX;
                                FVector BestInteriorPoint = BestConnectionPoint;
                                FVector BestInteriorTangent = BestConnectionTangent;
                                bool bFoundInterior = false;

                                for (int32 SegmentIdx = 0; SegmentIdx < PathPts.Num() - 1; ++SegmentIdx)
                                {
                                        const FVector A = PathPts[SegmentIdx];
                                        const FVector B = PathPts[SegmentIdx + 1];
                                        const FVector AB = B - A;
                                        const float LenSq = AB.SizeSquared();
                                        if (LenSq <= KINDA_SMALL_NUMBER)
                                        {
                                                continue;
                                        }

                                        const float RawT = FVector::DotProduct(PoiLocation - A, AB) / LenSq;
                                        const float ClampedT = FMath::Clamp(RawT, EndpointBias, 1.f - EndpointBias);
                                        if (ClampedT <= EndpointBias || ClampedT >= 1.f - EndpointBias)
                                        {
                                                continue;
                                        }

                                        const FVector Candidate = A + AB * ClampedT;
                                        const float DistSq = FVector::DistSquared(PoiLocation, Candidate);
                                        if (!bFoundInterior || DistSq < BestInteriorDistSq)
                                        {
                                                BestInteriorDistSq = DistSq;
                                                BestInteriorPoint = Candidate;
                                                BestInteriorTangent = AB.GetSafeNormal();
                                                bFoundInterior = true;
                                        }
                                }

                                if (bFoundInterior && BestInteriorDistSq <= BestDistSq * 1.15f)
                                {
                                        BestConnectionPoint = BestInteriorPoint;
                                        BestConnectionTangent = BestInteriorTangent;
                                        BestDistSq = BestInteriorDistSq;
                                        bConnectionNearEndpoint = false;
                                }
                        }

                        // Build path from connection point to POI with a gentle side branch.
                        const FTransform ActorXf = GetActorTransform();
                        const FVector ConnectionPointLocal = ActorXf.InverseTransformPosition(BestConnectionPoint);
                        const float ScalePOI = EdgeOffsetScale(
                                FVector2f(ConnectionPointLocal.X, ConnectionPointLocal.Y),
                                PtsLocal[i],
                                RoomHalfSize);

                        TArray<FVector> POIPath;
                        BuildPOIBranchPath(BestConnectionPoint,
                                BestConnectionTangent,
                                PoiLocation,
                                ScalePOI,
                                Rng,
                                POIPath);

                        if (POIPath.Num() >= 2)
                        {
                                AllPaths.Add(POIPath);
                                BuildOnePath(POIPath);
                        }
                }
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
