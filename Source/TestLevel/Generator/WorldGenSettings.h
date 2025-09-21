// Copyright notice placeholder

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "Templates/SubclassOf.h"
#include "WorldGenTypes.h"

#include "WorldGenSettings.generated.h"

class ARoadSegment;

/**
 * Data-driven parameters for room generation.
 * Place an instance in Content and assign in your GameMode or Subsystem.
 */
UCLASS(BlueprintType)
class TESTLEVEL_API UWorldGenSettings : public UDataAsset
{
    GENERATED_BODY()

public:
    /** Overall room size (X = depth, Y = width) in Unreal units. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Room")
    FVector2f RoomSize = FVector2f(6000.f, 4000.f);

    /** Height of walls along the perimeter. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Room|Wall")
    float WallHeight = 400.f;

    /** Desired distance between samples used to build wall segments. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Room|Wall")
    float WallSegmentLength = 400.f;

    /** How wide the walls should be scaled along Y. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Room|Wall")
    float WallThickness = 100.f;

    /** Width of a doorway/portal opening in the wall. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Room|Wall")
    float PortalWidth = 600.f;

    /** Extra padding from wall corners when placing portals. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Room|Wall")
    float PortalEdgePadding = 150.f;

    /** Minimal spacing between portals that live on the same wall. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Room|Wall")
    float PortalMinSeparation = 800.f;

    /** Offset applied along the outward normal when placing finish markers. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Room|Wall")
    float PortalSurfaceOffset = 30.f;

    /** Minimal distance from entrance to fallback point when there are no exits. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Room|Wall")
    float FallbackMinDistanceFromEntrance = 1200.f;

    /** Margin to keep randomly picked interior points away from walls. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Room")
    FVector2f InteriorMargin = FVector2f(300.f, 300.f);

    /** Instanced static mesh that represents a single wall chunk. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Room|Wall")
    UStaticMesh* WallSegmentMesh = nullptr;

    /** Maximal number of exits that can be generated. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Room|Portals")
    int32 MaxExitCount = 3;

    /** Optional blueprint class that implements spline road rendering. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road")
    TSubclassOf<ARoadSegment> RoadSegmentClass;

    /** Mesh used by spline mesh components to visualize the road. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road")
    UStaticMesh* RoadSplineMesh = nullptr;

    /** How far the road mesh should float above the ground. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road")
    float RoadSplineZOffset = 5.f;

    /** Scale applied to start/end of spline mesh components. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road")
    FVector2f RoadSplineScale = FVector2f(1.f, 1.f);

    /** Random deviations applied to intermediate road control points. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road")
    FVector2f RoadDeviation = FVector2f(700.f, 500.f);

    /** How close to the wall roads are allowed to travel. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road")
    FVector2f RoadClampMargin = FVector2f(400.f, 400.f);

    /** Minimal and maximal number of control points per generated road. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road")
    FIntPoint RoadControlPointRange = FIntPoint(1, 3);

    // --- POI ---
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "POI")
    float POISpawnChance = 0.35f;

    /** Safety margin from the walls when picking POI locations. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "POI")
    FVector2f POIMargin = FVector2f(300.f, 300.f);

    /** Minimal distance from any portal when spawning a POI. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "POI")
    float POIMinDistanceFromPortals = 700.f;

    /** Minimal distance between individual POIs. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "POI")
    float POIMinDistanceBetween = 900.f;

    /** Chance to connect a spawned POI with a road. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "POI")
    float ProbabilityOfRoadToPOI = 0.6f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "POI")
    TArray<FSpawnStruct> POITable;

    // --- Monsters ---
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Monsters")
    float MonsterSpawnChance = 0.7f;

    /** Safety margin from the walls when picking monster locations. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Monsters")
    FVector2f MonsterMargin = FVector2f(400.f, 400.f);

    /** Minimal distance from portals when spawning monsters. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Monsters")
    float MonsterMinDistanceFromPortals = 900.f;

    /** Minimal distance from any POI when spawning monsters. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Monsters")
    float MonsterMinDistanceFromPOI = 600.f;

    /** Minimal distance between monsters. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Monsters")
    float MonsterMinDistanceBetween = 600.f;

    /** Minimal distance from generated roads when spawning monsters. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Monsters")
    float MonsterMinDistanceFromRoads = 500.f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Monsters")
    TArray<FSpawnStruct> MonsterTable;

    // Randomness
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Random")
    int32 Seed = 1337;
};
