#pragma once

#include <CoreMinimal.h>
#include <Engine/DeveloperSettings.h>

#include "SVONavigationSettings.generated.h"

class USVORayCaster;

UCLASS( config = Engine, defaultconfig )
class SVONAVIGATION_API USVONavigationSettings final : public UDeveloperSettings
{
    GENERATED_BODY()

public:
    USVONavigationSettings();

    // Set to false to disable automatic navigation data rebuilds
    UPROPERTY( config, EditAnywhere, Category = "SVO Navigation" )
    uint8 bNavigationAutoUpdateEnabled : 1;

    /** 
     * If true, paths will be post-processed to remove redundant waypoints across chunk boundaries.
     * This requires the "Smart Portal" and "TraceStack" features to be active.
     */
    UPROPERTY( config, EditAnywhere, Category = "PathFinding" )
    uint8 bSmoothPaths : 1;

    // The algorithm to use to detect if there's a direct line of sight between the start of the path and the target.
    // If there's a direct LoS, the generated path will be a straight line from start to target.
    // Otherwise, the pathfinding algorithm will be executed.
    // If that option is not set, the pathfinding will always be executed.
    UPROPERTY( config, EditAnywhere, Category = "PathFinding" )
    TSubclassOf< USVORayCaster > DefaultRaycasterClass;
    
    /** * The minimum Z height for World Partition SVO generation. 
     * Forces all generated chunks to start at this height to prevent "flat world" issues.
     */
    UPROPERTY(config, EditAnywhere, Category = "World Partition")
    float WorldPartitionMinZ = -1000.0f;

    /** * The maximum Z height for World Partition SVO generation. 
     * Forces all generated chunks to extend to this height.
     */
    UPROPERTY(config, EditAnywhere, Category = "World Partition")
    float WorldPartitionMaxZ = 5000.0f;
};