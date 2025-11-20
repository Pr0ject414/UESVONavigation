#pragma once

#include "CoreMinimal.h"
#include "SVONavigationTypes.generated.h"

class USVOPathFindingAlgorithm;
class USVOPathHeuristicCalculator;
class USVOPathTraversalCostCalculator;

DECLARE_DELEGATE_ThreeParams( FSVONavigationPathQueryDelegate, uint32, ENavigationQueryResult::Type, FNavPathSharedPtr );

/** * Represents an open connection point on the boundary of a Navigation Volume.
 * Used for stitching paths between World Partition chunks.
 */
USTRUCT()
struct FSVOPortal
{
    GENERATED_BODY()

    FSVOPortal() 
        : Location(FVector::ZeroVector)
        , Extent(FVector::ZeroVector)
        , Direction(0)
    {}

    FSVOPortal(const FVector& InLocation, const FVector& InExtent, uint8 InDirection)
        : Location(InLocation)
        , Extent(InExtent)
        , Direction(InDirection)
    {}

    /** Center of the portal in World Space */
    UPROPERTY()
    FVector Location;

    /** Half-size of the portal opening */
    UPROPERTY()
    FVector Extent;

    /** * Direction of the face this portal sits on.
     * 0: +X, 1: -X, 2: +Y, 3: -Y, 4: +Z, 5: -Z
     */
    UPROPERTY()
    uint8 Direction;

    bool operator==(const FSVOPortal& Other) const
    {
        return Location.Equals(Other.Location, 1.0f) && Direction == Other.Direction;
    }

    friend FArchive& operator<<(FArchive& Ar, FSVOPortal& Portal)
    {
        Ar << Portal.Location;
        Ar << Portal.Extent;
        Ar << Portal.Direction;
        return Ar;
    }
};

/**
 * Represents a navigable voxel situated on the edge of a chunk.
 * Used during generation to identify where portals should be created.
 */
USTRUCT()
struct FSVOEdgeVoxel
{
    GENERATED_BODY()

    // The Morton Code of the leaf node
    uint_fast64_t MortonCode = 0;
    
    // The specific sub-node index (0-63) if applicable
    uint8 SubNodeIndex = 0;
    
    // The layer this voxel belongs to (usually 0 for leaves)
    uint8 LayerIndex = 0;

    // Bitmask indicating which faces of the chunk this voxel touches
    // 0: +X, 1: -X, 2: +Y, 3: -Y, 4: +Z, 5: -Z
    uint8 FaceMask = 0; 

    FSVOEdgeVoxel() = default;
    FSVOEdgeVoxel(uint_fast64_t InCode, uint8 InSubNode, uint8 InLayer)
        : MortonCode(InCode), SubNodeIndex(InSubNode), LayerIndex(InLayer) {}
};

USTRUCT()
struct FSVODataGenerationSettings
{
	GENERATED_USTRUCT_BODY()

	FSVODataGenerationSettings()
	{
		CollisionChannel = ECollisionChannel::ECC_WorldStatic;
		Clearance = 0.0f;

		CollisionQueryParameters.bFindInitialOverlaps = true;
		CollisionQueryParameters.bTraceComplex = false;
		CollisionQueryParameters.TraceTag = "SVONavigationRasterize";
	}

	UPROPERTY( EditAnywhere, Category = "Generation" )
	TEnumAsByte< ECollisionChannel > CollisionChannel;

	UPROPERTY( EditAnywhere, Category = "Generation" )
	float Clearance;

	FCollisionQueryParams CollisionQueryParameters;
};