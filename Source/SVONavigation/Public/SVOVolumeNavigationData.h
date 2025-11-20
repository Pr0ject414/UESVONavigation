#pragma once

#include <Templates/SubclassOf.h>

#include "Common/SVODataTypes.h"
#include "Common/SVONavigationTypes.h"

class UNavigationQueryFilter;
class USVONavigationQueryFilter;
enum class ESVOVersion : uint8;

struct FSVOVolumeNavigationDataGenerationSettings
{
    FSVOVolumeNavigationDataGenerationSettings();

    float VoxelExtent;
    UWorld * World;
    FSVODataGenerationSettings GenerationSettings;
};

class SVONAVIGATION_API FSVOVolumeNavigationData
{
public:
    typedef FSVONodeAddress FNodeRef;

    FSVOVolumeNavigationData() = default;

    // Used by FGraphAStar
    bool IsValidRef( const FSVONodeAddress ref ) const
    {
        return ref.IsValid();
    }

    bool IsInNavigationDataChunk() const;
    void SetInNavigationDataChunk( bool in_navigation_data_chunk );
    const FSVOVolumeNavigationDataGenerationSettings & GetDataGenerationSettings() const;
    const FBox & GetVolumeBounds() const;
    const FBox & GetNavigationBounds() const;
    const FSVOData & GetData() const;
    const FSVONode & GetNodeFromAddress( const FSVONodeAddress & address ) const;
    TSubclassOf< USVONavigationQueryFilter > GetVolumeNavigationQueryFilter() const;
    void SetVolumeNavigationQueryFilter( TSubclassOf< USVONavigationQueryFilter > navigation_query_filter );

    FVector GetNodePositionFromAddress( const FSVONodeAddress & address, bool try_get_sub_node_position ) const;
    FVector GetNodePositionFromLayerAndMortonCode( LayerIndex layer_index, MortonCode morton_code ) const;
    FVector GetLeafNodePositionFromMortonCode( MortonCode morton_code ) const;
    bool GetNodeAddressFromPosition( FSVONodeAddress & node_address, const FVector & position ) const;
    void GetNodeNeighbors( TArray< FSVONodeAddress > & neighbors, const FSVONodeAddress & node_address ) const;
    float GetLayerRatio( LayerIndex layer_index ) const;
    float GetLayerInverseRatio( LayerIndex layer_index ) const;
    float GetNodeExtentFromNodeAddress( FSVONodeAddress node_address ) const;
    TOptional< FNavLocation > GetRandomPoint() const;
    bool IsNodeAddressNavigable(const FSVONodeAddress& Address) const;
    void FindNodesInSphere(const FVector& Center, float Radius, TArray<FSVONodeAddress>& OutNodes) const;

    /**
     * Finds the closest navigable position to the target within a search radius.
     * Uses BFS starting from the node at TargetPos (even if occluded) to find a free neighbor.
     * @param TargetPos The desired world position.
     * @param OutNavigablePos The resulting safe position.
     * @param SearchRadius Maximum distance to search.
     * @return True if a navigable position was found.
     */
    bool GetClosestNavigablePosition(const FVector& TargetPos, FVector& OutNavigablePos, float SearchRadius) const;
    
    /** Returns the generated portals for this volume. Used for Macro Pathfinding. */
    const TArray<FSVOPortal>& GetPortals() const { return Portals; }

    void GenerateNavigationData( const FBox & volume_bounds, const FSVOVolumeNavigationDataGenerationSettings & generation_settings );
    void Serialize( FArchive & archive, const ESVOVersion version );
    void Reset();

private:
    int GetLayerCount() const;
    bool IsPositionOccluded( const FVector & position, float box_extent ) const;
    void FirstPassRasterization();
    void RasterizeLeaf( const FVector & node_position, const LeafIndex leaf_index );
    void RasterizeInitialLayer( TMap< LeafIndex, MortonCode > & leaf_index_to_layer_one_node_index_map );
    void RasterizeLayer( LayerIndex layer_index );
    int32 GetNodeIndexFromMortonCode( LayerIndex layer_index, MortonCode morton_code ) const;
    void BuildNeighborLinks( LayerIndex layer_index );
    bool FindNeighborInDirection( FSVONodeAddress & node_address, const LayerIndex layer_index, const NodeIndex node_index, const NeighborDirection direction );
    void GetLeafNeighbors( TArray< FSVONodeAddress > & neighbors, const FSVONodeAddress & leaf_address ) const;
    void GetFreeNodesFromNodeAddress( FSVONodeAddress node_address, TArray< FSVONodeAddress > & free_nodes ) const;
    void BuildParentLinkForLeafNodes( const TMap< LeafIndex, MortonCode > & leaf_index_to_parent_morton_code_map );
    void FindNodesInSphereRecursive(const FVector& Center, float RadiusSq, const FSVONodeAddress& CurrentNodeAddress, TArray<FSVONodeAddress>& OutNodes) const;
    
    /** Scans the boundaries of the volume to identify open voxels. */
    void IdentifyBoundaryVoxels();

    /** Converts identified boundary voxels into FSVOPortal structures. */
    void GeneratePortals();

    FSVOVolumeNavigationDataGenerationSettings Settings;
    FBox VolumeBounds;
    FSVOData SVOData;
    TSubclassOf< USVONavigationQueryFilter > VolumeNavigationQueryFilter;
    bool bInNavigationDataChunk;

    /** Intermediate storage for voxels found on the boundary */
    TArray<FSVOEdgeVoxel> BoundaryVoxels;

    /** Generated connection portals */
    TArray<FSVOPortal> Portals;
};

FORCEINLINE bool FSVOVolumeNavigationData::IsInNavigationDataChunk() const
{
    return bInNavigationDataChunk;
}

FORCEINLINE void FSVOVolumeNavigationData::SetInNavigationDataChunk( const bool in_navigation_data_chunk )
{
    bInNavigationDataChunk = in_navigation_data_chunk;
}

FORCEINLINE const FSVOVolumeNavigationDataGenerationSettings & FSVOVolumeNavigationData::GetDataGenerationSettings() const
{
    return Settings;
}

FORCEINLINE const FBox & FSVOVolumeNavigationData::GetVolumeBounds() const
{
    return VolumeBounds;
}

FORCEINLINE const FBox & FSVOVolumeNavigationData::GetNavigationBounds() const
{
    return SVOData.GetNavigationBounds();
}

FORCEINLINE const FSVOData & FSVOVolumeNavigationData::GetData() const
{
    return SVOData;
}

FORCEINLINE const FSVONode & FSVOVolumeNavigationData::GetNodeFromAddress( const FSVONodeAddress & address ) const
{
    return address.LayerIndex < 15
               ? SVOData.GetLayer( address.LayerIndex ).GetNode( address.NodeIndex )
               : SVOData.GetLastLayer().GetNode( 0 );
}

FORCEINLINE TSubclassOf< USVONavigationQueryFilter > FSVOVolumeNavigationData::GetVolumeNavigationQueryFilter() const
{
    return VolumeNavigationQueryFilter;
}

FORCEINLINE void FSVOVolumeNavigationData::SetVolumeNavigationQueryFilter( TSubclassOf< USVONavigationQueryFilter > navigation_query_filter )
{
    VolumeNavigationQueryFilter = navigation_query_filter;
}

FORCEINLINE int FSVOVolumeNavigationData::GetLayerCount() const
{
    return SVOData.GetLayerCount();
}