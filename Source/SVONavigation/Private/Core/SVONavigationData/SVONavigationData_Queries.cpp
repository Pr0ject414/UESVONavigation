#include "SVONavigationData.h"
#include "SVOVolumeNavigationData.h"
#include "PathFinding/SVOPathFinder.h"
#include "PathFinding/SVONavigationPath.h"
#include "Core/SVONavigationDataChunkActor.h"

FNavLocation ASVONavigationData::GetRandomPoint( FSharedConstNavQueryFilter /*filter*/, const UObject * /*querier*/ ) const
{
    // Gather all sources of volume data (Legacy + ChunkActors)
    TArray<const FSVOVolumeNavigationData*> AllVolumes;
    
    for (const auto& VolumeData : VolumeNavigationData)
    {
        if (VolumeData.GetData().IsValid())
        {
            AllVolumes.Add(&VolumeData);
        }
    }

    for (const auto& ChunkActor : ChunkActors)
    {
        if (ChunkActor && ChunkActor->GetVolumeNavigationData().GetData().IsValid())
        {
            AllVolumes.Add(&ChunkActor->GetVolumeNavigationData());
        }
    }

    FNavLocation result;
    if ( AllVolumes.Num() == 0 )
    {
        return result;
    }

    const int32 Index = FMath::RandRange(0, AllVolumes.Num() - 1);
    const auto* SelectedVolume = AllVolumes[Index];

    const auto random_point = SelectedVolume->GetRandomPoint();
    if ( random_point.IsSet() )
    {
        result = random_point.GetValue();
    }

    return result;
}

bool ASVONavigationData::GetRandomReachablePointInRadius( const FVector & origin, float radius, FNavLocation & out_result, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    // :TODO:
    ensure( false );
    return false;
}

bool ASVONavigationData::GetRandomPointInNavigableRadius( const FVector & origin, float Radius, FNavLocation & out_result, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    // :TODO:
    ensure( false );
    return false;
}

void ASVONavigationData::BatchRaycast( TArray< FNavigationRaycastWork > & workload, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    // :TODO:
    ensure( false );
}

bool ASVONavigationData::FindMoveAlongSurface( const FNavLocation & start_location, const FVector & target_position, FNavLocation & out_location, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    // :TODO:
    ensure( false );
    return false;
}

bool ASVONavigationData::ProjectPoint( const FVector & point, FNavLocation & out_location, const FVector & extent, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    const FSVOVolumeNavigationData* VolumeToSearch = GetVolumeNavigationDataContainingPoints({point});
    
    if ( !VolumeToSearch )
    {
        float MinDistSq = -1.0f;
        
        // Check Legacy Volumes
        for (const auto& Volume : VolumeNavigationData)
        {
            // Use VolumeBounds for tighter fit check, NavigationBounds for SVO logic
            const FBox& BoundingBox = Volume.GetVolumeBounds();
            if (!BoundingBox.IsValid) continue;

            const float DistSq = BoundingBox.ComputeSquaredDistanceToPoint(point);
            if (MinDistSq < 0 || DistSq < MinDistSq)
            {
                MinDistSq = DistSq;
                VolumeToSearch = &Volume;
            }
        }

        // Check World Partition Chunks
        for (const auto& Chunk : ChunkActors)
        {
            if (!Chunk) continue;
            const FSVOVolumeNavigationData& Volume = Chunk->GetVolumeNavigationData();
            const FBox& BoundingBox = Volume.GetVolumeBounds(); // Use tight bounds
            
            if (!BoundingBox.IsValid) continue;

            const float DistSq = BoundingBox.ComputeSquaredDistanceToPoint(point);
            if (MinDistSq < 0 || DistSq < MinDistSq)
            {
                MinDistSq = DistSq;
                VolumeToSearch = &Volume;
            }
        }
    }

    if ( !VolumeToSearch )
    {
        return false;
    }

    FVector StartPoint = point;
    const FBox& VolumeBounds = VolumeToSearch->GetNavigationBounds(); // Use SVO bounds for internal logic
    if (!VolumeBounds.IsInside(StartPoint))
    {
        StartPoint = VolumeBounds.GetClosestPointTo(StartPoint);
    }

    // 3. Attempt to find the initial node and check if it's already navigable.
    FSVONodeAddress InitialAddress;
    const bool bInitialNodeFound = VolumeToSearch->GetNodeAddressFromPosition(InitialAddress, StartPoint);
    if (bInitialNodeFound && VolumeToSearch->IsNodeAddressNavigable(InitialAddress))
    {
        out_location.Location = StartPoint;
        out_location.NodeRef = InitialAddress.GetNavNodeRef();
        return true;
    }

    // 4. BFS Initialization: The start point is either in an occluded node or couldn't be resolved.
    //    We must perform a search for the nearest navigable one.
    TQueue<FSVONodeAddress> OpenList;
    TSet<FSVONodeAddress> VisitedList;
    const FBox SearchBounds = FBox::BuildAABB(StartPoint, extent);
    
    if (bInitialNodeFound)
    {
        // Start point is in an occluded node, begin search from there.
        OpenList.Enqueue(InitialAddress);
        VisitedList.Add(InitialAddress);
    }
    else
    {
        // The start point could not be resolved to any node.
        // Seed the search with the nearest nodes to the start point instead of failing.
        
        TArray<FSVONodeAddress> SeedNodes;
        // A small radius, just enough to find the immediate surrounding nodes.
        const float SeedRadius = VolumeToSearch->GetData().GetLeafNodes().GetLeafNodeExtent() * 1.5f;
        VolumeToSearch->FindNodesInSphere(StartPoint, SeedRadius, SeedNodes);
        
        if (SeedNodes.IsEmpty())
        {
             return false;
        }

        for (const FSVONodeAddress& SeedNode : SeedNodes)
        {
            if (!VisitedList.Contains(SeedNode))
            {
                 OpenList.Enqueue(SeedNode);
                 VisitedList.Add(SeedNode);
            }
        }
    }
    
    // 5. BFS Loop
    FSVONodeAddress CurrentAddress;
    while ( OpenList.Dequeue(CurrentAddress))
    {
        TArray<FSVONodeAddress> Neighbors;
        VolumeToSearch->GetNodeNeighbors(Neighbors, CurrentAddress);

        for (const FSVONodeAddress& NeighborAddress : Neighbors)
        {
            if (!VisitedList.Contains( NeighborAddress))
            {
                VisitedList.Add(NeighborAddress);

                const FVector NeighborLocation = VolumeToSearch->GetNodePositionFromAddress(NeighborAddress, true);

                if (!SearchBounds.IsInside(NeighborLocation))
                {
                    continue;
                }

                if (VolumeToSearch->IsNodeAddressNavigable(NeighborAddress))
                {
                    out_location.Location = NeighborLocation;
                    out_location.NodeRef = NeighborAddress.GetNavNodeRef();
                    return true;
                }

                OpenList.Enqueue(NeighborAddress);
            }
        }
    }

    // 6. Failure
    return false;
}

void ASVONavigationData::BatchProjectPoints( TArray< FNavigationProjectionWork > & Workload, const FVector & Extent, FSharedConstNavQueryFilter Filter, const UObject * Querier ) const
{
    for (FNavigationProjectionWork& WorkItem : Workload)
    {
        WorkItem.bResult = ProjectPoint(WorkItem.Point, WorkItem.OutLocation, Extent, Filter, Querier);
    }
}

void ASVONavigationData::BatchProjectPoints( TArray< FNavigationProjectionWork > & Workload, FSharedConstNavQueryFilter Filter, const UObject * Querier ) const
{
    for (FNavigationProjectionWork& WorkItem : Workload)
    {
        WorkItem.bResult = ProjectPoint(WorkItem.Point, WorkItem.OutLocation, FVector::ZeroVector, Filter, Querier);
    }
}

ENavigationQueryResult::Type ASVONavigationData::CalcPathCost( const FVector & path_start, const FVector & path_end, FVector::FReal & out_path_cost, const FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    FVector::FReal path_length = 0.f;
    return CalcPathLengthAndCost( path_start, path_end, path_length, out_path_cost, filter, querier );
}

ENavigationQueryResult::Type ASVONavigationData::CalcPathLength( const FVector & path_start, const FVector & path_end, FVector::FReal & out_path_length, const FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    FVector::FReal path_cost = 0.f;
    return CalcPathLengthAndCost( path_start, path_end, out_path_length, path_cost, filter, querier );
}

ENavigationQueryResult::Type ASVONavigationData::CalcPathLengthAndCost( const FVector & path_start, const FVector & path_end, FVector::FReal & out_path_length, FVector::FReal & out_path_cost, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    if ( ( path_start - path_end ).IsNearlyZero() )
    {
        out_path_length = 0.f;
        return ENavigationQueryResult::Success;
    }

    auto * volume_navigation_data = GetVolumeNavigationDataContainingPoints( { path_start, path_end } );

    if ( volume_navigation_data == nullptr )
    {
        return ENavigationQueryResult::Error;
    }

    const TSharedRef< FSVONavigationPath > navigation_path = MakeShareable( new FSVONavigationPath() );

    const ENavigationQueryResult::Type Result = FSVOPathFinder::GetPath(navigation_path.Get(), *this, path_start, path_end, filter);

    if ( Result == ENavigationQueryResult::Success || ( Result == ENavigationQueryResult::Fail && navigation_path->IsPartial() ) )
    {
        out_path_length = navigation_path->GetLength();
        out_path_cost = navigation_path->GetCost();
    }

    return Result;
}

bool ASVONavigationData::DoesNodeContainLocation( NavNodeRef node_ref, const FVector & world_space_location ) const
{
    const FSVONodeAddress Address(node_ref);
    if (!Address.IsValid())
    {
        return false;
    }

    // Check Legacy Volumes
    for (const auto& Volume : VolumeNavigationData)
    {
        if (Volume.GetNavigationBounds().IsInside(world_space_location))
        {
            const FVector NodeLocation = Volume.GetNodePositionFromAddress(Address, true);
            const float NodeExtent = Volume.GetNodeExtentFromNodeAddress(Address);
            const FBox NodeBounds = FBox::BuildAABB(NodeLocation, FVector(NodeExtent));

            if (NodeBounds.IsInsideOrOn(world_space_location))
            {
                return true;
            }
        }
    }

    // Check Chunk Actors (World Partition)
    for (const auto& Chunk : ChunkActors)
    {
        if (!Chunk) continue;
        
        const FSVOVolumeNavigationData& Volume = Chunk->GetVolumeNavigationData();
        if (Volume.GetNavigationBounds().IsInside(world_space_location))
        {
            const FVector NodeLocation = Volume.GetNodePositionFromAddress(Address, true);
            const float NodeExtent = Volume.GetNodeExtentFromNodeAddress(Address);
            const FBox NodeBounds = FBox::BuildAABB(NodeLocation, FVector(NodeExtent));

            if (NodeBounds.IsInsideOrOn(world_space_location))
            {
                return true;
            }
        }
    }

    return false;
}

bool ASVONavigationData::IsNodeRefValid( const NavNodeRef node_ref ) const
{
    return FSVONodeAddress( node_ref ).IsValid();
}

FBox ASVONavigationData::GetBoundingBox() const
{
    FBox bounding_box( ForceInit );

    // Add legacy bounds
    for ( const auto & bounds : VolumeNavigationData )
    {
        // CHANGED: Use GetVolumeBounds (User requested size) instead of GetNavigationBounds (Expanded Octree)
        bounding_box += bounds.GetVolumeBounds();
    }

    // Add Chunk Actor bounds
    for (const auto& Chunk : ChunkActors)
    {
        if (Chunk)
        {
            // Chunk->GetBounds() is already the user-requested slice
            bounding_box += Chunk->GetBounds();
        }
    }

    return bounding_box;
}

const FSVOVolumeNavigationData * ASVONavigationData::GetVolumeNavigationDataContainingPoints( const TArray< FVector > & points ) const
{
    // 1. Check Chunk Actors (Prioritize loaded WP chunks)
    for (const auto& Chunk : ChunkActors)
    {
        if (!Chunk) continue;

        const FSVOVolumeNavigationData& Data = Chunk->GetVolumeNavigationData();
        
        // Check against tight VolumeBounds to prevent overlapping "phantom" octree space from stealing the query
        const auto& Bounds = Data.GetVolumeBounds();

        bool bAllPointsInside = true;
        for (const auto& Point : points)
        {
            if (!Bounds.IsInside(Point))
            {
                bAllPointsInside = false;
                break;
            }
        }

        if (bAllPointsInside)
        {
            return &Data;
        }
    }

    // 2. Check Legacy VolumeNavigationData
    return VolumeNavigationData.FindByPredicate( [ this, &points ]( const FSVOVolumeNavigationData & data ) {
        const auto & bounds = data.GetVolumeBounds(); // Use tight bounds
        for ( const auto & point : points )
        {
            if ( !bounds.IsInside( point ) )
            {
                return false;
            }
        }
        return true;
    } );
}