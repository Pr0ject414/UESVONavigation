#include "PathFinding/SVOPathFinder.h"

#include "PathFinding/SVONavigationQueryFilter.h"
#include "PathFinding/SVOPathFindingAlgorithm.h"
#include "Pathfinding/SVONavigationQueryFilterImpl.h"
#include "Raycasters/SVORayCaster.h"
#include "SVONavigationData.h"
#include "SVONavigationSettings.h"
#include "Grid/SVONavigationWorldSubsystem.h"
#include "Core/SVONavigationDataChunkActor.h"
#include "PathFinding/SVONavigationPath.h"
#include "PathFinding/SVOPathSmoother.h"

namespace
{
    USVOPathFindingAlgorithm * GetPathFindingAlgorithm( const FSharedConstNavQueryFilter & nav_query_filter )
    {
        if ( !ensureAlwaysMsgf( nav_query_filter.IsValid(), TEXT( "The query filter is not valid" ) ) )
        {
            return nullptr;
        }

        const FSVONavigationQueryFilterImpl * query_filter_implementation = static_cast< const FSVONavigationQueryFilterImpl * >( nav_query_filter->GetImplementation() );

        if ( !ensureAlwaysMsgf( query_filter_implementation != nullptr, TEXT( "The query filter implementation is not valid" ) ) )
        {
            return nullptr;
        }

        const auto & query_filter_settings = query_filter_implementation->QueryFilterSettings;

        if ( !ensureAlwaysMsgf( query_filter_settings.PathFinder != nullptr, TEXT( "The PathFinder is not valid" ) ) )
        {
            return nullptr;
        }

        if ( !ensureAlwaysMsgf( query_filter_settings.TraversalCostCalculator != nullptr, TEXT( "The TraversalCostCalculator is not valid" ) ) )
        {
            return nullptr;
        }

        if ( !ensureAlwaysMsgf( query_filter_settings.HeuristicCalculator != nullptr, TEXT( "The HeuristicCalculator is not valid" ) ) )
        {
            return nullptr;
        }

        return query_filter_settings.PathFinder;
    }
}

ENavigationQueryResult::Type FSVOPathFinder::GetPath( FSVONavigationPath & navigation_path, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, FSharedConstNavQueryFilter nav_query_filter )
{
    UWorld* World = navigation_data.GetWorld();
    if (!World) return ENavigationQueryResult::Fail;

    const USVOPathFindingAlgorithm* path_finder = GetPathFindingAlgorithm(nav_query_filter);
    if (!path_finder) return ENavigationQueryResult::Fail;

    // 1. Intra-Volume Pathfinding (Single Chunk or Monolithic)
    // If both points are in the same volume, we don't need macro pathfinding.
    if (const auto* SingleVolumeData = navigation_data.GetVolumeNavigationDataContainingPoints({ start_location, end_location }))
    {
        // Raycast Optimization (Try straight line first)
        if (auto* settings = GetDefault<USVONavigationSettings>())
        {
            if (settings->DefaultRaycasterClass != nullptr)
            {
                if (!settings->DefaultRaycasterClass->GetDefaultObject<USVORayCaster>()->Trace(*SingleVolumeData, start_location, end_location))
                {
                    auto& path_points = navigation_path.GetPathPoints();
                    path_points.Emplace(start_location);
                    path_points.Emplace(end_location);
                    navigation_path.MarkReady();
                    return ENavigationQueryResult::Success;
                }
            }
        }

        const auto volume_navigation_query_filter = SingleVolumeData->GetVolumeNavigationQueryFilter();
        const auto navigation_query_filter_copy = volume_navigation_query_filter != nullptr
                                                      ? volume_navigation_query_filter.GetDefaultObject()->GetQueryFilter( navigation_data, nullptr )
                                                      : nav_query_filter;

        const auto params = FSVOPathFindingParameters::Initialize(*SingleVolumeData, start_location, end_location, *navigation_query_filter_copy);
        
        if (params.IsSet())
        {
            return path_finder->GetPath(navigation_path, params.GetValue());
        }
        
        return ENavigationQueryResult::Fail;
    }
    
    // 2. Global Raycast Optimization (Cross-Chunk)
    // If we can see the target directly across multiple chunks, skip Macro Pathfinding.
    if (auto* settings = GetDefault<USVONavigationSettings>())
    {
        if (settings->DefaultRaycasterClass != nullptr)
        {
            const USVORayCaster* RayCaster = settings->DefaultRaycasterClass->GetDefaultObject<USVORayCaster>();
            
            // TraceStack handles traversing the World Partition grid.
            // It returns true if BLOCKED, so we check !TraceStack
            if (!RayCaster->TraceStack(navigation_data, start_location, end_location))
            {
                auto& path_points = navigation_path.GetPathPoints();
                path_points.Emplace(start_location);
                path_points.Emplace(end_location);
                navigation_path.MarkReady();
                return ENavigationQueryResult::Success;
            }
        }
    }

    // 3. Macro Pathfinding (Hierarchical / Multi-Chunk)
    // This is required for World Partition where start and end are in different loaded chunks.
    USVONavigationWorldSubsystem* Subsystem = World->GetSubsystem<USVONavigationWorldSubsystem>();
    if (!Subsystem) return ENavigationQueryResult::Fail;

    TArray<ASVONavigationDataChunkActor*> ChunkPath;
    // Note: FindChunkPath now verifies connectivity using FSVOPortals (Phase 2 Update)
    if (!Subsystem->FindChunkPath(start_location, end_location, ChunkPath))
    {
        UE_LOG(LogNavigation, Warning, TEXT("SVOPathFinder: Could not find macro path between chunks. Check if chunks are loaded and portals generated."));
        return ENavigationQueryResult::Fail;
    }

    if (ChunkPath.Num() == 0) return ENavigationQueryResult::Fail;

    // 4. Stitch Local Paths
    FVector CurrentStart = start_location;
    FVector CurrentEnd;
    TArray<FNavPathPoint>& AllPoints = navigation_path.GetPathPoints();
    AllPoints.Reset();
    
    // Add an initial start point
    AllPoints.Add(FNavPathPoint(start_location));

    // -- Prepare Filter Overrides --
    // We disable smoothing for intermediate segments to ensure the path hits the portals exactly.
    // We will smooth the *entire* stitched path at the end.
    
    const FSVONavigationQueryFilterImpl* OriginalImpl = static_cast<const FSVONavigationQueryFilterImpl*>(nav_query_filter->GetImplementation());
    
    const bool bOriginalSmoothPaths = OriginalImpl->QueryFilterSettings.bSmoothPaths;
    const int32 OriginalSubdivisions = OriginalImpl->QueryFilterSettings.SmoothingSubdivisions;

    TSharedRef<FNavigationQueryFilter> SegmentFilterContainer = MakeShared<FNavigationQueryFilter>();
    FSVONavigationQueryFilterImpl* SegmentImpl = new FSVONavigationQueryFilterImpl(*OriginalImpl);
    SegmentImpl->QueryFilterSettings.bSmoothPaths = false; // Disable local smoothing
    SegmentFilterContainer->SetFilterImplementation(SegmentImpl);
    
    FSharedConstNavQueryFilter SegmentFilter = SegmentFilterContainer;

    for (int32 i = 0; i < ChunkPath.Num(); ++i)
    {
        ASVONavigationDataChunkActor* CurrentChunk = ChunkPath[i];
        if (!CurrentChunk) continue;

        const FSVOVolumeNavigationData& VolumeData = CurrentChunk->GetVolumeNavigationData();

        if (i == ChunkPath.Num() - 1)
        {
            CurrentEnd = end_location;
        }
        else
        {
            ASVONavigationDataChunkActor* NextChunk = ChunkPath[i + 1];
            
            // CalculatePortalLocation uses the generated FSVOPortal data to find the exact open voxels.
            FVector IdealPortalPos = USVONavigationWorldSubsystem::CalculatePortalLocation(CurrentChunk, NextChunk, start_location, end_location);
            
            // Ensure the target portal position is actually inside a navigable node in the current volume.
            // This handles cases where the portal might be slightly offset or effectively blocked by dynamic obstacles.
            float SearchRadius = VolumeData.GetData().GetLeafNodes().GetLeafSubNodeExtent() * 4.0f; // 4 voxels tolerance
            
            if (!VolumeData.GetClosestNavigablePosition(IdealPortalPos, CurrentEnd, SearchRadius))
            {
                 // If we can't find a navigable point at the portal, the Macro path lied to us (or dynamic occlusion blocked it).
                 UE_LOG(LogNavigation, Warning, TEXT("SVOPathFinder: Blocked portal transition in chunk %s. Ideal: %s"), *CurrentChunk->GetName(), *IdealPortalPos.ToString());
                 return ENavigationQueryResult::Fail;
            }
        }

        // --- Raycast Optimization ---
        // Attempt to trace a straight line. If successful, we skip A* to avoid zigzag artifacts and improve performance.
        bool bFoundDirectPath = false;
        if (const USVONavigationSettings* Settings = GetDefault<USVONavigationSettings>())
        {
            if (Settings->DefaultRaycasterClass)
            {
                const USVORayCaster* RayCaster = Settings->DefaultRaycasterClass->GetDefaultObject<USVORayCaster>();
                
                // Trace returns true if BLOCKED, so we check !Trace
                if (!RayCaster->Trace(VolumeData, CurrentStart, CurrentEnd))
                {
                    // The path is clear! Directly add the end point.
                    // We skip adding CurrentStart because it was added by the previous iteration (or initialization).
                    AllPoints.Add(FNavPathPoint(CurrentEnd));
                    bFoundDirectPath = true;
                }
            }
        }
        // -----------------------------

        if (!bFoundDirectPath)
        {
            // Run Local A*
            const auto params = FSVOPathFindingParameters::Initialize(VolumeData, CurrentStart, CurrentEnd, *SegmentFilter);
            
            if (params.IsSet())
            {
                FSVONavigationPath SegmentPath;
                ENavigationQueryResult::Type SegmentResult = path_finder->GetPath(SegmentPath, params.GetValue());
                if (SegmentResult != ENavigationQueryResult::Success)
                {
                    UE_LOG(LogNavigation, Warning, TEXT("SVOPathFinder: Failed local path in chunk %s from %s to %s"), *CurrentChunk->GetName(), *CurrentStart.ToString(), *CurrentEnd.ToString());
                    return ENavigationQueryResult::Fail;
                }
                
                const TArray<FNavPathPoint>& SegmentPoints = SegmentPath.GetPathPoints();
                
                // Append points. Skip the first point because it duplicates the last point of the previous segment.
                for (int32 pt = 1; pt < SegmentPoints.Num(); ++pt)
                {
                    AllPoints.Add(SegmentPoints[pt]);
                }
            }
            else
            {
                 UE_LOG(LogNavigation, Warning, TEXT("SVOPathFinder: Failed to initialize path parameters in chunk %s (Start or End invalid)"), *CurrentChunk->GetName());
                 return ENavigationQueryResult::Fail;
            }
        }

        // Set up for the next iteration
        CurrentStart = CurrentEnd;
    }

    // 5. Post-Process Smoothing Sequence
    
    // A. Greedy Smoothing: Optimize node count across chunk boundaries (Stitch repair).
    // This removes the "kinks" where paths meet at portals.
    FSVOPathSmoother::SmoothPathGreedy(navigation_path, navigation_data);

    // B. Catmull-Rom Smoothing: Apply visual curves if requested by the original filter.
    // Now that the full path is stitched and optimized, we apply the curve.
    if (bOriginalSmoothPaths)
    {
        FSVOPathSmoother::SmoothPathCatmullRom(navigation_path, OriginalSubdivisions);
    }

    navigation_path.MarkReady();
    return ENavigationQueryResult::Success;
}

TSharedPtr< FSVOPathFindingAlgorithmStepper > FSVOPathFinder::GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FSharedConstNavQueryFilter & nav_query_filter )
{
    if ( const auto * path_finder = GetPathFindingAlgorithm( nav_query_filter ) )
    {
        if ( const auto * volume_navigation_data = navigation_data.GetVolumeNavigationDataContainingPoints( { start_location, end_location } ) )
        {
            const auto params = FSVOPathFindingParameters::Initialize( *volume_navigation_data, start_location, end_location, *nav_query_filter );

            if ( params.IsSet() )
            {
                return path_finder->GetDebugPathStepper( debug_infos, params.GetValue() );
            }
        }
    }

    return nullptr;
}