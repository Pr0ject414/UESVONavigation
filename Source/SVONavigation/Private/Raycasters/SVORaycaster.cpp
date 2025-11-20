#include "Raycasters/SVORayCaster.h"

#include "SVONavigationData.h"
#include "SVOVolumeNavigationData.h"
#include "Core/SVONavigationDataChunkActor.h"
#include "Grid/SVONavigationWorldSubsystem.h"

FSVORayCasterObserver_GenerateDebugInfos::FSVORayCasterObserver_GenerateDebugInfos( FSVORayCasterDebugInfos & debug_infos ) :
    DebugInfos( debug_infos )
{
}

void FSVORayCasterObserver_GenerateDebugInfos::Initialize( const FSVOVolumeNavigationData * navigation_data, const FVector from, const FVector to )
{
    DebugInfos.TraversedNodes.Reset();
    DebugInfos.TraversedLeafNodes.Reset();
    DebugInfos.TraversedLeafSubNodes.Reset();
    DebugInfos.RayCastStartLocation = from;
    DebugInfos.RayCastEndLocation = to;
    DebugInfos.NavigationData = navigation_data;
}

void FSVORayCasterObserver_GenerateDebugInfos::SetResult( const bool result )
{
    DebugInfos.Result = result;
}

void FSVORayCasterObserver_GenerateDebugInfos::AddTraversedNode( FSVONodeAddress node_address, bool is_occluded )
{
    UE_LOG( LogTemp, Warning, TEXT( "Node Address : %i - %i - %i" ), node_address.LayerIndex, node_address.NodeIndex, node_address.SubNodeIndex );
    DebugInfos.TraversedNodes.Emplace( node_address, is_occluded );
}

void FSVORayCasterObserver_GenerateDebugInfos::AddTraversedLeafSubNode( FSVONodeAddress node_address, bool is_occluded )
{
    UE_LOG( LogTemp, Warning, TEXT( "SubNode Address : %i - %i - %i" ), node_address.LayerIndex, node_address.NodeIndex, node_address.SubNodeIndex );
    DebugInfos.TraversedLeafSubNodes.Emplace( node_address, is_occluded );
}

bool USVORayCaster::Trace( const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to ) const
{
    if ( Observer.IsValid() )
    {
        Observer->Initialize( &volume_navigation_data, from, to );
    }

    const auto result = TraceInternal( volume_navigation_data, from, to );

    if ( Observer.IsValid() )
    {
        Observer->SetResult( result );
    }

    return result;
}

bool USVORayCaster::TraceStack(const ASVONavigationData& NavigationData, const FVector& From, const FVector& To) const
{
    UWorld* World = NavigationData.GetWorld();
    if (!World) return false;

    USVONavigationWorldSubsystem* Subsystem = World->GetSubsystem<USVONavigationWorldSubsystem>();
    if (!Subsystem) return false;

    FVector CurrentPos = From;
    FVector Direction = (To - From).GetSafeNormal();
    float RemainingDist = FVector::Dist(From, To);
    
    // Epsilon to push the ray slightly into the next chunk to avoid getting stuck on boundary faces

    int32 Steps = 0;
    constexpr int32 MaxSteps = 5000; // Increased significantly to handle long paths with micro-steps

    while (RemainingDist > KINDA_SMALL_NUMBER && Steps < MaxSteps)
    {
        constexpr float BoundaryPush = 5.0f;
        Steps++;

        // 1. Find current chunk
        ASVONavigationDataChunkActor* CurrentChunk = Subsystem->GetChunkAtLocation(CurrentPos);
        
        if (!CurrentChunk)
        {
            // We are in a void.
            // Try to step forward blindly to find the next chunk.
            if (constexpr float VoidTolerance = 50.0f; RemainingDist > VoidTolerance)
            {
                // Scan ahead to see if we re-enter a chunk quickly
                FVector TestPos = CurrentPos + (Direction * VoidTolerance);
                if (Subsystem->GetChunkAtLocation(TestPos))
                {
                    // We found a chunk ahead, skip the gap
                    CurrentPos = TestPos;
                    RemainingDist -= VoidTolerance;
                    continue;
                }
                else
                {
                     // Void is too large, assume blocked
                    return true; 
                }
            }
            else
            {
                // Just a tiny gap at the end of a path, assume clear
                return false;
            }
        }

        const FSVOVolumeNavigationData& VolumeData = CurrentChunk->GetVolumeNavigationData();
        const FBox& Bounds = VolumeData.GetNavigationBounds();

        // 2. Determine the exit point from this chunk
        // Calculate TMax for ray vs. AABB
        // Use safe inverse to avoid div/0
        FVector InvDir(
            FMath::IsNearlyZero(Direction.X) ? 1.e8f : 1.0f / Direction.X,
            FMath::IsNearlyZero(Direction.Y) ? 1.e8f : 1.0f / Direction.Y,
            FMath::IsNearlyZero(Direction.Z) ? 1.e8f : 1.0f / Direction.Z
        );

        FVector T0 = (Bounds.Min - CurrentPos) * InvDir;
        FVector T1 = (Bounds.Max - CurrentPos) * InvDir;
        FVector TMaxV = FVector(FMath::Max(T0.X, T1.X), FMath::Max(T0.Y, T1.Y), FMath::Max(T0.Z, T1.Z));
        
        // The distance to the boundary is the minimum of the max T values
        // Max(0) handles case where we are slightly outside due to push
        float DistToBoundary = FMath::Max(0.0f, FMath::Min3(TMaxV.X, TMaxV.Y, TMaxV.Z));
        
        // The actual segment length is the smaller of: distance to boundary OR distance to goal
        float SegmentLength = FMath::Min(DistToBoundary, RemainingDist);

        // Safety: If we are stuck on a boundary (Dist ~ 0), force a push
        if (SegmentLength <= KINDA_SMALL_NUMBER) 
        {
            // If we are at the goal, we are done
            if (RemainingDist <= KINDA_SMALL_NUMBER) break;
            
            // Otherwise, we are stuck on a seam. We MUST advance.
            // We don't trace this tiny step, just push.
            CurrentPos = CurrentPos + (Direction * BoundaryPush);
            RemainingDist -= BoundaryPush;
            continue;
        }

        FVector SegmentEnd = CurrentPos + (Direction * SegmentLength);

        // 3. Trace inside this chunk
        // TraceInternal returns true if BLOCKED (Hit)
        if (TraceInternal(VolumeData, CurrentPos, SegmentEnd))
        {
            return true; // Hit obstruction
        }

        // 4. Advance
        CurrentPos = SegmentEnd + (Direction * BoundaryPush);
        RemainingDist -= (SegmentLength + BoundaryPush);
    }

    return false; // No hit found along the entire stack
}

void USVORayCaster::SetObserver( const TSharedPtr< FSVORayCasterObserver > observer )
{
    Observer = observer;
}

bool USVORayCaster::TraceInternal( const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to ) const
{
    return false;
}

UWorld * USVORayCaster::GetWorldContext()
{
#if WITH_EDITOR
    return GEditor->GetEditorWorldContext( false ).World();
#else
    return GEngine->GetCurrentPlayWorld();
#endif
}