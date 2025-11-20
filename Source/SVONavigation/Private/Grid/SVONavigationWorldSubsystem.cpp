#include "Grid/SVONavigationWorldSubsystem.h"
#include "Core/SVONavigationDataChunkActor.h"
#include "SVONavigationData.h"
#include "EngineUtils.h"
#include "NavigationSystem.h"
#include "TimerManager.h"
#include "DrawDebugHelpers.h"
#include "SVONavigationSettings.h"
#include "Raycasters/SVORayCaster.h"

void USVONavigationWorldSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);
}

void USVONavigationWorldSubsystem::Deinitialize()
{
    FScopeLock Lock(&Mutex);
    Grid.Reset();
    
    if (GetWorld())
    {
        GetWorld()->GetTimerManager().ClearTimer(BatchTimerHandle);
    }
    PendingDirtyChunks.Empty();
    
    Super::Deinitialize();
}

void USVONavigationWorldSubsystem::RegisterChunkActor(ASVONavigationDataChunkActor* Actor)
{
    if (!Actor) return;
    FScopeLock Lock(&Mutex);
    const FBox& Bounds = Actor->GetBounds();
    TArray<FIntVector> Cells;
    GetGridCellsForBounds(Bounds, Cells);
    for (const FIntVector& CellCoord : Cells)
    {
        Grid.FindOrAdd(CellCoord).Actors.Add(Actor);
    }
}

void USVONavigationWorldSubsystem::UnregisterChunkActor(ASVONavigationDataChunkActor* Actor)
{
    if (!Actor) return;
    FScopeLock Lock(&Mutex);
    for (auto& Pair : Grid)
    {
        Pair.Value.Actors.RemoveAllSwap([Actor](const TWeakObjectPtr<ASVONavigationDataChunkActor>& Ptr)
        {
            return !Ptr.IsValid() || Ptr.Get() == Actor;
        });
    }
}

void USVONavigationWorldSubsystem::QueryChunksInBounds(const FBox& Bounds, TArray<ASVONavigationDataChunkActor*>& OutActors) const
{
    if (!Bounds.IsValid) return;
    FScopeLock Lock(&Mutex);
    
    TArray<FIntVector> Cells;
    GetGridCellsForBounds(Bounds, Cells);
    TSet<ASVONavigationDataChunkActor*> UniqueActors;
    
    for (const FIntVector& CellCoord : Cells)
    {
        if (const FSVOSpatialCell* Cell = Grid.Find(CellCoord))
        {
            for (const auto& WeakPtr : Cell->Actors)
            {
                if (ASVONavigationDataChunkActor* Actor = WeakPtr.Get())
                {
                    if (Actor->GetBounds().Intersect(Bounds))
                    {
                        UniqueActors.Add(Actor);
                    }
                }
            }
        }
    }
    OutActors = UniqueActors.Array();
}

ASVONavigationDataChunkActor* USVONavigationWorldSubsystem::GetChunkAtLocation(const FVector& Location) const
{
    FScopeLock Lock(&Mutex);
    const FIntVector CellCoord = LocationToGridCell(Location);
    
    if (const FSVOSpatialCell* Cell = Grid.Find(CellCoord))
    {
        for (const auto& WeakPtr : Cell->Actors)
        {
            if (ASVONavigationDataChunkActor* Actor = WeakPtr.Get())
            {
                if (Actor->GetBounds().IsInside(Location))
                {
                    return Actor;
                }
            }
        }
    }
    return nullptr;
}

// --- Macro Pathfinding ---

struct FChunkPathNode
{
    ASVONavigationDataChunkActor* Actor = nullptr;
    float GCost = MAX_flt;
    float HCost = 0.0f;
    ASVONavigationDataChunkActor* Parent = nullptr;
    
    float FCost() const { return GCost + HCost; }
    bool operator<(const FChunkPathNode& Other) const { return FCost() < Other.FCost(); }
};

bool USVONavigationWorldSubsystem::GetSharedPortal(const ASVONavigationDataChunkActor* ChunkA, const ASVONavigationDataChunkActor* ChunkB, FSVOPortal& OutPortal)
{
    if (!ChunkA || !ChunkB) return false;

    const TArray<FSVOPortal>& PortalsA = ChunkA->GetVolumeNavigationData().GetPortals();
    const TArray<FSVOPortal>& PortalsB = ChunkB->GetVolumeNavigationData().GetPortals();
    
    for (const FSVOPortal& PA : PortalsA)
    {
        // The XOR 1 flips the last bit, effectively swapping + and - on that axis
        const uint8 RequiredDirB = PA.Direction ^ 1;
        
        for (const FSVOPortal& PB : PortalsB)
        {
            // Extremely fast integer comparison instead of logical checks
            if (PB.Direction == RequiredDirB)
            {
                // Check overlap
                // We use a small epsilon for the plane check, but intersect the face extents
                FBox BoxA = FBox::BuildAABB(PA.Location, PA.Extent);
                FBox BoxB = FBox::BuildAABB(PB.Location, PB.Extent);

                if (BoxA.Intersect(BoxB))
                {
                    // Found a connection!
                    // Return the intersection box center as the shared portal point
                    // We collapse the axis of the direction to the exact midpoint between chunks
                    FBox IntersectBox = BoxA.Overlap(BoxB);
                    
                    OutPortal.Location = IntersectBox.GetCenter();
                    OutPortal.Extent = IntersectBox.GetExtent();
                    OutPortal.Direction = PA.Direction; // Keep A's direction convention
                    return true;
                }
            }
        }
    }

    return false;
}

bool USVONavigationWorldSubsystem::FindChunkPath(const FVector& StartLocation, const FVector& EndLocation, TArray<ASVONavigationDataChunkActor*>& OutChunkPath) const
{
    FScopeLock Lock(&Mutex);
    OutChunkPath.Reset();

    ASVONavigationDataChunkActor* StartChunk = GetChunkAtLocation(StartLocation);
    ASVONavigationDataChunkActor* EndChunk = GetChunkAtLocation(EndLocation);

    if (!StartChunk || !EndChunk)
    {
        UE_LOG(LogNavigation, Warning, TEXT("FindChunkPath Failed: StartChunk=%s, EndChunk=%s (One is null)"), 
            *GetNameSafe(StartChunk), *GetNameSafe(EndChunk));
        return false;
    }

    if (StartChunk == EndChunk)
    {
        OutChunkPath.Add(StartChunk);
        return true;
    }

    // A* on the Chunk Graph
    TMap<ASVONavigationDataChunkActor*, FChunkPathNode> NodeMap;
    TSet<ASVONavigationDataChunkActor*> OpenSet;
    TSet<ASVONavigationDataChunkActor*> ClosedSet;

    FChunkPathNode StartNode;
    StartNode.Actor = StartChunk;
    StartNode.GCost = 0.0f;
    StartNode.HCost = FVector::Dist(StartChunk->GetBounds().GetCenter(), EndLocation);
    
    NodeMap.Add(StartChunk, StartNode);
    OpenSet.Add(StartChunk);

    // Directions for neighbor lookup: +X, -X, +Y, -Y, +Z, -Z
    // Must match the order used in FSVOVolumeNavigationData::GeneratePortals
    const FVector Directions[] = {
        FVector(1, 0, 0), FVector(-1, 0, 0),
        FVector(0, 1, 0), FVector(0, -1, 0),
        FVector(0, 0, 1), FVector(0, 0, -1)
    };

    while (OpenSet.Num() > 0)
    {
        ASVONavigationDataChunkActor* CurrentActor = nullptr;
        float LowestF = MAX_flt;

        for (ASVONavigationDataChunkActor* OpenActor : OpenSet)
        {
            if (const FChunkPathNode* Node = NodeMap.Find(OpenActor))
            {
                if (Node->FCost() < LowestF)
                {
                    LowestF = Node->FCost();
                    CurrentActor = OpenActor;
                }
            }
        }

        if (!CurrentActor) break;

        if (CurrentActor == EndChunk)
        {
            // Reconstruct Path
            ASVONavigationDataChunkActor* Trace = EndChunk;
            while (Trace)
            {
                OutChunkPath.Insert(Trace, 0);
                const FChunkPathNode* Node = NodeMap.Find(Trace);
                Trace = Node ? Node->Parent : nullptr;
            }
            return true;
        }

        OpenSet.Remove(CurrentActor);
        ClosedSet.Add(CurrentActor);

        // Dynamic Neighbor Expansion (Fixes Race Condition)
        const FVector Extent = CurrentActor->GetBounds().GetExtent();
        const FVector Center = CurrentActor->GetBounds().GetCenter();
        // Chunks are grid aligned, so the neighbor center is exactly 2*Extent away
        const FVector Step = Extent * 2.0f; 

        for (uint8 Dir = 0; Dir < 6; ++Dir)
        {
            // Calculate theoretical neighbor position
            // Note: We multiply component-wise to step along the correct axis
            FVector NeighborPos = Center + (Directions[Dir] * Step);
            
            // Query the Subsystem (O(1) lookup) instead of asking the Actor
            ASVONavigationDataChunkActor* Neighbor = GetChunkAtLocation(NeighborPos);
            
            if (!Neighbor || ClosedSet.Contains(Neighbor)) continue;

            // --- PORTAL VALIDATION ---
            FSVOPortal SharedPortal;
            if (!GetSharedPortal(CurrentActor, Neighbor, SharedPortal))
            {
                // Adjacent in grid, but SVO generation did not find a walkable connection.
                continue; 
            }

            float DistToNeighbor = FVector::Dist(Center, Neighbor->GetBounds().GetCenter());
            float NewGCost = NodeMap[CurrentActor].GCost + DistToNeighbor;

            bool bIsNew = !NodeMap.Contains(Neighbor);
            if (bIsNew || NewGCost < NodeMap[Neighbor].GCost)
            {
                FChunkPathNode& NeighborNode = NodeMap.FindOrAdd(Neighbor);
                NeighborNode.Actor = Neighbor;
                NeighborNode.GCost = NewGCost;
                NeighborNode.HCost = FVector::Dist(Neighbor->GetBounds().GetCenter(), EndLocation);
                NeighborNode.Parent = CurrentActor;

                OpenSet.Add(Neighbor);
            }
        }
    }
    
    UE_LOG(LogNavigation, Warning, TEXT("FindChunkPath Failed: OpenSet Empty. StartChunk=%s EndChunk=%s. Portals likely missing."), *StartChunk->GetName(), *EndChunk->GetName());
    return false;
}

FVector USVONavigationWorldSubsystem::CalculatePortalLocation(const ASVONavigationDataChunkActor* ChunkA, const ASVONavigationDataChunkActor* ChunkB, const FVector& StartPos, const FVector& EndPos)
{
    if (!ChunkA || !ChunkB) return FVector::ZeroVector;

    const TArray<FSVOPortal>& PortalsA = ChunkA->GetVolumeNavigationData().GetPortals();
    const TArray<FSVOPortal>& PortalsB = ChunkB->GetVolumeNavigationData().GetPortals();

    // Structure to evaluate potential crossing points.
    struct FPortalCandidate
    {
        FVector Point;
        double GeomDist;   // Pure Euclidean distance (Start -> Point -> End)
        double FinalScore; // Score adjusted by visibility (Lower is better)
        bool bIsCenter;    // Flag to identify if this is the safe center point
    };
    TArray<FPortalCandidate> Candidates;

    // -----------------------------------------------------------------
    // 0. Prepare Raycaster
    // -----------------------------------------------------------------
    const USVONavigationSettings* Settings = GetDefault<USVONavigationSettings>();
    const USVORayCaster* RayCaster = nullptr;
    if (Settings && Settings->DefaultRaycasterClass)
    {
        RayCaster = Settings->DefaultRaycasterClass->GetDefaultObject<USVORayCaster>();
    }

    // Helper Lambda for checking visibility with Z-Bias
    // This prevents the ray from clipping the floor and returning false positives.
    auto IsPathClear = [&](const FVector& TargetPoint) -> bool
    {
        if (!RayCaster) return true; // Assume clear if no raycaster configured

        // LIFT the trace by 50 units (approx half-agent height) to avoid hitting the navmesh floor
        const FVector VerticalOffset(0.0f, 0.0f, 50.0f);
        
        const FVector TraceStart = StartPos + VerticalOffset;
        const FVector TraceEnd = EndPos + VerticalOffset;
        const FVector TraceTarget = TargetPoint + VerticalOffset;

        const auto& VolA = ChunkA->GetVolumeNavigationData();
        const auto& VolB = ChunkB->GetVolumeNavigationData();

        // Trace Start -> Portal -> End
        // RayCaster->Trace returns true if BLOCKED, so we return false if it hits.
        if (RayCaster->Trace(VolA, TraceStart, TraceTarget)) return false;
        if (RayCaster->Trace(VolB, TraceTarget, TraceEnd)) return false;

        return true;
    };

    // -----------------------------------------------------------------
    // 1. Gather Candidates from ALL overlapping portals
    // -----------------------------------------------------------------
    for (const FSVOPortal& PA : PortalsA)
    {
        // Find the required opposite direction index (0<->1, 2<->3, 4<->5)
        const uint8 RequiredDirB = PA.Direction ^ 1;

        for (const FSVOPortal& PB : PortalsB)
        {
            if (PB.Direction == RequiredDirB)
            {
                FBox BoxA = FBox::BuildAABB(PA.Location, PA.Extent);
                FBox BoxB = FBox::BuildAABB(PB.Location, PB.Extent);

                // Expand by a small tolerance to detect portals that are touching but not mathematically overlapping due to float precision.
                if (BoxA.ExpandBy(5.0f).Intersect(BoxB))
                {
                    // Calculate the actual intersection box.
                    FBox SharedBox = BoxA.Overlap(BoxB.ExpandBy(5.0f));
                    
                    // --- A. Geometric Ideal (Shortest Path) ---
                    // This is where the straight line pierces the portal. Usually hugs corners.
                    FVector IdealPoint;
                    FVector HitPoint, HitNormal;
                    float HitTime;

                    // Check if the straight line actually passes through the portal
                    if (FMath::LineExtentBoxIntersection(SharedBox, StartPos, EndPos, FVector::ZeroVector, HitPoint, HitNormal, HitTime))
                    {
                        IdealPoint = HitPoint;
                    }
                    else
                    {
                        // If the line misses the portal, clamp it to the nearest point on the portal surface.
                        FVector ClosestOnLine = FMath::ClosestPointOnSegment(SharedBox.GetCenter(), StartPos, EndPos);
                        IdealPoint = SharedBox.GetClosestPointTo(ClosestOnLine);
                    }

                    // --- B. Geometric Center (Safest Path) ---
                    FVector CenterPoint = SharedBox.GetCenter();

                    // --- C. "Nudge" Search (Smart Correction) ---
                    // Instead of just picking Ideal or Center, we scan the line between them.
                    // If the Ideal point is blocked (e.g., by a wall corner), we step towards the center
                    // until we find the first valid opening. This minimizes the "Arc" effect.
                    
                    FVector BestCandidate = IdealPoint;
                    bool bFoundValid = false;

                    // Steps: 0.0 (Ideal), 0.2, 0.4, 0.6, 0.8, 1.0 (Center)
                    const int32 Steps = 5; 
                    for (int32 i = 0; i <= Steps; ++i)
                    {
                        float Alpha = (float)i / (float)Steps;
                        FVector TestPoint = FMath::Lerp(IdealPoint, CenterPoint, Alpha);

                        if (IsPathClear(TestPoint))
                        {
                            BestCandidate = TestPoint;
                            bFoundValid = true;
                            
                            if (i > 0) 
                            {
                                // Debug log to confirm "Smart Nudge" is working
                                UE_LOG(LogNavigation, Verbose, TEXT("SVO Smart Portal: Nudged entry point by %.2f%% to clear obstacle."), Alpha * 100.0f);
                            }
                            break; // Stop as soon as we find a clear path to keep it as straight as possible
                        }
                    }

                    // --- Calculate Final Score ---
                    double Dist = FVector::Dist(StartPos, BestCandidate) + FVector::Dist(BestCandidate, EndPos);
                    double FinalScore = Dist;

                    // If we nudged all the way to the center, and it's STILL blocked, 
                    // apply a massive penalty so the pathfinder might choose a different portal entirely.
                    if (!bFoundValid)
                    {
                        constexpr double OcclusionPenaltyMult = 10.0;
                        FinalScore *= OcclusionPenaltyMult;
                        
                        // Warn if we are forced to pick a blocked path
                        UE_LOG(LogNavigation, Warning, TEXT("SVO Portal Blocked! No valid path found through portal at %s"), *SharedBox.GetCenter().ToString());
                    }

                    // Add to the candidates list
                    Candidates.Add({ BestCandidate, Dist, FinalScore, false });
                }
            }
        }
    }

    // Fallback: If no valid portals were found (rare, implies chunks aren't connected properly),
    // return the geometric center of the chunk boundary to prevent a crash.
    if (Candidates.Num() == 0)
    {
        return (ChunkA->GetBounds().GetCenter() + ChunkB->GetBounds().GetCenter()) * 0.5f;
    }

    // -----------------------------------------------------------------
    // 2. Pick Best Score
    // -----------------------------------------------------------------
    // Sort so the lowest score (the shortest visible path) is first.
    Candidates.Sort([](const FPortalCandidate& A, const FPortalCandidate& B) {
        return A.FinalScore < B.FinalScore;
    });

    return Candidates[0].Point;
}

void USVONavigationWorldSubsystem::NotifyNavigationDirty(const FBox& Bounds)
{
    if (!Bounds.IsValid) return;

    // 1. Identify affected chunks
    TArray<ASVONavigationDataChunkActor*> AffectedChunks;
    QueryChunksInBounds(Bounds, AffectedChunks);

    if (AffectedChunks.Num() == 0)
    {
        return;
    }

    // 2. Add to batch
    for (ASVONavigationDataChunkActor* Chunk : AffectedChunks)
    {
        if (Chunk)
        {
            PendingDirtyChunks.Add(Chunk);
        }
    }

    // 3. Reset/Start Timer (Debounce)
    if (UWorld* World = GetWorld())
    {
        World->GetTimerManager().ClearTimer(BatchTimerHandle);
        World->GetTimerManager().SetTimer(BatchTimerHandle, this, &USVONavigationWorldSubsystem::ProcessDirtyChunks, 0.1f, false);
    }
}

void USVONavigationWorldSubsystem::RegisterDynamicOccluder(const AActor* Occluder)
{
    if (!Occluder) return;

    ASVONavigationData* NavData = nullptr;
    
    // FIXED: Removed unreachable loop
    TActorIterator<ASVONavigationData> It(GetWorld());
    if (It)
    {
        NavData = *It;
    }

    if (NavData)
    {
        NavData->RegisterDynamicOccluder(Occluder);
    }
}

void USVONavigationWorldSubsystem::UnregisterDynamicOccluder(const AActor* Occluder)
{
    if (!Occluder) return;

    ASVONavigationData* NavData = nullptr;
    
    TActorIterator<ASVONavigationData> It(GetWorld());
    if (It)
    {
        NavData = *It;
    }

    if (NavData)
    {
        NavData->UnregisterDynamicOccluder(Occluder);
    }
}

void USVONavigationWorldSubsystem::ProcessDirtyChunks()
{
    if (PendingDirtyChunks.Num() == 0) return;

    ASVONavigationData* NavData = nullptr;
    
    TActorIterator<ASVONavigationData> It(GetWorld());
    if (It)
    {
        NavData = *It;
    }

    if (!NavData)
    {
        PendingDirtyChunks.Empty();
        return;
    }

    UE_LOG(LogNavigation, Log, TEXT("SVONavigationWorldSubsystem: Processing batch update for %d dirty chunks"), PendingDirtyChunks.Num());

    for (const TWeakObjectPtr<ASVONavigationDataChunkActor>& WeakChunk : PendingDirtyChunks)
    {
        if (ASVONavigationDataChunkActor* Chunk = WeakChunk.Get())
        {
            NavData->RequestBuildForChunk(Chunk);
        }
    }

    PendingDirtyChunks.Empty();
}

void USVONavigationWorldSubsystem::GetGridCellsForBounds(const FBox& Bounds, TArray<FIntVector>& OutCells) const
{
    if (!Bounds.IsValid) return;
    const FIntVector MinCell = LocationToGridCell(Bounds.Min);
    const FIntVector MaxCell = LocationToGridCell(Bounds.Max);
    for (int32 X = MinCell.X; X <= MaxCell.X; ++X)
    {
        for (int32 Y = MinCell.Y; Y <= MaxCell.Y; ++Y)
        {
            OutCells.Add(FIntVector(X, Y, 0));
        }
    }
}

void USVONavigationWorldSubsystem::DrawDebugPortals(bool bEnable, float Duration)
{
#if ENABLE_DRAW_DEBUG
    UWorld* World = GetWorld();
    if (!World) return;

    if (!bEnable)
    {
        FlushDebugStrings(World);
        FlushPersistentDebugLines(World);
        return;
    }

    FScopeLock Lock(&Mutex);
    
    // Iterate all loaded actors in the grid
    for (const auto& Elem : Grid)
    {
        for (const auto& WeakPtr : Elem.Value.Actors)
        {
            if (ASVONavigationDataChunkActor* Chunk = WeakPtr.Get())
            {
                const TArray<FSVOPortal>& Portals = Chunk->GetVolumeNavigationData().GetPortals();
                for (const FSVOPortal& Portal : Portals)
                {
                    FColor Color = FColor::Cyan;
                    DrawDebugBox(World, Portal.Location, Portal.Extent, FQuat::Identity, Color, false, Duration, 0, 5.0f);
                }
            }
        }
    }
#endif
}