#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Common/SVONavigationTypes.h"
#include "SVONavigationWorldSubsystem.generated.h"

class ASVONavigationDataChunkActor;

USTRUCT()
struct FSVOSpatialCell
{
    GENERATED_BODY()
    TArray<TWeakObjectPtr<ASVONavigationDataChunkActor>> Actors;
};

/**
 * Central subsystem for managing the spatial grid of loaded SVO navigation chunks.
 * Handles O(1) lookups and dynamic dirty area notifications.
 */
UCLASS()
class SVONAVIGATION_API USVONavigationWorldSubsystem : public UWorldSubsystem
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, Category = "SVONavigation")
    float CellSize = 25600.0f;

    virtual void Initialize(FSubsystemCollectionBase& Collection) override;
    virtual void Deinitialize() override;

    // -- Registry --
    void RegisterChunkActor(ASVONavigationDataChunkActor* Actor);
    void UnregisterChunkActor(ASVONavigationDataChunkActor* Actor);

    // -- Queries --
    void QueryChunksInBounds(const FBox& Bounds, TArray<ASVONavigationDataChunkActor*>& OutActors) const;
    ASVONavigationDataChunkActor* GetChunkAtLocation(const FVector& Location) const;

    // -- Pathfinding --
    
    /** 
     * Finds a sequence of connected chunks from Start to End.
     * Validates connectivity using generated Portals.
     */
    bool FindChunkPath(const FVector& StartLocation, const FVector& EndLocation, TArray<ASVONavigationDataChunkActor*>& OutChunkPath) const;

    /**
     * Calculates the ideal crossing point between two chunks.
     * Uses FSVOPortal data to find the best navigable opening.
     */
    static FVector CalculatePortalLocation(const ASVONavigationDataChunkActor* ChunkA, const ASVONavigationDataChunkActor* ChunkB, const FVector& StartPos, const FVector& EndPos);

    /**
     * Finds a valid portal connecting ChunkA to ChunkB.
     * @param ChunkA
     * @param ChunkB
     * @param OutPortal The resulting overlap region (intersection of A's portal and B's portal).
     * @return True if a valid connection exists.
     */
    static bool GetSharedPortal(const ASVONavigationDataChunkActor* ChunkA, const ASVONavigationDataChunkActor* ChunkB, FSVOPortal& OutPortal);

    // -- Dynamic Updates & Persistence --

    UFUNCTION(BlueprintCallable, Category = "SVONavigation")
    void NotifyNavigationDirty(const FBox& Bounds);

    UFUNCTION(BlueprintCallable, Category = "SVONavigation")
    void RegisterDynamicOccluder(const AActor* Occluder);

    UFUNCTION(BlueprintCallable, Category = "SVONavigation")
    void UnregisterDynamicOccluder(const AActor* Occluder);

    /** Debug helper to visualize portals in the world */
    UFUNCTION(BlueprintCallable, Category = "SVONavigation")
    void DrawDebugPortals(bool bEnable, float Duration = 0.0f);

private:
    mutable FCriticalSection Mutex;
    TMap<FIntVector, FSVOSpatialCell> Grid;

    // -- Batching State --
    FTimerHandle BatchTimerHandle;
    TSet<TWeakObjectPtr<ASVONavigationDataChunkActor>> PendingDirtyChunks;

    void ProcessDirtyChunks();

    FIntVector LocationToGridCell(const FVector& Location) const
    {
        return FIntVector(
            FMath::FloorToInt(Location.X / CellSize),
            FMath::FloorToInt(Location.Y / CellSize),
            0 
        );
    }

    void GetGridCellsForBounds(const FBox& Bounds, TArray<FIntVector>& OutCells) const;
};