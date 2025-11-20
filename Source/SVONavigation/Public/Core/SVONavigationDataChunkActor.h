#pragma once

#include "CoreMinimal.h"
#include "ActorPartition/PartitionActor.h"
#include "SVOVolumeNavigationData.h"
#include "SVONavigationDataChunkActor.generated.h"

class ASVONavigationData;

/**
 * An actor responsible for holding the SVO navigation data for a specific World Partition cell.
 * Automatically loaded/unloaded by the engine's World Partition system.
 */
UCLASS(NotPlaceable, BlueprintType)
class SVONAVIGATION_API ASVONavigationDataChunkActor : public APartitionActor
{
    GENERATED_BODY()

public:
    ASVONavigationDataChunkActor(const FObjectInitializer& ObjectInitializer);

    //-- AActor Interface --
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void GetActorBounds(bool bOnlyCollidingComponents, FVector& OutOrigin, FVector& OutBoxExtent, bool bIncludeFromChildActors) const override;
    virtual void Serialize(FArchive& Ar) override;
    //-- End AActor Interface --

#if WITH_EDITOR
    virtual uint32 GetDefaultGridSize(UWorld* InWorld) const override;
#endif

    /** Accessor for the raw SVO data container */
    const FSVOVolumeNavigationData& GetVolumeNavigationData() const { return VolumeNavigationData; }
    
    /** Mutable accessor for generation */
    FSVOVolumeNavigationData& GetVolumeNavigationDataMutable() { return VolumeNavigationData; }

    /** Get the bounds of this chunk */
    const FBox& GetBounds() const { return NavigationBounds; }

    /** Set bounds (used during generation) */
    void SetBounds(const FBox& InBounds);

    /** 
     * Returns cached neighbors.
     * Direction is 0..5 (matching standard neighbor order: +X, -X, +Y, -Y, +Z, -Z)
     * Returns nullptr if no neighbor is loaded in that direction.
     */
    ASVONavigationDataChunkActor* GetNeighbor(uint8 Direction) const;

protected:
    /**
     * The actual SVO octree data structure.
     * Note: FSVOVolumeNavigationData is a struct, not a UObject, so it serializes inline.
     */
    FSVOVolumeNavigationData VolumeNavigationData;

    /** Bounds of this chunk in world space */
    UPROPERTY(VisibleAnywhere, Category = "SVONavigation")
    FBox NavigationBounds;

    /** Cached weak pointers to neighbors. Index 0-5. */
    TArray<TWeakObjectPtr<ASVONavigationDataChunkActor>> CachedNeighbors;

private:
    /** Registers this actor with the SVO Subsystem and the main Navigation Data actor */
    void RegisterWithNavigationSystem();
    
    /** Unregisters from all systems */
    void UnregisterFromNavigationSystem();

    /** Queries the subsystem to populate CachedNeighbors */
    void CacheNeighbors();
};