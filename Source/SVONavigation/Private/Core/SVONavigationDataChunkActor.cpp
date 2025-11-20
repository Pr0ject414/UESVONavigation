#include "Core/SVONavigationDataChunkActor.h"
#include "Grid/SVONavigationWorldSubsystem.h"
#include "SVONavigationData.h"
#include "EngineUtils.h"
#include "SVOVersion.h"

ASVONavigationDataChunkActor::ASVONavigationDataChunkActor(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    SetCanBeDamaged(false);
    SetActorEnableCollision(false);
    
    NavigationBounds = FBox(EForceInit::ForceInit);
    CachedNeighbors.Init(nullptr, 6); // Pre-size for 6 directions
}

void ASVONavigationDataChunkActor::BeginPlay()
{
    Super::BeginPlay();
    RegisterWithNavigationSystem();
    
    // Cache neighbors immediately on Load
    CacheNeighbors();

    // === On-Demand Generation Logic (PIE / Development) ===
    // If we are in the editor (not a cooked build) and the data is invalid/empty, request a build.
    // This allows the "Just Press Play" workflow without running the builder commandlet.
#if WITH_EDITOR
    if (!VolumeNavigationData.GetData().IsValid())
    {
        UWorld* World = GetWorld();
        if (World && !IsRunningCommandlet())
        {
            // Find the main nav data to request a build
            for (TActorIterator<ASVONavigationData> It(World); It; ++It)
            {
                ASVONavigationData* NavData = *It;
                if (IsValid(NavData))
                {
                    NavData->RequestBuildForChunk(this);
                    break;
                }
            }
        }
    }
#endif
}

void ASVONavigationDataChunkActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    UnregisterFromNavigationSystem();
    Super::EndPlay(EndPlayReason);
}

void ASVONavigationDataChunkActor::GetActorBounds(bool bOnlyCollidingComponents, FVector& OutOrigin, FVector& OutBoxExtent, bool bIncludeFromChildActors) const
{
    NavigationBounds.GetCenterAndExtents(OutOrigin, OutBoxExtent);
}

void ASVONavigationDataChunkActor::Serialize(FArchive& Ar)
{
    Super::Serialize(Ar);

    // Serialize the actual SVO data. 
    // This enables the Builder to save the generated octree into the actor's asset.
    ESVOVersion SerializeVersion = ESVOVersion::Latest;
    Ar << SerializeVersion;
    
    VolumeNavigationData.Serialize(Ar, SerializeVersion);
}

#if WITH_EDITOR
uint32 ASVONavigationDataChunkActor::GetDefaultGridSize(UWorld* InWorld) const
{
    // Default to a reasonable grid size (e.g., 256m) for World Partition
    return 25600;
}
#endif

void ASVONavigationDataChunkActor::SetBounds(const FBox& InBounds)
{
    NavigationBounds = InBounds;
    
#if WITH_EDITOR
    // In editor, visual placement helps debugging
    SetActorLocation(InBounds.GetCenter());
#endif
}

void ASVONavigationDataChunkActor::RegisterWithNavigationSystem()
{
    UWorld* World = GetWorld();
    if (!World)
    {
        return;
    }

    // Register with the spatial subsystem
    if (USVONavigationWorldSubsystem* Subsystem = World->GetSubsystem<USVONavigationWorldSubsystem>())
    {
        Subsystem->RegisterChunkActor(this);
    }

    // Register with main navigation data for legacy support and global queries
    for (TActorIterator<ASVONavigationData> It(World); It; ++It)
    {
        ASVONavigationData* NavData = *It;
        if (IsValid(NavData))
        {
            NavData->RegisterChunkActor(this);
            break; 
        }
    }
}

void ASVONavigationDataChunkActor::UnregisterFromNavigationSystem()
{
    UWorld* World = GetWorld();
    if (!World)
    {
        return;
    }

    if (USVONavigationWorldSubsystem* Subsystem = World->GetSubsystem<USVONavigationWorldSubsystem>())
    {
        Subsystem->UnregisterChunkActor(this);
    }

    for (TActorIterator<ASVONavigationData> It(World); It; ++It)
    {
        ASVONavigationData* NavData = *It;
        if (IsValid(NavData))
        {
            NavData->UnregisterChunkActor(this);
            break;
        }
    }
}

void ASVONavigationDataChunkActor::CacheNeighbors()
{
    UWorld* World = GetWorld();
    if (!World) return;

    USVONavigationWorldSubsystem* Subsystem = World->GetSubsystem<USVONavigationWorldSubsystem>();
    if (!Subsystem) return;

    const FVector Center = NavigationBounds.GetCenter();
    const FVector Extent = NavigationBounds.GetExtent();
    
    // Small offset to poke into neighbor bounds
    // Directions: +X, -X, +Y, -Y, +Z, -Z
    const FVector Directions[] = {
        FVector(1, 0, 0), FVector(-1, 0, 0),
        FVector(0, 1, 0), FVector(0, -1, 0),
        FVector(0, 0, 1), FVector(0, 0, -1)
    };

    for (int32 i = 0; i < 6; ++i)
    {
        // Calculate a point strictly inside the theoretical neighbor
        FVector NeighborPos = Center + (Directions[i] * (Extent * 2.0f)); 
        
        // Query subsystem
        ASVONavigationDataChunkActor* Neighbor = Subsystem->GetChunkAtLocation(NeighborPos);
        CachedNeighbors[i] = Neighbor;
    }
}

ASVONavigationDataChunkActor* ASVONavigationDataChunkActor::GetNeighbor(uint8 Direction) const
{
    if (CachedNeighbors.IsValidIndex(Direction))
    {
        return CachedNeighbors[Direction].Get();
    }
    return nullptr;
}