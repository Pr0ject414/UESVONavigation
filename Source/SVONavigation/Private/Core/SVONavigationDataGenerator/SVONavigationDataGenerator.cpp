#include "SVONavigationDataGenerator.h"
#include "SVONavigationData.h"
#include "NavigationSystem.h"
#include "SVONavigationSettings.h"
#include "Core/SVONavigationDataChunkActor.h"
#include "Grid/SVONavigationWorldSubsystem.h"
#include "Kismet/GameplayStatics.h"

FSVOVolumeNavigationDataGenerator::FSVOVolumeNavigationDataGenerator( FSVONavigationDataGenerator & navigation_data_generator, const FBox & volume_bounds ) :
	ParentGenerator( navigation_data_generator ),
	BoundsNavigationData(),
	VolumeBounds( volume_bounds )
{
	NavDataConfig = navigation_data_generator.GetOwner()->GetConfig();
}

bool FSVOVolumeNavigationDataGenerator::DoWork()
{
	FSVOVolumeNavigationDataGenerationSettings generation_settings;
	generation_settings.GenerationSettings = ParentGenerator.GetGenerationSettings();
	generation_settings.World = ParentGenerator.GetWorld();
	generation_settings.VoxelExtent = NavDataConfig.AgentRadius * 2.0f;

	BoundsNavigationData.GenerateNavigationData( VolumeBounds, generation_settings );

	return true;
}

FSVONavigationDataGenerator::FSVONavigationDataGenerator( ASVONavigationData & navigation_data ) :
	NavigationData( navigation_data ),
	MaximumGeneratorTaskCount( 2 ),
	IsInitialized( false )
{
}

void FSVONavigationDataGenerator::Init()
{
	GenerationSettings = NavigationData.GenerationSettings;

	UpdateNavigationBounds();

	///** setup maximum number of active tile generator*/
	const int32 worker_threads_count = FTaskGraphInterface::Get().GetNumWorkerThreads();
	MaximumGeneratorTaskCount = FMath::Min( FMath::Max( worker_threads_count * 2, 1 ), NavigationData.MaxSimultaneousBoxGenerationJobsCount );
	UE_LOG( LogNavigation, Log, TEXT( "Using max of %d workers to build SVO navigation." ), MaximumGeneratorTaskCount );
}

TArray<FBox> FSVONavigationDataGenerator::PartitionVolume(const FBox& OriginalVolume) const
{
    TArray<FBox> Partitions;
    
    UWorld* World = GetWorld();
    if (!World) return Partitions;

    // 1. Get Grid Settings
    float CellSize = 25600.0f; // Default fallback
    float FixedMinZ = -1000.0f;
    float FixedMaxZ = 5000.0f;

    // Try to get from Settings (Best Practice)
    if (const USVONavigationSettings* Settings = GetDefault<USVONavigationSettings>())
    {
        FixedMinZ = Settings->WorldPartitionMinZ;
        FixedMaxZ = Settings->WorldPartitionMaxZ;
        
        // Optionally, if you add CellSize to settings later, read it here too
    }
    // Fallback to Subsystem if you prefer keeping CellSize dynamic there
    else if (const USVONavigationWorldSubsystem* Subsystem = World->GetSubsystem<USVONavigationWorldSubsystem>())
    {
        CellSize = Subsystem->CellSize;
    }

    const FVector Min = OriginalVolume.Min;
    const FVector Max = OriginalVolume.Max;

    const int32 StartX = FMath::FloorToInt(Min.X / CellSize);
    const int32 StartY = FMath::FloorToInt(Min.Y / CellSize);
    const int32 EndX = FMath::FloorToInt(Max.X / CellSize);
    const int32 EndY = FMath::FloorToInt(Max.Y / CellSize);

    for (int32 X = StartX; X <= EndX; X++)
    {
        for (int32 Y = StartY; Y <= EndY; Y++)
        {
            const float CellMinX = X * CellSize;
            const float CellMinY = Y * CellSize;
            const float CellMaxX = (X + 1) * CellSize;
            const float CellMaxY = (Y + 1) * CellSize;

            // 2. Apply Fixed Z Bounds
            // We ignore the Input Z (OriginalVolume.Min.Z) because automatic updates 
            // from tile spawns pass "Flat" bounds (Z=0 to Z=0). 
            // We FORCE the partition to cover the full navigable height.
            FBox CellBox(
                FVector(CellMinX, CellMinY, FixedMinZ),
                FVector(CellMaxX, CellMaxY, FixedMaxZ)
            );

            // Only add if it actually intersects 2D (we assume Z always intersects)
            // We check 2D intersection manually to avoid Z-fighting with the flat inputs
            const FBox InputBounds2D(FVector(Min.X, Min.Y, 0), FVector(Max.X, Max.Y, 1));
            const FBox CellBounds2D(FVector(CellMinX, CellMinY, 0), FVector(CellMaxX, CellMaxY, 1));

            if (CellBounds2D.Intersect(InputBounds2D))
            {
                Partitions.Add(CellBox);
            }
        }
    }

    return Partitions;
}

ASVONavigationDataChunkActor* FSVONavigationDataGenerator::SpawnOrUpdateChunkActor(const FBox& ChunkBounds, const FSVOVolumeNavigationData& NavData)
{
    UWorld* World = GetWorld();
    if (!World) return nullptr;

    USVONavigationWorldSubsystem* Subsystem = World->GetSubsystem<USVONavigationWorldSubsystem>();
    
    // 1. Check if Actor already exists (via Subsystem O(1) lookup)
    if (Subsystem)
    {
        // Check center of bounds
        if (ASVONavigationDataChunkActor* Existing = Subsystem->GetChunkAtLocation(ChunkBounds.GetCenter()))
        {
            Existing->GetVolumeNavigationDataMutable() = NavData;
            return Existing;
        }
    }

    // 2. Spawn new Actor
    FActorSpawnParameters SpawnParams;
    SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    SpawnParams.ObjectFlags = RF_Transient; // Don't save these if we are just playing in the editor

    ASVONavigationDataChunkActor* NewActor = World->SpawnActorDeferred<ASVONavigationDataChunkActor>(
        ASVONavigationDataChunkActor::StaticClass(),
        FTransform(ChunkBounds.GetCenter()),
        nullptr,
        nullptr,
        ESpawnActorCollisionHandlingMethod::AlwaysSpawn
    );

    if (NewActor)
    {
        NewActor->SetBounds(ChunkBounds);
        NewActor->GetVolumeNavigationDataMutable() = NavData;
        
        UGameplayStatics::FinishSpawningActor(NewActor, FTransform(ChunkBounds.GetCenter()));
        
        UE_LOG(LogNavigation, Log, TEXT("Spawned new SVO Chunk Actor at %s"), *ChunkBounds.GetCenter().ToString());
    }

    return NewActor;
}