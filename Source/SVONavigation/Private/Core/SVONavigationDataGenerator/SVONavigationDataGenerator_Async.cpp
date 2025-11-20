#include "SVONavigationDataGenerator.h"
#include "SVONavigationData.h"
#include "Core/SVONavigationDataChunkActor.h"
#include "NavigationSystem.h"

void FSVONavigationDataGenerator::EnsureBuildCompletion()
{
    const bool had_tasks = GetNumRemaningBuildTasks() > 0;

    do
    {
        const int32 tasks_to_process_count = MaximumGeneratorTaskCount - RunningBoundsDataGenerationElements.Num();
        ProcessAsyncTasks( tasks_to_process_count );

        // Block until tasks are finished
        for ( auto & element : RunningBoundsDataGenerationElements )
        {
            element.AsyncTask->EnsureCompletion();
        }
    } while ( GetNumRemaningBuildTasks() > 0 );

    if ( had_tasks )
    {
        NavigationData.RequestDrawingUpdate();
    }
}

void FSVONavigationDataGenerator::CancelBuild()
{
    PendingBoundsDataGenerationElements.Empty();

    for ( auto & element : RunningBoundsDataGenerationElements )
    {
        if ( element.AsyncTask )
        {
            element.AsyncTask->EnsureCompletion();
            delete element.AsyncTask;
            element.AsyncTask = nullptr;
        }
    }

    RunningBoundsDataGenerationElements.Empty();
}

void FSVONavigationDataGenerator::TickAsyncBuild( float delta_seconds )
{
    const UNavigationSystemV1 * navigation_system = FNavigationSystem::GetCurrent< UNavigationSystemV1 >( GetWorld() );
    if ( !ensureMsgf( navigation_system != nullptr, TEXT( "FSVONavigationDataGenerator can't find valid navigation system: Owner=[%s] World=[%s]" ), *GetFullNameSafe( GetOwner() ), *GetFullNameSafe( GetWorld() ) ) )
    {
        return;
    }

    const int32 running_tasks_count = navigation_system->GetNumRunningBuildTasks();
    const int32 tasks_to_submit_count = MaximumGeneratorTaskCount - running_tasks_count;

    const auto finished_boxes = ProcessAsyncTasks( tasks_to_submit_count );

    if ( finished_boxes.Num() > 0 )
    {
        NavigationData.OnNavigationDataUpdatedInBounds( finished_boxes );
        NavigationData.RequestDrawingUpdate();
    }
}

TArray< FBox > FSVONavigationDataGenerator::ProcessAsyncTasks( const int32 task_to_process_count )
{
    // 1. Submit new tasks
    const bool has_tasks_at_start = GetNumRemaningBuildTasks() > 0;
    int32 processed_tasks_count = 0;

    for ( int32 element_index = PendingBoundsDataGenerationElements.Num() - 1; element_index >= 0 && processed_tasks_count < task_to_process_count; element_index-- )
    {
        FPendingBoundsDataGenerationElement & PendingElement = PendingBoundsDataGenerationElements[ element_index ];
        FRunningBoundsDataGenerationElement running_element( PendingElement.VolumeBounds );

        if ( RunningBoundsDataGenerationElements.Contains( running_element ) )
        {
            continue;
        }

        // Create the task to run on a background thread
        TUniquePtr< FSVOBoxGeneratorTask > task = MakeUnique< FSVOBoxGeneratorTask >( CreateBoxNavigationGenerator( PendingElement.VolumeBounds ) );

        running_element.AsyncTask = task.Release();
        running_element.AsyncTask->StartBackgroundTask();

        RunningBoundsDataGenerationElements.Add( running_element );

        PendingBoundsDataGenerationElements.RemoveAt( element_index, 1, EAllowShrinking::No );
        processed_tasks_count++;
    }

    if ( processed_tasks_count > 0 && PendingBoundsDataGenerationElements.Num() == 0 )
    {
        PendingBoundsDataGenerationElements.Empty( 64 );
    }

    // 2. Harvest finished tasks
    // This function is always called on the GameThread by TickAsyncBuild, so it is safe to access Actors.
    TArray< FBox > finished_boxes;

    for ( int32 index = RunningBoundsDataGenerationElements.Num() - 1; index >= 0; --index )
    {
        // QUICK_SCOPE_CYCLE_COUNTER( STAT_RecastNavMeshGenerator_ProcessTileTasks_FinishedTasks );

        FRunningBoundsDataGenerationElement & element = RunningBoundsDataGenerationElements[ index ];
        check( element.AsyncTask != nullptr );

        if ( !element.AsyncTask->IsDone() )
        {
            continue;
        }

        if ( !element.ShouldDiscard )
        {
            auto & box_generator = *element.AsyncTask->GetTask().BoxNavigationDataGenerator;
            
            // Retrieve the data generated on the background thread
            FSVOVolumeNavigationData GeneratedData = box_generator.GetBoundsNavigationData();

            // ROUTING LOGIC:
            // Determine if this data belongs to a World Partition Chunk Actor or the legacy monolithic array.
            bool bHandledByChunkActor = false;
            
            // First, try to update existing actors (O(N) scan over loaded actors)
            const TArray<TObjectPtr<ASVONavigationDataChunkActor>>& Actors = NavigationData.GetChunkActors();
            
            for ( ASVONavigationDataChunkActor* ChunkActor : Actors )
            {
                if ( !ChunkActor ) continue;

                const FBox ActorBounds = ChunkActor->GetBounds();
                const FVector ActorCenter = ActorBounds.GetCenter();
                const FVector GeneratedCenter = element.VolumeBounds.GetCenter();

                // Check if centers match closely
                if ( ActorCenter.Equals(GeneratedCenter, 10.0f) ) // 10cm tolerance
                {
                    UE_LOG(LogNavigation, Log, TEXT("FSVONavigationDataGenerator: Updating existing Chunk Actor: %s"), *ChunkActor->GetName());
                    ChunkActor->GetVolumeNavigationDataMutable() = MoveTemp(GeneratedData);
                    bHandledByChunkActor = true;
                    break;
                }
            }

            // If no existing actor handled it...
            if (!bHandledByChunkActor)
            {
                UWorld* World = GetWorld();
                // In World Partition, we MUST have actors for the subsystem to find them.
                if (World && World->IsPartitionedWorld())
                {
                    // Spawn a new one!
                    SpawnOrUpdateChunkActor(element.VolumeBounds, GeneratedData);
                    bHandledByChunkActor = true; 
                }
                else
                {
                    // Legacy monolithic fallback
                    NavigationData.AddVolumeNavigationData( MoveTemp(GeneratedData) );
                }
            }

            finished_boxes.Emplace( MoveTemp( element.VolumeBounds ) );
        }

        delete element.AsyncTask;
        element.AsyncTask = nullptr;
        RunningBoundsDataGenerationElements.RemoveAtSwap( index, 1, EAllowShrinking::No);
    }

    const bool has_tasks_at_end = GetNumRemaningBuildTasks() > 0;
    if ( has_tasks_at_start && !has_tasks_at_end )
    {
        // QUICK_SCOPE_CYCLE_COUNTER( STAT_RecastNavMeshGenerator_OnNavMeshGenerationFinished );
        NavigationData.OnNavigationDataGenerationFinished();
    }

    return finished_boxes;
}

TSharedRef< FSVOVolumeNavigationDataGenerator > FSVONavigationDataGenerator::CreateBoxNavigationGenerator( const FBox & box )
{
    // SCOPE_CYCLE_COUNTER(STAT_SVONavigation_CreateBoxNavigationGenerator);

    TSharedRef< FSVOVolumeNavigationDataGenerator > box_navigation_data_generator = MakeShareable( new FSVOVolumeNavigationDataGenerator( *this, box ) );
    return box_navigation_data_generator;
}