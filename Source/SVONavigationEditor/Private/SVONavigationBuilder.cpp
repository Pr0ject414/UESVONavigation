#include "SVONavigationBuilder.h"

#include "SVONavigationData.h"
#include "Core/SVONavigationDataChunkActor.h"
#include "SVONavigationDataGenerator.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "NavigationSystem.h"

USVONavigationBuilder::USVONavigationBuilder(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

bool USVONavigationBuilder::RunInternal(UWorld* World, const FCellInfo& CellInfo, FPackageSourceControlHelper& PackageHelper)
{
	if (!World)
	{
		return false;
	}

	// 1. Find the Navigation Data Actor
	// Note: In a partitioned world, the NavData actor might be in the persistent level or streamed. 
	// We iterate to find it in the current context.
	ASVONavigationData* NavData = nullptr;
	for (TActorIterator<ASVONavigationData> It(World); It; ++It)
	{
		NavData = *It;
		break;
	}

	if (!NavData)
	{
		// It is possible the NavData actor itself isn't loaded in this cell context if it's not partitioned.
		// However, we need it to run the generator.
		UE_LOG(LogNavigation, Warning, TEXT("SVONavigationBuilder: No ASVONavigationData found in world context. Skipping generation."));
		return true; 
	}

	// Ensure the generator is initialized
	NavData->ConditionalConstructGenerator();
	FSVONavigationDataGenerator* Generator = static_cast<FSVONavigationDataGenerator*>(NavData->GetGenerator());
	if (!Generator)
	{
		UE_LOG(LogNavigation, Error, TEXT("SVONavigationBuilder: Failed to initialize Navigation Generator."));
		return false;
	}

	// 2. Find Chunk Actors loaded in this cell context
	TArray<ASVONavigationDataChunkActor*> ChunksToBuild;
	for (TActorIterator<ASVONavigationDataChunkActor> It(World); It; ++It)
	{
		ASVONavigationDataChunkActor* Chunk = *It;
		// Only build chunks that intersect the current cell bounds being processed
		// This prevents rebuilding the whole world for every cell iteration
		if (Chunk && Chunk->GetBounds().Intersect(CellInfo.EditorBounds))
		{
			ChunksToBuild.Add(Chunk);
		}
	}

	if (ChunksToBuild.Num() == 0)
	{
		return true; // Nothing to do in this cell
	}

	UE_LOG(LogNavigation, Display, TEXT("SVONavigationBuilder: Building %d chunks for cell bounds %s"), 
		ChunksToBuild.Num(), *CellInfo.EditorBounds.ToString());

	// 3. Execute Build
	bool bSuccess = true;

	for (ASVONavigationDataChunkActor* Chunk : ChunksToBuild)
	{
		// Configure generator for this specific chunk bound
		// We use the synchronous build approach here to ensure data is ready before saving
		Generator->RebuildBounds({ Chunk->GetBounds() });
		Generator->EnsureBuildCompletion();

		// IMPORTANT: Mark the package dirty so the builder knows to save it
		if (UPackage* Package = Chunk->GetExternalPackage())
		{
			Package->MarkPackageDirty();
			
			// In World Partition Builder, we usually delegate saving to the PackageHelper or 
			// simply mark dirty and let the iterative process handle it. 
			// However, checking out files is good practice.
			if (!PackageHelper.Checkout(Package))
			{
				UE_LOG(LogNavigation, Error, TEXT("SVONavigationBuilder: Failed to checkout chunk package %s"), *Package->GetName());
				bSuccess = false;
				continue;
			}
		}
		else
		{
			// Fallback for non-external actors (though WP actors should be external)
			Chunk->MarkPackageDirty();
		}
	}

	return bSuccess;
}