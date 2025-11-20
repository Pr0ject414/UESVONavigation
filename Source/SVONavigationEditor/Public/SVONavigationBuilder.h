#pragma once

#include "CoreMinimal.h"
#include "WorldPartition/WorldPartitionBuilder.h"
#include "SVONavigationBuilder.generated.h"

/**
 * World Partition Builder for generating SVO Navigation Data.
 * 
 * Usage: 
 * Run via commandlet to generate and save SVO data into World Partition cells.
 * This ensures that when cells are loaded at runtime, they come with pre-computed "pristine" navigation data.
 * 
 * Command:
 * UnrealEditor.exe ProjectName -run=WorldPartitionBuilder -Builder=SVONavigationBuilder -AllowCommandletRendering
 */
UCLASS()
class SVONAVIGATIONEDITOR_API USVONavigationBuilder : public UWorldPartitionBuilder
{
	GENERATED_BODY()

public:
	USVONavigationBuilder(const FObjectInitializer& ObjectInitializer);

	//-- UWorldPartitionBuilder Interface --
	virtual bool RequiresCommandletRendering() const override { return true; }
	virtual ELoadingMode GetLoadingMode() const override { return ELoadingMode::IterativeCells; }
	virtual bool RunInternal(UWorld* World, const FCellInfo& CellInfo, FPackageSourceControlHelper& PackageHelper) override;
	//-- End UWorldPartitionBuilder Interface --
};