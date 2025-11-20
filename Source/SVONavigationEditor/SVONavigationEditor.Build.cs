using UnrealBuildTool;

public class SVONavigationEditor : ModuleRules
{
	public SVONavigationEditor(ReadOnlyTargetRules target) : base(target) {

		PublicDependencyModuleNames.AddRange([
			"Core", 
			"CoreUObject", 
			"Engine",  
			"SVONavigation", 
			"InputCore",
			"UnrealEd"
		]);

		PrivateDependencyModuleNames.AddRange([
			"Slate", 
			"SlateCore", 
			"PropertyEditor", 
			"EditorStyle", 
			"GraphEditor", 
			"BlueprintGraph",
			"SourceControl" // Required for PackageHelper
		]);

		PrivateIncludePaths.AddRange([
			"SVONavigationEditor/Private"
		]);

		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	}
};