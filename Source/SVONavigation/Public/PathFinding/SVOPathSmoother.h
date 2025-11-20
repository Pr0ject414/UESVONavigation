#pragma once

#include "CoreMinimal.h"

struct FSVONavigationPath;
class ASVONavigationData;
struct FSVONavigationQueryFilterSettings;

/**
 * Utility class responsible for post-processing paths.
 * Handles both Optimization (Greedy Pruning) and Visuals (Catmull-Rom Splines).
 */
class SVONAVIGATION_API FSVOPathSmoother
{
public:
	/**
	 * Runs the Greedy Smoother (Line of Sight Pruning).
	 * Removes redundant nodes if a straight line exists between non-adjacent nodes.
	 */
	static void SmoothPathGreedy(FSVONavigationPath& Path, const ASVONavigationData& NavData);

	/**
	 * Runs Catmull-Rom Spline subdivision.
	 * Adds points to create a curved path passing through the original points.
	 * @param Path The path to modify.
	 * @param Subdivisions Number of points to insert between each node.
	 */
	static void SmoothPathCatmullRom(FSVONavigationPath& Path, const int32 Subdivisions);

private:
	static void PerformGreedySmoothing(FSVONavigationPath& Path, const ASVONavigationData& NavData);
};