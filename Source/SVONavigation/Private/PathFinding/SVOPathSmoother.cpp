#include "PathFinding/SVOPathSmoother.h"

#include "PathFinding/SVONavigationPath.h"
#include "SVONavigationData.h"
#include "Raycasters/SVORayCaster.h"
#include "Raycasters/SVORaycaster_OctreeTraversal.h"
#include "SVONavigationSettings.h"

void FSVOPathSmoother::SmoothPathGreedy(FSVONavigationPath& Path, const ASVONavigationData& NavData)
{
	TArray<FNavPathPoint>& Points = Path.GetPathPoints();
	if (Points.Num() < 3)
	{
		return;
	}

	// Check global settings
	const USVONavigationSettings* Settings = GetDefault<USVONavigationSettings>();
	if (Settings && !Settings->bSmoothPaths)
	{
		return;
	}

	PerformGreedySmoothing(Path, NavData);
}

// Helper for Catmull-Rom
namespace
{
	float GetT(float t, float alpha, const FVector& p0, const FVector& p1)
	{
		const auto d = p1 - p0;
		const auto a = d | d; // Dot product
		const auto b = FMath::Pow(a, alpha * .5f);
		return (b + t);
	}

	FVector GetPoint(const FVector& p0, const FVector& p1, const FVector& p2, const FVector& p3, float t /* between 0 and 1 */, float alpha = .5f)
	{
		constexpr auto t0 = 0.0f;
		const auto t1 = GetT(t0, alpha, p0, p1);
		const auto t2 = GetT(t1, alpha, p1, p2);
		const auto t3 = GetT(t2, alpha, p2, p3);
		t = FMath::Lerp(t1, t2, t);
		const auto a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1;
		const auto a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2;
		const auto a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3;
		const auto b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0 ) / (t2 - t0) * a2;
		const auto b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3;
		const auto c = (t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2;
		return c;
	}
}

void FSVOPathSmoother::SmoothPathCatmullRom(FSVONavigationPath& Path, const int32 Subdivisions)
{
	if (Subdivisions <= 0) return;

	auto& OldPoints = Path.GetPathPoints();
	if (OldPoints.Num() < 2) return;

	// We need to copy old points/costs because we will rewrite the array
	TArray<FNavPathPoint> SourcePoints = OldPoints;
	TArray<float> SourceCosts = Path.GetPathPointCosts();

	// Prepare new arrays
	auto& NewPoints = Path.GetPathPoints();
	auto& NewCosts = Path.GetPathPointCosts();
	
	// Calculate simplified size estimation
	const int32 EstimatedSize = (SourcePoints.Num() - 1) * Subdivisions + SourcePoints.Num();
	NewPoints.Reset(EstimatedSize);
	NewCosts.Reset(EstimatedSize);

	// Add phantom control points at start and end for spline continuity
	SourcePoints.Insert(FNavPathPoint(2 * SourcePoints[0].Location - SourcePoints[1].Location), 0);
	SourcePoints.Emplace(FNavPathPoint(2 * SourcePoints.Last().Location - SourcePoints.Last(1).Location));
	
	// Adjust costs array to match size (though we don't strictly use cost for phantom points)
	if (SourceCosts.Num() > 0)
	{
		SourceCosts.Insert(0.0f, 0);
		SourceCosts.Emplace(0.0f);
	}

	// Loop through original segments
	// SourcePoints now has Size + 2 elements. Real points start at index 1.
	for (int32 index = 1; index < SourcePoints.Num() - 2; ++index)
	{
		// Always add the start point of the segment
		NewPoints.Emplace(SourcePoints[index].Location);
		if (SourceCosts.IsValidIndex(index)) NewCosts.Add(SourceCosts[index]);

		for (int32 alpha = 1; alpha < Subdivisions; ++alpha)
		{
			FVector InterpPos = GetPoint(
				SourcePoints[index - 1].Location,
				SourcePoints[index].Location,
				SourcePoints[index + 1].Location,
				SourcePoints[index + 2].Location,
				static_cast<float>(alpha) / static_cast<float>(Subdivisions)
			);

			NewPoints.Emplace(InterpPos);
			
			// Distribute cost evenly (approximation)
			if (SourceCosts.IsValidIndex(index))
				NewCosts.Add(SourceCosts[index] / Subdivisions);
			else
				NewCosts.Add(0.0f);
		}
	}

	// Add the final point
	NewPoints.Emplace(SourcePoints[SourcePoints.Num() - 2].Location);
	if (SourceCosts.IsValidIndex(SourcePoints.Num() - 2))
	{
		NewCosts.Add(SourceCosts[SourcePoints.Num() - 2]);
	}
}

void FSVOPathSmoother::PerformGreedySmoothing(FSVONavigationPath& Path, const ASVONavigationData& NavData)
{
    TArray<FNavPathPoint>& Points = Path.GetPathPoints();
    TArray<float>& Costs = Path.GetPathPointCosts(); // Note: We must keep this synced

    if (Points.Num() < 3) return;

    // 1. Optimization: Avoid NewObject allocation. 
    // Use the Class Default Object (CDO) since we aren't storing state/observers in the raycaster for this operation.
    const USVORayCaster* RayCasterCDO = nullptr;
    if (const USVONavigationSettings* Settings = GetDefault<USVONavigationSettings>())
    {
        if (Settings->DefaultRaycasterClass)
        {
            RayCasterCDO = Settings->DefaultRaycasterClass->GetDefaultObject<USVORayCaster>();
        }
    }

    if (!RayCasterCDO)
    {
        RayCasterCDO = USVORayCaster_OctreeTraversal::StaticClass()->GetDefaultObject<USVORayCaster>();
    }

    bool bPathModified = true;
    int32 Iterations = 0;
    const int32 MaxIterations = 3; // Limit iterations to prevent infinite loops in edge cases

    // 2. Iterative Smoothing
    // We loop until no more nodes can be removed, or we hit the limit. 
    // This catches cases where removing node B allows A and D to connect, which wasn't possible before.
    while (bPathModified && Iterations < MaxIterations)
    {
        bPathModified = false;
        Iterations++;

        int32 AnchorIndex = 0;
        
        while (AnchorIndex < Points.Num() - 2)
        {
            // 3. Collinearity Check (Cheap Optimization)
            // Before raycasting, check if the next point is perfectly aligned with the current and subsequent point.
            // A -> B -> C. If B is on the line A-C, remove B immediately.
            const FVector& P1 = Points[AnchorIndex].Location;
            const FVector& P2 = Points[AnchorIndex + 1].Location;
            const FVector& P3 = Points[AnchorIndex + 2].Location;

            // Use a small tolerance for float errors
            if (FMath::PointDistToLine(P2, P3 - P1, P1) < 1.0f) 
            {
                // Remove the middle point (P2)
                Points.RemoveAt(AnchorIndex + 1);
                if (Costs.IsValidIndex(AnchorIndex + 1)) Costs.RemoveAt(AnchorIndex + 1);
                
                bPathModified = true;
                // Do not increment AnchorIndex; we want to test the new neighbor against P1
                continue; 
            }

            // 4. Furthest Reach (String Pulling)
            int32 FurthestVisibleIndex = -1;

            // Scan backwards from the end to find the longest possible jump
            for (int32 TestIndex = Points.Num() - 1; TestIndex > AnchorIndex + 1; --TestIndex)
            {
            	const FVector VerticalOffset(0.0f, 0.0f, 50.0f);
                const FVector& StartPos = Points[AnchorIndex].Location+ VerticalOffset;
                const FVector& EndPos = Points[TestIndex].Location+ VerticalOffset;

                // TraceStack returns true if BLOCKED
                const bool bHit = RayCasterCDO->TraceStack(NavData, StartPos, EndPos);

                if (!bHit)
                {
                    FurthestVisibleIndex = TestIndex;
                    break; // Found the longest shortcut
                }
            }

            if (FurthestVisibleIndex != -1)
            {
                // Calculate how many nodes lie between Anchor and Furthest
                const int32 NumToRemove = FurthestVisibleIndex - AnchorIndex - 1;
                
                if (NumToRemove > 0)
                {
                    Points.RemoveAt(AnchorIndex + 1, NumToRemove);
                    
                    // Keep costs synced
                    // We use a loop here to be safe because Costs array size might differ slightly in some implementations
                    int32 CostRemoveCount = FMath::Min(NumToRemove, Costs.Num() - (AnchorIndex + 1));
                    if (CostRemoveCount > 0)
                    {
                        Costs.RemoveAt(AnchorIndex + 1, CostRemoveCount);
                    }

                    bPathModified = true;
                }
                
                // If we found a shortcut, our new "Next" node is the one we just connected to.
                // We increment to start smoothing from that new node.
                AnchorIndex++; 
            }
            else
            {
                // No shortcut found, step forward one node
                AnchorIndex++;
            }
        }
    }
}
