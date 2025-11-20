#include "SVONavigationSettings.h"

#include "Raycasters/SVORaycaster_OctreeTraversal.h"

USVONavigationSettings::USVONavigationSettings()
{
    bNavigationAutoUpdateEnabled = true;
    bSmoothPaths = true;
    DefaultRaycasterClass = USVORayCaster_OctreeTraversal::StaticClass();
}