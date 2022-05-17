#include <ospray/ospray.h>
#include <ospray/version.h>
#include "common/OSPCommon.h"
#include "ellipsoid.h"

using namespace ospray;

extern "C" OSPError OSPRAY_DLLEXPORT ospray_module_init_example(int16_t versionMajor,
                                                                int16_t versionMinor,
                                                                int16_t /*versionPatch*/)
{
    Geometry::registerType<geometry::Ellipsoids>("ellipsoids");
    return OSP_NO_ERROR;
}
