#include <ospray/ospray.h>
#include <ospray/version.h>
#include "common/OSPCommon.h"
#include "ellipsoid.h"
#include "superquadric.h"

using namespace ospray;

extern "C" OSPError OSPRAY_DLLEXPORT ospray_module_init_tensor_geometry(int16_t versionMajor,
                                                                        int16_t versionMinor,
                                                                        int16_t /*versionPatch*/)
{
    Geometry::registerType<tensor_geometry::Ellipsoids>("ellipsoids");
    Geometry::registerType<tensor_geometry::Superquadrics>("superquadrics");
    return OSP_NO_ERROR;
}
