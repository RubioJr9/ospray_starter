// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

// ospray
#include "example_geometry.h"
#include "common/Data.h"
#include "common/World.h"
// ispc-generated files
#include "example_geometry_ispc.h"

namespace ospray {
namespace example {

    ExampleEllipsoids::ExampleEllipsoids()
    {
        ispcEquivalent = ispc::ExampleEllipsoids_create();
    }

    std::string ExampleEllipsoids::toString() const
    {
        return "ospray::ExampleEllipsoids";
    }

    void ExampleEllipsoids::commit()
    {
        if (!embreeDevice) {
            throw std::runtime_error("invalid Embree device");
        }
        if (!embreeGeometry) {
            embreeGeometry = rtcNewGeometry(embreeDevice, RTC_GEOMETRY_TYPE_USER);
        }
        radius = getParam<float>("radius", 0.01f);
        vertexData = getParamDataT<vec3f>("ellipsoid.position", true);
        radiiData = getParamDataT<vec3f>("ellipsoid.radii");
        radiusData = getParamDataT<float>("ellipsoid.radius");
        texcoordData = getParamDataT<vec2f>("ellipsoid.texcoord");
        eigvec1Data = getParamDataT<vec3f>("ellipsoid.eigvec1");
        eigvec2Data = getParamDataT<vec3f>("ellipsoid.eigvec2");

        ispc::ExampleEllipsoids_set(getIE(),
                                 embreeGeometry,
                                 ispc(vertexData),
                                 ispc(radiiData),
                                 ispc(radiusData),
                                 ispc(texcoordData),
                                 radius,
                                 ispc(eigvec1Data),
                                 ispc(eigvec2Data));

        postCreationInfo();
    }

    size_t ExampleEllipsoids::numPrimitives() const
    {
        return vertexData ? vertexData->size() : 0;
    }

}
}
