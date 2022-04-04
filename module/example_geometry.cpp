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

    ExampleSpheres::ExampleSpheres()
    {
        ispcEquivalent = ispc::ExampleSpheres_create();
    }

    std::string ExampleSpheres::toString() const
    {
        return "ospray::ExampleSpheres";
    }

    void ExampleSpheres::commit()
    {
        if (!embreeDevice) {
            throw std::runtime_error("invalid Embree device");
        }
        if (!embreeGeometry) {
            embreeGeometry = rtcNewGeometry(embreeDevice, RTC_GEOMETRY_TYPE_USER);
        }
        radius = getParam<float>("radius", 0.01f);
        vertexData = getParamDataT<vec3f>("sphere.position", true);
        radiiData = getParamDataT<vec3f>("sphere.radii");
        radiusData = getParamDataT<float>("sphere.radius");
        texcoordData = getParamDataT<vec2f>("sphere.texcoord");
        eigvec1Data = getParamDataT<vec3f>("sphere.eigvec1");
        eigvec2Data = getParamDataT<vec3f>("sphere.eigvec2");

        ispc::ExampleSpheres_set(getIE(),
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

    size_t ExampleSpheres::numPrimitives() const
    {
        return vertexData ? vertexData->size() : 0;
    }

}
}
