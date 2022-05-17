// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

// ospray
#include "superquadric.h"
#include "common/Data.h"
#include "common/World.h"
// ispc-generated files
#include "superquadric_ispc.h"

namespace ospray {
namespace tensor_geometry {

    Superquadrics::Superquadrics()
    {
        ispcEquivalent = ispc::Superquadrics_create();
    }

    std::string Superquadrics::toString() const
    {
        return "ospray::Superquadrics";
    }

    void Superquadrics::commit()
    {
        if (!embreeDevice) {
            throw std::runtime_error("invalid Embree device");
        }
        if (!embreeGeometry) {
            embreeGeometry = rtcNewGeometry(embreeDevice, RTC_GEOMETRY_TYPE_USER);
        }
        radius = getParam<float>("radius", 0.01f);
        vertexData = getParamDataT<vec3f>("superquadric.position", true);
        radiiData = getParamDataT<vec3f>("superquadric.radii");
        radiusData = getParamDataT<float>("superquadric.radius");
        texcoordData = getParamDataT<vec2f>("superquadric.texcoord");
        eigvec1Data = getParamDataT<vec3f>("superquadric.eigvec1");
        eigvec2Data = getParamDataT<vec3f>("superquadric.eigvec2");

        ispc::Superquadrics_set(getIE(),
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

    size_t Superquadrics::numPrimitives() const
    {
        return vertexData ? vertexData->size() : 0;
    }

}
}
