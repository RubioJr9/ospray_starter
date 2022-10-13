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
        getSh()->super.postIntersect = ispc::Superquadrics_postIntersect_addr();
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
        vertexData = getParamDataT<vec3f>("glyph.position", true);
        radiiData = getParamDataT<vec3f>("glyph.radii");
        radiusData = getParamDataT<float>("glyph.radius");
        texcoordData = getParamDataT<vec2f>("glyph.texcoord");
        eigvec1Data = getParamDataT<vec3f>("glyph.eigvec1");
        eigvec2Data = getParamDataT<vec3f>("glyph.eigvec2");

        createEmbreeUserGeometry((RTCBoundsFunction)&ispc::Superquadrics_bounds,
                                 (RTCIntersectFunctionN)&ispc::Superquadrics_intersect,
                                 (RTCOccludedFunctionN)&ispc::Superquadrics_occluded);
        getSh()->vertex = *ispc(vertexData);
        getSh()->radii = *ispc(radiiData);
        getSh()->radius = *ispc(radiusData);
        getSh()->texcoord = *ispc(texcoordData);
        getSh()->eigvec1 = *ispc(eigvec1Data);
        getSh()->eigvec2 = *ispc(eigvec2Data);

        postCreationInfo();
    }

    size_t Superquadrics::numPrimitives() const
    {
        return vertexData ? vertexData->size() : 0;
    }

}
}
