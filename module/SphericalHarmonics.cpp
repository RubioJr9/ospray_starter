// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

// ospray
#include "SphericalHarmonics.h"
#include "SphericalHarmonicsShared.h"
#include "common/Data.h"
#include "common/World.h"
// ispc-generated files
#include "spherical_harmonics_ispc.h"

namespace ospray {
namespace tensor_geometry {

    SphericalHarmonics::SphericalHarmonics()
    {
        getSh()->super.postIntersect = ispc::SphericalHarmonics_postIntersect_addr();
    }

    std::string SphericalHarmonics::toString() const
    {
        return "ospray::SphericalHarmonics";
    }

    void SphericalHarmonics::commit()
    {
        if (!embreeDevice) {
            throw std::runtime_error("invalid Embree device");
        }
        if (!embreeGeometry) {
            embreeGeometry = rtcNewGeometry(embreeDevice, RTC_GEOMETRY_TYPE_USER);
        }
        vertexData = getParamDataT<vec3f>("glyph.position", true);
        coefficientData = getParamDataT<float>("glyph.coefficients");
        degreeL = getParam<int>("glyph.degreeL");

        createEmbreeUserGeometry((RTCBoundsFunction)&ispc::SphericalHarmonics_bounds,
                                 (RTCIntersectFunctionN)&ispc::SphericalHarmonics_intersect,
                                 (RTCOccludedFunctionN)&ispc::SphericalHarmonics_occluded);
        getSh()->vertex = *ispc(vertexData);
        getSh()->coefficients = *ispc(coefficientData);
        getSh()->degreeL = degreeL;
        getSh()->boundRadius = new float[vertexData->size()];

        postCreationInfo();
        ispc::SphericalHarmonics_tests();
    }

    size_t SphericalHarmonics::numPrimitives() const
    {
        return vertexData ? vertexData->size() : 0;
    }

}
}
