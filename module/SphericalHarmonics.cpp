// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

// ospray
#include "SphericalHarmonics.h"
#include "SphericalHarmonicsShared.h"
#include "common/Data.h"
#include "common/World.h"
#include "camera/PerspectiveCamera.h"
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
        boundRadiusData = getParamDataT<float>("glyph.boundRadius");
        coefficientData = getParamDataT<float>("glyph.coefficients");
        rotatedCoefficientData = getParamDataT<float>("glyph.rotatedCoefficients");
        degreeL = getParam<int>("glyph.degreeL");
        auto cam = (PerspectiveCamera*)getParamObject("glyph.camera");

        createEmbreeUserGeometry((RTCBoundsFunction)&ispc::SphericalHarmonics_bounds,
                                 (RTCIntersectFunctionN)&ispc::SphericalHarmonics_intersect,
                                 (RTCOccludedFunctionN)&ispc::SphericalHarmonics_occluded);
        getSh()->vertex = *ispc(vertexData);
        getSh()->coefficients = *ispc(coefficientData);
        getSh()->rotatedCoefficients = *ispc(rotatedCoefficientData);
        getSh()->degreeL = degreeL;
        getSh()->boundRadius = *ispc(boundRadiusData);
        getSh()->camera = cam->getSh();

        postCreationInfo();
        ispc::SphericalHarmonics_tests();
    }

    size_t SphericalHarmonics::numPrimitives() const
    {
        return vertexData ? vertexData->size() : 0;
    }

}
}
