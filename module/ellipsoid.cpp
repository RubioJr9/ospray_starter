// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

// ospray
#include "ellipsoid.h"
#include "common/Data.h"
#include "common/World.h"
// ispc-generated files
#include "ellipsoid_ispc.h"

namespace ospray {
namespace tensor_geometry {

    Ellipsoids::Ellipsoids()
    {
        // ispcEquivalent = ispc::Ellipsoids_create();
        getSh()->super.postIntersect = ispc::Ellipsoids_postIntersect_addr();
    }

    std::string Ellipsoids::toString() const
    {
        return "ospray::Ellipsoids";
    }

    void Ellipsoids::commit()
    {
        vertexData = getParamDataT<vec3f>("glyph.position", true);
        radiiData = getParamDataT<vec3f>("glyph.radii");
        eigvec1Data = getParamDataT<vec3f>("glyph.eigvec1");
        eigvec2Data = getParamDataT<vec3f>("glyph.eigvec2");

        createEmbreeUserGeometry((RTCBoundsFunction)&ispc::Ellipsoids_bounds,
                                 (RTCIntersectFunctionN)&ispc::Ellipsoids_intersect,
                                 (RTCOccludedFunctionN)&ispc::Ellipsoids_occluded);
        getSh()->vertex = *ispc(vertexData);
        getSh()->radii = *ispc(radiiData);
        getSh()->eigvec1 = *ispc(eigvec1Data);
        getSh()->eigvec2 = *ispc(eigvec2Data);

        postCreationInfo();
    }

    size_t Ellipsoids::numPrimitives() const
    {
        return vertexData ? vertexData->size() : 0;
    }

}
}
