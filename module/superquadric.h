// Copyright 2009-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "geometry/Geometry.h"
// c++ shared
#include "SuperquadricsShared.h"

namespace ospray {
namespace tensor_geometry {
    struct OSPRAY_SDK_INTERFACE Superquadrics
        : public AddStructShared<Geometry, ispc::Superquadrics> {
        Superquadrics();

        virtual std::string toString() const override;

        virtual void commit() override;

        virtual size_t numPrimitives() const override;

    protected:
        float radius{0.01};  // default radius, if no per-sphere radius
        Ref<const DataT<vec3f>> vertexData;
        Ref<const DataT<vec3f>> radiiData;
        Ref<const DataT<float>> radiusData;
        Ref<const DataT<vec2f>> texcoordData;
        Ref<const DataT<vec3f>> eigvec1Data;
        Ref<const DataT<vec3f>> eigvec2Data;
    };
}}
