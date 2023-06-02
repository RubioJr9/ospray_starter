// Copyright 2009-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "geometry/Geometry.h"
// ispc shared
#include "SphericalHarmonicsShared.h"

namespace ospray {
namespace tensor_geometry {
    struct OSPRAY_SDK_INTERFACE SphericalHarmonics
        : public AddStructShared<Geometry, ispc::SphericalHarmonics> {
        SphericalHarmonics();

        virtual ~SphericalHarmonics() override = default;

        virtual std::string toString() const override;

        virtual void commit() override;

        virtual size_t numPrimitives() const override;

    protected:
        Ref<const DataT<vec3f>> vertexData;
        Ref<const DataT<float>> boundRadiusData;
        Ref<const DataT<float>> coefficientData;
        Ref<const DataT<float>> rotatedCoefficientData;
        SHRenderMethod shRenderMethod{SHRenderMethod::NewtonBisection};
        bool useCylinder;
    };
}}
