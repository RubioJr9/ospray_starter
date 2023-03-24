// Copyright 2022 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "geometry/GeometryShared.h"
#include "camera/PerspectiveCameraShared.h"

#ifdef __cplusplus
namespace ispc {
#endif // __cplusplus

struct SphericalHarmonics {
    Geometry super;
    Data1D vertex;
    Data1D coefficients;
    Data1D rotatedCoefficients;
    uint32 degreeL;
    Data1D boundRadius;
    PerspectiveCamera* camera;

#ifdef __cplusplus
  SphericalHarmonics() : degreeL(0) {}
};
} // namespace ispc
#else
};
#endif // __cplusplus
