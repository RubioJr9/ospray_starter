// Copyright 2022 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "geometry/GeometryShared.h"
#include "camera/PerspectiveCameraShared.h"

enum SHRenderMethod { NewtonBisection = 0, Laguerre, Wigner, Naive };

#ifdef __cplusplus
namespace ispc {
#endif // __cplusplus

struct SphericalHarmonics {
    Geometry super;
    Data1D vertex;
    Data1D coefficients;
    Data1D rotatedCoefficients;
    Data1D boundRadius;
    PerspectiveCamera* camera;
    SHRenderMethod shRenderMethod;
    bool useCylinder;

#ifdef __cplusplus
  SphericalHarmonics() : shRenderMethod(SHRenderMethod::NewtonBisection) {}
};
} // namespace ispc
#else
};
#endif // __cplusplus
