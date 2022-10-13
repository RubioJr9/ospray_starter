// Copyright 2022 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "geometry/GeometryShared.h"

#ifdef __cplusplus
namespace ispc {
#endif // __cplusplus

struct Ellipsoids {
    Geometry super;
    Data1D vertex;
    Data1D radii;
    Data1D radius;
    Data1D texcoord;
    float global_radius;
    Data1D eigvec1;
    Data1D eigvec2;
    affine3f basis;
    affine3f inv_basis;
#ifdef __cplusplus
    Ellipsoids() {}
};
} // namespace ispc
#else
};
#endif // __cplusplus
