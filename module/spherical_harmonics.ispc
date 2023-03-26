// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

// ospray
#include "common/Data.ih"
#include "common/FilterIntersect.ih"
#include "common/ISPCMessages.h"
#include "SphericalHarmonicsIntersect.ih"
#include "common/Intersect.ih"
#include "common/Ray.ih"
#include "common/World.ih"
#include "geometry/Geometry.ih"
#include "math/sampling.ih"
#include "rkcommon/math/box.ih"
#include "rkcommon/math/vec.ih"
#include "math/AffineSpace.ih"
// c++ shared
#include "SphericalHarmonicsShared.h"


void SphericalHarmonics_postIntersect(const Geometry *uniform geometry,
                                         varying DifferentialGeometry &dg,
                                         const varying Ray &ray,
                                         uniform int64 flags)
{
    SphericalHarmonics *uniform self = (SphericalHarmonics * uniform) geometry;

    dg.Ng = dg.Ns = ray.Ng;

    // make epsilon large enough to not get lost when computing
    // |CO| = |center-ray.org| ~ radius for 2ndary rays
    uniform float* coeffs = (uniform float*)self->coefficients.addr;
    dg.epsilon = coeffs[0] * ulpEpsilon;

    /* if (and(flags & DG_TEXCOORD, valid(self->texcoord))) */
        /* dg.st = get_vec2f(self->texcoord, ray.primID); */
}

uniform float sum(uniform float * uniform a, uniform int start, uniform int end) {
    // todo make uniform
    uniform float val = 0.0;
    while (start <= end)
      val += a[start++];
    return val;
}

export void SphericalHarmonics_bounds(const RTCBoundsFunctionArguments *uniform args)
{
    SphericalHarmonics *uniform self = (SphericalHarmonics * uniform) args->geometryUserPtr;
    uniform int primID = args->primID;

    const uniform vec3f center = get_vec3f(self->vertex, primID);
    uniform float r = get_float(self->boundRadius, primID);

    box3fa *uniform out = (box3fa * uniform) args->bounds_o;
    *out = make_box3fa(center + make_vec3f(-r,-r,-r), center + make_vec3f(r,r,r));
}

void SphericalHarmonics_intersect_kernel(const RTCIntersectFunctionNArguments *uniform args,
                                     const uniform bool isOcclusionTest)
{
    // make sure to set the mask
    if (!args->valid[programIndex])
        return;

    EmbreeIntersectionContext *uniform ctxt = ((EmbreeIntersectionContext *uniform)args->context);
    /* varying SHIntersections* uniform hitData = (varying SHIntersections* uniform)ctxt->userPtr; */
    varying SHIntersections* uniform hitData = (varying SHIntersections* uniform)ctxt->userPtr;

    SphericalHarmonics *uniform self = (SphericalHarmonics * uniform) args->geometryUserPtr;
    uniform int primID = args->primID;

    // this assumes that the args->rayhit is actually a pointer to a varying ray!
    varying Ray *uniform ray = (varying Ray * uniform) args->rayhit;



    const uniform vec3f center = get_vec3f(self->vertex, primID);
    const uniform uint32 coeffsCount = 15;
    const uniform float* uniform coeffs = (const uniform float* uniform)(self->coefficients.addr + self->coefficients.byteStride * primID * coeffsCount);
    const uniform float* uniform rotatedCoeffs = (const uniform float* uniform)(self->rotatedCoefficients.addr + self->rotatedCoefficients.byteStride * primID * coeffsCount);
    uniform float r = get_float(self->boundRadius, primID);
    Intersections isect = intersectSphericalHarmonics(ray->org, ray->dir, center, coeffs, rotatedCoeffs, self->degreeL, r, hitData, self->camera);

    // call intersection filtering callback and setup hit if accepted
    filterIntersectionBoth(args, isect, isOcclusionTest);
}

export void SphericalHarmonics_intersect(
    const struct RTCIntersectFunctionNArguments *uniform args)
{
    SphericalHarmonics_intersect_kernel(args, false);
}

export void SphericalHarmonics_occluded(const struct RTCOccludedFunctionNArguments *uniform args)
{
    SphericalHarmonics_intersect_kernel((RTCIntersectFunctionNArguments *)args, true);
}

export void *uniform SphericalHarmonics_postIntersect_addr()
{
  return SphericalHarmonics_postIntersect;
}

export void SphericalHarmonics_tests()
{
  testComplexMath();
  testPolyGen();
  testPolyGenInverseRotation();
}
