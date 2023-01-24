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
    const uint32 coeffsCount = 15;
    const uniform float* c = (const uniform float*)(self->coefficients.addr + self->coefficients.byteStride * primID * coeffsCount);
    const uniform vec3f center = get_vec3f(self->vertex, primID);

    const uniform float sqrt3 = sqrt(3.f);
    const uniform float invPi4 = 0.25 / PI;
    uniform float lCoeffs[] = {invPi4, 5.f*invPi4, 9.f*invPi4};

    uniform float uniform c2[15];
    for (int i = 0; i < 15; ++i)
        c2[i] = c[i]*c[i];
    uniform float squareSum[] = {c2[0],
                                 c2[1]+c2[2]+c2[3]+c2[4]+c2[5],
                                 c2[6]+c2[7]+c2[8]+c2[9]+c2[10]+c2[11]+c2[12]+c2[13]+c2[14]};


    uniform float v = sqrt3 * sqrt(lCoeffs[0]*squareSum[0] + lCoeffs[1]*squareSum[1] + lCoeffs[2]*squareSum[2]);
    self->boundRadius[primID] = v;

    box3fa *uniform out = (box3fa * uniform) args->bounds_o;
    *out = make_box3fa(center + make_vec3f(-v,-v,-v), center + make_vec3f(v,v,v));
}

void SphericalHarmonics_intersect_kernel(const RTCIntersectFunctionNArguments *uniform args,
                                     const uniform bool isOcclusionTest)
{
    // make sure to set the mask
    if (!args->valid[programIndex])
        return;

    EmbreeIntersectionContext *uniform ctxt = ((EmbreeIntersectionContext *uniform)args->context);
    varying SHIntersections* uniform hitData = (varying SHIntersections* uniform)ctxt->userPtr;

    SphericalHarmonics *uniform self = (SphericalHarmonics * uniform) args->geometryUserPtr;
    uniform int primID = args->primID;

    // this assumes that the args->rayhit is actually a pointer to a varying ray!
    varying Ray *uniform ray = (varying Ray * uniform) args->rayhit;



    const uniform vec3f center = get_vec3f(self->vertex, primID);
    const uint32 coeffsCount = 15;
    const uniform float* coeffs = (const uniform float*)(self->coefficients.addr + self->coefficients.byteStride * primID * coeffsCount);
    Intersections isect = intersectSphericalHarmonics(ray->org, ray->dir, center, coeffs, self->degreeL, self->boundRadius[primID], hitData);

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
    testPolyGen();
}
