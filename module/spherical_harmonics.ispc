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

static void SphericalHarmonics_postIntersect(const Geometry *uniform geometry,
                                         varying DifferentialGeometry &dg,
                                         const varying Ray &ray,
                                         uniform int64 flags)
{
    SphericalHarmonics *uniform self = (SphericalHarmonics * uniform) geometry;

    dg.Ng = dg.Ns = ray.Ng;

    // make epsilon large enough to not get lost when computing
    // |CO| = |center-ray.org| ~ radius for 2ndary rays
    uniform float* coeffs = (uniform float*)self->coefficients.addr;
    /* float* coefficients = ((float*)self->coefficients)[ray.primID]; */
    /* dg.epsilon = coefficients[0] * ulpEpsilon; */
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
    /* uniform vec3f coefficients = get_vec3f(self->coefficients, primID); */
    /* uniform float * uniform c = (uniform float * uniform)self->coefficients.addr; */
    const uint32 coeffsCount = (self->degreeL+1) * (self->degreeL+1);
    const uniform float* c = (const uniform float*)(self->coefficients.addr + self->coefficients.byteStride * primID * coeffsCount);
    /* const uniform int lMax = get_int32(self->degreeL, primID); */
    /* uniform float c = (const float)coeffs[0]; */
    const uniform vec3f center = get_vec3f(self->vertex, primID);

    const uniform float sqrt3 = sqrt(3.f);
    const uniform float invPi4 = 0.25 / PI;
    uniform float lCoeffs[] = {invPi4, 3.f*invPi4, 5.f*invPi4, 7.f*invPi4, 9.f*invPi4};

    uniform float uniform c2[25];
    for (int i = 0; i < 25; ++i)
        c2[i] = c[i]*c[i];
    uniform float squareSum[] = {c2[0],
                                 c2[1]+c2[2]+c2[3],
                                 c2[4]+c2[5]+c2[6]+c2[7]+c2[8],
                                 c2[9]+c2[10]+c2[11]+c2[12]+c2[13]+c2[14]+c2[15],
                                 c2[16]+c2[17]+c2[18]+c2[19]+c2[20]+c2[21]+c2[22]+c2[23]+c2[24]};

    /* for (int i = 0; i < 5; ++i) */
    /*     lCoeffs[i] = (2*i+1)/(4*PI); */

    uniform float v = sqrt3 * sqrt(lCoeffs[0]*squareSum[0] + lCoeffs[1]*squareSum[1] + lCoeffs[2]*squareSum[2] + lCoeffs[3]*squareSum[3] + lCoeffs[4]*squareSum[4]);
    /* uniform float v = sqrt3 * sqrt(lCoeffs[0]*c2[0] + lCoeffs[1]*sum(c2,1,3) + lCoeffs[2]*sum(c2,4,8) */
        /* + lCoeffs[3]*sum(c2,9,15) + lCoeffs[4]*sum(c2,16,24)); */
    self->boundRadius[primID] = v;
    /* float sum = 0.f; */
    /* for () */

    /* uniform vec3f min = make_vec3f(-coeffs[0], -coeffs[0], -coeffs[0]); */
    /* uniform vec3f max = make_vec3f(coeffs[0], coeffs[0], coeffs[0]); */

    /* uniform vec3f min = make_vec3f(0,0,0); */
    /* uniform vec3f max = make_vec3f(0,0,0); */
    /* for (uniform int x_sign = -1; x_sign <= 1; x_sign+=2) */
    /*     for (uniform int y_sign = -1; y_sign <= 1; y_sign+=2) */
    /*         for (uniform int z_sign = -1; z_sign <= 1; z_sign+=2) { */
    /*             uniform vec3f offset = x_sign*scaled_eigvec1 + y_sign*scaled_eigvec2 + z_sign*scaled_eigvec3; */
    /*             if (offset.x < min.x) min.x = offset.x; */
    /*             else if (offset.x > max.x) max.x = offset.x; */
    /*             if (offset.y < min.y) min.y = offset.y; */
    /*             else if (offset.y > max.y) max.y = offset.y; */
    /*             if (offset.z < min.z) min.z = offset.z; */
    /*             else if (offset.z > max.z) max.z = offset.z; */
    /*         } */

    box3fa *uniform out = (box3fa * uniform) args->bounds_o;
    /* *out = make_box3fa(center + min, center + max); */
    /* *out = make_box3fa(center + make_vec3f(-100,-100,-100), center + make_vec3f(100,100,100)); */
    *out = make_box3fa(center + make_vec3f(-v,-v,-v), center + make_vec3f(v,v,v));
}

void SphericalHarmonics_intersect_kernel(const RTCIntersectFunctionNArguments *uniform args,
                                     const uniform bool isOcclusionTest)
{
    // make sure to set the mask
    if (!args->valid[programIndex])
        return;

    SphericalHarmonics *uniform self = (SphericalHarmonics * uniform) args->geometryUserPtr;
    uniform int primID = args->primID;

    // this assumes that the args->rayhit is actually a pointer to a varying ray!
    varying Ray *uniform ray = (varying Ray * uniform) args->rayhit;



    const uniform vec3f center = get_vec3f(self->vertex, primID);
    /* const uniform float* coefficients = (float*)get_int32(self->coefficients, primID); */
    /* uniform vec3f coeffs = get_vec3f(self->coefficients, primID); */
    /* uniform vec3f coeffs = *((const uniform vec3f*)(self->coefficients.addr + self->coefficients.byteStride * primID)); */
    /* print("%f\n", get_float(self->coefficients, primID)); */
    /* print("0\n"); */
    /* uniform float* addr = *((uniform float**)(self->coefficients.addr)); */
    /* uint8* addr = self->coefficients.addr; */
    /* uniform uint32 coeffs = *(uniform vec3f*)addr; */
    const uint32 coeffsCount = (self->degreeL+1) * (self->degreeL+1);
    const uniform float* coeffs = (const uniform float*)(self->coefficients.addr + self->coefficients.byteStride * primID * coeffsCount);
    /* const uniform uint32 degreeL = get_uint32(self->degreeL, primID); */
    /* print("1\n"); */
    /* print("%f\n", coeffs[0]); */
    /* print("%f\n", coeffs[0]); */
    /* print("2\n"); */
    /* vec3f* v_addr = (vec3f*)addr; */
    /* vec3f coeffs = *v_addr; */
    /* uniform float* coeffs = (uniform float*)self->coefficients.addr; */
    /* uniform vec3f coeffs = *(uniform vec3f*)self->coefficients.addr + primID*sizeof(vec3f); */
    Intersections isect = intersectSphericalHarmonics(ray->org, ray->dir, center, coeffs, self->degreeL, self->boundRadius[primID]);

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

