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
#include "sh.ih"
#include "SphericalHarmonicsIntersectRelatedWork.ih"
// c++ shared
#include "SphericalHarmonicsShared.h"

#define COEFFS_COUNT 15


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

void compute_aabb_newton(uniform float uniform out_aabb[3], const uniform float uniform sh_coeffs[COEFFS_COUNT], uniform int sample_count) {
	// Prepare a spherical Fibonacci sampling on the upper hemisphere
	uniform float z_factor = -1.0f / ((float) sample_count);
	uniform float z_summand = 1.0f - 1.0f / (2.0f * ((float) sample_count));
	uniform float golden_ratio = 0.5f * (sqrtf(5.0f) + 1.0f);
	uniform float azimuth_factor = 2.0f * 3.141592653589793f / golden_ratio;
	// Find good candidates for extrema
	uniform float axis_extremum[3][3] = { { 0.0f } };
	for (uniform int i = 0; i != sample_count; ++i) {
		// Construct the spherical Fibonacci point
		uniform float z = z_factor * ((float) i) + z_summand;
		uniform float radius = sqrtf(1.0f - z * z);
		uniform float azimuth = azimuth_factor * ((float) i);
		uniform float x = cosf(azimuth) * radius;
		uniform float y = sinf(azimuth) * radius;
		// Evaluate the SH polynomial
		uniform float point[3] = { x, y, z };
		uniform float shs[COEFFS_COUNT];
		evaluate_sh_4(shs, point);
		uniform float value = 0.0f;
		for (uniform int j = 0; j != COEFFS_COUNT; ++j)
			value += sh_coeffs[j] * shs[j];
		// Scale the point accordingly
		for (uniform int j = 0; j != 3; ++j)
			point[j] *= value;
		// Take the maximum for each axis
		for (uniform int j = 0; j != 3; ++j)
			if (abs(axis_extremum[j][j]) < abs(point[j]))
				for (uniform int k = 0; k != 3; ++k)
					axis_extremum[j][k] = point[k];
	}
	// Store the maximum found thus far to ensure that Newton's method
	// cannot make matters worse
	for (uniform int i = 0; i != 3; ++i)
		out_aabb[i] = abs(axis_extremum[i][i]);
	// Now perform a few Newton iterations for each axis
	const uniform int uniform hessian_indices[3][3] = {
		{ 0, 1, 2 },
		{ 1, 3, 4 },
		{ 2, 4, 5 },
	};
	for (uniform int Z = 0; Z != 3; ++Z) {
		// Figure out the relevant entries of the Hessian matrix
		uniform int X = (Z + 1) % 3;
		uniform int Y = (Z + 2) % 3;
		uniform int XX = hessian_indices[X][X];
		uniform int XY = hessian_indices[X][Y];
		uniform int YY = hessian_indices[Y][Y];
		// We run Newton's method with a fixed iteration count. If that is not
		// sufficient, the method probably ended up moving away from the local
		// maximum found originally and we are better off keeping that.
		uniform float * uniform point = axis_extremum[Z];
		for (uniform int i = 0; i != 5; ++i) {
			// Normalizing saves us a few multiplications below
			uniform float norm = sqrt(point[0]*point[0] + point[1]*point[1] + point[2]*point[2]);
            for (uniform int j = 0; j < 3; ++j)
                point[j] /= norm;
			// Evaluate the polynomial and two of its derivatives
			uniform float uniform shs[COEFFS_COUNT], gradients[COEFFS_COUNT][3], hessians[COEFFS_COUNT][6];
			evaluate_sh_4_hess(shs, gradients, hessians, point);
			uniform float sh = 0.0f;
			uniform float uniform gradient[3] = { 0.0f, 0.0f, 0.0f };
			uniform float uniform hessian[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
			for (uniform int j = 0; j != COEFFS_COUNT; ++j) {
				sh += sh_coeffs[j] * shs[j];
				for (uniform int k = 0; k != 3; ++k)
					gradient[k] += sh_coeffs[j] * gradients[j][k];
				for (uniform int k = 0; k != 6; ++k)
					hessian[k] += sh_coeffs[j] * hessians[j][k];
			}
			// Compute the relevant entries of the homogenized normal
			uniform float gradient_x = gradient[X] - 5.0f * sh * point[X];
			uniform float gradient_y = gradient[Y] - 5.0f * sh * point[Y];
			// Compute the Hessian matrix
			uniform float hessian_xx = hessian[XX] - 3.0f * point[X] * gradient[X] - 5.0f * sh;
			uniform float hessian_xy = hessian[XY] + 2.0f * point[Y] * gradient[X] - 5.0f * point[X] * gradient[Y];
			uniform float hessian_yy = hessian[YY] - 3.0f * point[Y] * gradient[Y] - 5.0f * sh;
			// Perform the Newton step (implemented using Cramer's rule)
			uniform float inv_det = 1.0f / (hessian_xx * hessian_yy - hessian_xy * hessian_xy);
			point[X] -= inv_det * (hessian_yy * gradient_x - hessian_xy * gradient_y);
			point[Y] -= inv_det * (-hessian_xy * gradient_x + hessian_xx * gradient_y);
		}
		// Normalize the final point such that it is on the glyph
        uniform float norm = sqrt(point[0]*point[0] + point[1]*point[1] + point[2]*point[2]);
        for (uniform int j = 0; j < 3; ++j)
            point[j] /= norm;
		uniform float shs[COEFFS_COUNT];
		evaluate_sh_4(shs, point);
		uniform float value = 0.0f;
		for (uniform int j = 0; j != COEFFS_COUNT; ++j)
			value += sh_coeffs[j] * shs[j];
		for (uniform int j = 0; j != 3; ++j)
			point[j] *= value;
		// Now we know the bounding box extent. But we only keep it, if it is
		// better than what we had before.
		if (out_aabb[Z] < abs(point[Z]))
			out_aabb[Z] = abs(point[Z]);
	}
}

export void SphericalHarmonics_bounds(const RTCBoundsFunctionArguments *uniform args)
{
    SphericalHarmonics *uniform self = (SphericalHarmonics * uniform) args->geometryUserPtr;
    uniform int primID = args->primID;

    const uniform vec3f center = get_vec3f(self->vertex, primID);
    uniform float r = get_float(self->boundRadius, primID);

    box3fa *uniform out = (box3fa * uniform) args->bounds_o;

    uniform float uniform out_aabb[3];
    const uniform float* uniform coeffs = (const uniform float* uniform)(self->coefficients.addr + self->coefficients.byteStride * primID * COEFFS_COUNT);
    const uniform int sample_count = 100;
    if (self->useCylinder) {
        *out = make_box3fa(center - make_vec3f(r), center + make_vec3f(r));
    } else {
        compute_aabb_newton(out_aabb, coeffs, sample_count);
        for (uniform int i = 0; i < 3; ++i)
            out_aabb[i] *= 1.02;
        *out = make_box3fa(center + make_vec3f(-out_aabb[0],-out_aabb[1],-out_aabb[2]), center + make_vec3f(out_aabb[0],out_aabb[1],out_aabb[2]));
    }
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
    const uniform vec3f center = get_vec3f(self->vertex, primID);
    const uniform float* uniform coeffs = (const uniform float* uniform)(self->coefficients.addr + self->coefficients.byteStride * primID * COEFFS_COUNT);

    // this assumes that the args->rayhit is actually a pointer to a varying ray!
    varying Ray *uniform ray = (varying Ray * uniform) args->rayhit;


    #if 0
    uniform float uniform out_aabb[3];
    const uniform int sample_count = 100;
    compute_aabb_newton(out_aabb, coeffs, sample_count);
    uniform vec3f out_aabb_vec = 1.02 * make_vec3f(out_aabb[0], out_aabb[1], out_aabb[2]);
    uniform box3f box = make_box3f(center - out_aabb_vec, center + out_aabb_vec);
    Intersections isectBox = intersectBox(ray->org, ray->dir, box);
    #endif

    Intersections isect;
    isect.entry.hit = false;
    isect.exit.hit = false;
    isect.entry.t = -inf;
    isect.exit.t = -inf;
    SHRenderMethod shRenderMethod = self->shRenderMethod;
        float out_ray_roots[10];
    switch (shRenderMethod) {
        case 0:
            intersectSphericalHarmonicsNewtonBisection(out_ray_roots, ray->org, ray->dir, center, coeffs, hitData);
            break;
        case 1: {
            get_sh_glyph_intersections_complex(out_ray_roots, coeffs, center, ray->org, ray->dir);
            break;
        }
        case 2: {
        const uniform float* uniform rotatedCoeffs = (const uniform float* uniform)(self->rotatedCoefficients.addr + self->rotatedCoefficients.byteStride * primID * COEFFS_COUNT);
            get_sh_glyph_intersections_almsick(out_ray_roots, rotatedCoeffs, center, ray->org, ray->dir, false, self->camera);
            break;
        }
        case 3: {
            get_sh_glyph_intersections_naive(out_ray_roots, coeffs, center, ray);
            break;
        }
    }
            isect.entry.hit = out_ray_roots[0] != NO_INTERSECTION;
            isect.entry.t = out_ray_roots[0];
            if (isect.entry.hit)
                isect.entry.N = get_sh_glyph_normal(coeffs, ray->org + out_ray_roots[0] * ray->dir - center);

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
  /* testComplexMath(); */
  /* testPolyGen(); */
  /* testPolyGenMoebius(); */
  /* testPolyGenInverseRotation(); */
  /* testPolyGenNewton(); */
}
