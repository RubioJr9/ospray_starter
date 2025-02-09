// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

// ospray
#include "common/Data.ih"
#include "common/FilterIntersect.ih"
#include "common/ISPCMessages.h"
#include "EllipsoidIntersect.ih"
#include "common/Intersect.ih"
#include "common/Ray.ih"
#include "common/World.ih"
#include "geometry/Geometry.ih"
#include "math/sampling.ih"
#include "rkcommon/math/box.ih"
#include "rkcommon/math/vec.ih"
#include "math/AffineSpace.ih"

struct ExampleEllipsoids {
    Geometry super;
    Data1D vertex;
    Data1D radii;
    Data1D radius;
    Data1D texcoord;
    float global_radius;
    Data1D eigvec1;
    Data1D eigvec2;
    uniform affine3f basis;
    uniform affine3f inv_basis;
};

static void ExampleEllipsoids_postIntersect(const Geometry *uniform geometry,
                                         varying DifferentialGeometry &dg,
                                         const varying Ray &ray,
                                         uniform int64 flags)
{
    ExampleEllipsoids *uniform self = (ExampleEllipsoids * uniform) geometry;

    dg.Ng = dg.Ns = ray.Ng;

    float radius = self->global_radius;
    if (valid(self->radius))
        radius = get_float(self->radius, ray.primID);

    // make epsilon large enough to not get lost when computing
    // |CO| = |center-ray.org| ~ radius for 2ndary rays
    vec3f radii = get_vec3f(self->radii, ray.primID);
    dg.epsilon = radii.x * ulpEpsilon;

    if (and(flags & DG_TEXCOORD, valid(self->texcoord)))
        dg.st = get_vec2f(self->texcoord, ray.primID);
}

unmasked void ExampleEllipsoids_bounds(const RTCBoundsFunctionArguments *uniform args)
{
    ExampleEllipsoids *uniform self = (ExampleEllipsoids * uniform) args->geometryUserPtr;
    uniform int primID = args->primID;
    uniform vec3f radii = get_vec3f(self->radii, primID);
    uniform vec3f eigvec1 = get_vec3f(self->eigvec1, primID);
    uniform vec3f eigvec2 = get_vec3f(self->eigvec2, primID);
    uniform vec3f eigvec3 = cross(eigvec1, eigvec2);
    uniform vec3f scaled_eigvec1 = radii.x * eigvec1;
    uniform vec3f scaled_eigvec2 = radii.y * eigvec2;
    uniform vec3f scaled_eigvec3 = radii.z * eigvec3;
    uniform vec3f center = get_vec3f(self->vertex, primID);

    uniform vec3f min = make_vec3f(0,0,0);
    uniform vec3f max = make_vec3f(0,0,0);
    for (uniform int x_sign = -1; x_sign <= 1; x_sign+=2)
        for (uniform int y_sign = -1; y_sign <= 1; y_sign+=2)
            for (uniform int z_sign = -1; z_sign <= 1; z_sign+=2) {
                uniform vec3f offset = x_sign*scaled_eigvec1 + y_sign*scaled_eigvec2 + z_sign*scaled_eigvec3;
                if (offset.x < min.x) min.x = offset.x;
                else if (offset.x > max.x) max.x = offset.x;
                if (offset.y < min.y) min.y = offset.y;
                else if (offset.y > max.y) max.y = offset.y;
                if (offset.z < min.z) min.z = offset.z;
                else if (offset.z > max.z) max.z = offset.z;
            }

    box3fa *uniform out = (box3fa * uniform) args->bounds_o;
    *out = make_box3fa(center + min, center + max);
}

void ExampleEllipsoids_intersect_kernel(const RTCIntersectFunctionNArguments *uniform args,
                                     const uniform bool isOcclusionTest)
{
    // make sure to set the mask
    if (!args->valid[programIndex])
        return;

    ExampleEllipsoids *uniform self = (ExampleEllipsoids * uniform) args->geometryUserPtr;
    uniform int primID = args->primID;

    // this assumes that the args->rayhit is actually a pointer to a varying ray!
    varying Ray *uniform ray = (varying Ray * uniform) args->rayhit;



    const uniform vec3f radii = get_vec3f(self->radii, primID);
    const uniform vec3f center = get_vec3f(self->vertex, primID);
    const uniform vec3f eigvec1 = get_vec3f(self->eigvec1, primID);
    const uniform vec3f eigvec2 = get_vec3f(self->eigvec2, primID);
    const uniform vec3f eigvec3 = cross(eigvec1, eigvec2);
    const uniform vec3f scaled_eigvec1 = radii.x * eigvec1;
    const uniform vec3f scaled_eigvec2 = radii.y * eigvec2;
    const uniform vec3f scaled_eigvec3 = radii.z * eigvec3;
    self->basis = make_AffineSpace3f(scaled_eigvec1, scaled_eigvec2, scaled_eigvec3, center);
    self->inv_basis = rcp(self->basis);

    const Intersections isect = intersectEllipsoid(ray->org, ray->dir, center, radii, eigvec1, eigvec2, self->inv_basis);

    // call intersection filtering callback and setup hit if accepted
    filterIntersectionBoth(args, isect, isOcclusionTest);
}

unmasked void ExampleEllipsoids_intersect(
    const struct RTCIntersectFunctionNArguments *uniform args)
{
    ExampleEllipsoids_intersect_kernel(args, false);
}

unmasked void ExampleEllipsoids_occluded(const struct RTCOccludedFunctionNArguments *uniform args)
{
    ExampleEllipsoids_intersect_kernel((RTCIntersectFunctionNArguments *)args, true);
}

SampleAreaRes ExampleEllipsoids_sampleArea(const Geometry *uniform const _self,
                                        const int32 primID,
                                        const uniform affine3f &xfm,
                                        const uniform affine3f &rcp_xfm,
                                        const vec2f &s,
                                        const float)
{
    const ExampleEllipsoids *const uniform self = (const ExampleEllipsoids *uniform)_self;

    float radius = self->global_radius;
    if (valid(self->radius))
        radius = get_float(self->radius, primID);

    const vec3f localNormal = normalize(uniformSampleSphere(radius, s));
    const vec3f localPos = get_vec3f(self->vertex, primID) + localNormal;
    const vec3f n = make_vec3f(0,0,0);
    const vec3f p = xfmVector(self->basis, localPos);

    SampleAreaRes res;
    res.pos = xfmPoint(xfm, localPos);
    res.pos = p;
    res.normal = normalize(xfmVector(transposed(rcp_xfm.l), localNormal));
    return res;
}

void ExampleEllipsoids_getAreas(const Geometry *const uniform _self,
                             const int32 *const uniform primIDs,
                             const uniform int32 numPrims,
                             const uniform affine3f &xfm,
                             float *const uniform areas)
{
    ExampleEllipsoids *uniform self = (ExampleEllipsoids * uniform) _self;

    // detect (non-uniform) scaling; get length of transformed unit-vectors
    const uniform vec3f scaling3 =
        make_vec3f(length(xfm.l.vx), length(xfm.l.vy), length(xfm.l.vz));
    const uniform float min_scaling = reduce_min(scaling3);
    const uniform float max_scaling = reduce_max(scaling3);
    if ((max_scaling - min_scaling) > 1e-4f * min_scaling)
        postStatusMsg(ISPC_MSG_SPHERES, OSP_LOG_WARNING);

    // use average as approximation to closest uniform scaling
    const uniform float scale = four_pi * sqr(reduce_add(scaling3) / 3.f);

    if (!valid(self->radius)) {
        const uniform float sphereArea = scale * sqr(self->global_radius);
        foreach (i = 0 ... numPrims)
            areas[i] = sphereArea;
    } else {
        // TODO vectorize this loop, with foreach or ProgramCount & ProgramIndex
        for (uniform int32 i = 0; i < numPrims; i++)
            areas[i] = scale * sqr(get_float(self->radius, i));
    }
}

void ExampleEllipsoids_Constructor(ExampleEllipsoids *uniform self)
{
    Geometry_Constructor(&self->super, ExampleEllipsoids_postIntersect);
    self->super.getAreas = ExampleEllipsoids_getAreas;
    self->super.sampleArea = ExampleEllipsoids_sampleArea;
    Data1D_Constructor(&self->vertex);
    Data1D_Constructor(&self->radius);
    Data1D_Constructor(&self->texcoord);
    self->global_radius = 0.01;
}

export void *uniform ExampleEllipsoids_create()
{
    ExampleEllipsoids *uniform self = uniform new ExampleEllipsoids;
    ExampleEllipsoids_Constructor(self);
    return self;
}

export void ExampleEllipsoids_set(void *uniform _self,
                               void *uniform _embreeGeometry,
                               const Data1D *uniform vertex,
                               const Data1D *uniform radii,
                               const Data1D *uniform radius,
                               const Data1D *uniform texcoord,
                               uniform float global_radius,
                               const Data1D *uniform eigvec1,
                               const Data1D *uniform eigvec2)
{
    ExampleEllipsoids *uniform self = (ExampleEllipsoids * uniform) _self;

    self->vertex = *vertex;
    self->radius = *radius;
    self->radii = *radii;
    self->texcoord = *texcoord;
    self->global_radius = global_radius;
    self->super.numPrimitives = vertex->numItems;
    self->eigvec1 = *eigvec1;
    self->eigvec2 = *eigvec2;

    Geometry_setEmbreeUserGeometry(&self->super,
                                   (RTCGeometry)_embreeGeometry,
                                   ExampleEllipsoids_bounds,
                                   ExampleEllipsoids_intersect,
                                   ExampleEllipsoids_occluded);
}
