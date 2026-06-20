// @(#)root/geom
// Author: Sandro Wenzel   2026-01

/*************************************************************************
 * Copyright (C) 1995-2000, Rene Brun and Fons Rademakers.               *
 * All rights reserved.                                                  *
 *                                                                       *
 * For the licensing terms see $ROOTSYS/LICENSE.                         *
 * For the list of contributors see $ROOTSYS/README/CREDITS.             *
 *************************************************************************/

/** \class TGeoTessellatedEmbreeUser
\ingroup Geometry_classes

Tessellated solid navigated with Intel Embree used as a pure BVH provider.

Unlike TGeoTessellatedEmbree, which builds a single precision triangle scene and
lets Embree perform the ray/triangle intersection, this backend registers an
Embree `RTC_GEOMETRY_TYPE_USER` geometry. Embree is therefore only responsible
for building and traversing the bounding-volume hierarchy over per-facet
bounding boxes; the actual ray/primitive intersection runs in user callbacks, in
double precision, using the same geometric kernels as the base TGeoTessellated.

Each Embree user primitive maps one-to-one to a facet, so the Embree primitive id
is the facet index. The per-facet bounding boxes handed to Embree are rounded
outward to single precision so that they conservatively enclose the exact facet:
a real intersection can therefore never be culled, which removes the
watertightness gaps (and the associated neighbour rechecks / fallbacks) of the
triangle based backend.
*/

#include "TGeoTessellatedEmbreeUser.h"

#ifdef R__HAS_EMBREE

#include "TBuffer.h"
#include "TError.h"
#include "TGeoTessellatedHelpers.hxx"

#include <embree4/rtcore.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

ClassImp(TGeoTessellatedEmbreeUser);

namespace {

// Singleton Embree device shared within the process.
RTCDevice GetEmbreeDevice()
{
   static RTCDevice device = [] {
      RTCDevice d = rtcNewDevice(nullptr);
      if (!d) {
         throw std::runtime_error("TGeoTessellatedEmbreeUser: failed to create Embree device");
      }
      return d;
   }();
   return device;
}

// Pointers to the mesh data owned by the base class, handed to the Embree
// callbacks as the geometry user data (lifetime tied to the scene).
struct EmbreeMesh {
   const std::vector<TGeoFacet> *facets;
   const std::vector<Vertex_t> *vertices;
   const std::vector<Vertex_t> *normals;
};

// Per-query context carrying the query and the (double precision) result. A
// single context type is shared by the distance and the Contains queries so a
// single intersect callback can be registered on the geometry.
struct EmbreeQueryContext : public RTCRayQueryContext {
   enum Mode { kInside, kOutside, kContains } mode;
   Vertex_t org;    // ray origin (double)
   Vertex_t dir;    // ray direction (double)
   double bestT;    // nearest accepted distance (distance queries)
   int bestFacet;   // facet of the nearest accepted hit, -1 if none
   int crossings;   // number of surface crossings (Contains query)
};

// Round a double bound to a single precision value that is guaranteed to lie
// outside the true value, so the float bounding box conservatively encloses the
// exact facet (no real hit can ever be culled by Embree).
inline float roundDown(double v)
{
   return std::nextafter(static_cast<float>(v), -std::numeric_limits<float>::infinity());
}
inline float roundUp(double v)
{
   return std::nextafter(static_cast<float>(v), std::numeric_limits<float>::infinity());
}

// Bounds callback: conservative single precision AABB of one facet.
void boundsFunc(const RTCBoundsFunctionArguments *args)
{
   const auto *mesh = static_cast<const EmbreeMesh *>(args->geometryUserPtr);
   const auto &facet = (*mesh->facets)[args->primID];
   const auto &vertices = *mesh->vertices;

   const double INF = std::numeric_limits<double>::infinity();
   double lo[3] = {INF, INF, INF};
   double hi[3] = {-INF, -INF, -INF};
   const int nv = facet.GetNvert();
   for (int i = 0; i < nv; ++i) {
      const auto &v = vertices[facet[i]];
      for (int k = 0; k < 3; ++k) {
         lo[k] = std::min(lo[k], (double)v[k]);
         hi[k] = std::max(hi[k], (double)v[k]);
      }
   }

   args->bounds_o->lower_x = roundDown(lo[0]);
   args->bounds_o->lower_y = roundDown(lo[1]);
   args->bounds_o->lower_z = roundDown(lo[2]);
   args->bounds_o->upper_x = roundUp(hi[0]);
   args->bounds_o->upper_y = roundUp(hi[1]);
   args->bounds_o->upper_z = roundUp(hi[2]);
}

// Intersect callback: all primitive intersection happens here, in double
// precision. For distance queries Embree's traversal is pruned by ray.tfar
// (rounded outward), while the exact result is kept in the query context.
void intersectFunc(const RTCIntersectFunctionNArguments *args)
{
   if (!args->valid[0])
      return;

   auto *ctx = static_cast<EmbreeQueryContext *>(args->context);
   const auto *mesh = static_cast<const EmbreeMesh *>(args->geometryUserPtr);
   const unsigned int primID = args->primID;
   const auto &facet = (*mesh->facets)[primID];
   const auto &vertices = *mesh->vertices;

   if (ctx->mode == EmbreeQueryContext::kContains) {
      // Parity test: count every facet actually crossed (quads handled inside
      // rayFacetHit, so a facet is counted at most once). The ray is left
      // untouched so Embree keeps traversing all crossed primitives.
      if (rayFacetHit(ctx->org, ctx->dir, facet, vertices, 0.))
         ++ctx->crossings;
      return;
   }

   // Distance queries: keep only the relevant surface orientation.
   const double nd = (*mesh->normals)[primID].Dot(ctx->dir);
   if (ctx->mode == EmbreeQueryContext::kInside) {
      if (nd <= 0.) // from inside, only exiting surfaces matter
         return;
   } else {
      if (nd > 0.) // from outside, only entering surfaces matter
         return;
   }

   const double t = rayFacet(ctx->org, ctx->dir, facet, vertices, 0.);
   if (t >= ctx->bestT)
      return;

   ctx->bestT = t;
   ctx->bestFacet = static_cast<int>(primID);

   // Shrink the Embree traversal interval so farther BVH nodes get pruned. The
   // bound is rounded up so it never excludes a facet closer than t.
   auto *rayhit = reinterpret_cast<RTCRayHit *>(args->rayhit);
   rayhit->ray.tfar = roundUp(t);
   rayhit->hit.geomID = args->geomID;
   rayhit->hit.primID = primID;
   rayhit->hit.instID[0] = ctx->instID[0];
}

// Release a scene built by BuildEmbreeGeometry, also freeing the EmbreeMesh
// stored as the geometry user data.
void releaseEmbreeScene(RTCScene scene)
{
   if (!scene)
      return;
   if (RTCGeometry geom = rtcGetGeometry(scene, 0))
      delete static_cast<EmbreeMesh *>(rtcGetGeometryUserData(geom));
   rtcReleaseScene(scene);
}

} // anonymous namespace

////////////////////////////////////////////////////////////////////////////////
/// Destructor. Releases the Embree scene and its mesh user data.

TGeoTessellatedEmbreeUser::~TGeoTessellatedEmbreeUser()
{
   releaseEmbreeScene((RTCScene)fEmbreeScene);
   fEmbreeScene = nullptr;
}

////////////////////////////////////////////////////////////////////////////////
/// Custom streamer. The acceleration structures (base BVH, normals and the
/// Embree scene) are transient and are rebuilt on read via CloseShape.

void TGeoTessellatedEmbreeUser::Streamer(TBuffer &b)
{
   if (b.IsReading()) {
      b.ReadClassBuffer(TGeoTessellatedEmbreeUser::Class(), this);
      CloseShape(false); // rebuild without re-running the (expensive) closure checks
   } else {
      b.WriteClassBuffer(TGeoTessellatedEmbreeUser::Class(), this);
   }
}

////////////////////////////////////////////////////////////////////////////////
/// Close the shape: build the base BVH/normals and, in addition, the Embree user
/// geometry used by this navigation backend.

void TGeoTessellatedEmbreeUser::CloseShape(bool check, bool fixFlipped, bool verbose)
{
   TGeoTessellated::CloseShape(check, fixFlipped, verbose);
   BuildEmbreeGeometry();
}

////////////////////////////////////////////////////////////////////////////////
/// Build the Embree user geometry: one user primitive per facet. Embree only
/// builds/traverses the BVH; intersection is performed in the user callbacks.

void TGeoTessellatedEmbreeUser::BuildEmbreeGeometry()
{
   const auto &facets = GetFacets();

   RTCDevice device = GetEmbreeDevice();

   // release a previously built scene (e.g. when re-closing the shape)
   releaseEmbreeScene((RTCScene)fEmbreeScene);
   fEmbreeScene = nullptr;

   // mesh pointers, owned by the geometry as its user data
   auto *mesh = new EmbreeMesh{&GetFacets(), &GetVertices(), &GetOutwardNormals()};

   RTCScene scene = rtcNewScene(device);
   rtcSetSceneBuildQuality(scene, RTC_BUILD_QUALITY_HIGH);
   rtcSetSceneFlags(scene, RTC_SCENE_FLAG_ROBUST);

   RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_USER);
   rtcSetGeometryUserPrimitiveCount(geom, (unsigned int)facets.size());
   rtcSetGeometryUserData(geom, mesh);
   rtcSetGeometryBoundsFunction(geom, boundsFunc, nullptr);
   rtcSetGeometryIntersectFunction(geom, intersectFunc);
   rtcCommitGeometry(geom);
   rtcAttachGeometry(scene, geom);
   rtcReleaseGeometry(geom);
   rtcCommitScene(scene);

   fEmbreeScene = (void *)scene;
}

////////////////////////////////////////////////////////////////////////////////
/// Helper running a distance query against the Embree user geometry. Returns the
/// nearest distance (double precision) or Big() if no facet was hit.

namespace {

double distQuery(RTCScene scene, EmbreeQueryContext::Mode mode, const Vertex_t &org, const Vertex_t &dir,
                 double stepmax)
{
   EmbreeQueryContext ctx;
   rtcInitRayQueryContext(&ctx);
   ctx.mode = mode;
   ctx.org = org;
   ctx.dir = dir;
   ctx.bestT = TGeoShape::Big();
   ctx.bestFacet = -1;
   ctx.crossings = 0;

   RTCRayHit ray;
   std::memset(&ray, 0, sizeof(ray));
   ray.ray.org_x = (float)org[0];
   ray.ray.org_y = (float)org[1];
   ray.ray.org_z = (float)org[2];
   ray.ray.dir_x = (float)dir[0];
   ray.ray.dir_y = (float)dir[1];
   ray.ray.dir_z = (float)dir[2];
   ray.ray.tnear = 0.f;
   ray.ray.tfar = (float)stepmax;
   ray.ray.mask = (unsigned int)(-1);
   ray.hit.geomID = RTC_INVALID_GEOMETRY_ID;
   ray.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

   RTCIntersectArguments iargs;
   rtcInitIntersectArguments(&iargs);
   iargs.context = &ctx;
   rtcIntersect1(scene, &ray, &iargs);

   return (ctx.bestFacet >= 0) ? ctx.bestT : TGeoShape::Big();
}

} // anonymous namespace

////////////////////////////////////////////////////////////////////////////////
/// DistFromInside using Embree as a BVH provider.

Double_t TGeoTessellatedEmbreeUser::DistFromInside(const Double_t *point, const Double_t *dir, Int_t iact,
                                                   Double_t stepmax, Double_t *safe) const
{
   const Vertex_t p{point[0], point[1], point[2]};
   const Vertex_t d{dir[0], dir[1], dir[2]};

   const double dist = distQuery((RTCScene)fEmbreeScene, EmbreeQueryContext::kInside, p, d, stepmax);
   if (dist >= Big()) {
      // No facet seen (e.g. a degenerate query) -> robust base navigation.
      return TGeoTessellated::DistFromInside(point, dir, iact, stepmax, safe);
   }
   return dist;
}

////////////////////////////////////////////////////////////////////////////////
/// DistFromOutside using Embree as a BVH provider.

Double_t TGeoTessellatedEmbreeUser::DistFromOutside(const Double_t *point, const Double_t *dir, Int_t iact,
                                                    Double_t stepmax, Double_t *safe) const
{
   // Quickly approach the solid using the bounding box, so that the Embree query
   // starts close to the surface.
   Double_t cpoint[3] = {point[0], point[1], point[2]};
   double offset = 0.;
   if (!TGeoBBox::Contains(point)) {
      const double dist_to_box = TGeoBBox::DistFromOutside(point, dir, 3, stepmax);
      if (dist_to_box >= Big()) {
         return Big(); // the ray does not even cross the bounding box
      }
      offset = 0.999 * dist_to_box;
      cpoint[0] += offset * dir[0];
      cpoint[1] += offset * dir[1];
      cpoint[2] += offset * dir[2];
   }

   const Vertex_t p{cpoint[0], cpoint[1], cpoint[2]};
   const Vertex_t d{dir[0], dir[1], dir[2]};

   const double dist = distQuery((RTCScene)fEmbreeScene, EmbreeQueryContext::kOutside, p, d, stepmax);
   if (dist >= Big()) {
      // robust fallback using the base navigation (from the original point)
      return TGeoTessellated::DistFromOutside(point, dir, iact, stepmax, safe);
   }
   return dist + offset;
}

////////////////////////////////////////////////////////////////////////////////
/// Contains using Embree (parity test along an arbitrary direction).

bool TGeoTessellatedEmbreeUser::Contains(const Double_t *point) const
{
   if (!TGeoBBox::Contains(point)) {
      return false;
   }

   // An arbitrary skewed test direction, probing all normals without obvious symmetries.
   const Vertex_t test_dir{1.0, 1.41421356237, 1.73205080757};
   const Vertex_t p{point[0], point[1], point[2]};

   EmbreeQueryContext ctx;
   rtcInitRayQueryContext(&ctx);
   ctx.mode = EmbreeQueryContext::kContains;
   ctx.org = p;
   ctx.dir = test_dir;
   ctx.bestT = TGeoShape::Big();
   ctx.bestFacet = -1;
   ctx.crossings = 0;

   RTCRayHit ray;
   std::memset(&ray, 0, sizeof(ray));
   ray.ray.org_x = (float)p[0];
   ray.ray.org_y = (float)p[1];
   ray.ray.org_z = (float)p[2];
   ray.ray.dir_x = (float)test_dir[0];
   ray.ray.dir_y = (float)test_dir[1];
   ray.ray.dir_z = (float)test_dir[2];
   ray.ray.tnear = 0.f;
   ray.ray.tfar = std::numeric_limits<float>::infinity();
   ray.ray.mask = (unsigned int)(-1);
   ray.hit.geomID = RTC_INVALID_GEOMETRY_ID;
   ray.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

   RTCIntersectArguments iargs;
   rtcInitIntersectArguments(&iargs);
   iargs.context = &ctx;
   rtcIntersect1((RTCScene)fEmbreeScene, &ray, &iargs);

   return ctx.crossings & 1;
}

////////////////////////////////////////////////////////////////////////////////
/// Safety using the Embree closest-point query, evaluated in double precision.

Double_t TGeoTessellatedEmbreeUser::Safety(const Double_t *point, Bool_t /*in*/) const
{
   struct SafetyContext {
      const EmbreeMesh *mesh;
      Vec3f<double> p;
      double minDist2;
   };

   auto pointQueryFunc = [](RTCPointQueryFunctionArguments *args) -> bool {
      auto *ctx = static_cast<SafetyContext *>(args->userPtr);
      const double d2 = pointFacetDistSq<double>(ctx->p, (*ctx->mesh->facets)[args->primID], *ctx->mesh->vertices);
      if (d2 < ctx->minDist2) {
         ctx->minDist2 = d2;
         args->query->radius = (float)std::sqrt(d2); // prune the BVH traversal
         return true;                                // radius shrank
      }
      return false;
   };

   auto scene = (RTCScene)fEmbreeScene;

   SafetyContext ctx{static_cast<const EmbreeMesh *>(rtcGetGeometryUserData(rtcGetGeometry(scene, 0))),
                     Vec3f<double>(point[0], point[1], point[2]), std::numeric_limits<double>::infinity()};

   RTCPointQuery query;
   query.x = (float)point[0];
   query.y = (float)point[1];
   query.z = (float)point[2];
   query.radius = std::numeric_limits<float>::infinity();
   query.time = 0.0f;

   RTCPointQueryContext qctx;
   rtcInitPointQueryContext(&qctx);
   rtcPointQuery(scene, &query, &qctx, pointQueryFunc, &ctx);

   return std::nextafter(std::sqrt(ctx.minDist2), 0.0);
}

#endif // R__HAS_EMBREE
