// @(#)root/geom
// Author: Sandro Wenzel   2026-01

/*************************************************************************
 * Copyright (C) 1995-2000, Rene Brun and Fons Rademakers.               *
 * All rights reserved.                                                  *
 *                                                                       *
 * For the licensing terms see $ROOTSYS/LICENSE.                         *
 * For the list of contributors see $ROOTSYS/README/CREDITS.             *
 *************************************************************************/

/** \class TGeoTessellatedEmbree
\ingroup Geometry_classes

Tessellated solid navigated with the Intel Embree ray-tracing library.

Embree is used to cull the candidate facets for a given query; the final
intersection or distance is then evaluated in double precision using the same
geometric kernels as TGeoTessellated, so the results stay consistent with the
base class up to floating point precision. When Embree returns no candidate
(e.g. a watertightness gap of the single precision acceleration structure), the
query transparently falls back to the robust BVH navigation of the base class.
*/

#include "TGeoTessellatedEmbree.h"

#ifdef R__HAS_EMBREE

#include "TBuffer.h"
#include "TError.h"
#include "TGeoTessellatedHelpers.hxx"

#include <embree4/rtcore.h>

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

ClassImp(TGeoTessellatedEmbree);

namespace {

// Singleton Embree device shared within the process.
RTCDevice GetEmbreeDevice()
{
   static RTCDevice device = [] {
      RTCDevice d = rtcNewDevice(nullptr);
      if (!d) {
         throw std::runtime_error("TGeoTessellatedEmbree: failed to create Embree device");
      }
      return d;
   }();
   return device;
}

// Per-query context, used to collect the candidate primitives in a thread-safe
// way (the candidate buffer is thread-local and reached through this context
// rather than via shared geometry user data).
struct EmbreeQueryContext : public RTCRayQueryContext {
   std::vector<int> *candidates = nullptr;
};

// Initialize an Embree ray before a query.
template <typename T, typename W>
inline void initEmbreeRay(T px, T py, T pz, T dx, T dy, T dz, W stepmax, RTCRayHit &ray)
{
   std::memset(&ray, 0, sizeof(ray));
   ray.ray.org_x = static_cast<float>(px);
   ray.ray.org_y = static_cast<float>(py);
   ray.ray.org_z = static_cast<float>(pz);
   ray.ray.dir_x = static_cast<float>(dx);
   ray.ray.dir_y = static_cast<float>(dy);
   ray.ray.dir_z = static_cast<float>(dz);
   ray.ray.tnear = 0.f;
   ray.ray.tfar = static_cast<float>(stepmax);
   ray.ray.mask = static_cast<unsigned int>(-1);
   ray.hit.geomID = RTC_INVALID_GEOMETRY_ID;
   ray.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
}

// Filter used by the distance queries: collect every primitive Embree evaluates
// and keep traversing past coincident-at-origin hits (tfar <= 0).
void collectFilterDist(const RTCFilterFunctionNArguments *args)
{
   auto *ctx = static_cast<EmbreeQueryContext *>(args->context);
   const unsigned int objID = RTCHitN_primID(args->hit, 1, 0);
   ctx->candidates->push_back(static_cast<int>(objID));
   const float tfar = RTCRayN_tfar(args->ray, 1, 0);
   if (tfar <= 0.f) {
      args->valid[0] = 0; // reject so traversal continues to the next surface
   }
}

// Filter used by Contains: invalidate every hit so that all crossed surfaces
// are seen (needed for the parity test).
void collectFilterContains(const RTCFilterFunctionNArguments *args)
{
   auto *ctx = static_cast<EmbreeQueryContext *>(args->context);
   const unsigned int objID = RTCHitN_primID(args->hit, 1, 0);
   if (ctx->candidates->empty() || ctx->candidates->back() != static_cast<int>(objID)) {
      ctx->candidates->push_back(static_cast<int>(objID));
   }
   args->valid[0] = 0; // keep traversing
}

} // anonymous namespace

////////////////////////////////////////////////////////////////////////////////
/// Destructor. Releases the Embree scene.

TGeoTessellatedEmbree::~TGeoTessellatedEmbree()
{
   if (fEmbreeScene) {
      rtcReleaseScene((RTCScene)fEmbreeScene);
      fEmbreeScene = nullptr;
   }
}

////////////////////////////////////////////////////////////////////////////////
/// Custom streamer. The acceleration structures (BVH, normals, neighbours and
/// the Embree scene) are all transient and are rebuilt on read via CloseShape.

void TGeoTessellatedEmbree::Streamer(TBuffer &b)
{
   if (b.IsReading()) {
      b.ReadClassBuffer(TGeoTessellatedEmbree::Class(), this);
      CloseShape(false); // rebuild without re-running the (expensive) closure checks
   } else {
      b.WriteClassBuffer(TGeoTessellatedEmbree::Class(), this);
   }
}

////////////////////////////////////////////////////////////////////////////////
/// Close the shape: build the base BVH/normals and, in addition, the neighbour
/// topology and the Embree scene used by this navigation backend.

void TGeoTessellatedEmbree::CloseShape(bool check, bool fixFlipped, bool verbose)
{
   TGeoTessellated::CloseShape(check, fixFlipped, verbose);
   InitNeighbours();
   BuildEmbreeGeometry();
}

////////////////////////////////////////////////////////////////////////////////
/// Build the vertex -> facets lookup, used to recheck vertex-sharing neighbours
/// of an Embree candidate in the rare cases where the single precision hit could
/// not be confirmed in double precision.

void TGeoTessellatedEmbree::InitNeighbours()
{
   const auto &facets = GetFacets();
   const auto &vertices = GetVertices();

   fVertexIdToFacets.assign(vertices.size(), {});
   for (int f = 0; f < (int)facets.size(); ++f) {
      const auto &facet = facets[f];
      const int nv = facet.GetNvert();
      for (int i = 0; i < nv; ++i) {
         auto &container = fVertexIdToFacets[facet[i]];
         if (std::find(container.begin(), container.end(), f) == container.end()) {
            container.push_back(f);
         }
      }
   }
}

////////////////////////////////////////////////////////////////////////////////
/// Build the Embree scene from the facets. Quads are triangulated into two
/// triangles; fEmbreePrimToFacet maps every Embree triangle back to its facet.

void TGeoTessellatedEmbree::BuildEmbreeGeometry()
{
   const auto &facets = GetFacets();
   const auto &vertices = GetVertices();

   // count triangles (quads -> 2 triangles)
   int ntri = 0;
   for (const auto &facet : facets)
      ntri += (facet.GetNvert() == 4) ? 2 : 1;

   RTCDevice device = GetEmbreeDevice();

   // release a previously built scene (e.g. when re-closing the shape)
   if (fEmbreeScene) {
      rtcReleaseScene((RTCScene)fEmbreeScene);
      fEmbreeScene = nullptr;
   }

   RTCScene scene = rtcNewScene(device);
   // the BVH is built on the level of a scene
   rtcSetSceneBuildQuality(scene, RTC_BUILD_QUALITY_HIGH);
   rtcSetSceneFlags(scene, RTC_SCENE_FLAG_ROBUST);

   RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
   rtcSetGeometryBuildQuality(geom, RTC_BUILD_QUALITY_HIGH);
   rtcSetGeometryTimeStepCount(geom, 1);

   struct EVertex {
      float x, y, z;
   };
   struct ETriangle {
      int v0, v1, v2;
   };

   const int nvertices = 3 * ntri;
   auto *ev = (EVertex *)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(EVertex),
                                                 nvertices);
   auto *et = (ETriangle *)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(ETriangle),
                                                   ntri);

   fEmbreePrimToFacet.clear();
   fEmbreePrimToFacet.reserve(ntri);

   int vcount = 0;
   int tcount = 0;
   auto addTriangle = [&](const Vertex_t &a, const Vertex_t &b, const Vertex_t &c, int facetId) {
      ev[vcount] = {(float)a[0], (float)a[1], (float)a[2]};
      et[tcount].v0 = vcount++;
      ev[vcount] = {(float)b[0], (float)b[1], (float)b[2]};
      et[tcount].v1 = vcount++;
      ev[vcount] = {(float)c[0], (float)c[1], (float)c[2]};
      et[tcount].v2 = vcount++;
      fEmbreePrimToFacet.push_back(facetId);
      ++tcount;
   };

   for (int f = 0; f < (int)facets.size(); ++f) {
      const auto &facet = facets[f];
      const auto &v0 = vertices[facet[0]];
      const auto &v1 = vertices[facet[1]];
      const auto &v2 = vertices[facet[2]];
      addTriangle(v0, v1, v2, f);
      if (facet.GetNvert() == 4) {
         const auto &v3 = vertices[facet[3]];
         addTriangle(v0, v2, v3, f);
      }
   }

   // enable filter callbacks passed through the intersect arguments
   rtcSetGeometryEnableFilterFunctionFromArguments(geom, true);
   rtcCommitGeometry(geom);
   rtcAttachGeometry(scene, geom);
   rtcReleaseGeometry(geom);
   rtcCommitScene(scene);

   fEmbreeScene = (void *)scene;
}

////////////////////////////////////////////////////////////////////////////////
/// DistFromInside using Embree.

Double_t TGeoTessellatedEmbree::DistFromInside(const Double_t *point, const Double_t *dir, Int_t iact, Double_t stepmax,
                                              Double_t *safe) const
{
   auto scene = (RTCScene)fEmbreeScene;

   static thread_local std::vector<int> candidates;
   candidates.clear();

   EmbreeQueryContext ctx;
   rtcInitRayQueryContext(&ctx);
   ctx.candidates = &candidates;

   RTCRayHit ray;
   initEmbreeRay(point[0], point[1], point[2], dir[0], dir[1], dir[2], stepmax, ray);

   RTCIntersectArguments iargs;
   rtcInitIntersectArguments(&iargs);
   iargs.context = &ctx;
   iargs.filter = collectFilterDist;
   iargs.feature_mask = RTC_FEATURE_FLAG_ALL;
   rtcIntersect1(scene, &ray, &iargs);

   if (candidates.empty()) {
      // Embree did not see any surface (watertightness gap) -> robust fallback
      return TGeoTessellated::DistFromInside(point, dir, iact, stepmax, safe);
   }

   const auto &facets = GetFacets();
   const auto &vertices = GetVertices();
   const auto &normals = GetOutwardNormals();
   const Vertex_t p{point[0], point[1], point[2]};
   const Vertex_t d{dir[0], dir[1], dir[2]};

   double dist = Big();
   bool confirmed = false;
   for (int primID : candidates) {
      const int f = fEmbreePrimToFacet[primID];
      // only exiting surfaces are relevant (from inside the dot product is positive)
      if (normals[f].Dot(d) <= 0.)
         continue;
      const double t = rayFacet(p, d, facets[f], vertices, 0.);
      if (t < dist) {
         dist = t;
         confirmed = true;
      }
   }

   if (!confirmed) {
      // rare: also probe vertex-sharing neighbours of the candidates
      for (int primID : candidates) {
         const int cand = fEmbreePrimToFacet[primID];
         const auto &facet = facets[cand];
         const int nv = facet.GetNvert();
         for (int vi = 0; vi < nv; ++vi) {
            for (int nb : fVertexIdToFacets[facet[vi]]) {
               if (nb == cand || normals[nb].Dot(d) <= 0.)
                  continue;
               const double t = rayFacet(p, d, facets[nb], vertices, 0.);
               if (t < dist)
                  dist = t;
            }
         }
      }
   }

   return dist;
}

////////////////////////////////////////////////////////////////////////////////
/// DistFromOutside using Embree.

Double_t TGeoTessellatedEmbree::DistFromOutside(const Double_t *point, const Double_t *dir, Int_t iact, Double_t stepmax,
                                               Double_t *safe) const
{
   // Quickly approach the solid using the bounding box, so that the Embree query
   // starts close to the surface (better single precision behaviour).
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

   auto scene = (RTCScene)fEmbreeScene;

   static thread_local std::vector<int> candidates;
   candidates.clear();

   EmbreeQueryContext ctx;
   rtcInitRayQueryContext(&ctx);
   ctx.candidates = &candidates;

   RTCRayHit ray;
   initEmbreeRay(cpoint[0], cpoint[1], cpoint[2], dir[0], dir[1], dir[2], stepmax, ray);

   RTCIntersectArguments iargs;
   rtcInitIntersectArguments(&iargs);
   iargs.context = &ctx;
   iargs.filter = collectFilterDist;
   iargs.feature_mask = RTC_FEATURE_FLAG_ALL;
   rtcIntersect1(scene, &ray, &iargs);

   if (candidates.empty()) {
      // robust fallback using the base navigation (from the original point)
      return TGeoTessellated::DistFromOutside(point, dir, iact, stepmax, safe);
   }

   const auto &facets = GetFacets();
   const auto &vertices = GetVertices();
   const auto &normals = GetOutwardNormals();
   const Vertex_t p{cpoint[0], cpoint[1], cpoint[2]};
   const Vertex_t d{dir[0], dir[1], dir[2]};

   double dist = Big();
   bool confirmed = false;
   for (int primID : candidates) {
      const int f = fEmbreePrimToFacet[primID];
      // only entering surfaces are relevant (from outside the dot product is negative)
      if (normals[f].Dot(d) > 0.)
         continue;
      const double t = rayFacet(p, d, facets[f], vertices, 0.);
      if (t < dist) {
         dist = t;
         confirmed = true;
      }
   }

   if (!confirmed) {
      // rare: also probe vertex-sharing neighbours of the candidates
      for (int primID : candidates) {
         const int cand = fEmbreePrimToFacet[primID];
         const auto &facet = facets[cand];
         const int nv = facet.GetNvert();
         for (int vi = 0; vi < nv; ++vi) {
            for (int nb : fVertexIdToFacets[facet[vi]]) {
               if (nb == cand || normals[nb].Dot(d) > 0.)
                  continue;
               const double t = rayFacet(p, d, facets[nb], vertices, 0.);
               if (t < dist)
                  dist = t;
            }
         }
      }
   }

   return dist + offset;
}

////////////////////////////////////////////////////////////////////////////////
/// Contains using Embree (parity test along an arbitrary direction).

bool TGeoTessellatedEmbree::Contains(const Double_t *point) const
{
   if (!TGeoBBox::Contains(point)) {
      return false;
   }

   // An arbitrary skewed test direction, probing all normals without obvious symmetries.
   const Vertex_t test_dir{1.0, 1.41421356237, 1.73205080757};

   auto scene = (RTCScene)fEmbreeScene;

   static thread_local std::vector<int> candidates;
   candidates.clear();

   EmbreeQueryContext ctx;
   rtcInitRayQueryContext(&ctx);
   ctx.candidates = &candidates;

   RTCRayHit ray;
   initEmbreeRay(point[0], point[1], point[2], test_dir[0], test_dir[1], test_dir[2],
                 std::numeric_limits<float>::infinity(), ray);

   RTCIntersectArguments iargs;
   rtcInitIntersectArguments(&iargs);
   iargs.context = &ctx;
   iargs.filter = collectFilterContains;
   iargs.feature_mask = RTC_FEATURE_FLAG_ALL;
   rtcIntersect1(scene, &ray, &iargs);

   // Map the Embree triangles to unique facets (a quad maps to two triangles)
   // and count the actual crossings in double precision.
   static thread_local std::vector<int> facetHits;
   facetHits.clear();
   facetHits.reserve(candidates.size());
   for (int primID : candidates)
      facetHits.push_back(fEmbreePrimToFacet[primID]);
   std::sort(facetHits.begin(), facetHits.end());
   facetHits.erase(std::unique(facetHits.begin(), facetHits.end()), facetHits.end());

   const auto &facets = GetFacets();
   const auto &vertices = GetVertices();
   const Vertex_t p{point[0], point[1], point[2]};

   int crossings = 0;
   for (int f : facetHits) {
      if (rayFacetHit(p, test_dir, facets[f], vertices, 0.))
         ++crossings;
   }
   return crossings & 1;
}

////////////////////////////////////////////////////////////////////////////////
/// Safety using the Embree closest-point query.

Double_t TGeoTessellatedEmbree::Safety(const Double_t *point, Bool_t /*in*/) const
{
   struct ClosestPointContext {
      float minDist2;
      RTCGeometry geom;
   };

   auto pointQueryFunc = [](RTCPointQueryFunctionArguments *args) -> bool {
      auto *ctx = static_cast<ClosestPointContext *>(args->userPtr);
      const unsigned int primID = args->primID;

      const float *vertices = (const float *)rtcGetGeometryBufferData(ctx->geom, RTC_BUFFER_TYPE_VERTEX, 0);
      const unsigned int *indices = (const unsigned int *)rtcGetGeometryBufferData(ctx->geom, RTC_BUFFER_TYPE_INDEX, 0);

      const unsigned int i0 = indices[3 * primID + 0];
      const unsigned int i1 = indices[3 * primID + 1];
      const unsigned int i2 = indices[3 * primID + 2];
      const float *v0 = vertices + 3 * i0;
      const float *v1 = vertices + 3 * i1;
      const float *v2 = vertices + 3 * i2;

      const float dist2 =
         pointTriangleDistSq(Vec3f<float>(args->query->x, args->query->y, args->query->z),
                             Vec3f<float>(v0[0], v0[1], v0[2]), Vec3f<float>(v1[0], v1[1], v1[2]),
                             Vec3f<float>(v2[0], v2[1], v2[2]));

      if (dist2 < ctx->minDist2) {
         ctx->minDist2 = dist2;
         // update the traversal radius (used by Embree to prune the BVH)
         args->query->radius = std::sqrt(dist2);
      }
      return true; // continue traversal
   };

   auto scene = (RTCScene)fEmbreeScene;

   ClosestPointContext ctx;
   ctx.minDist2 = std::numeric_limits<float>::infinity();
   ctx.geom = rtcGetGeometry(scene, 0);

   RTCPointQuery query;
   query.x = static_cast<float>(point[0]);
   query.y = static_cast<float>(point[1]);
   query.z = static_cast<float>(point[2]);
   query.radius = std::numeric_limits<float>::infinity();
   query.time = 0.0f;

   RTCPointQueryContext qctx;
   rtcInitPointQueryContext(&qctx);
   rtcPointQuery(scene, &query, &qctx, pointQueryFunc, &ctx);

   return std::nextafter(std::sqrt((double)ctx.minDist2), 0.0);
}

#endif // R__HAS_EMBREE
