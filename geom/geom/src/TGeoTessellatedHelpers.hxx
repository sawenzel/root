// @(#)root/geom

/*************************************************************************
 * Copyright (C) 1995-2000, Rene Brun and Fons Rademakers.               *
 * All rights reserved.                                                  *
 *                                                                       *
 * For the licensing terms see $ROOTSYS/LICENSE.                         *
 * For the list of contributors see $ROOTSYS/README/CREDITS.             *
 *************************************************************************/

#ifndef ROOT_TGeoTessellatedHelpers
#define ROOT_TGeoTessellatedHelpers

// Internal (non-installed) header collecting the small geometric kernels shared
// between the BVH navigation in TGeoTessellated.cxx and the Embree navigation in
// TGeoTessellatedEmbree.cxx. The helpers live in an anonymous namespace so each
// translation unit gets its own internal-linkage copy.

#include "TGeoTessellated.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace {

using Vertex_t = Tessellated::Vertex_t;

// The classic Moeller-Trumbore ray triangle-intersection kernel:
// - Compute triangle edges e1, e2
// - Compute determinant det
// - Reject parallel rays
// - Compute barycentric coordinates u, v
// - Compute ray parameter t
inline double rayTriangle(const Vertex_t &orig, const Vertex_t &dir, const Vertex_t &v0, const Vertex_t &v1,
                          const Vertex_t &v2, double rayEPS = 1e-8)
{
   constexpr double EPS = 1e-8;
   const double INF = std::numeric_limits<double>::infinity();
   Vertex_t e1{v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]};
   Vertex_t e2{v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]};
   auto p = Vertex_t::Cross(dir, e2);
   auto det = e1.Dot(p);
   if (std::abs(det) <= EPS) {
      return INF;
   }

   Vertex_t tvec{orig[0] - v0[0], orig[1] - v0[1], orig[2] - v0[2]};
   auto invDet = 1.0 / det;
   auto u = tvec.Dot(p) * invDet;
   if (u < 0.0 || u > 1.0) {
      return INF;
   }
   auto q = Vertex_t::Cross(tvec, e1);
   auto v = dir.Dot(q) * invDet;
   if (v < 0.0 || u + v > 1.0) {
      return INF;
   }
   auto t = e2.Dot(q) * invDet;
   return (t > rayEPS) ? t : INF;
}

inline double rayFacet(const Vertex_t &orig, const Vertex_t &dir, const TGeoFacet &facet,
                       const std::vector<Vertex_t> &vertices, double rayEPS = 1e-8)
{
   // Keep the stored facet topology intact and triangulate quads only for geometric queries.
   const auto &v0 = vertices[facet[0]];
   const auto &v1 = vertices[facet[1]];
   const auto &v2 = vertices[facet[2]];
   auto t = rayTriangle(orig, dir, v0, v1, v2, rayEPS);
   if (facet.GetNvert() == 3)
      return t;
   const auto &v3 = vertices[facet[3]];
   auto t2 = rayTriangle(orig, dir, v0, v2, v3, rayEPS);
   return std::min(t, t2);
}

inline bool rayFacetHit(const Vertex_t &orig, const Vertex_t &dir, const TGeoFacet &facet,
                        const std::vector<Vertex_t> &vertices, double rayEPS = 1e-8)
{
   // Contains/parity checks only need a boolean hit, so keep the finite-distance test in one place.
   return rayFacet(orig, dir, facet, vertices, rayEPS) != std::numeric_limits<double>::infinity();
}

template <typename T = float>
struct Vec3f {
   T x, y, z;
   Vec3f(T x_, T y_, T z_) : x(x_), y(y_), z(z_){};
};

template <typename T>
inline Vec3f<T> operator-(const Vec3f<T> &a, const Vec3f<T> &b)
{
   return {a.x - b.x, a.y - b.y, a.z - b.z};
}

template <typename T>
inline Vec3f<T> cross(const Vec3f<T> &a, const Vec3f<T> &b)
{
   return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

template <typename T>
inline T dot(const Vec3f<T> &a, const Vec3f<T> &b)
{
   return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Kernel to get closest/shortest distance between a point and a triangle (a,b,c).
// Performed by default in float since Safety can be approximate.
// Project point onto triangle plane
// If projection lies inside → distance to plane
// Otherwise compute min distance to the three edges
// Return squared distance
template <typename T = float>
T pointTriangleDistSq(const Vec3f<T> &p, const Vec3f<T> &a, const Vec3f<T> &b, const Vec3f<T> &c)
{
   // Edges
   Vec3f<T> ab = b - a;
   Vec3f<T> ac = c - a;
   Vec3f<T> ap = p - a;

   auto d1 = dot(ab, ap);
   auto d2 = dot(ac, ap);
   if (d1 <= T(0.0) && d2 <= T(0.0)) {
      return dot(ap, ap); // barycentric (1,0,0)
   }

   Vec3f<T> bp = p - b;
   auto d3 = dot(ab, bp);
   auto d4 = dot(ac, bp);
   if (d3 >= T(0.0) && d4 <= d3) {
      return dot(bp, bp); // (0,1,0)
   }

   T vc = d1 * d4 - d3 * d2;
   if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
      T v = d1 / (d1 - d3);
      Vec3f<T> proj = {a.x + v * ab.x, a.y + v * ab.y, a.z + v * ab.z};
      Vec3f<T> d = p - proj;
      return dot(d, d); // edge AB
   }

   Vec3f<T> cp = p - c;
   T d5 = dot(ab, cp);
   T d6 = dot(ac, cp);
   if (d6 >= T(0.0f) && d5 <= d6) {
      return dot(cp, cp); // (0,0,1)
   }

   T vb = d5 * d2 - d1 * d6;
   if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
      T w = d2 / (d2 - d6);
      Vec3f<T> proj = {a.x + w * ac.x, a.y + w * ac.y, a.z + w * ac.z};
      Vec3f<T> d = p - proj;
      return dot(d, d); // edge AC
   }

   T va = d3 * d6 - d5 * d4;
   if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
      T w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
      Vec3f<T> proj = {b.x + w * (c.x - b.x), b.y + w * (c.y - b.y), b.z + w * (c.z - b.z)};
      Vec3f<T> d = p - proj;
      return dot(d, d); // edge BC
   }

   // Inside face region
   T denom = T(1.0f) / (va + vb + vc);
   T v = vb * denom;
   T w = vc * denom;

   Vec3f<T> proj = {a.x + ab.x * v + ac.x * w, a.y + ab.y * v + ac.y * w, a.z + ab.z * v + ac.z * w};

   Vec3f<T> d = p - proj;
   return dot(d, d);
}

template <typename T = float>
T pointFacetDistSq(const Vec3f<T> &p, const TGeoFacet &facet, const std::vector<Vertex_t> &vertices)
{
   // Safety uses the same on-the-fly split as ray queries so triangles and quads stay consistent.
   const auto &v0 = vertices[facet[0]];
   const auto &v1 = vertices[facet[1]];
   const auto &v2 = vertices[facet[2]];
   auto d = pointTriangleDistSq(p, Vec3f<T>(v0[0], v0[1], v0[2]), Vec3f<T>(v1[0], v1[1], v1[2]),
                                Vec3f<T>(v2[0], v2[1], v2[2]));
   if (facet.GetNvert() == 3)
      return d;
   const auto &v3 = vertices[facet[3]];
   auto d2 = pointTriangleDistSq(p, Vec3f<T>(v0[0], v0[1], v0[2]), Vec3f<T>(v2[0], v2[1], v2[2]),
                                 Vec3f<T>(v3[0], v3[1], v3[2]));
   return std::min(d, d2);
}

} // anonymous namespace

#endif
