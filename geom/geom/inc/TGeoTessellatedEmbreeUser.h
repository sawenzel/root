// @(#)root/geom
// Author: Sandro Wenzel   2026-01

/*************************************************************************
 * Copyright (C) 1995-2000, Rene Brun and Fons Rademakers.               *
 * All rights reserved.                                                  *
 *                                                                       *
 * For the licensing terms see $ROOTSYS/LICENSE.                         *
 * For the list of contributors see $ROOTSYS/README/CREDITS.             *
 *************************************************************************/

#ifndef ROOT_TGeoTessellatedEmbreeUser
#define ROOT_TGeoTessellatedEmbreeUser

#include "TGeoTessellated.h"

// The class is only available when ROOT is built with Intel Embree support.
#ifdef R__HAS_EMBREE

/** \class TGeoTessellatedEmbreeUser
\ingroup Geometry_classes

Tessellated solid navigated with Intel Embree used as a pure BVH provider.

This is a drop-in replacement for TGeoTessellated which overrides the navigation
functions (Contains, Safety, DistFromInside, DistFromOutside) with implementations
based on Intel Embree. In contrast to TGeoTessellatedEmbree (which builds a
triangle scene and intersects in single precision), this version registers an
Embree `RTC_GEOMETRY_TYPE_USER` geometry: Embree only builds and traverses the
bounding-volume hierarchy, while the ray/primitive intersection is performed
entirely in user space, in double precision, using the same geometric kernels as
the base class. Each Embree user primitive corresponds to exactly one facet, so
the Embree primitive id is the facet index (no remapping and no neighbour
rechecks are needed).

The class is only available when ROOT is configured with `-Dembree=ON` and a
suitable Embree 4 installation is found.
*/

class TGeoTessellatedEmbreeUser : public TGeoTessellated {
public:
   using Vertex_t = Tessellated::Vertex_t;

   // constructors (mirroring TGeoTessellated)
   TGeoTessellatedEmbreeUser() {}
   TGeoTessellatedEmbreeUser(const char *name, int nfacets = 0) : TGeoTessellated(name, nfacets) {}
   TGeoTessellatedEmbreeUser(const char *name, const std::vector<Vertex_t> &vertices) : TGeoTessellated(name, vertices)
   {
   }
   ~TGeoTessellatedEmbreeUser() override;

   // Build the BVH/normals (base class) and, in addition, the Embree user geometry.
   void CloseShape(bool check = true, bool fixFlipped = true, bool verbose = true) override;

   // navigation functions overridden with Embree implementations
   Double_t DistFromOutside(const Double_t *point, const Double_t *dir, Int_t iact = 1, Double_t step = TGeoShape::Big(),
                            Double_t *safe = nullptr) const override;
   Double_t DistFromInside(const Double_t *point, const Double_t *dir, Int_t iact = 1, Double_t step = TGeoShape::Big(),
                           Double_t *safe = nullptr) const override;
   bool Contains(const Double_t *point) const override;
   Double_t Safety(const Double_t *point, Bool_t in = kTRUE) const override;

private:
   TGeoTessellatedEmbreeUser(const TGeoTessellatedEmbreeUser &) = delete;
   TGeoTessellatedEmbreeUser &operator=(const TGeoTessellatedEmbreeUser &) = delete;

   void BuildEmbreeGeometry(); // builds the Embree user geometry from the facets

   // transient acceleration data (rebuilt on closure / after I/O)
   void *fEmbreeScene = nullptr; //! RTCScene holding the per-facet user geometry

   ClassDefOverride(TGeoTessellatedEmbreeUser, 1) // tessellated shape navigated with Intel Embree (user geometry)
};

#endif // R__HAS_EMBREE

#endif
