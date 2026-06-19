// @(#)root/geom
// Author: Sandro Wenzel   2026-01

/*************************************************************************
 * Copyright (C) 1995-2000, Rene Brun and Fons Rademakers.               *
 * All rights reserved.                                                  *
 *                                                                       *
 * For the licensing terms see $ROOTSYS/LICENSE.                         *
 * For the list of contributors see $ROOTSYS/README/CREDITS.             *
 *************************************************************************/

#ifndef ROOT_TGeoTessellatedEmbree
#define ROOT_TGeoTessellatedEmbree

#include "TGeoTessellated.h"

#include <vector>

// The class is only available when ROOT is built with Intel Embree support.
#ifdef R__HAS_EMBREE

/** \class TGeoTessellatedEmbree
\ingroup Geometry_classes

Tessellated solid navigated with the Intel Embree ray-tracing library.

This is a drop-in replacement for TGeoTessellated which overrides the navigation
functions (Contains, Safety, DistFromInside, DistFromOutside) with implementations
based on Intel Embree. Embree is used to quickly cull the candidate facets; the
final intersection / distance is then computed in double precision using the same
geometric kernels as the base class, so the results are consistent with
TGeoTessellated up to floating point precision.

The class is only available when ROOT is configured with `-Dembree=ON` and a
suitable Embree 4 installation is found.
*/

class TGeoTessellatedEmbree : public TGeoTessellated {
public:
   using Vertex_t = Tessellated::Vertex_t;

   // constructors (mirroring TGeoTessellated)
   TGeoTessellatedEmbree() {}
   TGeoTessellatedEmbree(const char *name, int nfacets = 0) : TGeoTessellated(name, nfacets) {}
   TGeoTessellatedEmbree(const char *name, const std::vector<Vertex_t> &vertices) : TGeoTessellated(name, vertices) {}
   ~TGeoTessellatedEmbree() override;

   // Build the BVH/normals (base class) and, in addition, the Embree scene and
   // the neighbour topology used for the robust double-precision rechecks.
   void CloseShape(bool check = true, bool fixFlipped = true, bool verbose = true) override;

   // navigation functions overridden with Embree implementations
   Double_t DistFromOutside(const Double_t *point, const Double_t *dir, Int_t iact = 1, Double_t step = TGeoShape::Big(),
                            Double_t *safe = nullptr) const override;
   Double_t DistFromInside(const Double_t *point, const Double_t *dir, Int_t iact = 1, Double_t step = TGeoShape::Big(),
                           Double_t *safe = nullptr) const override;
   bool Contains(const Double_t *point) const override;
   Double_t Safety(const Double_t *point, Bool_t in = kTRUE) const override;

private:
   TGeoTessellatedEmbree(const TGeoTessellatedEmbree &) = delete;
   TGeoTessellatedEmbree &operator=(const TGeoTessellatedEmbree &) = delete;

   void BuildEmbreeGeometry(); // builds the Embree scene from the facets
   void InitNeighbours();      // builds vertex->facet topology for robust rechecks

   // transient acceleration data (rebuilt on closure / after I/O)
   void *fEmbreeScene = nullptr;                  //! RTCScene with the triangulated facets
   std::vector<int> fEmbreePrimToFacet;           //! maps Embree triangle id -> facet index (quads -> 2 triangles)
   std::vector<std::vector<int>> fVertexIdToFacets; //! all facets sharing a given vertex

   ClassDefOverride(TGeoTessellatedEmbree, 1) // tessellated shape navigated with Intel Embree
};

#endif // R__HAS_EMBREE

#endif
