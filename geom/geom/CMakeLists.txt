# Copyright (C) 1995-2019, Rene Brun and Fons Rademakers.
# All rights reserved.
#
# For the licensing terms see $ROOTSYS/LICENSE.
# For the list of contributors see $ROOTSYS/README/CREDITS.

############################################################################
# CMakeLists.txt file for building ROOT geom/geom package
############################################################################

option(ENABLE_EMBREE "Enable Intel Embree" OFF)

# we look for Intel Embree which can help with geometry navigation
if(ENABLE_EMBREE)
  find_package(Embree 4 QUIET)
  if(Embree_FOUND)
    message(STATUS "Geom: Embree support enabled")
    set(GEOM_HAS_EMBREE TRUE)
  else()
    message(STATUS "Geom: Embree not found, disabling Embree support")
  endif()
endif()

set(_geom_extra_deps)
if(GEOM_HAS_EMBREE)
  get_target_property(_embree_includes embree INTERFACE_INCLUDE_DIRECTORIES)
  set_property(TARGET embree
    PROPERTY INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_embree_includes}"
  )
  list(APPEND _geom_extra_deps embree)
endif()

ROOT_STANDARD_LIBRARY_PACKAGE(Geom
  HEADERS
    TGDMLMatrix.h
    TGeoArb8.h
    TGeoAtt.h
    TGeoBBox.h
    TGeoBoolNode.h
    TGeoBranchArray.h
    TGeoBuilder.h
    TGeoCache.h
    TGeoCompositeShape.h
    TGeoCone.h
    TGeoElement.h
    TGeoEltu.h
    TGeoExtension.h
    TGeoGlobalMagField.h
    TGeoHalfSpace.h
    TGeoHelix.h
    TGeoHype.h
    TGeoManager.h
    TGeoMaterial.h
    TGeoMatrix.h
    TGeoMedium.h
    TGeoNavigator.h
    TGeoNode.h
    TGeoOpticalSurface.h
    TGeoPara.h
    TGeoParaboloid.h
    TGeoParallelWorld.h
    TGeoPatternFinder.h
    TGeoPcon.h
    TGeoPgon.h
    TGeoPhysicalConstants.h
    TGeoPhysicalNode.h
    TGeoPolygon.h
    TGeoRCPtr.h
    TGeoRegion.h
    TGeoScaledShape.h
    TGeoShape.h
    TGeoShapeAssembly.h
    TGeoSphere.h
    TGeoStateInfo.h
    TGeoSystemOfUnits.h
    TGeoTorus.h
    TGeoTrd1.h
    TGeoTrd2.h
    TGeoTube.h
    TGeoUniformMagField.h
    TGeoVolume.h
    TGeoVoxelFinder.h
    TGeoXtru.h
    TGeoTessellated.h
    TGeoVector3.h
    TVirtualGeoConverter.h
    TVirtualGeoPainter.h
    TVirtualGeoTrack.h
    TVirtualMagField.h
  SOURCES
    src/TGDMLMatrix.cxx
    src/TGeoArb8.cxx
    src/TGeoAtt.cxx
    src/TGeoBBox.cxx
    src/TGeoBoolNode.cxx
    src/TGeoBranchArray.cxx
    src/TGeoBuilder.cxx
    src/TGeoCache.cxx
    src/TGeoCompositeShape.cxx
    src/TGeoCone.cxx
    src/TGeoElement.cxx
    src/TGeoEltu.cxx
    src/TGeoExtension.cxx
    src/TGeoGlobalMagField.cxx
    src/TGeoHalfSpace.cxx
    src/TGeoHelix.cxx
    src/TGeoHype.cxx
    src/TGeoManager.cxx
    src/TGeoMaterial.cxx
    src/TGeoMatrix.cxx
    src/TGeoMedium.cxx
    src/TGeoNavigator.cxx
    src/TGeoNode.cxx
    src/TGeoOpticalSurface.cxx
    src/TGeoPara.cxx
    src/TGeoParaboloid.cxx
    src/TGeoParallelWorld.cxx
    src/TGeoPatternFinder.cxx
    src/TGeoPcon.cxx
    src/TGeoPgon.cxx
    src/TGeoPhysicalNode.cxx
    src/TGeoPolygon.cxx
    src/TGeoRegion.cxx
    src/TGeoScaledShape.cxx
    src/TGeoShape.cxx
    src/TGeoShapeAssembly.cxx
    src/TGeoSphere.cxx
    src/TGeoStateInfo.cxx
    src/TGeoTorus.cxx
    src/TGeoTrd1.cxx
    src/TGeoTrd2.cxx
    src/TGeoTube.cxx
    src/TGeoVolume.cxx
    src/TGeoVoxelFinder.cxx
    src/TGeoXtru.cxx
    src/TGeoTessellated.cxx
    src/TGeoVector3.cxx
    src/TVirtualGeoConverter.cxx
    src/TVirtualGeoPainter.cxx
    src/TVirtualGeoTrack.cxx
    src/TVirtualMagField.cxx
  DEPENDENCIES
    Thread
    RIO
    MathCore
    Hist
  )

# GCC has bugs with -O3 or -Ofast that break Geom
if(CMAKE_COMPILER_IS_GNUCXX)
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5 OR CMAKE_SIZEOF_VOID_P LESS 8)
    target_compile_options(Geom PRIVATE -O2)
  endif()
endif()

if(GEOM_HAS_EMBREE)
  target_compile_definitions(Geom PRIVATE ROOT_GEOM_EMBREE)
  target_link_libraries(Geom PRIVATE embree)
endif()
