#ifndef _COGENT_GEOMETRY_TYPES_H
#define _COGENT_GEOMETRY_TYPES_H

namespace Cogent {
namespace Geometry {

  enum struct Axis {X, Y, Z};
 
  enum struct SurfaceType {NotSet, Plane, CylinderSurface, FrustumSurface, SphereSurface, TorusSurface};

  struct BoundaryAlias {
    std::string aliasName;
    std::string boundaryName;
    int localIndex;
  };

}
}

#endif
