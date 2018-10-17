set(LIB_NAMES)

set(LIB_NAME GeometryCogent)
set(LIB_NAMES ${LIB_NAMES} ${LIB_NAME})

set(${LIB_NAME}_SOURCES
    core/Cogent_BodyIntegrator_Direct.cpp
    core/Cogent_BodyIntegrator_Projected.cpp
    core/Cogent_BoundaryIntegrator_Direct.cpp
    core/Cogent_BoundaryIntegrator_Projected.cpp
    core/Cogent_BoundaryIntegrator.cpp
    core/Cogent_Dicer.cpp
    core/Cogent_Model.cpp
    core/Cogent_ModelFactory.cpp
    core/Cogent_ParameterizedModel.cpp
    core/Cogent_NonParameterizedModel.cpp
    core/Cogent_Integrator.cpp
    core/Cogent_IntegratorFactory.cpp
    core/Cogent_Projector.cpp
    core/Cogent_WriteUtils.cpp
    core/Cogent_ParameterFunction.cpp
    core/Cogent_CoordinateSystem.cpp
    geometry/Cogent_PrimitiveFactory.cpp
    geometry/Cogent_Geometry_Primitive.cpp
    geometry/Cogent_Geometry_PrimitiveSurface.cpp
    geometry/Cogent_Geometry_Cylinder.cpp
    geometry/Cogent_Geometry_CylinderSurface.cpp
    geometry/Cogent_Geometry_Sphere.cpp
    geometry/Cogent_Geometry_SphereSurface.cpp
    geometry/Cogent_Geometry_Torus.cpp
    geometry/Cogent_Geometry_TorusSurface.cpp
    geometry/Cogent_Geometry_Frustum.cpp
    geometry/Cogent_Geometry_FrustumSurface.cpp
    geometry/Cogent_Geometry_Brick.cpp
    geometry/Cogent_Geometry_Plane.cpp
   )

set(${LIB_NAME}_HEADERS
    core/Cogent_BodyIntegrator_Def.hpp
    core/Cogent_BodyIntegrator_Direct.hpp
    core/Cogent_BodyIntegrator.hpp
    core/Cogent_BodyIntegrator_Projected.hpp
    core/Cogent_BoundaryIntegrator_Def.hpp
    core/Cogent_BoundaryIntegrator_Direct.hpp
    core/Cogent_BoundaryIntegrator.hpp
    core/Cogent_BoundaryIntegrator_Projected.hpp
    core/Cogent_Dicer_Def.hpp
    core/Cogent_Dicer.hpp
    core/Cogent_Model.hpp
    core/Cogent_ModelFactory.hpp
    core/Cogent_ParameterizedModel.cpp
    core/Cogent_NonParameterizedModel.cpp
    core/Cogent_Integrator_Def.hpp
    core/Cogent_IntegratorFactory.hpp
    core/Cogent_Integrator.hpp
    core/Cogent_Projector.hpp
    core/Cogent_Timer.hpp
    core/Cogent_Types.hpp
    core/Cogent_Utilities.hpp
    core/Cogent_WriteUtils_Def.hpp
    core/Cogent_WriteUtils.hpp
    core/Cogent_ParameterFunction.hpp
    core/Cogent_CoordinateSystem.hpp
    geometry/Cogent_Geometry_Types.hpp
    geometry/Cogent_PrimitiveFactory.hpp
    geometry/Cogent_Geometry_Primitive.hpp
    geometry/Cogent_Geometry_PrimitiveSurface.hpp
    geometry/Cogent_Geometry_Cylinder.hpp
    geometry/Cogent_Geometry_CylinderSurface.hpp
    geometry/Cogent_Geometry_Sphere.hpp
    geometry/Cogent_Geometry_SphereSurface.hpp
    geometry/Cogent_Geometry_Torus.hpp
    geometry/Cogent_Geometry_TorusSurface.hpp
    geometry/Cogent_Geometry_Frustum.hpp
    geometry/Cogent_Geometry_FrustumSurface.hpp
    geometry/Cogent_Geometry_Brick.hpp
    geometry/Cogent_Geometry_Plane.hpp

   )

INCLUDE_DIRECTORIES(${PLATO_INCLUDES})

FOREACH(PLATO_LIB ${LIB_NAMES})
  add_library(Plato${PLATO_LIB} ${${PLATO_LIB}_SOURCES} ${${PLATO_LIB}_HEADERS})
  set(ADD_PLATO_LIBRARIES ${ADD_PLATO_LIBRARIES} Plato${PLATO_LIB})
ENDFOREACH()
set(PLATO_LIBRARIES ${PLATO_LIBRARIES} ${ADD_PLATO_LIBRARIES} PARENT_SCOPE)

if( CMAKE_INSTALL_PREFIX )
 FOREACH(PLATO_LIB ${LIB_NAMES})
  install( TARGETS Plato${PLATO_LIB} EXPORT Plato${PLATO_LIB}
           LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
           ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
           PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
  install( FILES ${${PLATO_LIB}_HEADERS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
 ENDFOREACH()
 install( FILES ${LIBLESS_HEADERS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
endif()

if( UNIT_TESTING )
  add_subdirectory(./unittest)
endif()
