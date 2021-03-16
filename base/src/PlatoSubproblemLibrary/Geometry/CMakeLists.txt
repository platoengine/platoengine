set(LIB_NAMES)

set(LIB_NAME PSLGeometry)
set(LIB_NAMES ${LIB_NAMES} ${LIB_NAME})

set(${LIB_NAME}_SOURCES 
    PSL_AxisAlignedBoundingBox.cpp
    PSL_PointCloud.cpp
    PSL_Point.cpp
    PSL_Vector.cpp
    PSL_Triangle.cpp
    PSL_OrthogonalGridUtilities.cpp
    PSL_RegularHex8.cpp
    )

set(${LIB_NAME}_HEADERS 
    PSL_AxisAlignedBoundingBox.hpp
    PSL_PointCloud.hpp
    PSL_Point.hpp
    PSL_Vector.hpp
    PSL_Triangle.hpp
    PSL_OrthogonalGridUtilities.hpp
    PSL_RegularHex8.hpp
    )

if( AMFILTER_ENABLED )
    set(${LIB_NAME}_SOURCES ${${LIB_NAME}_SOURCES} PSL_TetMeshUtilities.cpp)
    set(${LIB_NAME}_HEADERS ${${LIB_NAME}_HEADERS} PSL_TetMeshUtilities.hpp)
endif()

INCLUDE_DIRECTORIES(${PLATO_INCLUDES})
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibraryInterface)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/AbstractInterface)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/ActivationFunction)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Agent)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/BoundedSupportFunction)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/DataLossFunction)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/DataOrganizer)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/DecisionMetric)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/DiscreteGlobalOptimization)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/DistanceVoting)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Eigen)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Example)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Filter)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Geometry)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Graph)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Helper)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/InputOutput)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/InvertibleUnitRange)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/LearningPreprocessor)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/NeighborsSearching)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/NodeSetMatching)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/ParameterData)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/SpatialSearching)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/SupervisedLearning)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/VectorKernel)

FOREACH(PLATO_LIB ${LIB_NAMES})
  add_library(Plato${PLATO_LIB}     ${${PLATO_LIB}_SOURCES}     ${${PLATO_LIB}_HEADERS}     )
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
endif()

