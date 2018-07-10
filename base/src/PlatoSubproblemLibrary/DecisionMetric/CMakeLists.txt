set(LIB_NAMES)

set(LIB_NAME PSLDecisionMetric)
set(LIB_NAMES ${LIB_NAMES} ${LIB_NAME})

set(${LIB_NAME}_SOURCES 
    PSL_DecisionMetric.cpp
    PSL_DecisionMetricFactory.cpp
    PSL_EntropyMetric.cpp
    PSL_GiniImpurityMetric.cpp
    )

set(${LIB_NAME}_HEADERS 
    PSL_DecisionMetricFactory.hpp
    PSL_DecisionMetric.hpp
    PSL_EntropyMetric.hpp
    PSL_GiniImpurityMetric.hpp
    )

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

