set(PLATO_INCLUDES ${PLATO_INCLUDES} 
  ${CMAKE_SOURCE_DIR}/base/src/analyze
  ${CMAKE_SOURCE_DIR}/base/src/optimize
  ${CMAKE_SOURCE_DIR}/base/src/operations
  ${CMAKE_SOURCE_DIR}/base/src/tools
  ${CMAKE_SOURCE_DIR}/base/src/proxy
  ${CMAKE_SOURCE_DIR}/base/src/models
  ${CMAKE_SOURCE_DIR}/base/src/data_layer
  ${CMAKE_SOURCE_DIR}/base/src/input_generator
  ${CMAKE_SOURCE_DIR}/base/src/Filters
  ${CMAKE_SOURCE_DIR}/base/src/interface
  ${CMAKE_SOURCE_DIR}/base/src/geometry/MovingLeastSquares
  ${CMAKE_SOURCE_DIR}/base/src/geometry/Cogent
  ${CMAKE_SOURCE_DIR}/base/src/EngineInterfaceToPlatoSubproblemLibrary
  ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Agent
  ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/BoundedSupportFunction
  ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/DiscreteGlobalOptimization
  ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Example
  ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Filter
  ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Geometry
  ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Helper
  ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/ParameterData
  ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/SpatialSearching
  ${CMAKE_SOURCE_DIR}/base/src/xtk
  ${CMAKE_SOURCE_DIR}/apps/services/unittest
    PARENT_SCOPE )
