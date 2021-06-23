###############################################################################
# Sources:
###############################################################################
SET(PlatoMainUnitTester_SRCS UnitMain.cpp
							 parse_unit_test.cpp
							 Plato_StkMeshUtils.cpp
							 Plato_PlatoProxyApp.cpp
							 Plato_PhysicsProxyApp.cpp
							 Plato_StructuralTopologyOptimizationProxyGoldResults.cpp
							 Plato_Test_GradFreeCriteria.cpp
							 Plato_Test_ParticleSwarm.cpp
							 Plato_Test_Proxy.cpp
							 Plato_Test_Hessian.cpp
							 Plato_Test_SimpleRocket.cpp
							 Plato_Test_KokkosAlgebra.cpp
							 Plato_Test_InputData.cpp
							 Plato_Test_LinearAlgebra.cpp
							 Plato_Test_OptimizersIO.cpp
							 Plato_Test_OptimalityCriteria.cpp
							 Plato_Test_DataFactory.cpp
							 Plato_Test_KelleySachsBounds.cpp
							 Plato_Test_ConservativeConvexSeparableApproximation.cpp
							 Plato_Test_AugmentedLagrangian.cpp
							 Plato_Test_Bounds.cpp
							 Plato_Test_SimpleTopOpt.cpp
							 Plato_Test_Statistics.cpp
							 Plato_Test_ElementWiseCCSA.cpp
							 Plato_Test_OptimizationTestFunctions.cpp
							 Plato_Test_Vector3DVariations.cpp
							 Plato_Test_UniqueCounter.cpp
							 Plato_Test_SimpleRocketOptimization.cpp
							 Plato_Test_UncertainLoadGeneratorXML.cpp
                                                         Plato_Test_UncertainMaterial.cpp
                                                         Plato_Test_Srom.cpp
							 Plato_Test_LocalStatisticsOperations.cpp
							 Plato_Test_MethodMovingAsymptotes.cpp
							 PSL_Test_Triangle.cpp  
							 PSL_Test_OverhangFilter.cpp  
							 PSL_Test_KernelThenHeavisideFilter.cpp  
							 PSL_Test_Standardization.cpp
							 PSL_Test_Random.cpp
							 PSL_Test_ParameterData.cpp
							 PSL_Test_ParallelExchanger.cpp
							 PSL_Test_NearestNeighborSearches.cpp
							 PSL_Test_MpiWrapperInterface.cpp
							 PSL_Test_MpiWrapperImplementation.cpp
							 PSL_Test_MeshScaleAgent.cpp
							 PSL_Test_Mesh.cpp
							 PSL_Test_KernelFilter.cpp
							 PSL_Test_GradientCheck.cpp
							 PSL_Test_GlobalUtilities.cpp
							 PSL_Test_FreeHelpers.cpp
							 PSL_Test_FixedRadiusNearestNeighborsSearches.cpp
							 PSL_Test_DenseMatrixImplementation.cpp
							 PSL_Test_DenseMatrixBuilder.cpp
							 PSL_Test_ConjugateGradient.cpp
							 PSL_Test_CompressedRowSparseMatrixImplementation.cpp
							 PSL_Test_CompressedRowSparseMatrixBuilder.cpp
							 PSL_Test_BoundingBoxSearches.cpp
							 PSL_Test_BoundingBoxMortonHierarchy.cpp
							 PSL_Test_BoundingBox.cpp
							 PSL_Test_BasicDiscreteObjective.cpp
							 PSL_Test_BasicDenseVectorOperations.cpp
							 PSL_Test_AbstractAuthority.cpp
							 PSL_Test_Point.cpp
							 PSL_Test_Vector.cpp
							 Plato_Test_TimersTree.cpp
                                                         PSL_Test_OrthogonalGridUtilities.cpp
                                                         PSL_Test_RegularHex8.cpp
							 )

if( AMFILTER_ENABLED )
    SET(PlatoMainUnitTester_SRCS ${PlatoMainUnitTester_SRCS} PSL_Test_TetMeshUtilities.cpp PSL_Test_AMFilterUtilities.cpp)
endif()

IF( ENABLE_ROL )
    SET(PlatoMainUnitTester_SRCS ${PlatoMainUnitTester_SRCS} Plato_Test_ROL.cpp)
ENDIF()

###############################################################################
# Headers:
###############################################################################
SET(PlatoMainUnitTester_HDRS Plato_StructuralTopologyOptimizationProxyGoldResults.hpp
							 ParseUnitTestStrings.hpp
							 Plato_UnitTestUtils.hpp
							 Plato_StkMeshUtils.hpp
							 )

set(PLATOUNIT_INCLUDES ${CMAKE_SOURCE_DIR}/tpls/gtest/include
                       ${CMAKE_SOURCE_DIR}/apps/services
                       ${CMAKE_SOURCE_DIR}/apps/services/proxy
                       ${CMAKE_SOURCE_DIR}/apps/services/unittest
                       ${CMAKE_SOURCE_DIR}/base/src/analyze
                       ${CMAKE_SOURCE_DIR}/base/src/interface
                       ${CMAKE_SOURCE_DIR}/base/src/input_generator
                       ${CMAKE_SOURCE_DIR}/base/src/data_layer
                       ${CMAKE_SOURCE_DIR}/base/src/Filters
                       ${CMAKE_SOURCE_DIR}/base/src/tools
                       ${CMAKE_SOURCE_DIR}/base/src/optimize
                       ${CMAKE_SOURCE_DIR}/base/src/operations
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibraryInterface
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/AbstractInterface
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/ActivationFunction
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Agent
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/BoundedSupportFunction
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/DataLossFunction
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/DataOrganizer
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/DecisionMetric
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/DiscreteGlobalOptimization
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/DistanceVoting
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Eigen
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Example
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Filter
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Geometry
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Graph
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Helper
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/InputOutput
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/InvertibleUnitRange
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/LearningPreprocessor
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/NeighborsSearching
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/NodeSetMatching
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/ParameterData
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/SpatialSearching
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/SupervisedLearning
                       ${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/VectorKernel
                       )

# Copy Input files from source to binary dir
SET( testFiles "rol_unit_test_inputs.xml")
Plato_add_test_files("${testFiles}")


#set(PLATOUNIT_INCLUDES ${CMAKE_SOURCE_DIR}/../tpls/gtest/include)
INCLUDE_DIRECTORIES(${PLATOUNIT_INCLUDES})

###INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/../tpls/gtest/include/gtest ${CMAKE_SOURCE_DIR}/../tpls/gtest/include/gtest/internal)

# actual target:
#set(PLATOMAINUNITTESTER_ADDITIONAL_LIBS ${CMAKE_SOURCE_DIR}/tpls/gtest/build) 
set(PLATOMAINUNITTESTER_LIBS ${PLATOMAINUNITTESTER_ADDITIONAL_LIBS} ${PLATO_LIBRARIES} ${PLATO_LIBRARIES} 
                             ${NLOPT_LIBRARY} ${GTEST_BOTH_LIBRARIES} ${Trilinos_LIBRARIES} ${Trilinos_TPL_LIBRARIES} ${PYTHON_LIBRARY} 
                             ${PYTHON_LINK_LIBS} ${Plato_EXTRA_LINK_FLAGS})
add_executable(PlatoMainUnitTester ${PlatoMainUnitTester_SRCS})
target_link_libraries(PlatoMainUnitTester PlatoApp ${PLATOMAINUNITTESTER_LIBS})
set(PLATOMAINUNITTESTER_BINARY ${CMAKE_BINARY_DIR}/apps/unittest/PlatoMainUnitTester PARENT_SCOPE)
add_test(NAME PlatoMainUnitTester COMMAND ${CMAKE_BINARY_DIR}/apps/services/unittest/PlatoMainUnitTester)


if( CMAKE_INSTALL_PREFIX )
  install( TARGETS PlatoMainUnitTester DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
#  set(PLATOMAIN_BINARY ${CMAKE_INSTALL_PREFIX}/bin/PlatoMain PARENT_SCOPE)
endif()
###############################################################################
###############################################################################
