set(LIB_NAMES)

set(LIB_NAME PSLExample)
set(LIB_NAMES ${LIB_NAMES} ${LIB_NAME})

set(${LIB_NAME}_SOURCES 
    PSL_Implementation_CompressedRowSparseMatrix.cpp
    PSL_Implementation_DenseMatrix.cpp
    PSL_Implementation_MeshModular.cpp
    PSL_Implementation_MeshMonolithic.cpp
    PSL_Implementation_MpiWrapper.cpp
    PSL_Interface_BasicDenseVectorOperations.cpp
    PSL_Interface_BasicGlobalUtilities.cpp
    PSL_Interface_ConjugateGradient.cpp
    PSL_Interface_DenseMatrixBuilder.cpp
    PSL_Interface_DenseMatrix.cpp
    PSL_Interface_Kernel_StructParameterDataBuilder.cpp
    PSL_Interface_MeshModular.cpp
    PSL_Interface_MeshMonolithic.cpp
    PSL_Interface_MpiWrapper.cpp
    PSL_Interface_NeuralNetwork_StructParameterDataBuilder.cpp
    PSL_Interface_ParallelExchanger_global.cpp
    PSL_Interface_ParallelExchanger_localAndNonlocal.cpp
    PSL_Interface_ParallelExchanger_ownershipAndGlobal.cpp
    PSL_Interface_ParallelExchanger_ownershipFlag.cpp
    PSL_Interface_ParallelVector.cpp
    PSL_Interface_PointCloud.cpp
    PSL_Interface_SparseMatrixBuilder.cpp
    )

set(${LIB_NAME}_HEADERS 
    PSL_Implementation_CompressedRowSparseMatrix.hpp
    PSL_Implementation_DenseMatrix.hpp
    PSL_Implementation_Kernel_StructParameterData.hpp
    PSL_Implementation_MeshModular.hpp
    PSL_Implementation_MeshMonolithic.hpp
    PSL_Implementation_MpiWrapper.hpp
    PSL_Implementation_NeuralNetwork_StructParameterData.hpp
    PSL_Interface_BasicDenseVectorOperations.hpp
    PSL_Interface_BasicGlobalUtilities.hpp
    PSL_Interface_ConjugateGradient.hpp
    PSL_Interface_DenseMatrixBuilder.hpp
    PSL_Interface_DenseMatrix.hpp
    PSL_Interface_Kernel_StructParameterDataBuilder.hpp
    PSL_Interface_MeshModular.hpp
    PSL_Interface_MeshMonolithic.hpp
    PSL_Interface_MpiWrapper.hpp
    PSL_Interface_NeuralNetwork_StructParameterDataBuilder.hpp
    PSL_Interface_ParallelExchanger_global.hpp
    PSL_Interface_ParallelExchanger_localAndNonlocal.hpp
    PSL_Interface_ParallelExchanger_ownershipAndGlobal.hpp
    PSL_Interface_ParallelExchanger_ownershipFlag.hpp
    PSL_Interface_ParallelVector.hpp
    PSL_Interface_PointCloud.hpp
    PSL_Interface_SparseMatrixBuilder.hpp
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

