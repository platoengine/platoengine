set(LIB_NAMES)

set(LIB_NAME Optimizer)
set(LIB_NAMES ${LIB_NAMES} ${LIB_NAME})
set(${LIB_NAME}_SOURCES Plato_OptimizersIO.cpp
                        )
set(${LIB_NAME}_HEADERS Plato_OptimizerFactory.hpp
                        Plato_TrustRegionUtilities.hpp
                        Plato_HostDualProbElementWise.hpp
                        Plato_HostPrimalProbElementWise.hpp
                        Plato_DeviceDualProbElementWise.hpp
                        Plato_DevicePrimalProbElementWise.hpp
                        Plato_DualProbElementWiseFunctions.hpp
                        Plato_PrimalProbElementWiseFunctions.hpp
                        Plato_CommWrapper.hpp
                        Plato_Types.hpp
                        Plato_Circle.hpp
                        Plato_HostBounds.hpp
                        Plato_Himmelblau.hpp
                        Plato_KokkosTypes.hpp
                        Plato_ErrorChecks.hpp
                        Plato_DeviceBounds.hpp
                        Plato_OptimizersIO.hpp
                        Plato_GoldsteinPrice.hpp
                        Plato_StateData.hpp
                        Plato_StateManager.hpp
                        Plato_AlgebraFactory.hpp
                        Plato_EngineObjective.hpp
                        Plato_EngineConstraint.hpp
                        Plato_SteihaugTointSolver.hpp
                        Plato_DualProblemSolver.hpp
                        Plato_DualProblemStageMng.hpp
                        Plato_GradientOperator.hpp
                        Plato_GradientOperatorList.hpp
                        Plato_IdentityHessian.hpp
                        Plato_IdentityPreconditioner.hpp
                        Plato_KelleySachsAlgorithm.hpp
                        Plato_MLKelleySachsAlgorithm.hpp
                        Plato_TrustRegionStageMng.hpp
                        Plato_TrustRegionStepMng.hpp
                        Plato_TrustRegionAlgorithmDataMng.hpp
                        Plato_KelleySachsAugmentedLagrangian.hpp
                        Plato_MLKelleySachsAugmentedLagrangian.hpp
                        Plato_KelleySachsBoundConstrained.hpp
                        Plato_ReducedSpaceTrustRegionStageMng.hpp
                        Plato_KelleySachsBoundLightInterface.hpp
                        Plato_KelleySachsBoundConstrainedInterface.hpp
                        Plato_KelleySachsAugmentedLagrangianInterface.hpp
                        Plato_KelleySachsAugmentedLagrangianLightInterface.hpp
                        Plato_KelleySachsStepMng.hpp
                        Plato_MLKelleySachsStepMng.hpp
                        Plato_LineSearch.hpp
                        Plato_LinearOperator.hpp
                        Plato_LinearOperatorList.hpp
                        Plato_MethodMovingAsymptotes.hpp
                        Plato_MethodMovingAsymptotesInterface.hpp
                        Plato_MethodMovingAsymptotesLightInterface.hpp
                        Plato_MultiVectorList.hpp
                        Plato_Preconditioner.hpp
                        Plato_PrimalProblemStageMng.hpp
                        Plato_ProjectedSteihaugTointPcg.hpp
                        Plato_QuadraticLineSearch.hpp
                        Plato_Radius.hpp
                        Plato_Rosenbrock.hpp
                        Plato_GloballyConvergentMethodMovingAsymptotes.hpp
                        Plato_GloballyConvergentMethodMovingAsymptotesInterface.hpp
                        Plato_GloballyConvergentMethodMovingAsymptotesLightInterface.hpp
                        Plato_CcsaTestObjective.hpp
                        Plato_CcsaTestInequality.hpp
                        Plato_AnalyticalHessian.hpp
                        Plato_AnalyticalGradient.hpp
                        Plato_AugmentedLagrangianStageMng.hpp
                        Plato_CubicLineSearch.hpp
                        Plato_ConservativeConvexSeparableAppxDataMng.hpp
                        Plato_ConservativeConvexSeparableAppxStageMng.hpp
                        Plato_ConservativeConvexSeparableApproximation.hpp
                        Plato_ConservativeConvexSeparableAppxAlgorithm.hpp
                        Plato_Criterion.hpp
                        Plato_CriterionList.hpp
                        Plato_DataFactory.hpp
                        Plato_DistributedReductionOperations.hpp
                        Plato_LinearAlgebra.hpp
                        Plato_MultiVector.hpp
                        Plato_OptimalityCriteriaDataMng.hpp
                        Plato_OptimalityCriteria.hpp
                        Plato_OptimalityCriteriaStageMngBase.hpp
                        Plato_OptimalityCriteriaStageMng.hpp
                        Plato_OptimalityCriteriaSubProblem.hpp
                        Plato_OptimalityCriteriaLightInterface.hpp
                        Plato_OptimalityCriteriaTestInequalityOne.hpp
                        Plato_OptimalityCriteriaTestInequalityTwo.hpp
                        Plato_OptimalityCriteriaTestObjectiveOne.hpp
                        Plato_OptimalityCriteriaTestObjectiveTwo.hpp
                        Plato_ReductionOperations.hpp
                        Plato_SingleConstraintSubProblemTypeLP.hpp
                        Plato_StandardMultiVector.hpp
                        Plato_StandardVector.hpp
                        Plato_OptimalityCriteriaUpdate.hpp
                        Plato_HostOptimalityCriteriaUpdate.hpp
                        Plato_StandardVectorReductionOperations.hpp
                        Plato_NonlinearProgrammingSubProblemOC.hpp
                        Plato_Vector.hpp
                        Plato_Diagnostics.hpp
                        Plato_DiagnosticsInterface.hpp
                        Plato_DistributedVector.hpp
                        Plato_OptimizerInterface.hpp
                        Plato_EpetraSerialDenseVector.hpp
                        Plato_OptimalityCriteriaInterface.hpp
                        Plato_EpetraSerialDenseMultiVector.hpp
                        Plato_MLTrustRegionStepPrediction.hpp
                        Plato_BetaDistribution.hpp
                        Plato_Distribution.hpp
                        Plato_NormalDistribution.hpp
                        Plato_UniformDistribution.hpp
                        Plato_SromUtilis.hpp
                        Plato_SromConstraint.hpp
                        Plato_SromObjective.hpp
                        Plato_SromStatistics.hpp
                        Plato_SromProbDataStruct.hpp
                        Plato_StatisticsUtils.hpp
                        Plato_LinearCriterionHessian.hpp
                        Plato_CentralDifferenceHessian.hpp
                        Plato_OptimizerUtilities.hpp
                        Plato_KokkosVector.hpp
                        Plato_KokkosReductionOperations.hpp
                        Plato_DeviceOptimalityCriteriaUpdate.hpp
                        Plato_SolveUncertaintyProblem.hpp
                        Plato_DistributionFactory.hpp
                        )
                        
IF( ENABLE_ROL )
SET(${LIB_NAME}_HEADERS ${${LIB_NAME}_HEADERS}
					    Plato_RapidOptimizationLibraryInterface.hpp
						Plato_SerialEpetraVectorROL.hpp
						Plato_DistributedVectorROL.hpp
						Plato_ReducedConstraintROL.hpp
						Plato_ReducedObjectiveROL.hpp
						Plato_ConstraintSimOptROL.hpp
						Plato_ObjectiveSimOptROL.hpp
						Plato_ProxyComplianceROL.hpp
					    Plato_SerialVectorROL.hpp
						Plato_ProxyVolumeROL.hpp
                        )
ENDIF()

INCLUDE_DIRECTORIES(${PLATO_INCLUDES})

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
