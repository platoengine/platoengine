/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
// *************************************************************************
//@HEADER
*/

/*
 * Plato_SolveUncertaintyProblem.hpp
 *
 * Created on: August 31, 2018
 */

#pragma once

#include "Plato_SromUtilis.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_SromObjective.hpp"
#include "Plato_CriterionList.hpp"
#include "Plato_SromConstraint.hpp"
#include "Plato_StandardVector.hpp"
#include "Plato_StatisticsUtils.hpp"
#include "Plato_SromProbDataStruct.hpp"
#include "Plato_DistributionFactory.hpp"
#include "Plato_ReductionOperations.hpp"
#include "Plato_AugmentedLagrangianStageMng.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"
#include "Plato_KelleySachsAugmentedLagrangian.hpp"

namespace Plato
{

/******************************************************************************//**
 *
 * @brief Set Stochastic Reduced Order Model (SROM) problem statistics.
 * @param [in] aDataMng optimization problem data manager
 * @param [in] aStageMng optimization problem stage manager
 * @param [in] aAlgorithm optimization algorithm main interface
 * @param [in,out] aAlgorithmParam inputs and outputs for/from optimizer
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void save_srom_problem_stats(const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng,
                                   const Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType> & aStageMng,
                                   const Plato::KelleySachsAugmentedLagrangian<ScalarType, OrdinalType> & aAlgorithm,
                                   Plato::AlgorithmParamStruct<ScalarType, OrdinalType> & aAlgorithmParam)
{
    // transfer stopping criterion, objective and constraint values to algorithm parameter data structure
    aAlgorithmParam.mStop = aAlgorithm.getStoppingCriterion();
    aAlgorithmParam.mObjectiveValue = aDataMng.getCurrentObjectiveFunctionValue();

    const OrdinalType t_NUM_VECTORS = 1;
    const OrdinalType tNumConstraints = aStageMng.getNumConstraints();
    Plato::StandardMultiVector<ScalarType, OrdinalType> tConstraintValues(t_NUM_VECTORS, tNumConstraints);
    aStageMng.getCurrentConstraintValues(tConstraintValues);
    aAlgorithmParam.mConstraintValue = tConstraintValues(0,0);
}

/******************************************************************************//**
 *
 * @brief Output Stochastic Reduced Order Model (SROM) and Monte Carlo cumulative distribution functions.
 * @param [in] aCommWrapper distributed memory communicator wrapper
 * @param [in] aFinalControl solution from SROM optimization problem
 * @param [in] aDistribution probability distribution function interface
 * @param [in] aStatsInputs probability distribution function inputs
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void output_cdf_comparison(const Plato::CommWrapper & aCommWrapper,
                                  const Plato::MultiVector<ScalarType> & aFinalControl,
                                  const Plato::Distribution<ScalarType, OrdinalType> & aDistribution,
                                  const Plato::UncertaintyInputStruct<ScalarType, OrdinalType>& aStatsInputs)
{
    try
    {
        const bool tPrintErrorMsg = true;
        Plato::StandardVector<ScalarType, OrdinalType> tMonteCarloCDF(aStatsInputs.mNumMonteCarloSamples + 1);
        Plato::StandardVector<ScalarType, OrdinalType> tNormalizedMonteCarloSamples(aStatsInputs.mNumMonteCarloSamples + 1);
        Plato::compute_monte_carlo_data(aStatsInputs.mNumMonteCarloSamples, aDistribution, tNormalizedMonteCarloSamples, tMonteCarloCDF, tPrintErrorMsg);

        const OrdinalType t_SAMPLES_VEC_INDEX = 0;
        const OrdinalType t_PROBABILITIES_VEC_INDEX = 1;
        Plato::StandardVector<ScalarType, OrdinalType> tSromCDF(aStatsInputs.mNumMonteCarloSamples + 1);
        Plato::compute_srom_cdf_plot(tNormalizedMonteCarloSamples, aFinalControl[t_SAMPLES_VEC_INDEX], aFinalControl[t_PROBABILITIES_VEC_INDEX], tSromCDF);

        Plato::StandardVector<ScalarType, OrdinalType> tUnnormalizedMonteCarloSamples(aStatsInputs.mNumMonteCarloSamples + 1);
        Plato::compute_unnormalized_samples(aStatsInputs.mLowerBound, aStatsInputs.mUpperBound, tNormalizedMonteCarloSamples, tUnnormalizedMonteCarloSamples);

        Plato::output_cumulative_distribution_function(aCommWrapper, tSromCDF, tMonteCarloCDF, tUnnormalizedMonteCarloSamples, tPrintErrorMsg);
    }
    catch(const std::invalid_argument & tErrorMsg)
    {
        throw tErrorMsg;
    }
}

/******************************************************************************//**
 *
 * @brief Set optimizer parameters for stochastic reduced order model (SROM) problem
 * @param [in] aAlgorithmParam data structure with inputs/outputs for/from optimizer
 * @param [in,out] aAlgorithm optimizer interface
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void set_srom_optimizer_parameters(const Plato::AlgorithmParamStruct<ScalarType, OrdinalType>& aAlgorithmParam,
                                          Plato::KelleySachsAugmentedLagrangian<ScalarType, OrdinalType>& aAlgorithm)
{
    aAlgorithm.setMaxNumIterations(aAlgorithmParam.mMaxNumIterations);
    aAlgorithm.setMinPenaltyParameter(aAlgorithmParam.mMinPenaltyParameter);
    aAlgorithm.setFeasibilityTolerance(aAlgorithmParam.mFeasibilityTolerance);
    aAlgorithm.setMaxTrustRegionRadius(aAlgorithmParam.mMaxTrustRegionRadius);
    aAlgorithm.setActualReductionTolerance(aAlgorithmParam.mActualReductionTolerance);
    aAlgorithm.setObjectiveStagnationTolerance(aAlgorithmParam.mObjectiveStagnationTolerance);
}

/******************************************************************************//**
 *
 * @brief Initialize memory
 * @param [in] aStatisticInputs data structure with inputs for probability distribution function
 * @param [in,out] aDataFactory data factory for optimizer
 * @param [in,out] aDistribution probability distribution function interface
 * @param [in,out] aSromObjective stochastic reduced order model problem objective
 * @param [in,out] aDataMng optimization algorithm data manager
 * @param [in,out] aStageMng optimization problem stage manager (manages criteria evaluations)
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void initialize(const Plato::UncertaintyInputStruct<ScalarType, OrdinalType> & aStatsInputs,
                       std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aDataFactory,
                       std::shared_ptr<Plato::Distribution<ScalarType, OrdinalType>> & aDistribution,
                       std::shared_ptr<Plato::SromObjective<ScalarType, OrdinalType>> & aSromObjective,
                       std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType>> & aDataMng,
                       std::shared_ptr<Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>> & aStageMng)
{
    // grab values from input struct
    const OrdinalType tNumSamples = aStatsInputs.mNumSamples;
    const OrdinalType tMaxNumMoments = aStatsInputs.mMaxNumDistributionMoments;

    // ********* ALLOCATE DATA FACTORY *********
    aDataFactory = std::make_shared<Plato::DataFactory<ScalarType, OrdinalType>>();
    const OrdinalType tNumConstraints = 1;
    const OrdinalType tNumControlVectors = 2; // samples values and sample weights
    aDataFactory->allocateDual(tNumConstraints);
    aDataFactory->allocateControl(tNumSamples, tNumControlVectors);

    // build distribution
    aDistribution = Plato::build_distrubtion<ScalarType, OrdinalType>(aStatsInputs);

    // ********* ALLOCATE OBJECTIVE AND CONSTRAINT CRITERIA *********
    aSromObjective = std::make_shared<Plato::SromObjective<ScalarType, OrdinalType> >(aDistribution, tMaxNumMoments, tNumSamples);
    aSromObjective->setMomentMisfitTermWeight(aStatsInputs.mMomentErrorCriterionWeight);
    aSromObjective->setCdfMisfitTermWeight(aStatsInputs.mCumulativeDistributionFuncErrorWeight);

    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType> > tReductions = aDataFactory->getControlReductionOperations().create();
    std::shared_ptr<Plato::SromConstraint<ScalarType, OrdinalType> > tSromConstraint = std::make_shared<Plato::SromConstraint<ScalarType, OrdinalType> >(tReductions);
    std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType> > tConstraintList = std::make_shared<Plato::CriterionList<ScalarType, OrdinalType> >();
    tConstraintList->add(tSromConstraint);

    // ********* ALLOCATE AUGMENTED LAGRANGIAN STAGE MANAGER *********
    aStageMng = std::make_shared<Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>>(aDataFactory, aSromObjective, tConstraintList);

    // ********* ALLOCATE TRUST REGION ALGORITHM DATA MANAGER *********
    aDataMng = std::make_shared<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType>>(aDataFactory);
}

/******************************************************************************//**
 *
 * @brief Set initial guess for stochastic reduced order model (SROM) problem
 * @param [in] aNumSamples number of SROM samples
 * @param [in,out] aDataMng optimization problem data manager
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void set_srom_problem_initial_guess(const OrdinalType & aNumSamples, Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
{
    Plato::StandardVector<ScalarType, OrdinalType> tInitialGuess(aNumSamples);
    Plato::make_uniform_sample(tInitialGuess);
    OrdinalType tVectorIndex = 0;
    aDataMng.setInitialGuess(tVectorIndex, tInitialGuess);
    // ********* SET INTIAL GUESS FOR VECTOR OF PROBABILITIES *********
    ScalarType tScalarValue = 1. / static_cast<ScalarType>(aNumSamples);
    tInitialGuess.fill(tScalarValue);
    tVectorIndex = 1;
    aDataMng.setInitialGuess(tVectorIndex, tInitialGuess);
    // ********* SET UPPER AND LOWER BOUNDS *********
    tScalarValue = 0;
    aDataMng.setControlLowerBounds(tScalarValue);
    tScalarValue = 1;
    aDataMng.setControlUpperBounds(tScalarValue);
}

/******************************************************************************//**
 *
 * @brief Solve optimization problem to construct an optimal stochastic reduced order model (SROM)
 * @param [in] aStatisticInputs data structure with inputs for probability distribution function
 * @param [in,out] aAlgorithmParam data structure with inputs/outputs for/from optimization algorithm
 * @param [in,out] aSromDiagnostics diagnostics associated with the SROM optimization problem
 * @param [in,out] aOutput data structure with outputs from SROM optimization problem
 * @param [in] aOutputData flag use to enable output to file (default = false)
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void solve_uncertainty(const Plato::UncertaintyInputStruct<ScalarType, OrdinalType>& aStatsInputs,
                              Plato::AlgorithmParamStruct<ScalarType, OrdinalType>& aAlgorithmParam,
                              Plato::SromProblemDiagnosticsStruct<ScalarType>& aSromDiagnostics,
                              std::vector<Plato::UncertaintyOutputStruct<ScalarType>>& aSolution,
                              bool aOutputData = false)
{
    std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> tDataFactory;
    std::shared_ptr<Plato::Distribution<ScalarType, OrdinalType>> tDistribution;
    std::shared_ptr<Plato::SromObjective<ScalarType, OrdinalType>> tSromObjective;
    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType>> tDataMng;
    std::shared_ptr<Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>> tStageMng;
    Plato::initialize(aStatsInputs, tDataFactory, tDistribution, tSromObjective, tDataMng, tStageMng);

    const OrdinalType tNumSamples = aStatsInputs.mNumSamples;
    Plato::set_srom_problem_initial_guess(tNumSamples, *tDataMng);

    Plato::KelleySachsAugmentedLagrangian<ScalarType, OrdinalType> tAlgorithm(tDataFactory, tDataMng, tStageMng);
    Plato::set_srom_optimizer_parameters(aAlgorithmParam, tAlgorithm);
    tAlgorithm.solve();

    aSolution.resize(tNumSamples);
    const Plato::MultiVector<ScalarType, OrdinalType>& tFinalControl = tDataMng->getCurrentControl();
    Plato::save_srom_solution(aStatsInputs, tFinalControl, aSolution);
    Plato::save_srom_cdf_diagnostics(*tSromObjective, aSromDiagnostics);
    Plato::save_srom_moments_diagnostics(*tSromObjective, aSromDiagnostics);
    Plato::save_srom_problem_stats(*tDataMng, *tStageMng, tAlgorithm, aAlgorithmParam);

    if(aOutputData == true)
    {
        // transfer to output data structure
        const Plato::CommWrapper& tCommWrapper = tDataMng->getCommWrapper();
        Plato::output_cdf_comparison(tCommWrapper, tFinalControl, *tDistribution, aStatsInputs);

        const bool tPrintErrorMsg = true;
        Plato::output_srom_diagnostics(tCommWrapper, aSromDiagnostics, aAlgorithmParam, tPrintErrorMsg);
    }
}
// function solve_uncertainty

} // namespace Plato
