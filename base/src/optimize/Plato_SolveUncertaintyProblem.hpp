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
#include "Plato_SromObjective.hpp"
#include "Plato_SromConstraint.hpp"
#include "Plato_StatisticsUtils.hpp"
#include "Plato_SromProbDataStruct.hpp"
#include "Plato_DistributionFactory.hpp"

#include "Plato_KelleySachsAugmentedLagrangianLightInterface.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Output Stochastic Reduced Order Model (SROM) and Monte Carlo cumulative distribution functions.
 * @param [in] aCommWrapper distributed memory communicator wrapper
 * @param [in] aFinalControl solution from SROM optimization problem
 * @param [in] aDistribution probability distribution function interface
 * @param [in] aStatsInputs probability distribution function inputs
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
 * @brief Build SROM objective and constraint
 * @param [in] aStatisticInputs data structure with inputs for probability distribution function
 * @param [in,out] aDataFactory data factory for optimizer
 * @param [in,out] aDistribution probability distribution function interface
 * @param [in,out] aSromObjective stochastic reduced order model problem objective
 * @param [in,out] aDataMng optimization algorithm data manager
 * @param [in,out] aStageMng optimization problem stage manager (manages criteria evaluations)
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void build_srom_criteria(const Plato::UncertaintyInputStruct<ScalarType, OrdinalType> & aStatsInputs,
                                const std::shared_ptr<Plato::ReductionOperations<ScalarType,OrdinalType>> & aReductionOps,
                                const std::shared_ptr<Plato::Distribution<ScalarType, OrdinalType>> & aDistribution,
                                std::shared_ptr<Plato::SromObjective<ScalarType, OrdinalType>> & aSromObjective,
                                std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> & aConstraintList)
{
    // ********* ALLOCATE OBJECTIVE AND CONSTRAINT CRITERIA *********
    const OrdinalType tNumSamples = aStatsInputs.mNumSamples;
    const OrdinalType tMaxNumMoments = aStatsInputs.mMaxNumDistributionMoments;
    aSromObjective = std::make_shared<Plato::SromObjective<ScalarType, OrdinalType> >(aDistribution, tMaxNumMoments, tNumSamples);
    aSromObjective->setMomentMisfitTermWeight(aStatsInputs.mMomentErrorCriterionWeight);
    aSromObjective->setCdfMisfitTermWeight(aStatsInputs.mCumulativeDistributionFuncErrorWeight);

    std::shared_ptr<Plato::SromConstraint<ScalarType, OrdinalType> > tSromConstraint =
            std::make_shared<Plato::SromConstraint<ScalarType, OrdinalType>>(aReductionOps);
    aConstraintList = std::make_shared<Plato::CriterionList<ScalarType, OrdinalType> >();
    aConstraintList->add(tSromConstraint);
}

/******************************************************************************//**
 * @brief Set initial guess for stochastic reduced order model (SROM) problem
 * @param [in] aNumSamples number of SROM samples
 * @param [in,out] aInputsKSAL input data structure for KSAL algorithm
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void set_srom_problem_initial_guess(const OrdinalType & aNumSamples, Plato::AlgorithmInputsKSAL<ScalarType, OrdinalType>& aInputsKSAL)
{
    const OrdinalType tNumConstraints = 1;
    const OrdinalType tNumDualVectors = 1;
    aInputsKSAL.mDual = std::make_shared<Plato::StandardMultiVector<double>>(tNumDualVectors, tNumConstraints);

    // ********* SET INTIAL GUESS FOR VECTOR OF SAMPLES *********
    OrdinalType tVectorIndex = 0;
    const OrdinalType tNumControlVectors = 2;
    aInputsKSAL.mInitialGuess = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(tNumControlVectors, aNumSamples);
    Plato::make_uniform_sample((*aInputsKSAL.mInitialGuess)[tVectorIndex]);

    // ********* SET INTIAL GUESS FOR VECTOR OF PROBABILITIES *********
    tVectorIndex = 1;
    ScalarType tScalarValue = static_cast<ScalarType>(1) / static_cast<ScalarType>(aNumSamples);
    (*aInputsKSAL.mInitialGuess)[tVectorIndex].fill(tScalarValue);

    // ********* SET UPPER AND LOWER BOUNDS *********
    tScalarValue = 0;
    aInputsKSAL.mLowerBounds = aInputsKSAL.mInitialGuess->create();
    Plato::fill(tScalarValue, *aInputsKSAL.mLowerBounds);
    tScalarValue = 1;
    aInputsKSAL.mUpperBounds = aInputsKSAL.mInitialGuess->create();
    Plato::fill(tScalarValue, *aInputsKSAL.mUpperBounds);
}

/******************************************************************************//**
 * @brief Output diagnostics for stochastic reduced order model (SROM) optimization problem
 * @param [in] aCommWrapper distributed memory communicator wrapper
 * @param [in] aSromDiagnostics SROM problem diagnostics
 * @param [in] aOutputsKSAL optimizer diagnostics
 * @param [in] aPrint flag use to enable terminal output (default = false)
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void output_srom_diagnostics(const Plato::CommWrapper & aCommWrapper,
                                    const Plato::SromProblemDiagnosticsStruct<ScalarType>& aSromDiagnostics,
                                    const Plato::AlgorithmOutputsKSAL<ScalarType, OrdinalType>& aOutputsKSAL,
                                    bool aPrint = false)
{
    try
    {
        Plato::error::check_null_comm(aCommWrapper);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        std::ostringstream tMessage;
        tMessage << "\n\n ********\n ERROR IN FILE: " << __FILE__ << "\nFUNCTION: " << __PRETTY_FUNCTION__ << "\nLINE: "
                 << __LINE__ << "\n ******** \n\n";
        tMessage << tErrorMsg.what();
        if(aPrint == true)
        {
            std::cout << tMessage.str().c_str() << std::flush;
        }
        throw std::invalid_argument(tMessage.str().c_str());
    }

    if(aCommWrapper.myProcID() == 0)
    {
        std::ofstream tOutputFile;
        tOutputFile.open("plato_srom_diagnostics.txt");
        Plato::print_plato_license(tOutputFile);
        tOutputFile << "Cumulative Distribution Function (CDF) Mismatch = " << std::setprecision(6) << std::scientific
        << aSromDiagnostics.mCumulativeDistributionFunctionError << "\n\n";

        tOutputFile << "--------------------------------\n";
        tOutputFile << "| Statistical Moments Mismatch |\n";
        tOutputFile << "--------------------------------\n";
        tOutputFile << "| Name" << std::setw(12) << "Order" << std::setw(10) << "Error" << std::setw(5) << "|\n";
        tOutputFile << "--------------------------------\n";
        std::vector<std::string> tMomentNames = {"Mean    ", "Variance", "Skewness", "Kurtosis"};
        const OrdinalType tMaxNumMoments = aSromDiagnostics.mMomentErrors.size();
        //NOTE: ONLY THE ERRORS IN THE FIRST FOUR MOMENTS ARE OUTPUTTED TO FILE
        OrdinalType tNumMomentsToOutput = tMaxNumMoments > tMomentNames.size() ? tMomentNames.size() : tMaxNumMoments;
        for(OrdinalType tMomentIndex = 0; tMomentIndex < tNumMomentsToOutput; tMomentIndex++)
        {
            const OrdinalType tMomentOrder = tMomentIndex + static_cast<OrdinalType>(1);
            std::string tMyName = tMomentIndex < tMaxNumMoments ? tMomentNames[tMomentIndex] : std::to_string(tMomentOrder) + "";
            tOutputFile << std::setprecision(3) << std::scientific << "| " << std::setw(8) << tMyName.c_str() << std::setw(6)
            << tMomentOrder << std::setw(14) << aSromDiagnostics.mMomentErrors[tMomentIndex] << " |\n";
        }
        tOutputFile << "--------------------------------\n\n";
        tOutputFile << "Stochastic Reduced Order Model (SROM) optimizer diagnostics.\n";
        tOutputFile << "Objective Function Value = " << std::setprecision(6) << std::scientific
        << aOutputsKSAL.mObjFuncValue << "\n";
        tOutputFile << "Constraint Value = " << std::setprecision(6) << std::scientific << (*aOutputsKSAL.mConstraints)[0]
        << "\n";
        tOutputFile.close();
    }
}

/******************************************************************************//**
 * @brief Solve optimization problem to construct an optimal stochastic reduced order model (SROM)
 * @param [in] aStatsInputs data structure with inputs for probability distribution function
 * @param [in,out] aInputsKSAL input data structure for KSAL algorithm
 * @param [in,out] aSromDiagnostics diagnostics associated with the SROM optimization problem
 * @param [in,out] aSolution outputs from SROM optimization problem
 * @param [in] aPrintDiagnostics flag use to enable output to file (default = false)
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void solve_uncertainty(const Plato::UncertaintyInputStruct<ScalarType, OrdinalType>& aStatsInputs,
                              Plato::AlgorithmInputsKSAL<ScalarType, OrdinalType>& aInputsKSAL,
                              Plato::SromProblemDiagnosticsStruct<ScalarType>& aSromDiagnostics,
                              std::vector<Plato::UncertaintyOutputStruct<ScalarType>>& aSolution,
                              bool aPrintDiagnostics = false)
{
    // build distribution
    std::shared_ptr<Plato::Distribution<ScalarType, OrdinalType>> tDistribution;
    tDistribution = Plato::build_distrubtion<ScalarType, OrdinalType>(aStatsInputs);

    // build srom objective and constraint
    std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> tDataFactory;
    std::shared_ptr<Plato::SromObjective<ScalarType, OrdinalType>> tSromObjective;
    std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> tSromConstraints;
    Plato::build_srom_criteria(aStatsInputs, aInputsKSAL.mReductionOperations, tDistribution, tSromObjective, tSromConstraints);

    // set initial guess and bounds
    const OrdinalType tNumSamples = aStatsInputs.mNumSamples;
    Plato::set_srom_problem_initial_guess(tNumSamples, aInputsKSAL);

    // solve srom optimization problem
    aInputsKSAL.mHaveHessian = false;
    aInputsKSAL.mMaxTrustRegionRadius = 1.0;
    aInputsKSAL.mDisablePostSmoothing = true;
    aInputsKSAL.mControlStagnationTolerance = 1e-5;
    aInputsKSAL.mPrintDiagnostics = aPrintDiagnostics;
    Plato::AlgorithmOutputsKSAL<ScalarType, OrdinalType> tOutputsKSAL;
    Plato::solve_ksal<ScalarType, OrdinalType>(tSromObjective, tSromConstraints, aInputsKSAL, tOutputsKSAL);

    // set output data
    aSolution.resize(tNumSamples);
    Plato::save_srom_solution(aStatsInputs, *tOutputsKSAL.mSolution, aSolution);
    Plato::save_srom_cdf_diagnostics(*tSromObjective, aSromDiagnostics);
    Plato::save_srom_moments_diagnostics(*tSromObjective, aSromDiagnostics);

    if(aPrintDiagnostics == true)
    {
        // transfer to output data structure
        Plato::output_cdf_comparison(aInputsKSAL.mCommWrapper, *tOutputsKSAL.mSolution, *tDistribution, aStatsInputs);

        const bool tPrintErrorMsg = true;
        Plato::output_srom_diagnostics(aInputsKSAL.mCommWrapper, aSromDiagnostics, tOutputsKSAL, tPrintErrorMsg);
    }
}
// function solve_uncertainty

} // namespace Plato
