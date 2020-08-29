/*
//\HEADER
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
// Questions? Contact the Plato team (plato3D-help\sandia.gov)
// *************************************************************************
//\HEADER
*/

/*
 * Plato_SolveUncertaintyProblem.hpp
 *
 * Created on: August 31, 2018
 */

#pragma once

#include "Plato_Macros.hpp"
#include "Plato_SromUtilis.hpp"
#include "Plato_SromHelpers.hpp"
#include "Plato_SromObjective.hpp"
#include "Plato_SromConstraint.hpp"
#include "Plato_StatisticsUtils.hpp"
#include "Plato_SromProbDataStruct.hpp"
#include "Plato_DistributionFactory.hpp"

#include "Plato_KelleySachsAugmentedLagrangianLightInterface.hpp"

namespace Plato
{

/******************************************************************************//**
 * \brief Output Stochastic Reduced Order Model (SROM) and Monte Carlo cumulative distribution functions.
 * \param [in] aCommWrapper distributed memory communicator wrapper
 * \param [in] aFinalControl solution from SROM optimization problem
 * \param [in] aDistribution probability distribution function interface
 * \param [in] aStatsInputs probability distribution function inputs
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void output_cdf_comparison(const Plato::CommWrapper & aCommWrapper,
                                  const Plato::MultiVector<ScalarType> & aFinalControl,
                                  const Plato::Distribution<ScalarType, OrdinalType> & aDistribution,
                                  const Plato::SromInputs<ScalarType, OrdinalType>& aSromInputs)
{
    try
    {
        Plato::StandardVector<ScalarType, OrdinalType> tMonteCarloCDF(aSromInputs.mNumMonteCarloSamples + 1);
        Plato::StandardVector<ScalarType, OrdinalType> tNormalizedMonteCarloSamples(aSromInputs.mNumMonteCarloSamples + 1);
        Plato::compute_monte_carlo_data(aSromInputs.mNumMonteCarloSamples, aDistribution, tNormalizedMonteCarloSamples, tMonteCarloCDF);

        const OrdinalType t_SAMPLES_VEC_INDEX = 0;
        const OrdinalType t_PROBABILITIES_VEC_INDEX = 1;
        Plato::StandardVector<ScalarType, OrdinalType> tSromCDF(aSromInputs.mNumMonteCarloSamples + 1);
        Plato::compute_srom_cdf_plot(tNormalizedMonteCarloSamples, aFinalControl[t_SAMPLES_VEC_INDEX], aFinalControl[t_PROBABILITIES_VEC_INDEX], tSromCDF);

        Plato::StandardVector<ScalarType, OrdinalType> tUnnormalizedMonteCarloSamples(aSromInputs.mNumMonteCarloSamples + 1);
        Plato::compute_unnormalized_samples(aSromInputs.mLowerBound, aSromInputs.mUpperBound, tNormalizedMonteCarloSamples, tUnnormalizedMonteCarloSamples);

        Plato::output_cumulative_distribution_function(aCommWrapper, tSromCDF, tMonteCarloCDF, tUnnormalizedMonteCarloSamples);
    }
    catch(const std::invalid_argument & tErrorMsg)
    {
        THROWERR(tErrorMsg.what())
    }
}

/******************************************************************************//**
 * \brief Set correlation matrix.
 * \param [in]  aMetaData   stochastic reduced order model (SROM) metadata
 * \param [out] aObjective  objective function interface
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void set_correlation_matrix
(const Plato::SromInputs<ScalarType, OrdinalType>& aMetaData,
 Plato::SromObjective<ScalarType, OrdinalType>& aObjective)
{
    if(!aMetaData.mCorrelationMatrixFilename.empty())
    {
        auto tInputCorrelationMatrix =
            Plato::io::read_matrix_from_file<ScalarType>(aMetaData.mCorrelationMatrixFilename);
        auto tTruthCorrelationMatrix = aObjective.getTruthCorrelationMatrix().create();
        Plato::copy(tInputCorrelationMatrix, *tTruthCorrelationMatrix);
        aObjective.setTruthCorrelationMatrix(*tTruthCorrelationMatrix);
    }
}

/******************************************************************************//**
 * \brief Build stochastic reduced order model (SROM) objective function.
 * \param [in]  aSromInputs    SROM problem metadata
 * \param [in]  aDistribution  probability distribution function interface
 * \param [out] aSromObjective objective function interface
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void build_srom_objective_function
(const Plato::SromInputs<ScalarType, OrdinalType> & aSromInputs,
 const std::shared_ptr<Plato::Distribution<ScalarType, OrdinalType>> & aDistribution,
 std::shared_ptr<Plato::SromObjective<ScalarType, OrdinalType>> & aSromObjective)
{
    // TODO ADD SUPPORT FOR CORRELATION MATRIX
    const OrdinalType tRandVecDim = aSromInputs.mDimension;
    const OrdinalType tNumSamples = aSromInputs.mNumSamples;
    const OrdinalType tMaxNumMoments = aSromInputs.mMaxNumDistributionMoments;
    aSromObjective = std::make_shared< Plato::SromObjective<ScalarType, OrdinalType> >
                     (aDistribution, tMaxNumMoments, tNumSamples, tRandVecDim);
    aSromObjective->setMomentMisfitTermWeight(aSromInputs.mMomentErrorCriterionWeight);
    aSromObjective->setCdfMisfitTermWeight(aSromInputs.mCumulativeDistributionFuncErrorWeight);
    aSromObjective->setCorrelationMisfitTermWeight(aSromInputs.mCorrelationErrorCriterionWeight);
    Plato::set_correlation_matrix(aSromInputs, *aSromObjective);
}

/******************************************************************************//**
 * \brief Build stochastic reduced order model (SROM) constraint function.
 * \param [in]   aReductionOps   reduction operation interface
 * \param [out]  aConstraintList constraint function interface
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void build_srom_constraint_function
(const std::shared_ptr<Plato::ReductionOperations<ScalarType,OrdinalType>> & aReductionOps,
 std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> & aConstraintList)
{
    std::shared_ptr<Plato::SromConstraint<ScalarType, OrdinalType> > tSromConstraint =
            std::make_shared<Plato::SromConstraint<ScalarType, OrdinalType>>(aReductionOps);
    aConstraintList = std::make_shared<Plato::CriterionList<ScalarType, OrdinalType> >();
    aConstraintList->add(tSromConstraint);
}

/******************************************************************************//**
 * \brief Set lower and upper bounds for sample-probability pairs.
 * \param [in,out] aInputsKSAL input data structure for KSAL algorithm
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void set_sample_probability_pairs_bounds
(const Plato::SromInputs<ScalarType, OrdinalType>& aSromInputs,
 Plato::AlgorithmInputsKSAL<ScalarType, OrdinalType>& aInputsKSAL)
{
    if(aSromInputs.mNumSamples <= static_cast<OrdinalType>(0))
    {
        THROWERR(std::string("Set SROM Problem Bounds: Number of samples is <= 0. The input number of samples is set to '")
           + std::to_string(aSromInputs.mNumSamples) + "'.")
    }

    if(aSromInputs.mDimension <= static_cast<OrdinalType>(0))
    {
        THROWERR(std::string("Set SROM Problem Bounds: Number of random vector dimensions is <= 0. The input number of samples is set to '")
           + std::to_string(aSromInputs.mDimension) + "'.")
    }

    // NumVectors = Random Vector Dim + Probability Vector
    const OrdinalType tNumVectors = aSromInputs.mDimension + static_cast<OrdinalType>(1);
    aInputsKSAL.mLowerBounds =
        std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(tNumVectors, aSromInputs.mNumSamples);
    ScalarType tScalarValue = 0;
    Plato::fill(tScalarValue, *aInputsKSAL.mLowerBounds);

    tScalarValue = 1;
    aInputsKSAL.mUpperBounds = aInputsKSAL.mLowerBounds->create();
    Plato::fill(tScalarValue, *aInputsKSAL.mUpperBounds);
}

/******************************************************************************//**
 * \brief Set initial guess for random vector of samples.
 * \param [in]  SromInputs   stochastic reduced order model inputs
 * \param [out] aInputsKSAL  input data structure for Kelley-Sachs Augmented Lagrangian algorithm
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void set_samples_initial_guess
(const Plato::SromInputs<ScalarType, OrdinalType>& aSromInputs,
 Plato::AlgorithmInputsKSAL<ScalarType, OrdinalType>& aInputsKSAL)
{
    switch(aSromInputs.mInitialGuess)
    {
        case Plato::SromInitialGuess::random:
        {
            Plato::random_sample_initial_guess(*aInputsKSAL.mLowerBounds, *aInputsKSAL.mUpperBounds, *aInputsKSAL.mInitialGuess);
            break;
        }
        default:
        case Plato::SromInitialGuess::uniform:
        {
            Plato::uniform_samples_initial_guess(*aInputsKSAL.mInitialGuess);
            break;
        }
    }
}

/******************************************************************************//**
 * \brief Set initial guess for sample-probability pairs.
 * \param [in]  SromInputs   stochastic reduced order model inputs
 * \param [out] aInputsKSAL  input data structure for Kelley-Sachs Augmented Lagrangian algorithm
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void set_sample_probability_pairs_initial_guess
(const Plato::SromInputs<ScalarType, OrdinalType>& aSromInputs,
 Plato::AlgorithmInputsKSAL<ScalarType, OrdinalType>& aInputsKSAL)
{
    if(aInputsKSAL.mLowerBounds->getNumVectors() <= 0)
    {
        THROWERR("Set Sample-Probability Pairs Initial Guess: input template multi-vector is empty, i.e. aInputsKSAL.mLowerBounds.")
    }
    const OrdinalType tNumConstraints = 1;
    const OrdinalType tNumDualVectors = 1;
    aInputsKSAL.mDual =
        std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(tNumDualVectors, tNumConstraints);

    // ********* SET INTIAL GUESS FOR VECTOR OF SAMPLES *********
    aInputsKSAL.mInitialGuess = aInputsKSAL.mLowerBounds->create();
    Plato::set_samples_initial_guess(aSromInputs, aInputsKSAL);

    // ********* SET INTIAL GUESS FOR VECTOR OF PROBABILITIES *********
    auto tProbVecIndex = aInputsKSAL.mInitialGuess->getNumVectors() - static_cast<OrdinalType>(1);
    auto tScalarValue = static_cast<ScalarType>(1.0) / static_cast<ScalarType>(aSromInputs.mNumSamples);
    (*aInputsKSAL.mInitialGuess)[tProbVecIndex].fill(tScalarValue);
}
// function set_sample_probability_pairs_initial_guess

/******************************************************************************//**
 * \brief Output diagnostics for stochastic reduced order model (SROM) optimization problem
 * \param [in] aCommWrapper distributed memory communicator wrapper
 * \param [in] aSromDiagnostics SROM problem diagnostics
 * \param [in] aOutputsKSAL optimizer diagnostics
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void output_srom_diagnostics(const Plato::CommWrapper & aCommWrapper,
                                    const Plato::SromDiagnostics<ScalarType>& aSromDiagnostics,
                                    const Plato::AlgorithmOutputsKSAL<ScalarType, OrdinalType>& aOutputsKSAL)
{
    try
    {
        Plato::error::check_null_comm(aCommWrapper);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        THROWERR(tErrorMsg.what())
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
 * \brief Solve optimization problem to construct a stochastic reduced order model (SROM)
 * \param [in] aStatsInputs data structure with inputs for probability distribution function
 * \param [in,out] aInputsKSAL input data structure for KSAL algorithm
 * \param [in,out] aSromDiagnostics diagnostics associated with the SROM optimization problem
 * \param [in,out] aSolution outputs from SROM optimization problem
 * \param [in] aPrintDiagnostics flag use to enable output to file (default = false)
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void solve_srom_problem
(const Plato::SromInputs<ScalarType, OrdinalType>& aStatsInputs,
 Plato::AlgorithmInputsKSAL<ScalarType, OrdinalType>& aInputsKSAL,
 Plato::SromDiagnostics<ScalarType>& aSromDiagnostics,
 std::vector<Plato::SromOutputs<ScalarType>>& aSolution,
 bool aPrintDiagnostics = false)
{
    // build distribution
    std::shared_ptr<Plato::Distribution<ScalarType, OrdinalType>> tDistribution;
    tDistribution = Plato::build_distrubtion<ScalarType, OrdinalType>(aStatsInputs);

    // build srom objective and constraint
    std::shared_ptr<Plato::SromObjective<ScalarType, OrdinalType>> tSromObjective;
    std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> tSromConstraints;
    Plato::build_srom_objective_function(aStatsInputs, tDistribution, tSromObjective);
    Plato::build_srom_constraint_function(aInputsKSAL.mReductionOperations, tSromConstraints);

    // set initial guess and bounds
    Plato::set_sample_probability_pairs_bounds(aStatsInputs, aInputsKSAL);
    Plato::set_sample_probability_pairs_initial_guess(aStatsInputs, aInputsKSAL);

    // solve srom optimization problem
    aInputsKSAL.mMaxTrustRegionRadius = 1.0;
    aInputsKSAL.mDisablePostSmoothing = true;
    aInputsKSAL.mControlStagnationTolerance = 1e-5;
    aInputsKSAL.mPrintDiagnostics = aPrintDiagnostics;
    aInputsKSAL.mHessianMethod = Plato::Hessian::DISABLED;
    Plato::AlgorithmOutputsKSAL<ScalarType, OrdinalType> tOutputsKSAL;
    Plato::solve_ksal<ScalarType, OrdinalType>(tSromObjective, tSromConstraints, aInputsKSAL, tOutputsKSAL);

    // denormalize output
    aSolution.resize(aStatsInputs.mNumSamples); // set output data
    Plato::save_srom_solution(aStatsInputs, *tOutputsKSAL.mSolution, aSolution);
    Plato::save_srom_cdf_diagnostics(*tSromObjective, aSromDiagnostics);
    Plato::save_srom_moments_diagnostics(*tSromObjective, aSromDiagnostics);

    if(aPrintDiagnostics == true)
    {
        // transfer to output data structure
        Plato::output_cdf_comparison(aInputsKSAL.mCommWrapper, *tOutputsKSAL.mSolution, *tDistribution, aStatsInputs);
        Plato::output_srom_diagnostics(aInputsKSAL.mCommWrapper, aSromDiagnostics, tOutputsKSAL);
    }
}
// function solve_uncertainty

} // namespace Plato
