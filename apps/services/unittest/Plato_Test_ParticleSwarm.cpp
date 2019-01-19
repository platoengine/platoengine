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
 //S
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
 //
 // *************************************************************************
 //@HEADER
 */

/*
 * Plato_Test_ParticleSwarm.cpp
 *
 *  Created on: Dec 26, 2018
 */

#include "gtest/gtest.h"

#include <cmath>
#include <limits>
#include <random>
#include <iomanip>
#include <fstream>

#include "Plato_DataFactory.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_UnitTestUtils.hpp"
#include "Plato_AlgebraicRocketModel.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Return target thrust profile for gradient-based unit test
 * @return standard vector with target thrust profile
**********************************************************************************/
std::vector<double> get_target_thrust_profile()
{
    std::vector<double> tTargetThrustProfile =
        { 0, 1656714.377766964, 1684717.520617273, 1713123.001583093, 1741935.586049868, 1771160.083875437,
                1800801.349693849, 1830864.28322051, 1861353.829558637, 1892274.979507048, 1923632.769869272,
                1955432.283763989, 1987678.650936801, 2020377.048073344, 2053532.699113719, 2087150.875568287,
                2121236.896834771, 2155796.130516737, 2190833.992743404, 2226355.948490792, 2262367.511904243,
                2298874.246622283, 2335881.766101836, 2373395.733944806, 2411421.864226017, 2449965.921822503,
                2489033.722744186, 2528631.134465915, 2568764.076260844, 2609438.519535244, 2650660.488164633,
                2692436.058831303, 2734771.361363255, 2777672.579074459, 2821145.949106557, 2865197.762771913,
                2909834.365898075, 2955062.159173611, 3000887.598495364, 3047317.195317072, 3094357.516999425,
                3142015.18716148, 3190296.886033527, 3239209.350811319, 3288759.376011737, 3338953.813829865,
                3389799.574497465, 3441303.626642879, 3493472.997652346, 3546314.774032734, 3599836.101775718,
                3654044.186723352, 3708946.294935087, 3764549.753056224, 3820861.948687783, 3877890.330757833,
                3935642.409894215, 3994125.758798767, 4053348.012622938, 4113316.869344868, 4174040.090147917,
                4235525.499800648, 4297780.987038235, 4360814.504945371, 4424634.071340578, 4489247.76916203,
                4554663.746854796, 4620890.218759571, 4687935.465502855, 4755807.834388626, 4824515.739791448,
                4894067.663551098, 4964472.155368621, 5035737.83320389, 5107873.383674653, 5180887.562457044,
                5254789.194687578, 5329587.175366664, 5405290.469763565, 5481908.11382287, 5559449.214572486,
                5637922.950533082, 5717338.572129052, 5797705.402100981, 5879032.835919643, 5961330.342201422,
                6044607.46312535, 6128873.814851565, 6214139.087941348, 6300413.047778608, 6387705.534992979,
                6476026.465884338, 6565385.832848894, 6655793.704806847, 6747260.227631442, 6839795.624579719,
                6933410.196724654, 7028114.32338894, 7123918.462580209, 7220833.151427887 };

    return (tTargetThrustProfile);
}

struct particle_swarm
{
    enum stop_t
    {
        DID_NOT_CONVERGE = 1,
        MAX_NUMBER_ITERATIONS = 2,
        TRUE_OBJECTIVE_TOLERANCE = 3,
        MEAN_OBJECTIVE_TOLERANCE = 4,
    };
};

/******************************************************************************//**
 * @brief Diagnostic data for the Particle Swarm Optimization (PSO) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct OutputDataPSO
{
    /******************************************************************************//**
     * @brief Constructor
    **********************************************************************************/
    OutputDataPSO() :
            mNumIter(0),
            mObjFuncCount(0),
            mNumConstraints(0),
            mTrustRegionMultiplier(0),
            mMeanCurrentBestObjFuncValues(0),
            mCurrentGlobalBestObjFuncValue(0),
            mStdDevCurrentBestObjFuncValues(0)
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~OutputDataPSO()
    {
    }

    OrdinalType mNumIter;  /*!< number of outer iterations */
    OrdinalType mObjFuncCount;  /*!< number of objective function evaluations */
    OrdinalType mNumConstraints;  /*!< number of constraints - only needed for ALPSO output */

    ScalarType mTrustRegionMultiplier;  /*!< trust region multiplier */
    ScalarType mMeanCurrentBestObjFuncValues;  /*!< mean - objective function value */
    ScalarType mCurrentGlobalBestObjFuncValue;  /*!< best - objective function value */
    ScalarType mStdDevCurrentBestObjFuncValues;  /*!< standard deviation - objective function value */
};
// struct OutputDataPSO

/******************************************************************************//**
 * @brief Diagnostic data for the augmented Lagrangian Particle Swarm Optimization (ALPSO) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct OutputDataALPSO
{
    OutputDataALPSO() :
        mNumIter(0),
        mNumConstraints(0),
        mAugLagFuncCount(0),
        mMeanCurrentBestObjFuncValues(0),
        mCurrentGlobalBestObjFuncValue(0),
        mStdDevCurrentBestObjFuncValues(0),
        mMeanCurrentBestAugLagValues(0),
        mCurrentGlobalBestAugLagValue(0),
        mStdDevCurrentBestAugLagValues(0),
        mMeanCurrentBestConstraintValues(),
        mCurrentGlobalBestConstraintValues(),
        mStdDevCurrentBestConstraintValues(),
        mMeanCurrentPenaltyMultipliers(),
        mStdDevCurrentPenaltyMultipliers(),
        mMeanCurrentLagrangeMultipliers(),
        mStdDevCurrentLagrangeMultipliers()
    {
    }

    explicit OutputDataALPSO(const OrdinalType & aNumConstraints) :
        mNumIter(0),
        mNumConstraints(aNumConstraints),
        mAugLagFuncCount(0),
        mMeanCurrentBestObjFuncValues(0),
        mCurrentGlobalBestObjFuncValue(0),
        mStdDevCurrentBestObjFuncValues(0),
        mMeanCurrentBestAugLagValues(0),
        mCurrentGlobalBestAugLagValue(0),
        mStdDevCurrentBestAugLagValues(0),
        mMeanCurrentBestConstraintValues(std::vector<ScalarType>(aNumConstraints, 0)),
        mCurrentGlobalBestConstraintValues(std::vector<ScalarType>(aNumConstraints, 0)),
        mStdDevCurrentBestConstraintValues(std::vector<ScalarType>(aNumConstraints, 0)),
        mMeanCurrentPenaltyMultipliers(std::vector<ScalarType>(aNumConstraints, 0)),
        mStdDevCurrentPenaltyMultipliers(std::vector<ScalarType>(aNumConstraints, 0)),
        mMeanCurrentLagrangeMultipliers(std::vector<ScalarType>(aNumConstraints, 0)),
        mStdDevCurrentLagrangeMultipliers(std::vector<ScalarType>(aNumConstraints, 0))
    {
    }

    OrdinalType mNumIter;  /*!< number of outer iterations */
    OrdinalType mNumConstraints;  /*!< number of constraints */
    OrdinalType mAugLagFuncCount;  /*!< number of augmented Lagrangian function evaluations */

    ScalarType mMeanCurrentBestObjFuncValues;  /*!< mean objective function value */
    ScalarType mCurrentGlobalBestObjFuncValue;  /*!< best objective function value */
    ScalarType mStdDevCurrentBestObjFuncValues;  /*!< standard deviation for objective function value */

    ScalarType mMeanCurrentBestAugLagValues;  /*!< mean augmented Lagrangian function */
    ScalarType mCurrentGlobalBestAugLagValue;  /*!< best augmented Lagrangian function */
    ScalarType mStdDevCurrentBestAugLagValues;  /*!< standard deviation for augmented Lagrangian function */

    std::vector<ScalarType> mMeanCurrentBestConstraintValues;  /*!< mean constraint values */
    std::vector<ScalarType> mCurrentGlobalBestConstraintValues;  /*!< constraint values at global best particle location */
    std::vector<ScalarType> mStdDevCurrentBestConstraintValues;  /*!< standard deviation for constraint values */

    std::vector<ScalarType> mMeanCurrentPenaltyMultipliers;  /*!< mean penalty multipliers */
    std::vector<ScalarType> mStdDevCurrentPenaltyMultipliers;  /*!< standard deviation for penalty multipliers */

    std::vector<ScalarType> mMeanCurrentLagrangeMultipliers;  /*!< mean Lagrange multipliers */
    std::vector<ScalarType> mStdDevCurrentLagrangeMultipliers;  /*!< standard deviation for Lagrange multipliers */
};
// struct OutputDataALPSO

namespace pso
{

/******************************************************************************//**
 * @brief Check if output file is open. An error is thrown if output file is not open.
 * @param [in] aOutputFile output file
**********************************************************************************/
template<typename Type>
inline void is_file_open(const Type & aOutputFile)
{
    try
    {
        if(aOutputFile.is_open() == false)
        {
            throw std::invalid_argument("\n\n ******** MESSAGE: OUTPUT FILE IS NOT OPEN. ABORT! ******** \n\n");
        }
    }
    catch(const std::invalid_argument & tError)
    {
        throw tError;
    }
}

/******************************************************************************//**
 * @brief Check if the input vector is empty. If empty, thrown exception.
 * @param [in] aInput vector/container
**********************************************************************************/
template<typename ScalarType>
inline void is_vector_empty(const std::vector<ScalarType>& aInput)
{
    try
    {
        if(aInput.empty() == true)
        {
            throw std::invalid_argument("\n\n ******** MESSAGE: CONTAINER SIZE IS 0. ABORT! ******** \n\n");;
        }
    }
    catch(const std::invalid_argument & tError)
    {
        throw tError;
    }
}

/******************************************************************************//**
 * @brief Output a brief sentence explaining why the optimizer stopped.
 * @param [in] aStopCriterion stopping criterion flag
 * @param [in,out] aOutput string with brief description
 **********************************************************************************/
inline void get_stop_criterion(const Plato::particle_swarm::stop_t & aCriterion, std::string & aOutput)
{
    aOutput.clear();
    switch(aCriterion)
    {
        case Plato::particle_swarm::stop_t::DID_NOT_CONVERGE:
        {
            aOutput = "\n\n****** Optimization algorithm did not converge. ******\n\n";
            break;
        }
        case Plato::particle_swarm::stop_t::MAX_NUMBER_ITERATIONS:
        {
            aOutput = "\n\n****** Optimization stopping due to exceeding maximum number of iterations. ******\n\n";
            break;
        }
        case Plato::particle_swarm::stop_t::TRUE_OBJECTIVE_TOLERANCE:
        {
            aOutput = "\n\n****** Optimization stopping due to global best objective function tolerance being met. ******\n\n";
            break;
        }
        case Plato::particle_swarm::stop_t::MEAN_OBJECTIVE_TOLERANCE:
        {
            aOutput = "\n\n****** Optimization stopping due to mean objective function tolerance being met. ******\n\n";
            break;
        }
    }
}

/******************************************************************************//**
 * @brief Print header for Kelley-Sachs-Bound-Constrained (KSBC) diagnostics file
 * @param [in,out] aOutputFile output file
 * @param [in] aPrint flag use to enable/disable output (default = disabled)
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void print_pso_diagnostics_header(const Plato::OutputDataPSO<ScalarType, OrdinalType>& aData,
                                         std::ofstream& aOutputFile,
                                         bool aPrint = false)
{
    try
    {
        Plato::pso::is_file_open(aOutputFile);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {

        std::ostringstream tMessage;
        tMessage << "\n\n ******** ERROR IN FILE: " << __FILE__ << ", FUNCTION: " << __PRETTY_FUNCTION__
        << ", LINE: " << __LINE__ << " ******** \n\n";
        tMessage << tErrorMsg.what();
        if(aPrint == true)
        {
            std::cout << tMessage.str().c_str() << std::flush;
        }
        throw std::invalid_argument(tMessage.str().c_str());
    }

    aOutputFile << std::scientific << std::setprecision(6) << std::right << "Iter" << std::setw(10) << "F-count"
            << std::setw(15) << "Best(F)" << std::setw(15) << "Mean(F)" << std::setw(15) << "StdDev(F)" << std::setw(15)
            << "TR-Radius" << "\n" << std::flush;
}

/******************************************************************************//**
 * @brief Print diagnostics for bound constrained Particle Swarm Optimization (PSO) algorithm.
 * @param [in] aData diagnostic data PSO algorithm
 * @param [in,out] aOutputFile output file
 * @param [in] aPrint flag use to enable/disable output (default = disabled)
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void print_pso_diagnostics(const Plato::OutputDataPSO<ScalarType, OrdinalType>& aData,
                                  std::ofstream& aOutputFile,
                                  bool aPrint = false)
{
    try
    {
        Plato::pso::is_file_open(aOutputFile);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        std::ostringstream tMessage;
        tMessage << "\n\n ******** ERROR IN FILE: " << __FILE__ << ", FUNCTION: " << __PRETTY_FUNCTION__ << ", LINE: "
                << __LINE__ << " ******** \n\n";
        tMessage << tErrorMsg.what();
        if(aPrint == true)
        {
            std::cout << tMessage.str().c_str() << std::flush;
        }
        throw std::invalid_argument(tMessage.str().c_str());
    }

    aOutputFile << std::scientific << std::setprecision(6) << std::right << aData.mNumIter << std::setw(10)
            << aData.mObjFuncCount << std::setw(20) << aData.mCurrentGlobalBestObjFuncValue << std::setw(15)
            << aData.mMeanCurrentBestObjFuncValues << std::setw(15) << aData.mStdDevCurrentBestObjFuncValues << std::setw(15)
            << aData.mTrustRegionMultiplier << "\n" << std::flush;
}

template<typename ScalarType, typename OrdinalType>
inline void print_alpso_constraint_headers(const Plato::OutputDataALPSO<ScalarType, OrdinalType>& aData,
                                           std::ofstream& aOutputFile)
{
    const OrdinalType tNumConstraints = aData.mCurrentGlobalBestConstraintValues.size();
    for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
    {
        const OrdinalType tConstraintIndex = tIndex + static_cast<OrdinalType>(1);
        const OrdinalType tWidth = tIndex == static_cast<OrdinalType>(0) ? 13 : 12;

        aOutputFile << std::setw(tWidth) << "Best(H" << tConstraintIndex << ")" << std::setw(13)
                << "Mean(H" << tConstraintIndex << ")" << std::setw(14) << "StdDev(H" << tConstraintIndex << ")"
                << std::setw(12) << "Mean(P" << tConstraintIndex << ")" << std::setw(14) << "StdDev(P"
                << tConstraintIndex << ")" << std::setw(12) << "Mean(l" << tConstraintIndex << ")" << std::setw(14)
                << "StdDev(l" << tConstraintIndex << ")";
    }
}

/******************************************************************************//**
 * @brief Print header in augmented Lagrangian Particle Swarm Optimization (ALPSO)
 *        algorithm's diagnostic file.
 * @param [in] aData diagnostic data for ALPSO algorithm
 * @param [in,out] aOutputFile output/diagnostics file
 * @param [in] aPrint flag use to enable/disable output (default = disabled)
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void print_alpso_diagnostics_header(const Plato::OutputDataALPSO<ScalarType, OrdinalType>& aData,
                                           std::ofstream& aOutputFile,
                                           bool aPrint = false)
{
    try
    {
        Plato::pso::is_file_open(aOutputFile);
        Plato::pso::is_vector_empty(aData.mCurrentGlobalBestConstraintValues);
        Plato::pso::is_vector_empty(aData.mMeanCurrentBestConstraintValues);
        Plato::pso::is_vector_empty(aData.mStdDevCurrentBestConstraintValues);
        Plato::pso::is_vector_empty(aData.mMeanCurrentPenaltyMultipliers);
        Plato::pso::is_vector_empty(aData.mStdDevCurrentPenaltyMultipliers);
        Plato::pso::is_vector_empty(aData.mMeanCurrentLagrangeMultipliers);
        Plato::pso::is_vector_empty(aData.mStdDevCurrentLagrangeMultipliers);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {

        std::ostringstream tMessage;
        tMessage << "\n\n ********\n ERROR IN FILE: " << __FILE__ << "\n FUNCTION: " << __PRETTY_FUNCTION__
        << "\n LINE: " << __LINE__ << "\n ********";
        tMessage << tErrorMsg.what();
        if(aPrint == true)
        {
            std::cout << tMessage.str().c_str() << std::flush;
        }
        throw std::invalid_argument(tMessage.str().c_str());
    }

    aOutputFile << std::scientific << std::setprecision(6) << std::right << "Iter" << std::setw(10) << "F-count"
            << std::setw(15) << "Best(L)" << std::setw(15) << "Mean(L)" << std::setw(15) << "StdDev(L)" << std::setw(15)
            << "Best(F)" << std::setw(15) << "Mean(F)" << std::setw(15) << "StdDev(F)" << std::setw(15) << "TR-Radius";
    Plato::pso::print_alpso_constraint_headers(aData, aOutputFile);
    aOutputFile << "\n" << std::flush;
}

template<typename ScalarType, typename OrdinalType>
inline void print_alpso_outer_constraint_diagnostics(const Plato::OutputDataALPSO<ScalarType, OrdinalType>& aData,
                                                     std::ofstream& aOutputFile)
{
    for(OrdinalType tIndex = 0; tIndex < aData.mNumConstraints; tIndex++)
    {
        aOutputFile << std::setw(15) << aData.mCurrentGlobalBestConstraintValues[tIndex] << std::setw(15)
                << aData.mMeanCurrentBestConstraintValues[tIndex] << std::setw(15) << aData.mStdDevCurrentBestConstraintValues[tIndex]
                << std::setw(15) << aData.mMeanCurrentPenaltyMultipliers[tIndex] << std::setw(15)
                << aData.mStdDevCurrentPenaltyMultipliers[tIndex] << std::setw(15) << aData.mMeanCurrentLagrangeMultipliers[tIndex]
                << std::setw(15) << aData.mStdDevCurrentLagrangeMultipliers[tIndex];
    }
}

/******************************************************************************//**
 * @brief Print diagnostics for augmented Lagrangian Particle Swarm Optimization (ALPSO) algorithm.
 * @param [in] aData diagnostics data for ALPSO algorithm
 * @param [in,out] aOutputFile output file
 * @param [in] aPrint flag use to enable/disable output (default = disabled)
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void print_alpso_outer_diagnostics(const Plato::OutputDataALPSO<ScalarType, OrdinalType>& aData,
                                          std::ofstream& aOutputFile,
                                          bool aPrint = false)
{
    try
    {
        Plato::pso::is_file_open(aOutputFile);
        Plato::pso::is_vector_empty(aData.mCurrentGlobalBestConstraintValues);
        Plato::pso::is_vector_empty(aData.mMeanCurrentBestConstraintValues);
        Plato::pso::is_vector_empty(aData.mStdDevCurrentBestConstraintValues);
        Plato::pso::is_vector_empty(aData.mMeanCurrentPenaltyMultipliers);
        Plato::pso::is_vector_empty(aData.mStdDevCurrentPenaltyMultipliers);
        Plato::pso::is_vector_empty(aData.mMeanCurrentLagrangeMultipliers);
        Plato::pso::is_vector_empty(aData.mStdDevCurrentLagrangeMultipliers);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        std::ostringstream tMessage;
        tMessage << "\n\n ********\n ERROR IN FILE: " << __FILE__ << "\n FUNCTION: " << __PRETTY_FUNCTION__
                << "\n LINE: " << __LINE__ << "\n ********";
        tMessage << tErrorMsg.what();
        if(aPrint == true)
        {
            std::cout << tMessage.str().c_str() << std::flush;
        }
        throw std::invalid_argument(tMessage.str().c_str());
    }

    // ******** PRINT DIAGNOSTICS ********
    aOutputFile << std::scientific << std::setprecision(6) << std::right << aData.mNumIter << std::setw(10)
            << aData.mAugLagFuncCount << std::setw(20) << aData.mCurrentGlobalBestAugLagValue << std::setw(15)
            << aData.mMeanCurrentBestAugLagValues << std::setw(15) << aData.mStdDevCurrentBestAugLagValues << std::setw(15)
            << aData.mCurrentGlobalBestObjFuncValue << std::setw(15) << aData.mMeanCurrentBestObjFuncValues << std::setw(15)
            << aData.mStdDevCurrentBestObjFuncValues << std::setw(15) << "*";
    Plato::pso::print_alpso_outer_constraint_diagnostics(aData, aOutputFile);
    aOutputFile << "\n" << std::flush;
}

template<typename ScalarType, typename OrdinalType>
inline void print_alpso_inner_constraint_diagnostics(const Plato::OutputDataPSO<ScalarType, OrdinalType>& aData,
                                                     std::ofstream& aOutputFile)
{
    const OrdinalType tNumConstraints = aData.mNumConstraints;
    for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
    {
        aOutputFile << std::setw(15) << "*" << std::setw(15) << "*" << std::setw(15) << "*" << std::setw(15) << "*"
                << std::setw(15) << "*" << std::setw(15) << "*" << std::setw(15) << "*";
    }
}

/******************************************************************************//**
 * @brief Print inner-loop diagnostics for for augmented Lagrangian Particle Swarm Optimization (ALPSO) algorithm.
 * @param [in] aData diagnostics data for bound constrained PSO algorithm
 * @param [in,out] aOutputFile output/diagnostics file
 * @param [in] aPrint flag use to enable/disable output (default = disabled)
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void print_alpso_inner_diagnostics(const Plato::OutputDataPSO<ScalarType, OrdinalType>& aData,
                                          std::ofstream& aOutputFile,
                                          bool aPrint = false)
{
    try
    {
        Plato::pso::is_file_open(aOutputFile);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        std::ostringstream tMessage;
        tMessage << "\n\n ********\n ERROR IN FILE: " << __FILE__ << "\n FUNCTION: " << __PRETTY_FUNCTION__
        << "\n LINE: " << __LINE__ << "\n ********";
        tMessage << tErrorMsg.what();
        if(aPrint == true)
        {
            std::cout << tMessage.str().c_str() << std::flush;
        }
        throw std::invalid_argument(tMessage.str().c_str());
    }

    // ******** DIAGNOSTICS FOR CONSTRAINTS ********
    aOutputFile << std::scientific << std::setprecision(6) << std::right << aData.mNumIter << std::setw(10)
            << "*" << std::setw(20) << aData.mCurrentGlobalBestObjFuncValue << std::setw(15)
            << aData.mMeanCurrentBestObjFuncValues << std::setw(15) << aData.mStdDevCurrentBestObjFuncValues << std::setw(15) << "*"
            << std::setw(15) << "*" << std::setw(15) << "*" << std::setw(15) << aData.mTrustRegionMultiplier;
    Plato::pso::print_alpso_inner_constraint_diagnostics(aData, aOutputFile);
    aOutputFile << "\n" << std::flush;
}

} // namespace pso

template<typename ScalarType, typename OrdinalType>
inline ScalarType mean(const Plato::ReductionOperations<ScalarType, OrdinalType> & aReductions,
                       const Plato::Vector<ScalarType, OrdinalType> & aInput)
{
    const OrdinalType tSize = 1;
    const OrdinalType tELEMENT_INDEX = 0;
    Plato::StandardVector<ScalarType, OrdinalType> tWork(tSize);

    tWork[tELEMENT_INDEX] = aInput.size(); /* local number of elements */
    const OrdinalType tGlobalNumElements = aReductions.sum(tWork);
    ScalarType tOutput = aReductions.sum(aInput);
    tOutput = tOutput / tGlobalNumElements;
    return (tOutput);
}

template<typename ScalarType, typename OrdinalType>
inline ScalarType standard_deviation(const ScalarType & aMean,
                                     const Plato::Vector<ScalarType, OrdinalType> & aInput,
                                     const Plato::ReductionOperations<ScalarType, OrdinalType> & aReductions)
{
    const OrdinalType tSize = 1;
    const OrdinalType tELEMENT_INDEX = 0;
    Plato::StandardVector<ScalarType, OrdinalType> tWork(tSize);

    const OrdinalType tLocalNumElements = aInput.size();
    for(OrdinalType tIndex = 0; tIndex < tLocalNumElements; tIndex++)
    {
        const ScalarType tMisfit = aInput[tIndex] - aMean;
        tWork[tELEMENT_INDEX] += tMisfit * tMisfit;
    }

    ScalarType tOutput = aReductions.sum(tWork);
    tWork[tELEMENT_INDEX] = tLocalNumElements;
    const ScalarType tGlobalNumElements = aReductions.sum(tWork);
    tOutput = tOutput / (tGlobalNumElements - static_cast<OrdinalType>(1));
    tOutput = std::pow(tOutput, static_cast<ScalarType>(0.5));

    return (tOutput);
}

template<typename ScalarType, typename OrdinalType>
void find_best_criterion_values(const Plato::Vector<ScalarType, OrdinalType> & aCurrentValues,
                                Plato::Vector<ScalarType, OrdinalType> & aCurrentBestValues)
{
    assert(aCurrentValues.size() == aCurrentBestValues.size());

    const OrdinalType tNumParticles = aCurrentValues.size();
    for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
    {
        if(aCurrentValues[tIndex] < aCurrentBestValues[tIndex])
        {
            aCurrentBestValues[tIndex] = aCurrentValues[tIndex];
        }
    }
}

template<typename ScalarType, typename OrdinalType = size_t>
class GradFreeCriteria
{
public:
    virtual ~GradFreeCriteria()
    {
    }

    /******************************************************************************//**
     * @brief Evaluates general criterion
     * @param [in] aControl multi-vector of control variables (i.e. optimization variables)
     * @param [out] aOutput criterion value for each control vector
     **********************************************************************************/
    virtual void value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                       Plato::Vector<ScalarType, OrdinalType> & aOutput) = 0;
};
// class GradFreeCriteria

template<typename ScalarType, typename OrdinalType = size_t>
class GradFreeRocketObjFunc : public Plato::GradFreeCriteria<ScalarType, OrdinalType>
{
public:
    GradFreeRocketObjFunc(const Plato::Vector<ScalarType, OrdinalType>& aUpperBounds,
                          const Plato::AlgebraicRocketInputs<ScalarType>& aRocketInputs,
                          const std::shared_ptr<Plato::GeometryModel<ScalarType>>& aChamberGeom) :
            mTargetThrustProfileSet(false),
            mNormTargetThrustProfile(0),
            mAlgebraicRocketModel(aRocketInputs, aChamberGeom),
            mTargetThrustProfile(),
            mNormalizationConstants(aUpperBounds.create())
    {
        this->initialize(aUpperBounds);
    }

    virtual ~GradFreeRocketObjFunc()
    {
    }

    ScalarType getNormTargetThrustProfile() const
    {
        return (mNormTargetThrustProfile);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getNormalizationConstants() const
    {
        return (*mNormalizationConstants);
    }

    /******************************************************************************//**
     * @brief Disables output to console from algebraic rocket model.
    **********************************************************************************/
    void disableOutput()
    {
        mAlgebraicRocketModel.disableOutput();
    }

    void setTargetThrustProfile(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        if(mTargetThrustProfile.get() == nullptr)
        {
            mTargetThrustProfile = aInput.create();
        }
        assert(mTargetThrustProfile->size() == aInput.size());
        mTargetThrustProfile->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
        mNormTargetThrustProfile = mTargetThrustProfile->dot(*mTargetThrustProfile);
        //mNormTargetValues = std::pow(mNormTargetValues, 0.5);
        mTargetThrustProfileSet = true;
    }

    void value(const Plato::MultiVector<ScalarType, OrdinalType> & aControls,
               Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {
        try
        {
            this->isTargetThrustProfileSet();
        }
        catch(const std::invalid_argument& tErrorMsg)
        {

            std::ostringstream tMessage;
            tMessage << "\n\n ******** ERROR IN FILE: " << __FILE__ << ", FUNCTION: " << __PRETTY_FUNCTION__
            << ", LINE: " << __LINE__ << " ******** \n\n";
            tMessage << tErrorMsg.what();
            throw std::invalid_argument(tMessage.str().c_str());
        }

        const OrdinalType tNumParticles = aControls.getNumVectors();
        assert(tNumParticles > static_cast<OrdinalType>(0));
        assert(aOutput.size() == tNumParticles);
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            aOutput[tIndex] = this->evaluate(aControls[tIndex]);
        }
    }

    void solve(const Plato::Vector<ScalarType, OrdinalType> & aControls,
               Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {
        this->update(aControls);
        mAlgebraicRocketModel.solve();
        std::vector<ScalarType> tTrialThrustProfile = mAlgebraicRocketModel.getThrustProfile();
        assert(aOutput.size() == tTrialThrustProfile.size());
        for(OrdinalType tIndex = 0; tIndex < aOutput.size(); tIndex++)
        {
            aOutput[tIndex] = tTrialThrustProfile[tIndex];
        }
    }

private:
    void initialize(const Plato::Vector<ScalarType, OrdinalType> & aUpperBounds)
    {
        assert(aUpperBounds.size() > static_cast<OrdinalType>(0));
        std::vector<ScalarType> tNormalizationConstants(aUpperBounds.size());
        for(OrdinalType tIndex = 0; tIndex < aUpperBounds.size(); tIndex++)
        {
            (*mNormalizationConstants)[tIndex] = aUpperBounds[tIndex];
        }
    }

    void update(const Plato::Vector<ScalarType, OrdinalType> & aControls)
    {
        std::map<std::string, ScalarType> tChamberGeomParam;
        const ScalarType tRadius = aControls[0] * (*mNormalizationConstants)[0];
        tChamberGeomParam.insert(std::pair<std::string, ScalarType>("Radius", tRadius));
        tChamberGeomParam.insert(std::pair<std::string, ScalarType>("Configuration", Plato::Configuration::INITIAL));
        mAlgebraicRocketModel.updateInitialChamberGeometry(tChamberGeomParam);

        const ScalarType tRefBurnRate = aControls[1] * (*mNormalizationConstants)[1];
        std::map<std::string, ScalarType> tSimParam;
        tSimParam.insert(std::pair<std::string, ScalarType>("RefBurnRate", tRefBurnRate));
        mAlgebraicRocketModel.updateSimulation(tSimParam);
    }

    void isTargetThrustProfileSet()
    {
        try
        {
            if(mTargetThrustProfileSet == false)
            {
                throw std::invalid_argument("\n\n ******** MESSAGE: TARGET THRUST PROFILE IS NOT SET. ABORT! ******** \n\n");
            }
        }
        catch(const std::invalid_argument & tError)
        {
            throw tError;
        }
    }

    ScalarType evaluate(const Plato::Vector<ScalarType, OrdinalType> & aControls)
    {
        this->update(aControls);

        mAlgebraicRocketModel.solve();
        std::vector<ScalarType> tTrialThrustProfile = mAlgebraicRocketModel.getThrustProfile();
        assert(tTrialThrustProfile.size() == mTargetThrustProfile->size());

        ScalarType tObjFuncValue = 0;
        const OrdinalType tNumThrustEvalPoints = mTargetThrustProfile->size();
        for(OrdinalType tIndex = 0; tIndex < tNumThrustEvalPoints; tIndex++)
        {
            ScalarType tMisfit = tTrialThrustProfile[tIndex] - (*mTargetThrustProfile)[tIndex];
            tObjFuncValue += tMisfit * tMisfit;
        }
        tObjFuncValue = static_cast<ScalarType>(1.0 / (2.0 * tNumThrustEvalPoints * mNormTargetThrustProfile)) * tObjFuncValue;

        return tObjFuncValue;
    }

private:
    bool mTargetThrustProfileSet;
    ScalarType mNormTargetThrustProfile;
    Plato::AlgebraicRocketModel<ScalarType> mAlgebraicRocketModel;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mTargetThrustProfile;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mNormalizationConstants;
};
// class GradFreeAlgebraicRocketObjFunc

template<typename ScalarType, typename OrdinalType = size_t>
class GradFreeRosenbrock : public Plato::GradFreeCriteria<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
    **********************************************************************************/
    GradFreeRosenbrock()
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~GradFreeRosenbrock()
    {
    }

    /******************************************************************************//**
     * @brief Evaluate criterion at each particle.
     * @param [in] aControl multi-vector of control variables (i.e. optimization variables)
     * @param [out] aOutput criterion value for each control vector
     **********************************************************************************/
    void value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
               Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {
        const OrdinalType tNumParticles = aControl.getNumVectors();
        assert(tNumParticles > static_cast<OrdinalType>(0));
        assert(aOutput.size() == tNumParticles);
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            aOutput[tIndex] = this->evaluate(aControl[tIndex]);
        }
    }

private:
    /******************************************************************************//**
     * @brief Evaluate function:
     * \f$ 100 * \left(x_2 - x_1^2\right)^2 + \left(1 - x_1\right)^2 \f$
     * @param [in] aControl vector of control variables (i.e. optimization variables)
     * @return Rosenbrock function evaluation
     **********************************************************************************/
    ScalarType evaluate(const Plato::Vector<ScalarType, OrdinalType> & aControl)
    {
        const ScalarType tOutput = static_cast<ScalarType>(100.)
                * std::pow((aControl[1] - aControl[0] * aControl[0]), static_cast<ScalarType>(2))
                + std::pow(static_cast<ScalarType>(1) - aControl[0], static_cast<ScalarType>(2));
        return (tOutput);
    }

private:
    GradFreeRosenbrock(const Plato::GradFreeRosenbrock<ScalarType, OrdinalType> & aRhs);
    Plato::GradFreeRosenbrock<ScalarType, OrdinalType> & operator=(const Plato::GradFreeRosenbrock<ScalarType, OrdinalType> & aRhs);
};
// class GradFreeRosenbrock

template<typename ScalarType, typename OrdinalType = size_t>
class GradFreeRadius : public Plato::GradFreeCriteria<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aInput upper bound (i.e. limit) on constraint
    **********************************************************************************/
    explicit GradFreeRadius(ScalarType aInput = 1.0) :
        mLimit(aInput)
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~GradFreeRadius()
    {
    }

    /******************************************************************************//**
     * @brief Evaluate Rosenbrock criterion
     * @param [in] aInput radius upper bound
     **********************************************************************************/
    void setLimit(const ScalarType & aInput)
    {
        mLimit = aInput;
    }

    /******************************************************************************//**
     * @brief Evaluate criterion
     * @param [in] aControl multi-vector of control variables (i.e. optimization variables)
     * @param [out] aOutput criterion value for each control vector
     **********************************************************************************/
    void value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
               Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {
        const OrdinalType tNumParticles = aControl.getNumVectors();
        assert(tNumParticles > static_cast<OrdinalType>(0));
        assert(aOutput.size() == tNumParticles);
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            aOutput[tIndex] = this->evaluate(aControl[tIndex]);
        }
    }

private:
    /******************************************************************************//**
     * @brief Evaluate function:
     * \f$ \left(\mathbf{z}(0)\right)^2 + \left(\mathbf{z}(1)\right)^2 - limit \f$
     * @param [in] aControl vector of control variables (i.e. optimization variables)
     * @return function evaluation
     **********************************************************************************/
    ScalarType evaluate(const Plato::Vector<ScalarType, OrdinalType> & aControl)
    {
        assert(aControl.size() > static_cast<OrdinalType>(0));

        ScalarType tOutput = std::pow(aControl[0], static_cast<ScalarType>(2.))
                + std::pow(aControl[1], static_cast<ScalarType>(2.));
        tOutput = tOutput - mLimit;

        return (tOutput);
    }

private:
    ScalarType mLimit; /*!< radius upper bound */

private:
    GradFreeRadius(const Plato::GradFreeRadius<ScalarType, OrdinalType> & aRhs);
    Plato::GradFreeRadius<ScalarType, OrdinalType> & operator=(const Plato::GradFreeRadius<ScalarType, OrdinalType> & aRhs);
};
// class GradFreeRadius

/******************************************************************************//**
 * @brief Generic interface that gives access to the list of criteria
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class GradFreeCriteriaList
{
public:
    /******************************************************************************//**
     * @brief Constructor
    **********************************************************************************/
    GradFreeCriteriaList() :
            mWeights(),
            mList()
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~GradFreeCriteriaList()
    {
    }

    /******************************************************************************//**
     * @brief Return size of list
     * @return size
    **********************************************************************************/
    OrdinalType size() const
    {
        return (mList.size());
    }

    /******************************************************************************//**
     * @brief Initializes criterion list.
     * @param [in] aCriteria list of Plato criterion
    **********************************************************************************/
    void add(const std::vector<std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>>> & aCriteria)
    {
        assert(aCriteria.empty() == false);

        mList.clear();
        const OrdinalType tNumCriteria = aCriteria.size();
        for(OrdinalType tIndex = 0; tIndex < tNumCriteria; tIndex++)
        {
            assert(aCriteria[tIndex] != nullptr);
            mList.push_back(aCriteria[tIndex]);
            mWeights.push_back(static_cast<ScalarType>(1));
        }
    }

    /******************************************************************************//**
     * @brief Adds a new element at the end of the vector, after its current last element.
     * @param [in] aCriterion Plato criterion
     * @param [in] aMyWeight weight for input Plato criterion
    **********************************************************************************/
    void add(const std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> & aCriterion, ScalarType aMyWeight = 1)
    {
        assert(aCriterion != nullptr);
        mList.push_back(aCriterion);
        mWeights.push_back(aMyWeight);
    }

    /******************************************************************************//**
     * @brief Initializes criterion list and corresponding weights.
     * @param [in] aCriteria list of Plato criteria
     * @param [in] aWeights list of weights
    **********************************************************************************/
    void add(const std::vector<std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>>> & aCriteria, const std::vector<ScalarType> & aWeights)
    {
        assert(aCriteria.empty() == false);

        mList.clear();
        const OrdinalType tNumCriteria = aCriteria.size();
        for(OrdinalType tIndex = 0; tIndex < tNumCriteria; tIndex++)
        {
            assert(aCriteria[tIndex] != nullptr);
            mList.push_back(aCriteria[tIndex]);
        }
        assert(aWeights.empty() == false);
        assert(aWeights.size() == aCriteria.size());
        mWeights = aWeights;
    }

    /******************************************************************************//**
     * @brief Returns weight at position aIndex in the list.
     * @param [in] aIndex  Position of an element in the list.
     * @return weight
    **********************************************************************************/
    ScalarType weight(const OrdinalType & aIndex) const
    {
        assert(mWeights.empty() == false);
        assert(aIndex < mWeights.size());
        return (mWeights[aIndex]);
    }

    /******************************************************************************//**
     * @brief Returns a reference to the element at position aIndex in the list.
     * @param [in] aIndex Position of an element in the list.
     * @return The criterion at the specified position in the list.
    **********************************************************************************/
    Plato::GradFreeCriteria<ScalarType, OrdinalType> & operator [](const OrdinalType & aIndex)
    {
        assert(aIndex < mList.size());
        assert(mList[aIndex].get() != nullptr);
        return (mList[aIndex].operator*());
    }

    /******************************************************************************//**
     * @brief Returns a const reference to the element at position aIndex in the list.
     * @param [in] aIndex Position of an element in the list.
     * @return The criterion at the specified position in the list.
    **********************************************************************************/
    const Plato::GradFreeCriteria<ScalarType, OrdinalType> & operator [](const OrdinalType & aIndex) const
    {
        assert(aIndex < mList.size());
        assert(mList[aIndex].get() != nullptr);
        return (mList[aIndex].operator*());
    }

    /******************************************************************************//**
     * @brief Returns a copy of the list of criteria.
     * @return Copy of the list of criteria.
    **********************************************************************************/
    std::shared_ptr<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>> create() const
    {
        assert(this->size() > static_cast<OrdinalType>(0));
        std::shared_ptr<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>> tOutput =
                std::make_shared<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>>();
        const OrdinalType tNumCriterion = this->size();
        for(OrdinalType tIndex = 0; tIndex < tNumCriterion; tIndex++)
        {
            assert(mList[tIndex].get() != nullptr);
            const std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> & tCriterion = mList[tIndex];
            ScalarType tMyWeight = mWeights[tIndex];
            tOutput->add(tCriterion, tMyWeight);
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Returns a const shared pointer reference to the element at position aIndex in the list.
     * @param [in] aIndex Position of an element in the list.
     * @return The criterion at the specified position in the list.
    **********************************************************************************/
    const std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> & ptr(const OrdinalType & aIndex) const
    {
        assert(aIndex < mList.size());
        assert(mList[aIndex].get() != nullptr);
        return(mList[aIndex]);
    }

private:
    std::vector<ScalarType> mWeights; /*!< list of weights */
    std::vector<std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>>> mList; /*!< list of grad-free criteria */

private:
    GradFreeCriteriaList(const Plato::GradFreeCriteriaList<ScalarType, OrdinalType>&);
    Plato::GradFreeCriteriaList<ScalarType, OrdinalType> & operator=(const Plato::GradFreeCriteriaList<ScalarType, OrdinalType>&);
};
// class GradFreeCriteriaList

template<typename ScalarType, typename OrdinalType = size_t>
class ParticleSwarmDataMng
{
public:
    explicit ParticleSwarmDataMng(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory) :
            mGlobalNumParticles(0),
            mCurrentGlobalBestParticleRank(0),
            mCurrentGlobalBestParticleIndex(0),
            mTimeStep(1),
            mCurrentGlobalBestObjFuncValue(std::numeric_limits<ScalarType>::max()),
            mPreviousGlobalBestObjFuncValue(std::numeric_limits<ScalarType>::max()),
            mBestObjFunValueMean(std::numeric_limits<ScalarType>::max()),
            mBestObjFunValueStdDev(std::numeric_limits<ScalarType>::max()),
            mCurrentObjFuncValues(aFactory->objective().create()),
            mCurrentBestObjFuncValues(aFactory->objective().create()),
            mLowerBounds(),
            mUpperBounds(),
            mMeanParticlePosition(),
            mStdDevParticlePosition(),
            mGlobalBestParticlePosition(),
            mCurrentParticles(aFactory->control().create()),
            mCurrentVelocities(aFactory->control().create()),
            mPreviousVelocities(aFactory->control().create()),
            mBestParticlePositions(aFactory->control().create()),
            mCommWrapper(aFactory->getCommWrapper().create()),
            mCriteriaReductions(aFactory->getObjFuncReductionOperations().create())
    {
        this->initialize();
    }

    ~ParticleSwarmDataMng()
    {
    }

    OrdinalType getNumParticles() const
    {
        assert(static_cast<OrdinalType>(mCurrentParticles.use_count()) > static_cast<OrdinalType>(0));
        return (mCurrentParticles->getNumVectors());
    }

    OrdinalType getCurrentGlobalBestParticleRank() const
    {
        return (mCurrentGlobalBestParticleRank);
    }

    OrdinalType getCurrentGlobalBestParticleIndex() const
    {
        return (mCurrentGlobalBestParticleIndex);
    }

    void setCurrentGlobalBestParticleRank(const OrdinalType & aInput)
    {
        mCurrentGlobalBestParticleRank = aInput;
    }

    void setCurrentGlobalBestParticleIndex(const OrdinalType & aInput)
    {
        mCurrentGlobalBestParticleIndex = aInput;
    }

    ScalarType getTimeStep() const
    {
        return (mTimeStep);
    }

    ScalarType getMeanCurrentBestObjFuncValues() const
    {
        return (mBestObjFunValueMean);
    }

    ScalarType getStdDevCurrentBestObjFuncValues() const
    {
        return (mBestObjFunValueStdDev);
    }

    ScalarType getCurrentGlobalBestObjFuncValue() const
    {
        return (mCurrentGlobalBestObjFuncValue);
    }

    ScalarType getPreviousGlobalBestObjFuncValue() const
    {
        return (mPreviousGlobalBestObjFuncValue);
    }

    void setCurrentGlobalBestObjFunValue(const ScalarType & aInput)
    {
        mCurrentGlobalBestObjFuncValue = aInput;
    }

    void cacheGlobalBestObjFunValue()
    {
        mPreviousGlobalBestObjFuncValue = mCurrentGlobalBestObjFuncValue;
    }

    ScalarType getCurrentObjFuncValue(const OrdinalType & aIndex) const
    {
        assert(mCurrentObjFuncValues.get() != nullptr);
        return ((*mCurrentObjFuncValues)[aIndex]);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getCurrentObjFuncValues() const
    {
        assert(mCurrentObjFuncValues.get() != nullptr);
        return(*mCurrentObjFuncValues);
    }

    void setCurrentObjFuncValues(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        mCurrentObjFuncValues->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    const Plato::Vector<ScalarType, OrdinalType> & getCurrentBestObjFuncValues() const
    {
        assert(mCurrentBestObjFuncValues.get() != nullptr);
        return(*mCurrentBestObjFuncValues);
    }

    void setCurrentBestObjFuncValue(const OrdinalType & aIndex, const ScalarType & aInput)
    {
        (*mCurrentBestObjFuncValues)[aIndex] = aInput;
    }

    void setCurrentBestObjFuncValues(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        mCurrentBestObjFuncValues->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentParticles() const
    {
        assert(static_cast<OrdinalType>(mCurrentParticles.use_count()) > static_cast<OrdinalType>(0));
        return (*mCurrentParticles);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getCurrentParticle(const OrdinalType & aIndex) const
    {
        assert(static_cast<OrdinalType>(mCurrentParticles.use_count()) > static_cast<OrdinalType>(0));
        return ((*mCurrentParticles)[aIndex]);
    }

    void setCurrentParticle(const ScalarType & aInput)
    {
        assert(static_cast<OrdinalType>(mCurrentParticles.use_count()) > static_cast<OrdinalType>(0));
        Plato::fill(aInput, *mCurrentParticles);
    }

    void setCurrentParticle(const OrdinalType & aIndex, const ScalarType & aInput)
    {
        assert(static_cast<OrdinalType>(mCurrentParticles.use_count()) > static_cast<OrdinalType>(0));
        (*mCurrentParticles)[aIndex].fill(aInput);
    }

    void setCurrentParticle(const OrdinalType & aIndex, const Plato::Vector<ScalarType, OrdinalType> & aParticle)
    {
        assert(static_cast<OrdinalType>(mCurrentParticles.use_count()) > static_cast<OrdinalType>(0));
        assert(aParticle.size() > static_cast<OrdinalType>(0));
        (*mCurrentParticles)[aIndex].update(static_cast<ScalarType>(1), aParticle, static_cast<ScalarType>(0));
    }

    void setCurrentParticles(const Plato::MultiVector<ScalarType, OrdinalType> & aParticles)
    {
        assert(aParticles.getNumVectors() > static_cast<OrdinalType>(0));
        assert(static_cast<OrdinalType>(mCurrentParticles.use_count()) > static_cast<OrdinalType>(0));
        Plato::update(static_cast<ScalarType>(1), aParticles, static_cast<ScalarType>(0), *mCurrentParticles);
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentVelocities() const
    {
        assert(static_cast<OrdinalType>(mCurrentVelocities.use_count()) > static_cast<OrdinalType>(0));
        return (*mCurrentVelocities);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getCurrentVelocity(const OrdinalType & aIndex) const
    {
        assert(static_cast<OrdinalType>(mCurrentVelocities.use_count()) > static_cast<OrdinalType>(0));
        return ((*mCurrentVelocities)[aIndex]);
    }

    void setCurrentVelocity(const OrdinalType & aIndex, const Plato::Vector<ScalarType, OrdinalType> & aParticleVel)
    {
        assert(static_cast<OrdinalType>(mCurrentVelocities.use_count()) > static_cast<OrdinalType>(0));
        assert(aParticleVel.size() > static_cast<OrdinalType>(0));
        (*mCurrentVelocities)[aIndex].update(static_cast<ScalarType>(1), aParticleVel, static_cast<ScalarType>(0));
    }

    void setCurrentVelocities(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() > static_cast<OrdinalType>(0));
        assert(static_cast<OrdinalType>(mCurrentVelocities.use_count()) > static_cast<OrdinalType>(0));
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mCurrentVelocities);
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getPreviousVelocities() const
    {
        assert(static_cast<OrdinalType>(mPreviousVelocities.use_count()) > static_cast<OrdinalType>(0));
        return (*mPreviousVelocities);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getPreviousVelocity(const OrdinalType & aIndex) const
    {
        assert(static_cast<OrdinalType>(mPreviousVelocities.use_count()) > static_cast<OrdinalType>(0));
        return ((*mPreviousVelocities)[aIndex]);
    }

    void setPreviousVelocities(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() > static_cast<OrdinalType>(0));
        assert(static_cast<OrdinalType>(mPreviousVelocities.use_count()) > static_cast<OrdinalType>(0));
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mPreviousVelocities);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getLowerBounds() const
    {
        assert(static_cast<OrdinalType>(mLowerBounds.use_count()) > static_cast<OrdinalType>(0));
        return (*mLowerBounds);
    }

    void setLowerBounds(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(static_cast<OrdinalType>(mLowerBounds.use_count()) > static_cast<OrdinalType>(0));
        assert(aInput.size() > static_cast<OrdinalType>(0));
        mLowerBounds->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    void setLowerBounds(const ScalarType & aInput)
    {
        assert(static_cast<OrdinalType>(mLowerBounds.use_count()) > static_cast<OrdinalType>(0));
        mLowerBounds->fill(aInput);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getUpperBounds() const
    {
        assert(static_cast<OrdinalType>(mUpperBounds.use_count()) > static_cast<OrdinalType>(0));
        return (*mUpperBounds);
    }

    void setUpperBounds(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(static_cast<OrdinalType>(mUpperBounds.use_count()) > static_cast<OrdinalType>(0));
        assert(aInput.size() > static_cast<OrdinalType>(0));
        mUpperBounds->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    void setUpperBounds(const ScalarType & aInput)
    {
        assert(static_cast<OrdinalType>(mUpperBounds.use_count()) > static_cast<OrdinalType>(0));
        mUpperBounds->fill(aInput);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getParticlePositionMean() const
    {
        assert(static_cast<OrdinalType>(mMeanParticlePosition.use_count()) > static_cast<OrdinalType>(0));
        return (*mMeanParticlePosition);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getParticlePositionStdDev() const
    {
        assert(static_cast<OrdinalType>(mStdDevParticlePosition.use_count()) > static_cast<OrdinalType>(0));
        return (*mStdDevParticlePosition);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getGlobalBestParticlePosition() const
    {
        assert(static_cast<OrdinalType>(mGlobalBestParticlePosition.use_count()) > static_cast<OrdinalType>(0));
        return (*mGlobalBestParticlePosition);
    }

    void setGlobalBestParticlePosition(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));
        assert(static_cast<OrdinalType>(mGlobalBestParticlePosition.use_count()) > static_cast<OrdinalType>(0));
        mGlobalBestParticlePosition->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    const Plato::Vector<ScalarType, OrdinalType> & getBestParticlePosition(const OrdinalType & aIndex) const
    {
        assert(static_cast<OrdinalType>(mBestParticlePositions.use_count()) > static_cast<OrdinalType>(0));
        return ((*mBestParticlePositions)[aIndex]);
    }

    void setBestParticlePosition(const OrdinalType & aIndex, const Plato::Vector<ScalarType, OrdinalType> & aInput) const
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));
        assert(static_cast<OrdinalType>(mBestParticlePositions.use_count()) > static_cast<OrdinalType>(0));
        (*mBestParticlePositions)[aIndex].update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    void setBestParticlePositions(const Plato::MultiVector<ScalarType, OrdinalType> & aInput) const
    {
        assert(aInput.getNumVectors() > static_cast<OrdinalType>(0));
        assert(static_cast<OrdinalType>(mBestParticlePositions.use_count()) > static_cast<OrdinalType>(0));
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mBestParticlePositions);
    }

    void setInitialParticles()
    {
        std::default_random_engine tGenerator;
        std::uniform_real_distribution<ScalarType> tDistribution(0.0 /* lower bound */, 1.0 /* upper bound */);

        const OrdinalType tNumParticles = this->getNumParticles();
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            mWorkVector->update(static_cast<ScalarType>(1), *mUpperBounds, static_cast<ScalarType>(0));
            mWorkVector->update(static_cast<ScalarType>(-1), *mLowerBounds, static_cast<ScalarType>(1));

            ScalarType tMyRandomNum = tDistribution(tGenerator);
            (*mCurrentParticles)[tIndex].update(static_cast<ScalarType>(1), *mLowerBounds, static_cast<ScalarType>(0));
            (*mCurrentParticles)[tIndex].update(tMyRandomNum, *mWorkVector, static_cast<ScalarType>(1));
            (*mCurrentVelocities)[tIndex].update(static_cast<ScalarType>(1), (*mCurrentParticles)[tIndex], static_cast<ScalarType>(0));
        }
    }

    void cachePreviousVelocities()
    {
        Plato::update(static_cast<ScalarType>(1), *mCurrentVelocities, static_cast<ScalarType>(0), *mPreviousVelocities);
    }

    void findGlobalBestParticle()
    {
        // TODO: THINK PARALLEL IMPLEMENTATION
        Plato::ReductionOutputs<ScalarType, OrdinalType> tOutput;
        mCriteriaReductions->minloc(*mCurrentObjFuncValues, tOutput);
        const bool tFoundNewGlobalBest = tOutput.mOutputValue < mCurrentGlobalBestObjFuncValue;
        if(tFoundNewGlobalBest == true)
        {
            mCurrentGlobalBestObjFuncValue = tOutput.mOutputValue;
            mCurrentGlobalBestParticleRank = tOutput.mOutputRank;
            mCurrentGlobalBestParticleIndex = tOutput.mOutputIndex;
            const Plato::Vector<ScalarType, OrdinalType> & tCurrentParticle = this->getCurrentParticle(tOutput.mOutputIndex);
            this->setGlobalBestParticlePosition(tCurrentParticle);
        }
    }

    void updateBestParticlesData()
    {
        const OrdinalType tNumParticles = this->getNumParticles();
        const Plato::Vector<ScalarType, OrdinalType> & tCurrentObjFuncValues = this->getCurrentObjFuncValues();
        const Plato::Vector<ScalarType, OrdinalType> & tCurrentBestObjFuncValues = this->getCurrentBestObjFuncValues();
        for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
        {
            const ScalarType tCurrentObjFuncVal = tCurrentObjFuncValues[tParticleIndex];
            if(tCurrentObjFuncVal < tCurrentBestObjFuncValues[tParticleIndex])
            {
                const Plato::Vector<ScalarType, OrdinalType> & tCurrentParticle = this->getCurrentParticle(tParticleIndex);
                this->setBestParticlePosition(tParticleIndex, tCurrentParticle);
                this->setCurrentBestObjFuncValue(tParticleIndex, tCurrentObjFuncVal);
            }
        }
    }

    void computeGlobalBestParticlePositionStatistics()
    {
        this->computeGlobalBestParticlePositionMean();
        this->computeGlobalBestParticlePositionStdDev();
    }

    void computeCurrentBestObjFuncStatistics()
    {
        mBestObjFunValueMean = Plato::mean(*mCriteriaReductions, *mCurrentBestObjFuncValues);
        mBestObjFunValueStdDev = Plato::standard_deviation(mBestObjFunValueMean, *mCurrentBestObjFuncValues, *mCriteriaReductions);
    }

    /******************************************************************************//**
     * @brief Return a const reference to the distributed memory communication wrapper
     * @return const reference to the distributed memory communication wrapper
    **********************************************************************************/
    const Plato::CommWrapper& getCommWrapper() const
    {
        return (mCommWrapper.operator*());
    }

private:
    void initialize()
    {
        const OrdinalType tPARTICLE_INDEX = 0;
        const Plato::Vector<ScalarType, OrdinalType> & tMyParticle = (*mCurrentParticles)[tPARTICLE_INDEX];
        mWorkVector = tMyParticle.create();
        mLowerBounds = tMyParticle.create();
        mUpperBounds = tMyParticle.create();
        mMeanParticlePosition = tMyParticle.create();
        mStdDevParticlePosition = tMyParticle.create();

        mCurrentObjFuncValues->fill(std::numeric_limits<ScalarType>::max());
        mCurrentBestObjFuncValues->fill(std::numeric_limits<ScalarType>::max());

        mGlobalBestParticlePosition = tMyParticle.create();
        mGlobalBestParticlePosition->fill(std::numeric_limits<ScalarType>::max());
        Plato::fill(std::numeric_limits<ScalarType>::max(), *mBestParticlePositions);

        const OrdinalType tLength = 1;
        Plato::StandardVector<ScalarType, OrdinalType> tWork(tLength, this->getNumParticles());
        mGlobalNumParticles = mCriteriaReductions->sum(tWork);
    }

    void computeGlobalBestParticlePositionMean()
    {
        /* local sum */
        mWorkVector->fill(static_cast<ScalarType>(0));
        const OrdinalType tLocalNumParticles = this->getNumParticles();
        for(OrdinalType tParticleIndex = 0; tParticleIndex < tLocalNumParticles; tParticleIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyParticle = (*mBestParticlePositions)[tParticleIndex];
            const OrdinalType tMyParticleDims = tMyParticle.size();
            for(OrdinalType tDim = 0; tDim < tMyParticleDims; tDim++)
            {
                (*mWorkVector)[tDim] += tMyParticle[tDim];
            }
        }

        /* global sum */
        const OrdinalType tLength = 1;
        const OrdinalType tVECTOR_INDEX = 0;
        Plato::StandardVector<ScalarType, OrdinalType> tWork(tLength);
        const OrdinalType tNumDims = mWorkVector->size();
        for(OrdinalType tDim = 0; tDim < tNumDims; tDim++)
        {
            tWork[tVECTOR_INDEX] = (*mWorkVector)[tDim];
            (*mMeanParticlePosition)[tDim] = mCriteriaReductions->sum(tWork);
        }
        const ScalarType tMultiplier = static_cast<ScalarType>(1) / mGlobalNumParticles;
        mMeanParticlePosition->scale(tMultiplier);
    }

    void computeGlobalBestParticlePositionStdDev()
    {
        /* local operations */
        mWorkVector->fill(static_cast<ScalarType>(0));
        const OrdinalType tLocalNumParticles = this->getNumParticles();
        const OrdinalType tNumControls = mStdDevParticlePosition->size();
        for(OrdinalType tParticleIndex = 0; tParticleIndex < tLocalNumParticles; tParticleIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyBestParticlePosition = (*mBestParticlePositions)[tParticleIndex];
            for(OrdinalType tDim = 0; tDim < tNumControls; tDim++)
            {
                const ScalarType tMisfit = tMyBestParticlePosition[tDim] - (*mMeanParticlePosition)[tDim];
                (*mWorkVector)[tDim] += tMisfit * tMisfit;
            }
        }

        /* global operations */
        const OrdinalType tLength = 1;
        const OrdinalType tVECTOR_INDEX = 0;
        Plato::StandardVector<ScalarType, OrdinalType> tWork(tLength);
        for(OrdinalType tDim = 0; tDim < tNumControls; tDim++)
        {
            tWork[tVECTOR_INDEX] = (*mWorkVector)[tDim];
            const ScalarType tGlobalBestParticlePositionMinusMean = mCriteriaReductions->sum(tWork);
            const ScalarType tValue = tGlobalBestParticlePositionMinusMean / static_cast<ScalarType>(mGlobalNumParticles - 1);
            (*mStdDevParticlePosition)[tDim] = std::pow(tValue, static_cast<ScalarType>(0.5));
        }
    }

private:
    OrdinalType mGlobalNumParticles;
    OrdinalType mCurrentGlobalBestParticleRank;
    OrdinalType mCurrentGlobalBestParticleIndex;

    ScalarType mTimeStep;
    ScalarType mBestObjFunValueMean;
    ScalarType mBestObjFunValueStdDev;
    ScalarType mCurrentGlobalBestObjFuncValue;
    ScalarType mPreviousGlobalBestObjFuncValue;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentObjFuncValues;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentBestObjFuncValues;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mWorkVector;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mLowerBounds;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mUpperBounds;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mMeanParticlePosition;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mStdDevParticlePosition;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mGlobalBestParticlePosition;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentParticles;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentVelocities;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousVelocities;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mBestParticlePositions;

    std::shared_ptr<Plato::CommWrapper> mCommWrapper;
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mCriteriaReductions;

private:
    ParticleSwarmDataMng(const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType>&);
    Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & operator=(const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType>&);
};

template<typename ScalarType, typename OrdinalType = size_t>
class ParticleSwarmStageMng
{
public:
    virtual ~ParticleSwarmStageMng()
    {
    }

    virtual void evaluateObjective(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng) = 0;
    virtual void findBestParticlePositions(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng) = 0;
};
// class ParticleSwarmStageMng

template<typename ScalarType, typename OrdinalType = size_t>
class BoundConstrainedStageMngPSO : public Plato::ParticleSwarmStageMng<ScalarType, OrdinalType>
{
public:
    explicit BoundConstrainedStageMngPSO(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory,
                                         const std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> & aObjective) :
            mCurrentObjFuncValues(aFactory->objective().create()),
            mObjective(aObjective)
    {
    }

    virtual ~BoundConstrainedStageMngPSO()
    {
    }

    void evaluateObjective(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> & tParticles = aDataMng.getCurrentParticles();
        mObjective->value(tParticles, *mCurrentObjFuncValues);
        aDataMng.setCurrentObjFuncValues(*mCurrentObjFuncValues);
    }

    void findBestParticlePositions(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        aDataMng.cacheGlobalBestObjFunValue();
        aDataMng.updateBestParticlesData();
        aDataMng.findGlobalBestParticle();
    }

private:
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentObjFuncValues;
    std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> mObjective;

private:
    BoundConstrainedStageMngPSO(const Plato::BoundConstrainedStageMngPSO<ScalarType, OrdinalType>&);
    Plato::BoundConstrainedStageMngPSO<ScalarType, OrdinalType> & operator=(const Plato::BoundConstrainedStageMngPSO<ScalarType, OrdinalType>&);
};
// class BoundConstrainedStageMngPSO

template<typename ScalarType, typename OrdinalType = size_t>
class AugmentedLagrangianStageMngPSO : public Plato::ParticleSwarmStageMng<ScalarType, OrdinalType>
{
public:
    explicit AugmentedLagrangianStageMngPSO(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory,
                                            const std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> & aObjective,
                                            const std::shared_ptr<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>> & aConstraints) :
            mNumAugLagFuncEval(0),
            mMeanCurrentBestObjFuncValue(0),
            mStdDevCurrentBestObjFuncValue(0),
            mCurrentGlobalBestObjFuncValue(std::numeric_limits<ScalarType>::max()),
            mCurrentGlobalBestAugLagFuncValue(std::numeric_limits<ScalarType>::max()),
            mPenaltyExpansionMultiplier(2),
            mPenaltyContractionMultiplier(0.5),
            mFeasibilityInexactnessTolerance(1e-4),
            mCriteriaWorkVec(aFactory->objective().create()),
            mCurrentObjFuncValues(aFactory->objective().create()),
            mCurrentBestObjFuncValues(aFactory->objective().create()),
            mPreviousBestObjFuncValues(aFactory->objective().create()),
            mCurrentAugLagFuncValues(aFactory->objective().create()),
            mMeanBestConstraintValues(),
            mStdDevBestConstraintValues(),
            mCurrentGlobalBestConstraintValues(),
            mCurrentConstraintValues(aFactory->dual().create()),
            mCurrentPenaltyMultipliers(aFactory->dual().create()),
            mCurrentLagrangeMultipliers(aFactory->dual().create()),
            mPreviousBestConstraintValues(aFactory->dual().create()),
            mCurrentBestConstraintValues(aFactory->dual().create()),
            mObjective(aObjective),
            mConstraints(aConstraints),
            mCriteriaReductions(aFactory->getObjFuncReductionOperations().create())
    {
        this->initialize();
    }

    virtual ~AugmentedLagrangianStageMngPSO()
    {
    }

    void setPenaltyExpansionMultiplier(const ScalarType & aInput)
    {
        mPenaltyExpansionMultiplier = aInput;
    }

    void setPenaltyContractionMultiplier(const ScalarType & aInput)
    {
        mPenaltyContractionMultiplier = aInput;
    }

    void setFeasibilityInexactnessTolerance(const ScalarType & aInput)
    {
        mFeasibilityInexactnessTolerance = aInput;
    }

    OrdinalType getNumConstraints() const
    {
        return (mConstraints->size());
    }

    OrdinalType getNumAugLagFuncEvaluations() const
    {
        return (mNumAugLagFuncEval);
    }

    ScalarType getMeanCurrentBestObjFuncValues() const
    {
        return (mMeanCurrentBestObjFuncValue);
    }

    ScalarType getStdDevCurrentBestObjFuncValues() const
    {
        return (mStdDevCurrentBestObjFuncValue);
    }

    ScalarType getCurrentGlobalBestObjFuncValue() const
    {
        return (mCurrentGlobalBestObjFuncValue);
    }

    ScalarType getMeanCurrentPenaltyMultipliers(const OrdinalType & aInput) const
    {
        assert(aInput < mMeanCurrentPenaltyMultipliers->size());
        return ((*mMeanCurrentPenaltyMultipliers)[aInput]);
    }

    ScalarType getStdDevCurrentPenaltyMultipliers(const OrdinalType & aInput) const
    {
        assert(aInput < mStdDevCurrentPenaltyMultipliers->size());
        return ((*mStdDevCurrentPenaltyMultipliers)[aInput]);
    }

    ScalarType getMeanCurrentLagrangeMultipliers(const OrdinalType & aInput) const
    {
        assert(aInput < mMeanCurrentLagrangeMultipliers->size());
        return ((*mMeanCurrentLagrangeMultipliers)[aInput]);
    }

    ScalarType getStdDevCurrentLagrangeMultipliers(const OrdinalType & aInput) const
    {
        assert(aInput < mStdDevCurrentLagrangeMultipliers->size());
        return ((*mStdDevCurrentLagrangeMultipliers)[aInput]);
    }

    ScalarType getMeanCurrentBestConstraintValues(const OrdinalType & aInput) const
    {
        assert(aInput < mMeanBestConstraintValues->size());
        return ((*mMeanBestConstraintValues)[aInput]);
    }

    ScalarType getStdDevCurrentBestConstraintValues(const OrdinalType & aInput) const
    {
        assert(aInput < mStdDevBestConstraintValues->size());
        return ((*mStdDevBestConstraintValues)[aInput]);
    }

    ScalarType getCurrentGlobalBestConstraintValue(const OrdinalType & aInput) const
    {
        assert(aInput < mCurrentGlobalBestConstraintValues->size());
        return ((*mCurrentGlobalBestConstraintValues)[aInput]);
    }

    ScalarType getCurrentObjFuncValue(const OrdinalType & aIndex) const
    {
        assert(mCurrentObjFuncValues.get() != nullptr);
        return((*mCurrentObjFuncValues)[aIndex]);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getCurrentObjFuncValues() const
    {
        assert(mCurrentObjFuncValues.get() != nullptr);
        return(*mCurrentObjFuncValues);
    }

    ScalarType getPreviousBestObjFuncValue(const OrdinalType & aIndex) const
    {
        assert(mPreviousBestObjFuncValues.get() != nullptr);
        return((*mPreviousBestObjFuncValues)[aIndex]);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getPreviousBestObjFuncValues() const
    {
        assert(mPreviousBestObjFuncValues.get() != nullptr);
        return(*mPreviousBestObjFuncValues);
    }

    ScalarType getCurrentBestObjFuncValue(const OrdinalType & aIndex) const
    {
        assert(mCurrentBestObjFuncValues.get() != nullptr);
        return((*mCurrentBestObjFuncValues)[aIndex]);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getCurrentBestObjFuncValues() const
    {
        assert(mCurrentBestObjFuncValues.get() != nullptr);
        return(*mCurrentBestObjFuncValues);
    }

    ScalarType getCurrentConstraintValue(const OrdinalType & aConstraintIndex, const OrdinalType & aParticleIndex) const
    {
        assert(mCurrentConstraintValues.get() != nullptr);
        assert(aConstraintIndex < mCurrentConstraintValues->getNumVectors());
        return((*mCurrentConstraintValues)(aConstraintIndex, aParticleIndex));
    }

    const Plato::Vector<ScalarType, OrdinalType> & getCurrentConstraintValues(const OrdinalType & aIndex) const
    {
        assert(mCurrentConstraintValues.get() != nullptr);
        assert(aIndex < mCurrentConstraintValues->getNumVectors());
        return((*mCurrentConstraintValues)[aIndex]);
    }

    ScalarType getCurrentBestConstraintValue(const OrdinalType & aConstraintIndex, const OrdinalType & aParticleIndex) const
    {
        assert(mCurrentBestConstraintValues.get() != nullptr);
        assert(aConstraintIndex < mCurrentBestConstraintValues->getNumVectors());
        return((*mCurrentBestConstraintValues)(aConstraintIndex, aParticleIndex));
    }

    const Plato::Vector<ScalarType, OrdinalType> & getCurrentBestConstraintValues(const OrdinalType & aIndex) const
    {
        assert(mCurrentBestConstraintValues.get() != nullptr);
        assert(aIndex < mCurrentBestConstraintValues->getNumVectors());
        return((*mCurrentBestConstraintValues)[aIndex]);
    }

    ScalarType getPreviousBestConstraintValue(const OrdinalType & aConstraintIndex, const OrdinalType & aParticleIndex) const
    {
        assert(mPreviousBestConstraintValues.get() != nullptr);
        assert(aConstraintIndex < mPreviousBestConstraintValues->getNumVectors());
        return((*mPreviousBestConstraintValues)(aConstraintIndex, aParticleIndex));
    }

    const Plato::Vector<ScalarType, OrdinalType> & getPreviousBestConstraintValues(const OrdinalType & aIndex) const
    {
        assert(mPreviousBestConstraintValues.get() != nullptr);
        assert(aIndex < mPreviousBestConstraintValues->getNumVectors());
        return((*mPreviousBestConstraintValues)[aIndex]);
    }

    ScalarType getPenaltyMultiplier(const OrdinalType & aConstraintIndex, const OrdinalType & aParticleIndex) const
    {
        assert(mCurrentPenaltyMultipliers.get() != nullptr);
        assert(aConstraintIndex < mCurrentPenaltyMultipliers->getNumVectors());
        return((*mCurrentPenaltyMultipliers)(aConstraintIndex, aParticleIndex));
    }

    const Plato::Vector<ScalarType, OrdinalType> & getPenaltyMultipliers(const OrdinalType & aIndex) const
    {
        assert(mCurrentPenaltyMultipliers.get() != nullptr);
        assert(aIndex < mCurrentPenaltyMultipliers->getNumVectors());
        return((*mCurrentPenaltyMultipliers)[aIndex]);
    }

    ScalarType getLagrangeMultiplier(const OrdinalType & aConstraintIndex, const OrdinalType & aParticleIndex) const
    {
        assert(mCurrentLagrangeMultipliers.get() != nullptr);
        assert(aConstraintIndex < mCurrentLagrangeMultipliers->getNumVectors());
        return((*mCurrentLagrangeMultipliers)(aConstraintIndex, aParticleIndex));
    }

    const Plato::Vector<ScalarType, OrdinalType> & getLagrangeMultipliers(const OrdinalType & aIndex) const
    {
        assert(mCurrentLagrangeMultipliers.get() != nullptr);
        assert(aIndex < mCurrentLagrangeMultipliers->getNumVectors());
        return((*mCurrentLagrangeMultipliers)[aIndex]);
    }

    void findBestParticlePositions(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        aDataMng.cacheGlobalBestObjFunValue();
        this->updateBestParticlesData(aDataMng);
        aDataMng.findGlobalBestParticle();
    }

    void evaluateObjective(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        this->cacheCriteriaValues();
        // Evaluate objective function
        mNumAugLagFuncEval += aDataMng.getNumParticles();
        mCurrentObjFuncValues->fill(static_cast<ScalarType>(0));
        const Plato::MultiVector<ScalarType, OrdinalType> & tParticles = aDataMng.getCurrentParticles();
        mObjective->value(tParticles, *mCurrentObjFuncValues);
        // Evaluate constraints
        this->evaluateConstraints(tParticles);
        // Evaluate augmented Lagrangian
        mCurrentAugLagFuncValues->update(static_cast<ScalarType>(1), *mCurrentObjFuncValues, static_cast<ScalarType>(0));
        this->evaluateAugmentedLagrangian();
        aDataMng.setCurrentObjFuncValues(*mCurrentAugLagFuncValues);
    }

    void updateLagrangeMultipliers()
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tPenaltyValues = (*mCurrentPenaltyMultipliers)[tConstraintIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tBestConstraintValues = (*mCurrentBestConstraintValues)[tConstraintIndex];
            Plato::Vector<ScalarType, OrdinalType> & tLagrangeMultipliers = (*mCurrentLagrangeMultipliers)[tConstraintIndex];

            const OrdinalType tNumParticles = tBestConstraintValues.size();
            for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
            {
                const ScalarType tLagMultiplierOverPenalty = -tLagrangeMultipliers[tParticleIndex]
                        / (static_cast<ScalarType>(2) * tPenaltyValues[tParticleIndex]);
                const ScalarType tSuggestedConstraintValue = std::max(tBestConstraintValues[tParticleIndex], tLagMultiplierOverPenalty);
                ScalarType tNewLagMultiplier = tLagrangeMultipliers[tParticleIndex] +
                        (static_cast<ScalarType>(2) * tPenaltyValues[tParticleIndex] * tSuggestedConstraintValue);
                tNewLagMultiplier = std::max(tNewLagMultiplier, static_cast<ScalarType>(0));
                tLagrangeMultipliers[tParticleIndex] = tNewLagMultiplier;
            }
        }
    }

    void updatePenaltyMultipliers()
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tCurrentBestConstraint = (*mCurrentBestConstraintValues)[tConstraintIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tPreviousBestConstraint = (*mPreviousBestConstraintValues)[tConstraintIndex];
            Plato::Vector<ScalarType, OrdinalType> & tPenaltyMultipliers = (*mCurrentPenaltyMultipliers)[tConstraintIndex];
            Plato::Vector<ScalarType, OrdinalType> & tLagrangeMultipliers = (*mCurrentLagrangeMultipliers)[tConstraintIndex];

            const OrdinalType tNumParticles = tPenaltyMultipliers.size();
            for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
            {
                const bool tIsConstraintGreaterThanPrevious = tCurrentBestConstraint[tParticleIndex] > tPreviousBestConstraint[tParticleIndex];
                const bool tIsConstraintGreaterThanFeasibilityTol = tCurrentBestConstraint[tParticleIndex] > mFeasibilityInexactnessTolerance;
                tPenaltyMultipliers[tParticleIndex] = tIsConstraintGreaterThanPrevious || tIsConstraintGreaterThanFeasibilityTol ?
                        mPenaltyExpansionMultiplier * tPenaltyMultipliers[tParticleIndex] : tPenaltyMultipliers[tParticleIndex];

                const bool tIsConstraintLessThanFeasibilityTol = tCurrentBestConstraint[tParticleIndex] <= mFeasibilityInexactnessTolerance;
                tPenaltyMultipliers[tParticleIndex] = tIsConstraintLessThanFeasibilityTol ?
                        mPenaltyContractionMultiplier * tPenaltyMultipliers[tParticleIndex] : tPenaltyMultipliers[tParticleIndex];

                const ScalarType tLagrangeMultiplierOverFeasibilityTolerance =
                        tLagrangeMultipliers[tParticleIndex] / mFeasibilityInexactnessTolerance;
                const ScalarType tLowerBound = static_cast<ScalarType>(0.5) *
                        std::pow(tLagrangeMultiplierOverFeasibilityTolerance, static_cast<ScalarType>(0.5));
                tPenaltyMultipliers[tParticleIndex] = std::max(tPenaltyMultipliers[tParticleIndex], tLowerBound);
            }
        }
    }

    void computeCriteriaStatistics()
    {
        this->computeObjFuncStatistics();
        this->computeDualDataStatistics();
        this->computeConstraintStatistics();
    }

private:
    void initialize()
    {
        Plato::fill(static_cast<ScalarType>(1), *mCurrentPenaltyMultipliers);
        Plato::fill(static_cast<ScalarType>(0), *mCurrentLagrangeMultipliers);

        mCurrentObjFuncValues->fill(std::numeric_limits<ScalarType>::max());
        mCurrentBestObjFuncValues->fill(std::numeric_limits<ScalarType>::max());
        mCurrentAugLagFuncValues->fill(std::numeric_limits<ScalarType>::max());

        const OrdinalType tNumConstraints = mConstraints->size();
        mMeanBestConstraintValues = std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(tNumConstraints);
        mStdDevBestConstraintValues = std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(tNumConstraints);
        mMeanCurrentPenaltyMultipliers = std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(tNumConstraints);
        mStdDevCurrentPenaltyMultipliers = std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(tNumConstraints);
        mMeanCurrentLagrangeMultipliers = std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(tNumConstraints);
        mStdDevCurrentLagrangeMultipliers = std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(tNumConstraints);
        mCurrentGlobalBestConstraintValues = std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(tNumConstraints);

        mCurrentGlobalBestConstraintValues->fill(std::numeric_limits<ScalarType>::max());
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            (*mCurrentConstraintValues)[tIndex].fill(std::numeric_limits<ScalarType>::max());
            (*mCurrentBestConstraintValues)[tIndex].fill(std::numeric_limits<ScalarType>::max());
        }
    }

    void cacheCriteriaValues()
    {
        mPreviousBestObjFuncValues->update(static_cast<ScalarType>(1), *mCurrentBestObjFuncValues, static_cast<ScalarType>(0));
        Plato::update(static_cast<ScalarType>(1), *mCurrentBestConstraintValues, static_cast<ScalarType>(0), *mPreviousBestConstraintValues);
    }

    void updateBestParticlesData(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const OrdinalType tNumParticles = aDataMng.getNumParticles();
        const Plato::Vector<ScalarType, OrdinalType> & tCurrentBestAugLagFuncValues = aDataMng.getCurrentBestObjFuncValues();
        for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
        {
            if((*mCurrentAugLagFuncValues)[tParticleIndex] < tCurrentBestAugLagFuncValues[tParticleIndex])
            {
                const Plato::Vector<ScalarType, OrdinalType> & tCurrentParticle = aDataMng.getCurrentParticle(tParticleIndex);
                aDataMng.setBestParticlePosition(tParticleIndex, tCurrentParticle);
                this->updateBestCriteriaValues(tParticleIndex, aDataMng);
            }
        }
    }

    void updateBestCriteriaValues(const OrdinalType & aParticleIndex,
                                  Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        aDataMng.setCurrentBestObjFuncValue(aParticleIndex, (*mCurrentAugLagFuncValues)[aParticleIndex]);
        (*mCurrentBestObjFuncValues)[aParticleIndex] = (*mCurrentObjFuncValues)[aParticleIndex];
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            (*mCurrentBestConstraintValues)(tConstraintIndex, aParticleIndex) =
                    (*mCurrentConstraintValues)(tConstraintIndex, aParticleIndex);
        }
    }

    void computeObjFuncStatistics()
    {
        mMeanCurrentBestObjFuncValue = Plato::mean(*mCriteriaReductions, *mCurrentBestObjFuncValues);
        mStdDevCurrentBestObjFuncValue =
                Plato::standard_deviation(mMeanCurrentBestObjFuncValue, *mCurrentBestObjFuncValues, *mCriteriaReductions);
        const ScalarType tValue = mCriteriaReductions->min(*mCurrentBestObjFuncValues);
        const bool tFoundNewGlobalBest = tValue < mCurrentGlobalBestObjFuncValue;
        mCurrentGlobalBestObjFuncValue = tFoundNewGlobalBest == true ? tValue : mCurrentGlobalBestObjFuncValue;
    }

    void computeDualDataStatistics()
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyPenaltyMultipliers = (*mCurrentPenaltyMultipliers)[tIndex];
            ScalarType tMean = Plato::mean(*mCriteriaReductions, tMyPenaltyMultipliers);
            (*mMeanCurrentPenaltyMultipliers)[tIndex] = tMean;
            (*mStdDevCurrentPenaltyMultipliers)[tIndex] =
                    Plato::standard_deviation(tMean, tMyPenaltyMultipliers, *mCriteriaReductions);

            const Plato::Vector<ScalarType, OrdinalType> & tMyLagrangeMultipliers = (*mCurrentPenaltyMultipliers)[tIndex];
            tMean = Plato::mean(*mCriteriaReductions, tMyLagrangeMultipliers);
            (*mMeanCurrentLagrangeMultipliers)[tIndex] = tMean;
            (*mStdDevCurrentLagrangeMultipliers)[tIndex] =
                    Plato::standard_deviation(tMean, tMyLagrangeMultipliers, *mCriteriaReductions);
        }
    }

    void computeConstraintStatistics()
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            Plato::Vector<ScalarType, OrdinalType> & tMyBestConstraintValues = (*mCurrentBestConstraintValues)[tIndex];
            const ScalarType tMean = Plato::mean(*mCriteriaReductions, tMyBestConstraintValues);
            (*mMeanBestConstraintValues)[tIndex] = tMean;
            (*mStdDevBestConstraintValues)[tIndex] =
                    Plato::standard_deviation(tMean, tMyBestConstraintValues, *mCriteriaReductions);

            mCriteriaWorkVec->update(static_cast<ScalarType>(1), tMyBestConstraintValues, static_cast<ScalarType>(0));
            mCriteriaWorkVec->modulus();
            const ScalarType tValue = mCriteriaReductions->min(*mCriteriaWorkVec);
            const bool tFoundNewGlobalBest = tValue < (*mCurrentGlobalBestConstraintValues)[tIndex];
            (*mCurrentGlobalBestConstraintValues)[tIndex] =
                    tFoundNewGlobalBest == true ? tValue : (*mCurrentGlobalBestConstraintValues)[tIndex];
        }
    }

    /******************************************************************************//**
     * @brief Evaluate inequality constraints
     * @param [in] aControl control multi-vector
    **********************************************************************************/
    void evaluateConstraints(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            Plato::GradFreeCriteria<ScalarType, OrdinalType> & tConstraint = (*mConstraints)[tIndex];
            Plato::Vector<ScalarType, OrdinalType> & tMyValues = (*mCurrentConstraintValues)[tIndex];
            tMyValues.fill(static_cast<ScalarType>(0));
            tConstraint.value(aControl, tMyValues);
        }
    }

    /******************************************************************************//**
     * @brief Evaluate augmented Lagrangian criterion:
     * \f$ \ell(z, \lambda, \mu) = f(z) + sum_{i=1}^{N}\lambda_i \theta_i(z) + sum_{i=1}^{N}\mu_i \theta_i^2(z) \f$,
     * where \f$ \theta_i = \max( h_i(x), \frac{-\lambda_i}{2\mu_i} ) \f$
     * @param [in,out] aOutput augmented Lagrangian values
    **********************************************************************************/
    void evaluateAugmentedLagrangian()
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tConstraintValues = (*mCurrentConstraintValues)[tConstraintIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tPenaltyMultipliers = (*mCurrentPenaltyMultipliers)[tConstraintIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tLagrangeMultipliers = (*mCurrentLagrangeMultipliers)[tConstraintIndex];

            const OrdinalType tNumParticles = mCurrentObjFuncValues->size();
            for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
            {
                const ScalarType tLagMultiplierOverPenalty = -tLagrangeMultipliers[tParticleIndex]
                        / (static_cast<ScalarType>(2) * tPenaltyMultipliers[tParticleIndex]);
                ScalarType tSuggestedConstraintValue = std::max(tConstraintValues[tParticleIndex], tLagMultiplierOverPenalty);
                const bool tIsConstraintFeasible = tSuggestedConstraintValue < static_cast<ScalarType>(0);
                tSuggestedConstraintValue = tIsConstraintFeasible == true ? static_cast<ScalarType>(0) : tSuggestedConstraintValue;
                const ScalarType tLagrangeMultipliersTimesConstraint = tLagrangeMultipliers[tParticleIndex] * tSuggestedConstraintValue;
                const ScalarType tConstraintTimesConstraint = tSuggestedConstraintValue * tSuggestedConstraintValue;
                const ScalarType tMyConstraintContribution = tLagrangeMultipliersTimesConstraint
                        + tPenaltyMultipliers[tParticleIndex] * tConstraintTimesConstraint;
                (*mCurrentAugLagFuncValues)[tParticleIndex] += tMyConstraintContribution;
            }
        }
    }

private:
    OrdinalType mNumAugLagFuncEval;

    ScalarType mMeanCurrentBestObjFuncValue;
    ScalarType mStdDevCurrentBestObjFuncValue;
    ScalarType mCurrentGlobalBestObjFuncValue;
    ScalarType mCurrentGlobalBestAugLagFuncValue;

    ScalarType mPenaltyExpansionMultiplier;
    ScalarType mPenaltyContractionMultiplier;
    ScalarType mFeasibilityInexactnessTolerance;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCriteriaWorkVec;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentObjFuncValues;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentBestObjFuncValues;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mPreviousBestObjFuncValues;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentAugLagFuncValues;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mMeanBestConstraintValues;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mStdDevBestConstraintValues;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mMeanCurrentPenaltyMultipliers;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mStdDevCurrentPenaltyMultipliers;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mMeanCurrentLagrangeMultipliers;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mStdDevCurrentLagrangeMultipliers;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentGlobalBestConstraintValues;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentConstraintValues;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentPenaltyMultipliers;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentLagrangeMultipliers;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentBestConstraintValues;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousBestConstraintValues;

    std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> mObjective;
    std::shared_ptr<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>> mConstraints;
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mCriteriaReductions;

private:
    AugmentedLagrangianStageMngPSO(const Plato::AugmentedLagrangianStageMngPSO<ScalarType, OrdinalType>&);
    Plato::AugmentedLagrangianStageMngPSO<ScalarType, OrdinalType> & operator=(const Plato::AugmentedLagrangianStageMngPSO<ScalarType, OrdinalType>&);
};
// class AugmentedLagrangianStageMngPSO

template<typename ScalarType, typename OrdinalType = size_t>
class ParticleSwarmOperations
{
public:
    explicit ParticleSwarmOperations(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory) :
            mNumConsecutiveFailures(0),
            mNumConsecutiveSuccesses(0),
            mMaxNumConsecutiveFailures(10),
            mMaxNumConsecutiveSuccesses(10),
            mInertiaMultiplier(0.9),
            mSocialBehaviorMultiplier(0.8),
            mCognitiveBehaviorMultiplier(0.8),
            mTrustRegionMultiplier(1),
            mTrustRegionExpansionMultiplier(4.0),
            mTrustRegionContractionMultiplier(0.75),
            mControlWorkVector()
    {
        this->initialize(*aFactory);
    }

    virtual ~ParticleSwarmOperations()
    {
    }

    ScalarType getInertiaMultiplier() const
    {
        return (mInertiaMultiplier);
    }

    ScalarType getTrustRegionMultiplier() const
    {
        return (mTrustRegionMultiplier);
    }

    void setInertiaMultiplier(const ScalarType & aInput)
    {
        mInertiaMultiplier = aInput;
    }

    void setCognitiveMultiplier(const ScalarType & aInput)
    {
        mCognitiveBehaviorMultiplier = aInput;
    }

    void setSocialBehaviorMultiplier(const ScalarType & aInput)
    {
        mSocialBehaviorMultiplier = aInput;
    }

    void setTrustRegionMultiplier(const ScalarType & aInput)
    {
        mTrustRegionMultiplier = aInput;
    }

    void setTrustRegionExpansionMultiplier(const ScalarType & aInput)
    {
        mTrustRegionExpansionMultiplier = aInput;
    }

    void setTrustRegionContractionMultiplier(const ScalarType & aInput)
    {
        mTrustRegionContractionMultiplier = aInput;
    }

    void setNumConsecutiveFailures(const OrdinalType & aInput)
    {
        mNumConsecutiveFailures = aInput;
    }

    void setNumConsecutiveSuccesses(const OrdinalType & aInput)
    {
        mNumConsecutiveSuccesses = aInput;
    }

    void setMaxNumConsecutiveFailures(const OrdinalType & aInput)
    {
        mMaxNumConsecutiveFailures = aInput;
    }

    void setMaxNumConsecutiveSuccesses(const OrdinalType & aInput)
    {
        mMaxNumConsecutiveSuccesses = aInput;
    }

    bool checkInertiaMultiplier()
    {
        const ScalarType tUpperBound = 1.0;
        const ScalarType tLowerBound = (static_cast<ScalarType>(0.5)
                * (mCognitiveBehaviorMultiplier + mSocialBehaviorMultiplier)) - static_cast<ScalarType>(1.0);
        const bool tMultiplierOutsideBounds = (mInertiaMultiplier < tLowerBound) || (mInertiaMultiplier > tUpperBound);
        if(tMultiplierOutsideBounds == true)
        {
            // use default values
            mInertiaMultiplier = 0.9;
            mSocialBehaviorMultiplier = 0.8;
            mCognitiveBehaviorMultiplier = 0.8;
        }
        return(tMultiplierOutsideBounds);
    }

    void updateTrustRegionMultiplier()
    {
        if(mNumConsecutiveSuccesses >= mMaxNumConsecutiveSuccesses)
        {
            mTrustRegionMultiplier *= mTrustRegionExpansionMultiplier;
        }
        else if(mNumConsecutiveFailures >= mMaxNumConsecutiveFailures)
        {
            mTrustRegionMultiplier *= mTrustRegionContractionMultiplier;
        }
    }

    void updateParticleVelocities(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        aDataMng.cachePreviousVelocities();
        std::uniform_real_distribution<ScalarType> tDistribution(0.0 /* lower bound */, 1.0 /* upper bound */);
        const OrdinalType tNumParticles = aDataMng.getNumParticles();
        const OrdinalType tCurrentGlobalBestParticleIndex = aDataMng.getCurrentGlobalBestParticleIndex();

        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            if(tIndex != tCurrentGlobalBestParticleIndex)
            {
                this->updateParticleVelocity(tIndex, tDistribution, aDataMng);
            }
                else
                {
                    assert(tIndex == tCurrentGlobalBestParticleIndex);
                    this->updateGlobalBestParticleVelocity(tDistribution, aDataMng);
            }
        }
    }

    void updateParticlePositions(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const OrdinalType tNumParticles = aDataMng.getNumParticles();
        const OrdinalType tCurrentGlobalBestParticleIndex = aDataMng.getCurrentGlobalBestParticleIndex();

        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            if(tIndex != tCurrentGlobalBestParticleIndex)
            {
                this->updateParticlePosition(tIndex, aDataMng);
            }
            else
            {
                assert(tIndex == tCurrentGlobalBestParticleIndex);
                this->updateGlobalBestParticlePosition(aDataMng);
            }
        }
    }

    void checkGlobalBestParticleUpdateSuccessRate(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const ScalarType tCurrentGlobalBestObjFunValue = aDataMng.getCurrentGlobalBestObjFuncValue();
        const ScalarType tPreviousGlobalBestObjFunValue = aDataMng.getPreviousGlobalBestObjFuncValue();

        if(tCurrentGlobalBestObjFunValue < tPreviousGlobalBestObjFunValue)
        {
            mNumConsecutiveSuccesses++;
            mNumConsecutiveFailures = 0;
        }
        else
        {
            mNumConsecutiveFailures++;
            mNumConsecutiveSuccesses = 0;
        }
    }

private:
    void initialize(const Plato::DataFactory<ScalarType, OrdinalType> & aFactory)
    {
        const OrdinalType tPARTICLE_INDEX = 0;
        mControlWorkVector = aFactory.control(tPARTICLE_INDEX).create();
    }

    void updateParticleVelocity(const OrdinalType & aParticleIndex,
                                std::uniform_real_distribution<ScalarType> & aDistribution,
                                Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const Plato::Vector<ScalarType, OrdinalType> & tPreviousVel = aDataMng.getPreviousVelocity(aParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tCurrentParticle = aDataMng.getCurrentParticle(aParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tGlobalBestParticlePosition = aDataMng.getGlobalBestParticlePosition();
        const Plato::Vector<ScalarType, OrdinalType> & tBestParticlePosition = aDataMng.getBestParticlePosition(aParticleIndex);

        const OrdinalType tNumControls = tPreviousVel.size();
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            // inertia contribution
            const ScalarType tInertiaValue = mInertiaMultiplier * tPreviousVel[tIndex];
            // cognitive behavior contribution
            const ScalarType tRandomNumOne = aDistribution(mGenerator);
            const ScalarType tCognitiveMultiplier = tRandomNumOne * mCognitiveBehaviorMultiplier;
            const ScalarType tCognitiveValue = tCognitiveMultiplier * (tBestParticlePosition[tIndex] - tCurrentParticle[tIndex]);
            // social behavior contribution
            const ScalarType tRandomNumTwo = aDistribution(mGenerator);
            const ScalarType tSocialMultiplier = tRandomNumTwo * mSocialBehaviorMultiplier;
            const ScalarType tSocialValue = tSocialMultiplier * (tGlobalBestParticlePosition[tIndex] - tCurrentParticle[tIndex]);
            // set new velocity
            (*mControlWorkVector)[tIndex] = tInertiaValue + tCognitiveValue + tSocialValue;
        }

        aDataMng.setCurrentVelocity(aParticleIndex, *mControlWorkVector);
    }

    void updateGlobalBestParticleVelocity(std::uniform_real_distribution<ScalarType> & aDistribution,
                                          Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        // TODO: THINK PARALLEL IMPLEMENTATION
        const OrdinalType tCurrentGlobalBestParticleIndex = aDataMng.getCurrentGlobalBestParticleIndex();
        const Plato::Vector<ScalarType, OrdinalType> & tPreviousVel = aDataMng.getPreviousVelocity(tCurrentGlobalBestParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tGlobalBestParticlePosition = aDataMng.getGlobalBestParticlePosition();
        const Plato::Vector<ScalarType, OrdinalType> & tCurrentParticlePosition = aDataMng.getCurrentParticle(tCurrentGlobalBestParticleIndex);

        const OrdinalType tNumControls = tPreviousVel.size();
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            const ScalarType tRandomNum = aDistribution(mGenerator);
            const ScalarType tStochasticTrustRegionMultiplier = mTrustRegionMultiplier
                    * (static_cast<ScalarType>(1) - static_cast<ScalarType>(2) * tRandomNum);
            (*mControlWorkVector)[tIndex] = (static_cast<ScalarType>(-1) * tCurrentParticlePosition[tIndex]) + tGlobalBestParticlePosition[tIndex]
                    + (mInertiaMultiplier * tPreviousVel[tIndex]) + tStochasticTrustRegionMultiplier;
        }

        aDataMng.setCurrentVelocity(tCurrentGlobalBestParticleIndex, *mControlWorkVector);
    }

    void updateParticlePosition(const OrdinalType & aParticleIndex,
                                Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const ScalarType tTimeStep = aDataMng.getTimeStep();
        const Plato::Vector<ScalarType, OrdinalType> & tLowerBounds = aDataMng.getLowerBounds();
        const Plato::Vector<ScalarType, OrdinalType> & tUpperBounds = aDataMng.getUpperBounds();
        const Plato::Vector<ScalarType, OrdinalType> & tParticleVel = aDataMng.getCurrentVelocity(aParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tParticlePosition = aDataMng.getCurrentParticle(aParticleIndex);
        mControlWorkVector->update(static_cast<ScalarType>(1), tParticlePosition, static_cast<ScalarType>(0));

        const OrdinalType tNumControls = tParticleVel.size();
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            (*mControlWorkVector)[tIndex] = (*mControlWorkVector)[tIndex] + (tTimeStep * tParticleVel[tIndex]);
            (*mControlWorkVector)[tIndex] = std::max((*mControlWorkVector)[tIndex], tLowerBounds[tIndex]);
            (*mControlWorkVector)[tIndex] = std::min((*mControlWorkVector)[tIndex], tUpperBounds[tIndex]);
        }

        aDataMng.setCurrentParticle(aParticleIndex, *mControlWorkVector);
    }

    void updateGlobalBestParticlePosition(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        // TODO: THINK PARALLEL IMPLEMENTATION
        const OrdinalType tCurrentGlobalBestParticleIndex = aDataMng.getCurrentGlobalBestParticleIndex();
        const Plato::Vector<ScalarType, OrdinalType> & tGlobalBestParticleVel = aDataMng.getCurrentVelocity(tCurrentGlobalBestParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tGlobalBestParticlePosition = aDataMng.getGlobalBestParticlePosition();

        std::uniform_real_distribution<ScalarType> tDistribution(0.0 /* lower bound */, 1.0 /* upper bound */);
        const ScalarType tRandomNum = tDistribution(mGenerator);
        const ScalarType tStochasticTrustRegionMultiplier = mTrustRegionMultiplier
                * (static_cast<ScalarType>(1) - static_cast<ScalarType>(2) * tRandomNum);

        const OrdinalType tNumControls = tGlobalBestParticlePosition.size();
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            (*mControlWorkVector)[tIndex] = tGlobalBestParticlePosition[tIndex]
                    + (mInertiaMultiplier * tGlobalBestParticleVel[tIndex]) + tStochasticTrustRegionMultiplier;
        }

        aDataMng.setCurrentParticle(tCurrentGlobalBestParticleIndex, *mControlWorkVector);
    }

private:
    std::default_random_engine mGenerator;

    OrdinalType mNumConsecutiveFailures;
    OrdinalType mNumConsecutiveSuccesses;
    OrdinalType mMaxNumConsecutiveFailures;
    OrdinalType mMaxNumConsecutiveSuccesses;

    ScalarType mInertiaMultiplier;
    ScalarType mSocialBehaviorMultiplier;
    ScalarType mCognitiveBehaviorMultiplier;

    ScalarType mTrustRegionMultiplier;
    ScalarType mTrustRegionExpansionMultiplier;
    ScalarType mTrustRegionContractionMultiplier;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWorkVector;

private:
    ParticleSwarmOperations(const Plato::ParticleSwarmOperations<ScalarType, OrdinalType>&);
    Plato::ParticleSwarmOperations<ScalarType, OrdinalType> & operator=(const Plato::ParticleSwarmOperations<ScalarType, OrdinalType>&);
};
// class ParticleSwarmOperations

template<typename ScalarType, typename OrdinalType = size_t>
class BoundConstrainedPSO
{
public:
    explicit BoundConstrainedPSO(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory,
                                 const std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> & aObjective) :
            mPrintDiagnostics(false),
            mNumIterations(0),
            mMaxNumIterations(1000),
            mBestObjFuncTolerance(1e-10),
            mMeanObjFuncTolerance(5e-4),
            mStopCriterion(Plato::particle_swarm::DID_NOT_CONVERGE),
            mDataMng(std::make_shared<Plato::ParticleSwarmDataMng<ScalarType, OrdinalType>>(aFactory)),
            mStageMng(std::make_shared<Plato::BoundConstrainedStageMngPSO<ScalarType, OrdinalType>>(aFactory, aObjective)),
            mOperations(std::make_shared<Plato::ParticleSwarmOperations<ScalarType, OrdinalType>>(aFactory))
    {
    }

    explicit BoundConstrainedPSO(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory,
                                 const std::shared_ptr<Plato::ParticleSwarmStageMng<ScalarType, OrdinalType>> & aStageMng) :
            mPrintDiagnostics(false),
            mNumIterations(0),
            mMaxNumIterations(1000),
            mBestObjFuncTolerance(1e-10),
            mMeanObjFuncTolerance(5e-4),
            mStopCriterion(Plato::particle_swarm::DID_NOT_CONVERGE),
            mDataMng(std::make_shared<Plato::ParticleSwarmDataMng<ScalarType, OrdinalType>>(aFactory)),
            mStageMng(aStageMng),
            mOperations(std::make_shared<Plato::ParticleSwarmOperations<ScalarType, OrdinalType>>(aFactory))
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~BoundConstrainedPSO()
    {
    }

    /******************************************************************************//**
     * @brief Enable output of diagnostics (i.e. optimization problem status)
    **********************************************************************************/
    void enableDiagnostics()
    {
        mPrintDiagnostics = true;
    }

    void setMaxNumIterations(const OrdinalType & aInput)
    {
        mMaxNumIterations = aInput;
    }

    void setMaxNumConsecutiveFailures(const OrdinalType & aInput)
    {
        mOperations->setMaxNumConsecutiveFailures(aInput);
    }

    void setMaxNumConsecutiveSuccesses(const OrdinalType & aInput)
    {
        mOperations->setMaxNumConsecutiveSuccesses(aInput);
    }

    void setMeanObjFuncTolerance(const ScalarType & aInput)
    {
        mMeanObjFuncTolerance = aInput;
    }

    void setBestObjFuncTolerance(const ScalarType & aInput)
    {
        mBestObjFuncTolerance = aInput;
    }

    void setInertiaMultiplier(const ScalarType & aInput)
    {
        mOperations->setInertiaMultiplier(aInput);
    }

    void setCognitiveMultiplier(const ScalarType & aInput)
    {
        mOperations->setCognitiveMultiplier(aInput);
    }

    void setSocialBehaviorMultiplier(const ScalarType & aInput)
    {
        mOperations->setSocialBehaviorMultiplier(aInput);
    }

    void setTrustRegionMultiplier(const ScalarType & aInput)
    {
        mOperations->setTrustRegionMultiplier(aInput);
    }

    void setTrustRegionExpansionMultiplier(const ScalarType & aInput)
    {
        mOperations->setTrustRegionExpansionMultiplier(aInput);
    }

    void setTrustRegionContractionMultiplier(const ScalarType & aInput)
    {
        mOperations->setTrustRegionContractionMultiplier(aInput);
    }

    void setUpperBounds(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        mDataMng->setUpperBounds(aInput);
    }

    void setLowerBounds(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        mDataMng->setLowerBounds(aInput);
    }

    void setUpperBounds(const ScalarType & aInput)
    {
        mDataMng->setUpperBounds(aInput);
    }

    void setLowerBounds(const ScalarType & aInput)
    {
        mDataMng->setLowerBounds(aInput);
    }

    /******************************************************************************//**
     * @brief Set number of constraints. Required for ALPSO diagnostics output.
     * @param [in] aInput number of constraints
    **********************************************************************************/
    void setNumConstraints(const OrdinalType & aInput)
    {
        mOutputData.mNumConstraints = aInput;
    }

    void computeGlobalBestParticlePositionStatistics()
    {
        mDataMng->computeGlobalBestParticlePositionStatistics();
    }

    OrdinalType getNumIterations() const
    {
        return (mNumIterations);
    }

    ScalarType getMeanCurrentBestObjFuncValues() const
    {
        return (mDataMng->getMeanCurrentBestObjFuncValues());
    }

    ScalarType getStdDevCurrentBestObjFuncValues() const
    {
        return (mDataMng->getStdDevCurrentBestObjFuncValues());
    }

    ScalarType getCurrentGlobalBestObjFuncValue() const
    {
        return (mDataMng->getCurrentGlobalBestObjFuncValue());
    }

    std::string getStoppingCriterion() const
    {
        std::string tReason;
        Plato::pso::get_stop_criterion(mStopCriterion, tReason);
        return (tReason);
    }

    const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & getDataMng() const
    {
        return (*mDataMng);
    }

    /******************************************************************************//**
     * @brief Solve particle swarm bound constrained optimization problem
    **********************************************************************************/
    void solve()
    {
        assert(static_cast<OrdinalType>(mDataMng.use_count()) > static_cast<OrdinalType>(0));
        assert(static_cast<OrdinalType>(mStageMng.use_count()) > static_cast<OrdinalType>(0));

        this->openOutputFile();
        this->initialize();

        mNumIterations = 0;
        while(1)
        {
            mNumIterations++;
            mStageMng->evaluateObjective(*mDataMng);
            mStageMng->findBestParticlePositions(*mDataMng);

            mOperations->checkGlobalBestParticleUpdateSuccessRate(*mDataMng);
            mOperations->updateParticleVelocities(*mDataMng);
            mOperations->updateParticlePositions(*mDataMng);
            mOperations->updateTrustRegionMultiplier();

            mDataMng->computeCurrentBestObjFuncStatistics();
            this->outputDiagnostics();

            if(this->checkStoppingCriteria())
            {
                mDataMng->computeGlobalBestParticlePositionStatistics();
                this->outputStoppingCriterion();
                this->closeOutputFile();
                break;
            }
        }
    }

    /******************************************************************************//**
     * @brief Solve constrained optimization problem
     * @param [in,out] aOutputStream output/diagnostics file
    **********************************************************************************/
    void solve(std::ofstream & aOutputStream)
    {
        mNumIterations = 0;

        while(1)
        {
            mNumIterations++;
            mStageMng->evaluateObjective(*mDataMng);
            mStageMng->findBestParticlePositions(*mDataMng);

            mOperations->checkGlobalBestParticleUpdateSuccessRate(*mDataMng);
            mOperations->updateParticleVelocities(*mDataMng);
            mOperations->updateParticlePositions(*mDataMng);
            mOperations->updateTrustRegionMultiplier();

            mDataMng->computeCurrentBestObjFuncStatistics();
            this->outputDiagnostics(aOutputStream);

            if(this->checkStoppingCriteria())
            {
                break;
            }
        }
    }

    void initialize()
    {
        mOperations->checkInertiaMultiplier();
        mDataMng->setInitialParticles();
    }

private:
    bool checkStoppingCriteria()
    {
        bool tStop = false;
        const ScalarType tBestObjFunValueMean = mDataMng->getMeanCurrentBestObjFuncValues();
        const ScalarType tCurrentGlobalBestObjFunValue = mDataMng->getCurrentGlobalBestObjFuncValue();

        if(mNumIterations >= mMaxNumIterations)
        {
            tStop = true;
            mStopCriterion = Plato::particle_swarm::MAX_NUMBER_ITERATIONS;
        }
        else if(tCurrentGlobalBestObjFunValue < mBestObjFuncTolerance)
        {
            tStop = true;
            mStopCriterion = Plato::particle_swarm::TRUE_OBJECTIVE_TOLERANCE;
        }
        else if(tBestObjFunValueMean < mMeanObjFuncTolerance)
        {
            tStop = true;
            mStopCriterion = Plato::particle_swarm::MEAN_OBJECTIVE_TOLERANCE;
        }

        return (tStop);
    }

    /******************************************************************************//**
     * @brief Open output file (i.e. diagnostics file)
    **********************************************************************************/
    void openOutputFile()
    {
        if(mPrintDiagnostics == true)
        {
            const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
            if(tMyCommWrapper.myProcID() == 0)
            {
                mOutputStream.open("plato_pso_algorithm_diagnostics.txt");
                Plato::pso::print_pso_diagnostics_header(mOutputData, mOutputStream, mPrintDiagnostics);
            }
        }
    }

    /******************************************************************************//**
     * @brief Close output file (i.e. diagnostics file)
    **********************************************************************************/
    void closeOutputFile()
    {
        if(mPrintDiagnostics == true)
        {
            const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
            if(tMyCommWrapper.myProcID() == 0)
            {
                mOutputStream.close();
            }
        }
    }

    /******************************************************************************//**
     * @brief Print stopping criterion into diagnostics file.
    **********************************************************************************/
    void outputStoppingCriterion()
    {
        if(mPrintDiagnostics == true)
        {
            const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
            if(tMyCommWrapper.myProcID() == 0)
            {
                std::string tReason;
                Plato::pso::get_stop_criterion(mStopCriterion, tReason);
                mOutputStream << tReason.c_str();
            }
        }
    }

    /******************************************************************************//**
     * @brief Print diagnostics particle swarm bound constrained optimization algorithm.
    **********************************************************************************/
    void outputDiagnostics()
    {
        if(mPrintDiagnostics == false)
        {
            return;
        }

        const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
        if(tMyCommWrapper.myProcID() == 0)
        {
            this->cacheOutputData();
            Plato::pso::print_pso_diagnostics(mOutputData, mOutputStream, mPrintDiagnostics);
        }
    }

    /******************************************************************************//**
     * @brief Print diagnostics particle swarm constrained optimization algorithm.
    **********************************************************************************/
    void outputDiagnostics(std::ofstream & aOutputStream)
    {
        if(mPrintDiagnostics == false)
        {
            return;
        }

        const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
        if(tMyCommWrapper.myProcID() == 0)
        {
            this->cacheOutputData();
            Plato::pso::print_alpso_inner_diagnostics(mOutputData, aOutputStream, mPrintDiagnostics);
        }
    }

    /******************************************************************************//**
     * @brief Cache output/diagnostics data.
    **********************************************************************************/
    void cacheOutputData()
    {
        mOutputData.mNumIter = mNumIterations;
        mOutputData.mObjFuncCount = mNumIterations * mDataMng->getNumParticles();
        mOutputData.mCurrentGlobalBestObjFuncValue = mDataMng->getCurrentGlobalBestObjFuncValue();
        mOutputData.mMeanCurrentBestObjFuncValues = mDataMng->getMeanCurrentBestObjFuncValues();
        mOutputData.mStdDevCurrentBestObjFuncValues = mDataMng->getStdDevCurrentBestObjFuncValues();
        mOutputData.mTrustRegionMultiplier = mOperations->getTrustRegionMultiplier();
    }

private:
    bool mPrintDiagnostics;
    std::ofstream mOutputStream;

    OrdinalType mNumIterations;
    OrdinalType mMaxNumIterations;

    ScalarType mBestObjFuncTolerance;
    ScalarType mMeanObjFuncTolerance;

    Plato::particle_swarm::stop_t mStopCriterion;
    Plato::OutputDataPSO<ScalarType, OrdinalType> mOutputData;
    std::shared_ptr<Plato::ParticleSwarmDataMng<ScalarType, OrdinalType>> mDataMng;
    std::shared_ptr<Plato::ParticleSwarmStageMng<ScalarType, OrdinalType>> mStageMng;
    std::shared_ptr<Plato::ParticleSwarmOperations<ScalarType, OrdinalType>> mOperations;

private:
    BoundConstrainedPSO(const Plato::BoundConstrainedPSO<ScalarType, OrdinalType>&);
    Plato::BoundConstrainedPSO<ScalarType, OrdinalType> & operator=(const Plato::BoundConstrainedPSO<ScalarType, OrdinalType>&);
};
// class BoundConstrainedPSO

template<typename ScalarType, typename OrdinalType = size_t>
class AugmentedLagrangianPSO
{
public:
    AugmentedLagrangianPSO(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory,
                           const std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> & aObjective,
                           const std::shared_ptr<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>> & aConstraints) :
            mPrintDiagnostics(false),
            mNumIterations(0),
            mMaxNumIterations(200),
            mBestAugLagFuncTolerance(1e-10),
            mMeanAugLagFuncTolerance(5e-4),
            mStopCriterion(Plato::particle_swarm::DID_NOT_CONVERGE),
            mOutputData(aConstraints->size()),
            mStageMng(std::make_shared<Plato::AugmentedLagrangianStageMngPSO<ScalarType, OrdinalType>>(aFactory, aObjective, aConstraints)),
            mOptimizer(std::make_shared<Plato::BoundConstrainedPSO<ScalarType, OrdinalType>>(aFactory, mStageMng))
    {
        mOptimizer->setMaxNumIterations(5); /* augmented Lagrangian subproblem iterations */
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~AugmentedLagrangianPSO()
    {
    }

    /******************************************************************************//**
     * @brief Enable output of diagnostics (i.e. optimization problem status)
    **********************************************************************************/
    void enableDiagnostics()
    {
        mPrintDiagnostics = true;
        mOptimizer->enableDiagnostics();
    }

    void setMaxNumIterations(const OrdinalType & aInput)
    {
        mMaxNumIterations = aInput;
    }

    void serMaxNumAugLagSolverIterations(const OrdinalType & aInput)
    {
        mOptimizer->setMaxNumIterations(aInput);
    }

    void setMaxNumConsecutiveFailures(const OrdinalType & aInput)
    {
        mOptimizer->setMaxNumConsecutiveFailures(aInput);
    }

    void setMaxNumConsecutiveSuccesses(const OrdinalType & aInput)
    {
        mOptimizer->setMaxNumConsecutiveSuccesses(aInput);
    }

    void setInertiaMultiplier(const ScalarType & aInput)
    {
        mOptimizer->setInertiaMultiplier(aInput);
    }

    void setCognitiveMultiplier(const ScalarType & aInput)
    {
        mOptimizer->setCognitiveMultiplier(aInput);
    }

    void setSocialBehaviorMultiplier(const ScalarType & aInput)
    {
        mOptimizer->setSocialBehaviorMultiplier(aInput);
    }

    void setMeanAugLagFuncTolerance(const ScalarType & aInput)
    {
        mMeanAugLagFuncTolerance = aInput;
    }

    void setBestAugLagFuncTolerance(const ScalarType & aInput)
    {
        mBestAugLagFuncTolerance = aInput;
    }

    void setTrustRegionMultiplier(const ScalarType & aInput)
    {
        mOptimizer->setTrustRegionMultiplier(aInput);
    }

    void setTrustRegionExpansionMultiplier(const ScalarType & aInput)
    {
        mOptimizer->setTrustRegionExpansionMultiplier(aInput);
    }

    void setTrustRegionContractionMultiplier(const ScalarType & aInput)
    {
        mOptimizer->setTrustRegionContractionMultiplier(aInput);
    }

    void setUpperBounds(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        mOptimizer->setUpperBounds(aInput);
    }

    void setLowerBounds(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        mOptimizer->setLowerBounds(aInput);
    }

    void setUpperBounds(const ScalarType & aInput)
    {
        mOptimizer->setUpperBounds(aInput);
    }

    void setLowerBounds(const ScalarType & aInput)
    {
        mOptimizer->setLowerBounds(aInput);
    }

    OrdinalType getNumIterations() const
    {
        return (mNumIterations);
    }

    OrdinalType getNumConstraints() const
    {
        return (mStageMng->getNumConstraints());
    }

    ScalarType getMeanCurrentBestAugLagValues() const
    {
        return (mOptimizer->getMeanCurrentBestObjFuncValues());
    }

    ScalarType getStdDevCurrentBestAugLagValues() const
    {
        return (mOptimizer->getStdDevCurrentBestObjFuncValues());
    }

    ScalarType getCurrentGlobalBestAugLagValue() const
    {
        return (mOptimizer->getCurrentGlobalBestObjFuncValue());
    }

    ScalarType getMeanCurrentBestConstraintValues(const OrdinalType & aIndex) const
    {
        return (mStageMng->getMeanCurrentBestConstraintValues(aIndex));
    }

    ScalarType getStdDevCurrentBestConstraintValues(const OrdinalType & aIndex) const
    {
        return (mStageMng->getStdDevCurrentBestConstraintValues(aIndex));
    }

    ScalarType getCurrentGlobalBestConstraintValue(const OrdinalType & aIndex) const
    {
        return (mStageMng->getCurrentGlobalBestConstraintValue(aIndex));
    }

    std::string getStoppingCriterion() const
    {
        std::string tReason;
        Plato::pso::get_stop_criterion(mStopCriterion, tReason);
        return (tReason);
    }

    const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & getDataMng() const
    {
        return (mOptimizer->getDataMng());
    }

    /******************************************************************************//**
     * @brief Solve bound constrained particle swarm optimization problem
    **********************************************************************************/
    void solve()
    {
        assert(mStageMng.use_count() > static_cast<OrdinalType>(0));
        assert(mOptimizer.use_count() > static_cast<OrdinalType>(0));

        mNumIterations = 0;
        this->openOutputFile();
        this->initialize();

        while(1)
        {
            mNumIterations++;
            mOptimizer->solve(mOutputStream);
            mStageMng->updatePenaltyMultipliers();
            mStageMng->updateLagrangeMultipliers();
            mStageMng->computeCriteriaStatistics();

            this->outputDiagnostics();
            if(this->checkStoppingCriteria())
            {
                mOptimizer->computeGlobalBestParticlePositionStatistics();
                this->outputStoppingCriterion();
                this->closeOutputFile();
                break;
            }
        }
    }

private:
    void initialize()
    {
        mOptimizer->initialize();
        const OrdinalType tNumConstraints = mStageMng->getNumConstraints();
        mOptimizer->setNumConstraints(tNumConstraints);
    }

    bool checkStoppingCriteria()
    {
        bool tStop = false;
        const ScalarType tBestObjFunValueMean = mOptimizer->getMeanCurrentBestObjFuncValues();
        const ScalarType tCurrentGlobalBestAugLagFuncValue = mOptimizer->getCurrentGlobalBestObjFuncValue();

        if(mNumIterations >= mMaxNumIterations)
        {
            tStop = true;
            mStopCriterion = Plato::particle_swarm::MAX_NUMBER_ITERATIONS;
        }
        else if(tCurrentGlobalBestAugLagFuncValue < mBestAugLagFuncTolerance)
        {
            tStop = true;
            mStopCriterion = Plato::particle_swarm::TRUE_OBJECTIVE_TOLERANCE;
        }
        else if(tBestObjFunValueMean < mMeanAugLagFuncTolerance)
        {
            tStop = true;
            mStopCriterion = Plato::particle_swarm::MEAN_OBJECTIVE_TOLERANCE;
        }

        return (tStop);
    }

    /******************************************************************************//**
     * @brief Open output file (i.e. diagnostics file)
    **********************************************************************************/
    void openOutputFile()
    {
        if(mPrintDiagnostics == true)
        {
            const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & tDataMng = mOptimizer->getDataMng();
            const Plato::CommWrapper& tMyCommWrapper = tDataMng.getCommWrapper();
            if(tMyCommWrapper.myProcID() == 0)
            {
                mOutputStream.open("plato_alpso_algorithm_diagnostics.txt");
                Plato::pso::print_alpso_diagnostics_header(mOutputData, mOutputStream, mPrintDiagnostics);
            }
        }
    }

    /******************************************************************************//**
     * @brief Close output file (i.e. diagnostics file)
    **********************************************************************************/
    void closeOutputFile()
    {
        if(mPrintDiagnostics == true)
        {
            const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & tDataMng = mOptimizer->getDataMng();
            const Plato::CommWrapper& tMyCommWrapper = tDataMng.getCommWrapper();
            if(tMyCommWrapper.myProcID() == 0)
            {
                mOutputStream.close();
            }
        }
    }

    /******************************************************************************//**
     * @brief Print stopping criterion into diagnostics file.
    **********************************************************************************/
    void outputStoppingCriterion()
    {
        if(mPrintDiagnostics == true)
        {
            const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & tDataMng = mOptimizer->getDataMng();
            const Plato::CommWrapper& tMyCommWrapper = tDataMng.getCommWrapper();
            if(tMyCommWrapper.myProcID() == 0)
            {
                std::string tReason;
                Plato::pso::get_stop_criterion(mStopCriterion, tReason);
                mOutputStream << tReason.c_str();
            }
        }
    }

    /******************************************************************************//**
     * @brief Print diagnostics for Kelley-Sachs bound constrained optimization algorithm
    **********************************************************************************/
    void outputDiagnostics()
    {
        if(mPrintDiagnostics == false)
        {
            return;
        }

        const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & tDataMng = mOptimizer->getDataMng();
        const Plato::CommWrapper& tMyCommWrapper = tDataMng.getCommWrapper();
        if(tMyCommWrapper.myProcID() == 0)
        {
            this->cacheObjFuncOutputData();
            this->cacheConstraintOutputData();
            Plato::pso::print_alpso_outer_diagnostics(mOutputData, mOutputStream, mPrintDiagnostics);
        }
    }

    void cacheObjFuncOutputData()
    {
        mOutputData.mNumIter = mNumIterations;
        mOutputData.mAugLagFuncCount = mStageMng->getNumAugLagFuncEvaluations();
        mOutputData.mMeanCurrentBestAugLagValues = mOptimizer->getMeanCurrentBestObjFuncValues();
        mOutputData.mCurrentGlobalBestAugLagValue = mOptimizer->getCurrentGlobalBestObjFuncValue();
        mOutputData.mStdDevCurrentBestAugLagValues = mOptimizer->getStdDevCurrentBestObjFuncValues();

        mOutputData.mMeanCurrentBestObjFuncValues = mStageMng->getMeanCurrentBestObjFuncValues();
        mOutputData.mCurrentGlobalBestObjFuncValue = mStageMng->getCurrentGlobalBestObjFuncValue();
        mOutputData.mStdDevCurrentBestObjFuncValues = mStageMng->getStdDevCurrentBestObjFuncValues();
    }

    void cacheConstraintOutputData()
    {
        for(OrdinalType tIndex = 0; tIndex < mOutputData.mNumConstraints; tIndex++)
        {
            mOutputData.mMeanCurrentBestConstraintValues[tIndex] = mStageMng->getMeanCurrentBestConstraintValues(tIndex);
            mOutputData.mStdDevCurrentBestConstraintValues[tIndex] = mStageMng->getStdDevCurrentBestConstraintValues(tIndex);
            mOutputData.mCurrentGlobalBestConstraintValues[tIndex] = mStageMng->getCurrentGlobalBestConstraintValue(tIndex);

            mOutputData.mMeanCurrentPenaltyMultipliers[tIndex] = mStageMng->getMeanCurrentPenaltyMultipliers(tIndex);
            mOutputData.mStdDevCurrentPenaltyMultipliers[tIndex] = mStageMng->getStdDevCurrentPenaltyMultipliers(tIndex);
            mOutputData.mMeanCurrentLagrangeMultipliers[tIndex] = mStageMng->getMeanCurrentLagrangeMultipliers(tIndex);
            mOutputData.mStdDevCurrentLagrangeMultipliers[tIndex] = mStageMng->getStdDevCurrentLagrangeMultipliers(tIndex);
        }
    }

private:
    bool mPrintDiagnostics;
    std::ofstream mOutputStream;

    OrdinalType mNumIterations;
    OrdinalType mMaxNumIterations;

    ScalarType mBestAugLagFuncTolerance;
    ScalarType mMeanAugLagFuncTolerance;

    Plato::particle_swarm::stop_t mStopCriterion;
    Plato::OutputDataALPSO<double> mOutputData;

    std::shared_ptr<Plato::AugmentedLagrangianStageMngPSO<double>> mStageMng;
    std::shared_ptr<Plato::BoundConstrainedPSO<double>> mOptimizer;

private:
    AugmentedLagrangianPSO(const Plato::AugmentedLagrangianPSO<ScalarType, OrdinalType>&);
    Plato::AugmentedLagrangianPSO<ScalarType, OrdinalType> & operator=(const Plato::AugmentedLagrangianPSO<ScalarType, OrdinalType>&);
};
// class AugmentedLagrangianPSO

}// namespace Plato

namespace ParticleSwarmTest
{

TEST(PlatoTest, PSO_DataMng)
{
    // ********* Allocate Core Optimization Data Templates *********
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 20;
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateControl(tNumControls, tNumParticles);

    // TEST setInitialParticles
    Plato::ParticleSwarmDataMng<double> tDataMng(tFactory);
    EXPECT_EQ(20u, tDataMng.getNumParticles());
    Plato::StandardVector<double> tVector(tNumControls);
    tVector.fill(5);
    tDataMng.setUpperBounds(tVector);
    tVector.fill(-5);
    tDataMng.setLowerBounds(tVector);
    tDataMng.setInitialParticles();
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentParticles(), tDataMng.getCurrentVelocities());

    // TEST cachePreviousVelocities
    tDataMng.cachePreviousVelocities();
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentVelocities(), tDataMng.getPreviousVelocities());
}

TEST(PlatoTest, GradFreeRadius)
{
    // ********* Allocate Core Optimization Data Templates *********
    const size_t tNumControls = 2;
    const size_t tNumParticles = 3;
    Plato::StandardMultiVector<double> tControls(tNumParticles, tNumControls);
    tControls[0].fill(0.705);
    tControls[1].fill(0.695);
    tControls[2].fill(0.725);

    // TEST OBJECTIVE
    Plato::StandardVector<double> tObjVals(tNumParticles);
    Plato::GradFreeRadius<double> tCriterion;
    tCriterion.value(tControls, tObjVals);

    const double tTolerance = 1e-6;
    EXPECT_NEAR(-0.00595, tObjVals[0], tTolerance);
    EXPECT_NEAR(-0.03395, tObjVals[1], tTolerance);
    EXPECT_NEAR(0.05125, tObjVals[2], tTolerance);
}

TEST(PlatoTest, GradFreeRosenbrock)
{
    // ********* Allocate Core Optimization Data Templates *********
    const size_t tNumControls = 2;
    const size_t tNumParticles = 3;
    Plato::StandardMultiVector<double> tControls(tNumParticles, tNumControls);
    tControls[0].fill(2);
    tControls[1].fill(0.95);
    tControls[2].fill(1);

    // TEST OBJECTIVE
    Plato::StandardVector<double> tObjVals(tNumParticles);
    Plato::GradFreeRosenbrock<double> tCriterion;
    tCriterion.value(tControls, tObjVals);

    const double tTolerance = 1e-6;
    EXPECT_NEAR(401, tObjVals[0], tTolerance);
    EXPECT_NEAR(0.228125, tObjVals[1], tTolerance);
    EXPECT_NEAR(0, tObjVals[2], tTolerance);
}

TEST(PlatoTest, PSO_IsFileOpenExeption)
{
    int tMySize = 0;
    MPI_Comm_size(MPI_COMM_WORLD, &tMySize);
    if(tMySize <= 1)
    {
        std::ofstream tFile;
        ASSERT_THROW(Plato::pso::is_file_open(tFile), std::invalid_argument);

        tFile.open("MyFile.txt");
        ASSERT_NO_THROW(Plato::pso::is_file_open(tFile));
        tFile.close();
        std::system("rm -f MyFile.txt");
    }
}

TEST(PlatoTest, PSO_IsVectorEmpty)
{
    std::vector<double> tVector;
    ASSERT_THROW(Plato::pso::is_vector_empty(tVector), std::invalid_argument);

    const size_t tLength = 1;
    tVector.resize(tLength);
    ASSERT_NO_THROW(Plato::pso::is_vector_empty(tVector));
}

TEST(PlatoTest, PSO_PrintStoppingCriterion)
{
    std::string tDescription;
    Plato::particle_swarm::stop_t tFlag = Plato::particle_swarm::DID_NOT_CONVERGE;
    Plato::pso::get_stop_criterion(tFlag, tDescription);
    std::string tGold("\n\n****** Optimization algorithm did not converge. ******\n\n");
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::particle_swarm::MAX_NUMBER_ITERATIONS;
    Plato::pso::get_stop_criterion(tFlag, tDescription);
    tGold = "\n\n****** Optimization stopping due to exceeding maximum number of iterations. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::particle_swarm::TRUE_OBJECTIVE_TOLERANCE;
    Plato::pso::get_stop_criterion(tFlag, tDescription);
    tGold = "\n\n****** Optimization stopping due to global best objective function tolerance being met. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::particle_swarm::MEAN_OBJECTIVE_TOLERANCE;
    Plato::pso::get_stop_criterion(tFlag, tDescription);
    tGold = "\n\n****** Optimization stopping due to mean objective function tolerance being met. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());
}

TEST(PlatoTest, PSO_PrintDiagnosticsInvalidArgumentsPSO)
{
    std::ofstream tFile1;
    Plato::OutputDataPSO<double> tData;
    ASSERT_THROW(Plato::pso::print_pso_diagnostics_header(tData, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::pso::print_pso_diagnostics_header(tData, tFile1, true /* print message */), std::invalid_argument);
    ASSERT_THROW(Plato::pso::print_pso_diagnostics(tData, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::pso::print_pso_diagnostics(tData, tFile1, true /* print message */), std::invalid_argument);
}

TEST(PlatoTest, PSO_PrintDiagnostics)
{
    int tMySize = 0;
    MPI_Comm_size(MPI_COMM_WORLD, &tMySize);
    if(tMySize <= 1)
    {
        std::ofstream tWriteFile;
        tWriteFile.open("MyFile1.txt");
        Plato::OutputDataPSO<double> tData;
        tData.mNumIter = 1;
        tData.mObjFuncCount = 20;
        tData.mNumConstraints = 0;
        tData.mCurrentGlobalBestObjFuncValue = 4.2321;
        tData.mMeanCurrentBestObjFuncValues = 8.2321;
        tData.mStdDevCurrentBestObjFuncValues = 2.2321;
        tData.mTrustRegionMultiplier = 1.0;
        ASSERT_NO_THROW(Plato::pso::print_pso_diagnostics_header(tData, tWriteFile));
        ASSERT_NO_THROW(Plato::pso::print_pso_diagnostics(tData, tWriteFile));

        tData.mNumIter = 2;
        tData.mObjFuncCount = 40;
        tData.mCurrentGlobalBestObjFuncValue = 2.2321;
        tData.mMeanCurrentBestObjFuncValues = 7.2321;
        tData.mStdDevCurrentBestObjFuncValues = 2.4321;
        tData.mTrustRegionMultiplier = 1.0;
        ASSERT_NO_THROW(Plato::pso::print_pso_diagnostics(tData, tWriteFile));
        tWriteFile.close();

        std::ifstream tReadFile;
        tReadFile.open("MyFile1.txt");
        std::string tInputString;
        std::stringstream tReadData;
        while(tReadFile >> tInputString)
        {
            tReadData << tInputString.c_str();
        }
        tReadFile.close();
        std::system("rm -f MyFile1.txt");

        std::stringstream tGold;
        tGold << "IterF-countBest(F)Mean(F)StdDev(F)TR-Radius";
        tGold << "1204.232100e+008.232100e+002.232100e+001.000000e+00";
        tGold << "2402.232100e+007.232100e+002.432100e+001.000000e+00";
        ASSERT_STREQ(tReadData.str().c_str(), tGold.str().c_str());
    }
}

TEST(PlatoTest, PSO_PrintDiagnosticsInvalidArgumentsALPSO)
{
    std::ofstream tFile1;
    Plato::OutputDataALPSO<double> tDataALPSO;
    ASSERT_THROW(Plato::pso::print_alpso_diagnostics_header(tDataALPSO, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::pso::print_alpso_diagnostics_header(tDataALPSO, tFile1, true /* print message */), std::invalid_argument);
    ASSERT_THROW(Plato::pso::print_alpso_outer_diagnostics(tDataALPSO, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::pso::print_alpso_outer_diagnostics(tDataALPSO, tFile1, true /* print message */), std::invalid_argument);

    Plato::OutputDataPSO<double> tDataPSO;
    ASSERT_THROW(Plato::pso::print_alpso_inner_diagnostics(tDataPSO, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::pso::print_alpso_inner_diagnostics(tDataPSO, tFile1, true /* print message */), std::invalid_argument);
}

TEST(PlatoTest, PSO_PrintDiagnosticsALPSO)
{
    int tMySize = 0;
    MPI_Comm_size(MPI_COMM_WORLD, &tMySize);
    if(tMySize <= 1)
    {
        std::ofstream tWriteFile;
        tWriteFile.open("MyFile.txt");
        Plato::OutputDataPSO<double> tDataPSO;
        const size_t tNumConstraints = 2;
        Plato::OutputDataALPSO<double> tDataALPSO(tNumConstraints);
        tDataPSO.mNumConstraints = tNumConstraints;

        // **** AUGMENTED LAGRANGIAN OUTPUT ****
        tDataPSO.mNumIter = 0;
        tDataPSO.mObjFuncCount = 1;
        tDataPSO.mCurrentGlobalBestObjFuncValue = 1;
        tDataPSO.mMeanCurrentBestObjFuncValues = 1.5;
        tDataPSO.mStdDevCurrentBestObjFuncValues = 2.34e-2;
        tDataPSO.mTrustRegionMultiplier = 0.5;
        ASSERT_NO_THROW(Plato::pso::print_alpso_diagnostics_header(tDataALPSO, tWriteFile));
        ASSERT_NO_THROW(Plato::pso::print_alpso_outer_diagnostics(tDataALPSO, tWriteFile));

        tDataPSO.mNumIter = 1;
        tDataPSO.mObjFuncCount = 10;
        tDataPSO.mCurrentGlobalBestObjFuncValue = 0.1435;
        tDataPSO.mMeanCurrentBestObjFuncValues = 0.78;
        tDataPSO.mStdDevCurrentBestObjFuncValues = 0.298736;
        tDataPSO.mTrustRegionMultiplier = 3.45656e-1;
        ASSERT_NO_THROW(Plato::pso::print_alpso_inner_diagnostics(tDataPSO, tWriteFile));

        // **** AUGMENTED LAGRANGIAN OUTPUT ****
        tDataALPSO.mNumIter = 1;
        tDataALPSO.mAugLagFuncCount = 10;
        tDataALPSO.mCurrentGlobalBestAugLagValue = 1.2359e-1;
        tDataALPSO.mMeanCurrentBestAugLagValues = 3.2359e-1;
        tDataALPSO.mStdDevCurrentBestAugLagValues = 3.2359e-2;
        tDataALPSO.mCurrentGlobalBestObjFuncValue = 8.2359e-2;
        tDataALPSO.mMeanCurrentBestObjFuncValues = 9.2359e-2;
        tDataALPSO.mStdDevCurrentBestObjFuncValues = 2.2359e-2;
        tDataALPSO.mCurrentGlobalBestConstraintValues[0] = 1.23e-5;
        tDataALPSO.mCurrentGlobalBestConstraintValues[1] = 3.65e-3;
        tDataALPSO.mMeanCurrentBestConstraintValues[0] = 4.23e-5;
        tDataALPSO.mMeanCurrentBestConstraintValues[1] = 6.65e-3;
        tDataALPSO.mStdDevCurrentBestConstraintValues[0] = 1.23e-5;
        tDataALPSO.mStdDevCurrentBestConstraintValues[1] = 8.65e-4;
        tDataALPSO.mMeanCurrentPenaltyMultipliers[0] = 1;
        tDataALPSO.mMeanCurrentPenaltyMultipliers[1] = 2;
        tDataALPSO.mStdDevCurrentPenaltyMultipliers[0] = 0.25;
        tDataALPSO.mStdDevCurrentPenaltyMultipliers[1] = 0.1;
        tDataALPSO.mMeanCurrentLagrangeMultipliers[0] = 1.23e-2;
        tDataALPSO.mMeanCurrentLagrangeMultipliers[1] = 8.65e-1;
        tDataALPSO.mStdDevCurrentLagrangeMultipliers[0] = 9.23e-3;
        tDataALPSO.mStdDevCurrentLagrangeMultipliers[1] = 5.65e-1;
        ASSERT_NO_THROW(Plato::pso::print_alpso_outer_diagnostics(tDataALPSO, tWriteFile));
        tDataPSO.mNumIter = 1;
        tDataPSO.mObjFuncCount = 10;
        ASSERT_NO_THROW(Plato::pso::print_alpso_inner_diagnostics(tDataPSO, tWriteFile));
        tDataPSO.mNumIter = 2;
        tDataPSO.mObjFuncCount = 20;
        ASSERT_NO_THROW(Plato::pso::print_alpso_inner_diagnostics(tDataPSO, tWriteFile));
        tDataPSO.mNumIter = 3;
        tDataPSO.mObjFuncCount = 30;
        ASSERT_NO_THROW(Plato::pso::print_alpso_inner_diagnostics(tDataPSO, tWriteFile));
        tDataALPSO.mNumIter = 2;
        tDataALPSO.mAugLagFuncCount = 40;
        ASSERT_NO_THROW(Plato::pso::print_alpso_outer_diagnostics(tDataALPSO, tWriteFile));
        tDataPSO.mNumIter = 1;
        tDataPSO.mObjFuncCount = 10;
        ASSERT_NO_THROW(Plato::pso::print_alpso_inner_diagnostics(tDataPSO, tWriteFile));
        tDataALPSO.mNumIter = 3;
        tDataALPSO.mAugLagFuncCount = 50;
        ASSERT_NO_THROW(Plato::pso::print_alpso_outer_diagnostics(tDataALPSO, tWriteFile));
        tWriteFile.close();

        std::ifstream tReadFile;
        tReadFile.open("MyFile.txt");
        std::string tInputString;
        std::stringstream tReadData;
        while(tReadFile >> tInputString)
        {
            tReadData << tInputString.c_str();
        }
        tReadFile.close();
        std::system("rm -f MyFile.txt");

        std::stringstream tGold;
        tGold << "IterF-countBest(L)Mean(L)StdDev(L)Best(F)Mean(F)StdDev(F)TR-RadiusBest(H1)Mean(H1)StdDev(H1)Mean(P1)StdDev(P1)Mean(l1)StdDev(l1)";
        tGold << "Best(H2)Mean(H2)StdDev(H2)Mean(P2)StdDev(P2)Mean(l2)StdDev(l2)";
        tGold << "000.000000e+000.000000e+000.000000e+000.000000e+000.000000e+000.000000e+00*0.000000e+000.000000e+00";
        tGold << "0.000000e+000.000000e+000.000000e+000.000000e+000.000000e+00";
        tGold << "0.000000e+000.000000e+000.000000e+000.000000e+000.000000e+000.000000e+000.000000e+00";
        tGold << "1*1.435000e-017.800000e-012.987360e-01***3.456560e-01**************";
        tGold << "1101.235900e-013.235900e-013.235900e-028.235900e-029.235900e-022.235900e-02*1.230000e-054.230000e-051.230000e-051.000000e+002.500000e-01";
        tGold << "1.230000e-029.230000e-033.650000e-036.650000e-038.650000e-042.000000e+001.000000e-018.650000e-015.650000e-01";
        tGold << "1*1.435000e-017.800000e-012.987360e-01***3.456560e-01**************";
        tGold << "2*1.435000e-017.800000e-012.987360e-01***3.456560e-01**************";
        tGold << "3*1.435000e-017.800000e-012.987360e-01***3.456560e-01**************";
        tGold << "2401.235900e-013.235900e-013.235900e-028.235900e-029.235900e-022.235900e-02*1.230000e-054.230000e-051.230000e-051.000000e+002.500000e-01";
        tGold << "1.230000e-029.230000e-033.650000e-036.650000e-038.650000e-042.000000e+001.000000e-018.650000e-015.650000e-01";
        tGold << "1*1.435000e-017.800000e-012.987360e-01***3.456560e-01**************";
        tGold << "3501.235900e-013.235900e-013.235900e-028.235900e-029.235900e-022.235900e-02*1.230000e-054.230000e-051.230000e-051.000000e+002.500000e-01";
        tGold << "1.230000e-029.230000e-033.650000e-036.650000e-038.650000e-042.000000e+001.000000e-018.650000e-015.650000e-01";
        ASSERT_STREQ(tReadData.str().c_str(), tGold.str().c_str());
    }
}

TEST(PlatoTest, PSO_checkInertiaMultiplier)
{
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 20;
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateControl(tNumControls, tNumParticles);

    Plato::ParticleSwarmOperations<double> tOperations(tFactory);
    EXPECT_FALSE(tOperations.checkInertiaMultiplier());

    // MULTIPLIER ABOVE UPPER BOUND = 1, SET VALUE TO DEFAULT = 0.9
    tOperations.setInertiaMultiplier(2);
    EXPECT_TRUE(tOperations.checkInertiaMultiplier());
    const double tTolerance = 1e-6;
    EXPECT_NEAR(0.9, tOperations.getInertiaMultiplier(), tTolerance);

    // MULTIPLIER BELOW LOWER BOUND = -0.2, SET VALUE TO DEFAULT = 0.9
    tOperations.setInertiaMultiplier(-0.3);
    EXPECT_TRUE(tOperations.checkInertiaMultiplier());
    EXPECT_NEAR(0.9, tOperations.getInertiaMultiplier(), tTolerance);
}

TEST(PlatoTest, PSO_updateTrustRegionMultiplier)
{
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 20;
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateControl(tNumControls, tNumParticles);
    Plato::ParticleSwarmOperations<double> tOperations(tFactory);

    // MULTIPLIERS STAYS UNCHANGED
    tOperations.updateTrustRegionMultiplier();
    const double tTolerance = 1e-6;
    EXPECT_NEAR(1.0, tOperations.getTrustRegionMultiplier(), tTolerance);

    // MULTIPLIERS - CONTRACTED
    tOperations.setNumConsecutiveFailures(10 /* default limit */);
    tOperations.updateTrustRegionMultiplier();
    EXPECT_NEAR(0.75, tOperations.getTrustRegionMultiplier(), tTolerance);

    // MULTIPLIERS - EXPANDED
    tOperations.setNumConsecutiveFailures(0);
    tOperations.setNumConsecutiveSuccesses(10 /* default limit */);
    tOperations.updateTrustRegionMultiplier();
    EXPECT_NEAR(3.0, tOperations.getTrustRegionMultiplier(), tTolerance);
}

TEST(PlatoTest, PSO_computeBestObjFunStatistics)
{
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 20;
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateControl(tNumControls, tNumParticles);

    std::vector<double> tData =
        { 2.47714e-10, 1.85455e-10, 6.77601e-09, 1.31141e-09, 0.00147344, 4.75417e-07, 4.00712e-09, 2.52841e-10,
                3.88818e-10, 0.000664043, 5.2746e-10, 3.68332e-07, 1.21143e-09, 8.75453e-10, 1.81673e-10, 1.42615e-08,
                5.58984e-10, 2.70975e-08, 3.36991e-10, 1.55175e-09 };
    Plato::StandardVector<double> tBestObjFuncValues(tData);
    Plato::ParticleSwarmDataMng<double> tDataMng(tFactory);
    tDataMng.setCurrentBestObjFuncValues(tBestObjFuncValues);
    tDataMng.computeCurrentBestObjFuncStatistics();

    const double tTolerance = 1e-6;
    EXPECT_NEAR(0.00010692, tDataMng.getMeanCurrentBestObjFuncValues(), tTolerance);
    EXPECT_NEAR(0.000354175, tDataMng.getStdDevCurrentBestObjFuncValues(), tTolerance);
}

TEST(PlatoTest, PSO_find_best_criterion_values)
{
    std::vector<double> tData = { 0.00044607, 0.0639247, 3.9283e-05, 0.0318453, 0.000420515 };
    Plato::StandardVector<double> tCurrentObjFuncValues(tData);

    tData = { 0.000423009, 0.0008654, 0.00174032, 0.000871822, 0.000426448 };
    Plato::StandardVector<double> tCurrentBestObjFuncValues(tData);

    // TEST 1: FOUND TWO NEW BEST VALUES
    Plato::find_best_criterion_values(tCurrentObjFuncValues, tCurrentBestObjFuncValues);

    const double tTolerance = 1e-6;
    EXPECT_NEAR(0.000423009, tCurrentBestObjFuncValues[0], tTolerance);
    EXPECT_NEAR(0.0008654, tCurrentBestObjFuncValues[1], tTolerance);
    EXPECT_NEAR(3.9283e-05, tCurrentBestObjFuncValues[2], tTolerance);
    EXPECT_NEAR(0.000871822, tCurrentBestObjFuncValues[3], tTolerance);
    EXPECT_NEAR(0.000420515, tCurrentBestObjFuncValues[4], tTolerance);

    // TEST 2: FOUND NO NEW BEST VALUES - CURRENT BEST VALUES ARE NOT UNCHANGED
    tCurrentObjFuncValues[2] = 4.9283e-05;
    tCurrentObjFuncValues[4] = 0.000430515;
    Plato::find_best_criterion_values(tCurrentObjFuncValues, tCurrentBestObjFuncValues);

    EXPECT_NEAR(0.000423009, tCurrentBestObjFuncValues[0], tTolerance);
    EXPECT_NEAR(0.0008654, tCurrentBestObjFuncValues[1], tTolerance);
    EXPECT_NEAR(3.9283e-05, tCurrentBestObjFuncValues[2], tTolerance);
    EXPECT_NEAR(0.000871822, tCurrentBestObjFuncValues[3], tTolerance);
    EXPECT_NEAR(0.000420515, tCurrentBestObjFuncValues[4], tTolerance);
}

TEST(PlatoTest, PSO_findBestParticlePositions_BoundConstrainedStageMng)
{
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 5;
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateControl(tNumControls, tNumParticles);

    Plato::ParticleSwarmDataMng<double> tDataMng(tFactory);
    tDataMng.setCurrentGlobalBestObjFunValue(0.000423009);
    std::vector<double> tData = { 0.00044607, 0.0639247, 3.9283e-05, 0.0318453, 0.000420515 };
    Plato::StandardVector<double> tCurrentObjFuncValues(tData);
    tDataMng.setCurrentObjFuncValues(tCurrentObjFuncValues);

    tData = { 0.000423009, 0.0008654, 0.00174032, 0.000871822, 0.000426448 };
    Plato::StandardVector<double> tBestObjFuncValues(tData);
    tDataMng.setCurrentBestObjFuncValues(tBestObjFuncValues);

    Plato::StandardMultiVector<double> tControls(tNumParticles, tNumControls);
    tControls(0, 0) = 1.02069; tControls(0, 1) = 1.04138;
    tControls(1, 0) = 1.03309; tControls(1, 1) = 1.0422;
    tControls(2, 0) = 0.998152; tControls(2, 1) = 0.996907;
    tControls(3, 0) = 1.01857; tControls(3, 1) = 1.05524;
    tControls(4, 0) = 1.0205; tControls(4, 1) = 1.04138;
    tDataMng.setCurrentParticles(tControls);

    tControls(0, 0) = 1.02056; tControls(0, 1) = 1.0415;
    tControls(1, 0) = 1.02941; tControls(1, 1) = 1.05975;
    tControls(2, 0) = 0.98662; tControls(2, 1) = 0.97737;
    tControls(3, 0) = 1.02876; tControls(3, 1) = 1.05767;
    tControls(4, 0) = 1.02065; tControls(4, 1) = 1.04175;
    tDataMng.setBestParticlePositions(tControls);

    // FIND BEST PARTICLE POSITIONS
    tDataMng.cacheGlobalBestObjFunValue();
    tDataMng.updateBestParticlesData();
    tDataMng.findGlobalBestParticle();

    const double tTolerance = 1e-6;
    EXPECT_NEAR(3.9283e-05, tDataMng.getCurrentGlobalBestObjFuncValue(), tTolerance);
    EXPECT_NEAR(0.000423009, tDataMng.getPreviousGlobalBestObjFuncValue(), tTolerance);
    EXPECT_NEAR(0.998152, tDataMng.getGlobalBestParticlePosition()[0], tTolerance);
    EXPECT_NEAR(0.996907, tDataMng.getGlobalBestParticlePosition()[1], tTolerance);
    // CHECK CURRENT BEST PARTICLE POSITIONS
    EXPECT_NEAR(1.02056, tDataMng.getBestParticlePosition(0)[0], tTolerance);
    EXPECT_NEAR(1.0415, tDataMng.getBestParticlePosition(0)[1], tTolerance);
    EXPECT_NEAR(1.02941, tDataMng.getBestParticlePosition(1)[0], tTolerance);
    EXPECT_NEAR(1.05975, tDataMng.getBestParticlePosition(1)[1], tTolerance);
    EXPECT_NEAR(0.998152, tDataMng.getBestParticlePosition(2)[0], tTolerance);
    EXPECT_NEAR(0.996907, tDataMng.getBestParticlePosition(2)[1], tTolerance);
    EXPECT_NEAR(1.02876, tDataMng.getBestParticlePosition(3)[0], tTolerance);
    EXPECT_NEAR(1.05767, tDataMng.getBestParticlePosition(3)[1], tTolerance);
    EXPECT_NEAR(1.0205, tDataMng.getBestParticlePosition(4)[0], tTolerance);
    EXPECT_NEAR(1.04138, tDataMng.getBestParticlePosition(4)[1], tTolerance);
    // CHECK CURRENT BEST OBJECTIVE FUNCTION VALUES
    EXPECT_NEAR(0.000423009, tDataMng.getCurrentBestObjFuncValues()[0], tTolerance);
    EXPECT_NEAR(0.0008654, tDataMng.getCurrentBestObjFuncValues()[1], tTolerance);
    EXPECT_NEAR(3.9283e-05, tDataMng.getCurrentBestObjFuncValues()[2], tTolerance);
    EXPECT_NEAR(0.000871822, tDataMng.getCurrentBestObjFuncValues()[3], tTolerance);
    EXPECT_NEAR(0.000420515, tDataMng.getCurrentBestObjFuncValues()[4], tTolerance);
}

TEST(PlatoTest, PSO_BoundConstrainedStageMng)
{
    const size_t tNumControls = 2;
    const size_t tNumParticles = 1;
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateControl(tNumControls, tNumParticles);

    Plato::ParticleSwarmDataMng<double> tDataMng(tFactory);
    tDataMng.setCurrentParticle(2.0);

    std::shared_ptr<Plato::GradFreeRosenbrock<double>> tObjective = std::make_shared<Plato::GradFreeRosenbrock<double>>();
    Plato::BoundConstrainedStageMngPSO<double> tStageMng(tFactory, tObjective);
    tStageMng.evaluateObjective(tDataMng);

    const double tTolerance = 1e-6;
    EXPECT_NEAR(401, tDataMng.getCurrentObjFuncValue(0 /* particle index */), tTolerance);
}

TEST(PlatoTest, PSO_AugmentedLagrangianStageMng)
{
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 2;
    const size_t tNumConstraints = 1;
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateDual(tNumParticles, tNumConstraints);
    tFactory->allocateControl(tNumControls, tNumParticles);

    std::shared_ptr<Plato::GradFreeRosenbrock<double>> tObjective = std::make_shared<Plato::GradFreeRosenbrock<double>>();
    std::shared_ptr<Plato::GradFreeRadius<double>> tConstraint = std::make_shared<Plato::GradFreeRadius<double>>();
    std::shared_ptr<Plato::GradFreeCriteriaList<double>> tConstraintList = std::make_shared<Plato::GradFreeCriteriaList<double>>();
    tConstraintList->add(tConstraint);

    Plato::AugmentedLagrangianStageMngPSO<double> tStageMng(tFactory, tObjective, tConstraintList);

    // FIRST AUGMENTED LAGRANGIAN FUNCTION EVALUATION
    Plato::StandardVector<double> tLagrangianValues(tNumParticles);
    Plato::StandardMultiVector<double> tControl(tNumParticles, tNumControls);
    Plato::ParticleSwarmDataMng<double> tDataMng(tFactory);
    tDataMng.setCurrentParticle(0 /* particle index */, 0.725 /* scalar value */);
    tDataMng.setCurrentParticle(1 /* particle index */, 0.705 /* scalar value */);
    tStageMng.evaluateObjective(tDataMng);
    const double tTolerance = 1e-6;
    EXPECT_NEAR(4.053290625, tDataMng.getCurrentObjFuncValue(0 /* particle index */), tTolerance); // augmented Lagrangian
    EXPECT_NEAR(4.4123850625, tDataMng.getCurrentObjFuncValue(1 /* particle index */), tTolerance); // augmented Lagrangian

    // FIND CURRENT SET OF BEST PARTICLES
    tStageMng.findBestParticlePositions(tDataMng);
    EXPECT_NEAR(4.0506640625, tStageMng.getCurrentBestObjFuncValue(0 /* particle index */), tTolerance); // objective function
    EXPECT_NEAR(4.4123850625, tStageMng.getCurrentBestObjFuncValues()[1 /* particle index */], tTolerance); // objective function

    // UPDATE PENALTY MULTIPLIERS BASED ON BEST CONSTRAINT VALUES
    tStageMng.updatePenaltyMultipliers();
    EXPECT_NEAR(2, tStageMng.getPenaltyMultipliers(0  /* constraint index */)[0], tTolerance);
    EXPECT_NEAR(0.5, tStageMng.getPenaltyMultiplier(0  /* constraint index */, 1  /* particle index */), tTolerance);

    // UPDATE LAGRANGE MULTIPLIERS BASED ON BEST CONSTRAINT VALUES
    tStageMng.updateLagrangeMultipliers();
    EXPECT_NEAR(0.205, tStageMng.getLagrangeMultipliers(0  /* constraint index */)[0], tTolerance);
    EXPECT_NEAR(0, tStageMng.getLagrangeMultiplier(0  /* constraint index */, 1  /* particle index */), tTolerance);

    tStageMng.computeCriteriaStatistics();
    EXPECT_NEAR(4.0506640625, tStageMng.getCurrentGlobalBestObjFuncValue(), tTolerance);
    EXPECT_NEAR(4.2315245625, tStageMng.getMeanCurrentBestObjFuncValues(), tTolerance);
    EXPECT_NEAR(0.255775372, tStageMng.getStdDevCurrentBestObjFuncValues(), tTolerance);
    EXPECT_NEAR(0.02265, tStageMng.getMeanCurrentBestConstraintValues(0  /* constraint index */), tTolerance);
    EXPECT_NEAR(0.04044650788, tStageMng.getStdDevCurrentBestConstraintValues(0  /* constraint index */), tTolerance);
    EXPECT_NEAR(0.00595, tStageMng.getCurrentGlobalBestConstraintValue(0  /* constraint index */), tTolerance);

    // SECOND CALL: EVALUATE AUGMENTED LAGRANGIAN FUNCTION
    tDataMng.setCurrentParticle(0  /* particle index */, 0.715  /* scalar value */);
    tDataMng.setCurrentParticle(1  /* particle index */, 0.695  /* scalar value */);
    tStageMng.evaluateObjective(tDataMng);
    EXPECT_NEAR(4.2392603175, tDataMng.getCurrentObjFuncValue(0  /* particle index */), tTolerance); // augmented Lagrangian
    EXPECT_NEAR(4.5863650625, tDataMng.getCurrentObjFuncValue(1  /* particle index */), tTolerance); // augmented Lagrangian
    EXPECT_NEAR(4.2336500625, tStageMng.getCurrentObjFuncValue(0  /* particle index */), tTolerance); // objective function
    EXPECT_NEAR(4.5863650625, tStageMng.getCurrentObjFuncValues()[1 /* particle index */], tTolerance); // objective function
    EXPECT_NEAR(4.0506640625, tStageMng.getPreviousBestObjFuncValue(0  /* particle index */), tTolerance); // objective function
    EXPECT_NEAR(4.4123850625, tStageMng.getPreviousBestObjFuncValues()[1 /* particle index */], tTolerance); // objective function
    EXPECT_NEAR(0.02245, tStageMng.getCurrentConstraintValue(0  /* constraint index */, 0  /* particle index */), tTolerance);
    EXPECT_NEAR(-0.03395, tStageMng.getCurrentConstraintValues(0  /* constraint index */)[1], tTolerance);

    // FIND CURRENT SET OF BEST PARTICLES
    tStageMng.findBestParticlePositions(tDataMng);
    EXPECT_NEAR(4.0506640625, tStageMng.getPreviousBestObjFuncValue(0  /* particle index */), tTolerance); // objective function
    EXPECT_NEAR(4.4123850625, tStageMng.getPreviousBestObjFuncValues()[1 /* particle index */], tTolerance); // objective function
    EXPECT_NEAR(0.05125, tStageMng.getPreviousBestConstraintValue(0  /* constraint index */, 0  /* particle index */), tTolerance);
    EXPECT_NEAR(-0.00595, tStageMng.getPreviousBestConstraintValues(0  /* constraint index */)[1], tTolerance);
    EXPECT_NEAR(0.05125, tStageMng.getCurrentBestConstraintValue(0  /* constraint index */, 0  /* particle index */), tTolerance);
    EXPECT_NEAR(-0.00595, tStageMng.getCurrentBestConstraintValues(0  /* constraint index */)[1 /* particle index */], tTolerance);

    tStageMng.updatePenaltyMultipliers();
    EXPECT_NEAR(22.63846284534, tStageMng.getPenaltyMultipliers(0  /* constraint index */)[0], tTolerance);
    EXPECT_NEAR(0.25, tStageMng.getPenaltyMultipliers(0  /* constraint index */)[1], tTolerance);

    tStageMng.updateLagrangeMultipliers();
    EXPECT_NEAR(2.5254424416477, tStageMng.getLagrangeMultipliers(0  /* constraint index */)[0], tTolerance);
    EXPECT_NEAR(0, tStageMng.getLagrangeMultipliers(0  /* constraint index */)[1], tTolerance);

    tStageMng.computeCriteriaStatistics();
    EXPECT_NEAR(4.0506640625, tStageMng.getCurrentGlobalBestObjFuncValue(), tTolerance);
    EXPECT_NEAR(4.2315245625, tStageMng.getMeanCurrentBestObjFuncValues(), tTolerance);
    EXPECT_NEAR(0.255775372, tStageMng.getStdDevCurrentBestObjFuncValues(), tTolerance);
    EXPECT_NEAR(0.02265, tStageMng.getMeanCurrentBestConstraintValues(0  /* constraint index */), tTolerance);
    EXPECT_NEAR(0.04044650788, tStageMng.getStdDevCurrentBestConstraintValues(0  /* constraint index */), tTolerance);
    EXPECT_NEAR(0.00595, tStageMng.getCurrentGlobalBestConstraintValue(0  /* constraint index */), tTolerance);
}

TEST(PlatoTest, PSO_SolveBCPSO)
{
    // ********* Allocate Core Optimization Data Templates *********
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 10;
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateControl(tNumControls, tNumParticles);

    // TEST ALGORITHM
    std::shared_ptr<Plato::GradFreeRosenbrock<double>> tObjective = std::make_shared<Plato::GradFreeRosenbrock<double>>();
    Plato::BoundConstrainedPSO<double> tAlgorithm(tFactory, tObjective);
    tAlgorithm.setLowerBounds(-5);
    tAlgorithm.setUpperBounds(5);
    tAlgorithm.solve();

    const double tTolerance = 1e-2;
    EXPECT_NEAR(0, tAlgorithm.getCurrentGlobalBestObjFuncValue(), tTolerance);

    std::cout << "NUM ITERATIONS = " << tAlgorithm.getNumIterations() << "\n";
    std::cout << "OBJECTIVE: BEST = " << tAlgorithm.getCurrentGlobalBestObjFuncValue() << ", MEAN = "
            << tAlgorithm.getMeanCurrentBestObjFuncValues() << ", STDDEV = "
            << tAlgorithm.getStdDevCurrentBestObjFuncValues() << "\n";
    std::cout << tAlgorithm.getStoppingCriterion() << "\n";

    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = "
                << tAlgorithm.getDataMng().getGlobalBestParticlePosition()[tIndex] << ", MEAN = "
                << tAlgorithm.getDataMng().getParticlePositionMean()[tIndex] << ", STDDEV = "
                << tAlgorithm.getDataMng().getParticlePositionStdDev()[tIndex] << "\n";
    }
}

TEST(PlatoTest, PSO_SolveBCPSO_Rocket)
{
    // ********* Allocate Core Optimization Data Templates *********
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 15;
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateControl(tNumControls, tNumParticles);

    // ********* ALLOCATE NORMALIZATION CONSTANTS *********
    Plato::StandardVector<double> tVector(tNumControls);
    tVector[0] = 0.08; tVector[1] = 0.006;

    // ********* ALLOCATE OBJECTIVE FUNCTION *********
    Plato::AlgebraicRocketInputs<double> tRocketInputs;
    std::shared_ptr<Plato::GeometryModel<double>> tGeomModel =
            std::make_shared<Plato::Cylinder<double>>(tRocketInputs.mChamberRadius, tRocketInputs.mChamberLength);
    std::shared_ptr<Plato::GradFreeRocketObjFunc<double>> tObjective =
            std::make_shared<Plato::GradFreeRocketObjFunc<double>>(tVector, tRocketInputs, tGeomModel);
    tObjective->disableOutput();

    // ********* SET TARGET THRUST PROFILE *********
    std::vector<double> tData = Plato::get_target_thrust_profile();
    Plato::StandardVector<double> tTargetThrustProfile(tData);
    tObjective->setTargetThrustProfile(tTargetThrustProfile);

    // ********* TEST ALGORITHM *********
    Plato::BoundConstrainedPSO<double> tAlgorithm(tFactory, tObjective);
    tVector[0] = 1; tVector[1] = 1;
    tAlgorithm.setUpperBounds(tVector);  /* bounds are normalized */
    tVector[0] = 0.06 / tVector[0]; tVector[1] = 0.003 / tVector[1];
    tAlgorithm.setLowerBounds(tVector);  /* bounds are normalized */
    tAlgorithm.setMeanObjFuncTolerance(1e-6);
    tAlgorithm.solve();

    const double tTolerance = 1e-2;
    EXPECT_NEAR(0, tAlgorithm.getCurrentGlobalBestObjFuncValue(), tTolerance);

    // ********* OUTPUT DIAGNOSTICS *********
    std::cout << "NUM ITERATIONS = " << tAlgorithm.getNumIterations() << "\n";
    std::cout << "OBJECTIVE: BEST = " << tAlgorithm.getCurrentGlobalBestObjFuncValue() << ", MEAN = "
            << tAlgorithm.getMeanCurrentBestObjFuncValues() << ", STDDEV = "
            << tAlgorithm.getStdDevCurrentBestObjFuncValues() << "\n";
    std::cout << tAlgorithm.getStoppingCriterion() << "\n";

    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = "
                << tAlgorithm.getDataMng().getGlobalBestParticlePosition()[tIndex]
                        * tObjective->getNormalizationConstants()[tIndex]
                << ", MEAN = "
                << tAlgorithm.getDataMng().getParticlePositionMean()[tIndex]
                        * tObjective->getNormalizationConstants()[tIndex]
                << ", STDDEV = "
                << tAlgorithm.getDataMng().getParticlePositionStdDev()[tIndex]
                        * tObjective->getNormalizationConstants()[tIndex] << "\n";
    }

    // ********* TEST THRUST PROFILE SOLUTION *********
    Plato::StandardVector<double> tBestThrustProfileSolution(tData.size());
    tObjective->solve(tAlgorithm.getDataMng().getGlobalBestParticlePosition(), tBestThrustProfileSolution);
    const double tMultiplier = 1.0 / tObjective->getNormTargetThrustProfile();
    tTargetThrustProfile.scale(tMultiplier);
    tBestThrustProfileSolution.scale(tMultiplier);
    PlatoTest::checkVectorData(tTargetThrustProfile, tBestThrustProfileSolution);
}

TEST(PlatoTest, PSO_SolveALPSO)
{
    // ********* Allocate Core Optimization Data Templates *********
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 20;
    const size_t tNumConstraints = 1;
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateDual(tNumParticles, tNumConstraints);
    tFactory->allocateControl(tNumControls, tNumParticles);

    // TEST ALGORITHM
    const double tBound = 2;
    std::shared_ptr<Plato::GradFreeRadius<double>> tConstraintOne = std::make_shared<Plato::GradFreeRadius<double>>(tBound);
    std::shared_ptr<Plato::GradFreeCriteriaList<double>> tConstraints = std::make_shared<Plato::GradFreeCriteriaList<double>>();
    tConstraints->add(tConstraintOne);
    std::shared_ptr<Plato::GradFreeRosenbrock<double>> tObjective = std::make_shared<Plato::GradFreeRosenbrock<double>>();
    Plato::AugmentedLagrangianPSO<double> tAlgorithm(tFactory, tObjective, tConstraints);
    tAlgorithm.setMeanAugLagFuncTolerance(1e-6);
    tAlgorithm.setLowerBounds(-5);
    tAlgorithm.setUpperBounds(5);
    tAlgorithm.solve();

    const double tTolerance = 1e-2;
    EXPECT_NEAR(0, tAlgorithm.getCurrentGlobalBestAugLagValue(), tTolerance);

    std::cout << "\nNUM ITERATIONS = " << tAlgorithm.getNumIterations() << "\n";
    std::cout << "\nOBJECTIVE: BEST = " << tAlgorithm.getCurrentGlobalBestAugLagValue() << ", MEAN = "
            << tAlgorithm.getMeanCurrentBestAugLagValues() << ", STDDEV = "
            << tAlgorithm.getStdDevCurrentBestAugLagValues() << "\n";

    std::cout << "\nCONSTRAINT #0: BEST = " << tAlgorithm.getCurrentGlobalBestConstraintValue(0) << ", MEAN = "
            << tAlgorithm.getMeanCurrentBestConstraintValues(0) << ", STDDEV = "
            << tAlgorithm.getStdDevCurrentBestConstraintValues(0) << "\n";

    std::cout << tAlgorithm.getStoppingCriterion() << "\n";

    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = "
                << tAlgorithm.getDataMng().getGlobalBestParticlePosition()[tIndex] << ", MEAN = "
                << tAlgorithm.getDataMng().getParticlePositionMean()[tIndex] << ", STDDEV = "
                << tAlgorithm.getDataMng().getParticlePositionStdDev()[tIndex] << "\n";
    }
}

} // ParticleSwarmTest
