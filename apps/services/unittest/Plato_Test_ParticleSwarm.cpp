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

#include "Plato_GradFreeCircle.hpp"
#include "Plato_GradFreeRadius.hpp"
#include "Plato_GradFreeHimmelblau.hpp"
#include "Plato_GradFreeRosenbrock.hpp"
#include "Plato_GradFreeRocketObjFunc.hpp"
#include "Plato_GradFreeShiftedEllipse.hpp"
#include "Plato_GradFreeGoldsteinPrice.hpp"

#include "Plato_GradFreeCriteriaList.hpp"
#include "Plato_ParticleSwarmDataMng.hpp"

#include "Plato_Parser.hpp"
#include "Plato_InputData.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_UnitTestUtils.hpp"


#include "Plato_Interface.hpp"
#include "Plato_OptimizerEngineStageData.hpp"

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
        STDDEV_OBJECTIVE_TOLERANCE = 5,
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
// function is_file_open

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
// function is_vector_empty

/******************************************************************************//**
 * @brief Output a brief description of the stopping criterion.
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
        case Plato::particle_swarm::stop_t::STDDEV_OBJECTIVE_TOLERANCE:
        {
            aOutput = "\n\n****** Optimization stopping due to standard deviation tolerance being met. ******\n\n";
            break;
        }
    }
}
// function print_bcpso_diagnostics_header

/******************************************************************************//**
 * @brief Print header for Kelley-Sachs-Bound-Constrained (KSBC) diagnostics file
 * @param [in,out] aOutputFile output file
 * @param [in] aPrint flag use to enable/disable output (default = disabled)
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void print_bcpso_diagnostics_header(const Plato::OutputDataPSO<ScalarType, OrdinalType>& aData,
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
// function print_bcpso_diagnostics_header

/******************************************************************************//**
 * @brief Print diagnostics for bound constrained Particle Swarm Optimization (PSO) algorithm.
 * @param [in] aData diagnostic data PSO algorithm
 * @param [in,out] aOutputFile output file
 * @param [in] aPrint flag use to enable/disable output (default = disabled)
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void print_bcpso_diagnostics(const Plato::OutputDataPSO<ScalarType, OrdinalType>& aData,
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
// function print_bcpso_diagnostics

/******************************************************************************//**
 * @brief Print outer constraint headers in augmented Lagrangian Particle Swarm
 *        Optimization (ALPSO) algorithm's diagnostic file.
 * @param [in] aData diagnostic data for ALPSO algorithm
 * @param [in,out] aOutputFile output/diagnostics file
 **********************************************************************************/
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
// function print_alpso_constraint_headers

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
// function print_alpso_diagnostics_header

/******************************************************************************//**
 * @brief Print outer constraint diagnostics in augmented Lagrangian Particle Swarm
 *        Optimization (ALPSO) algorithm
 * @param [in] aData diagnostic data for ALPSO algorithm
 * @param [in,out] aOutputFile output/diagnostics file
 **********************************************************************************/
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
// function print_alpso_outer_constraint_diagnostics

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
// function print_alpso_outer_diagnostics

/******************************************************************************//**
 * @brief Print outer constraint diagnostics in augmented Lagrangian Particle Swarm
 *        Optimization (ALPSO) algorithm
 * @param [in] aData diagnostic data for ALPSO algorithm
 * @param [in,out] aOutputFile output/diagnostics file
 **********************************************************************************/
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
// function print_alpso_inner_constraint_diagnostics

/******************************************************************************//**
 * @brief Print inner-loop diagnostics for the augmented Lagrangian Particle Swarm
 *        Optimization (ALPSO) algorithm.
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
// function print_alpso_inner_diagnostics

} // namespace pso





/******************************************************************************//**
 * @brief Interface for gradient free algebraic rocket criterion
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class GradFreeEngineObjective : public Plato::GradFreeCriterion<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aDataFactory data factory
     * @param [in] aInputData data structure with engine's options and input keywords
     * @param [in] aInterface interface to data motion coordinator
    **********************************************************************************/
    explicit GradFreeEngineObjective(const Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory,
                                     const Plato::OptimizerEngineStageData & aInputData,
                                     Plato::Interface* aInterface = nullptr) :
            mControl(std::vector<ScalarType>(aDataFactory.getNumControls())),
            mObjFuncValues(std::vector<ScalarType>(aDataFactory.getNumCriterionValues())),
            mInterface(aInterface),
            mEngineInputData(aInputData),
            mParameterList(std::make_shared<Teuchos::ParameterList>())
    {
    }

    /******************************************************************************//**
     * @brief Set interface to data motion coordinator
     * @param [in] aInterface interface to data motion coordinator
    **********************************************************************************/
    void set(Plato::Interface* aInterface)
    {
        assert(aInterface != nullptr);
        mInterface = aInterface;
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~GradFreeEngineObjective()
    {
    }

    /******************************************************************************//**
     * @brief Evaluates generic objective function through the Plato Engine
     * @param [in] aControl set of particles, each particle denotes a set of optimization variables
     * @param [out] aOutput criterion value for each particle
     **********************************************************************************/
    virtual void value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                       Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {

    }

private:
    std::vector<ScalarType> mControl; /*!< set of particles */
    std::vector<ScalarType> mObjFuncValues; /*!< set of objective function values */

    Plato::Interface* mInterface; /*!< interface to data motion coordinator */
    Plato::OptimizerEngineStageData mEngineInputData; /*!< holds Plato Engine's options and inputs */
    std::shared_ptr<Teuchos::ParameterList> mParameterList; /*!< Plato Engine parameter list */

private:
    GradFreeEngineObjective(const Plato::GradFreeEngineObjective<ScalarType, OrdinalType>&);
    Plato::GradFreeEngineObjective<ScalarType, OrdinalType> & operator=(const Plato::GradFreeEngineObjective<ScalarType, OrdinalType>&);
};
// class GradFreeEngineObjective
















/******************************************************************************//**
 * @brief Abstract interface for Particle Swarm Optimization (PSO) algorithm stage
 *        managers. The PSO stage manages calls to criteria (e.g. objective functions).
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class ParticleSwarmStageMng
{
public:
    virtual ~ParticleSwarmStageMng()
    {
    }

    /******************************************************************************//**
     * @brief Evaluate objective function
     * @param [in,out] aDataMng PSO data manager
    **********************************************************************************/
    virtual void evaluateObjective(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng) = 0;

    /******************************************************************************//**
     * @brief Find current best particle positions
     * @param [in,out] aDataMng PSO data manager
    **********************************************************************************/
    virtual void findBestParticlePositions(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng) = 0;
};
// class ParticleSwarmStageMng

/******************************************************************************//**
 * @brief Bound Constrained Particle Swarm Optimization (BCPSO) algorithm stage
 *        managers. The BCPSO stage manages calls to criteria (e.g. objective functions).
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class BoundConstrainedStageMngPSO : public Plato::ParticleSwarmStageMng<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aFactory data factory
     * @param [in] aObjective gradient free objective function interface
    **********************************************************************************/
    explicit BoundConstrainedStageMngPSO(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory,
                                         const std::shared_ptr<Plato::GradFreeCriterion<ScalarType, OrdinalType>> & aObjective) :
            mCurrentObjFuncValues(aFactory->objective().create()),
            mObjective(aObjective)
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~BoundConstrainedStageMngPSO()
    {
    }

    /******************************************************************************//**
     * @brief Evaluate objective function
     * @param [in,out] aDataMng PSO data manager
    **********************************************************************************/
    void evaluateObjective(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        mCurrentObjFuncValues->fill(static_cast<ScalarType>(0));
        const Plato::MultiVector<ScalarType, OrdinalType> & tParticles = aDataMng.getCurrentParticles();
        mObjective->value(tParticles, *mCurrentObjFuncValues);
        aDataMng.setCurrentObjFuncValues(*mCurrentObjFuncValues);
    }

    /******************************************************************************//**
     * @brief Find current best particle positions
     * @param [in,out] aDataMng PSO data manager
    **********************************************************************************/
    void findBestParticlePositions(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        aDataMng.cacheGlobalBestObjFunValue();
        aDataMng.updateBestParticlesData();
        aDataMng.findGlobalBestParticle();
    }

private:
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentObjFuncValues; /*!< current objective function values */
    std::shared_ptr<Plato::GradFreeCriterion<ScalarType, OrdinalType>> mObjective; /*!< grad-free objective function interface */

private:
    BoundConstrainedStageMngPSO(const Plato::BoundConstrainedStageMngPSO<ScalarType, OrdinalType>&);
    Plato::BoundConstrainedStageMngPSO<ScalarType, OrdinalType> & operator=(const Plato::BoundConstrainedStageMngPSO<ScalarType, OrdinalType>&);
};
// class BoundConstrainedStageMngPSO

/******************************************************************************//**
 * @brief Augmented Lagrangian Particle Swarm Optimization (ALPSO) algorithm stage
 *        managers. The ALPSO stage coordinates calls to grad-free criteria (e.g.
 *        objective functions).
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class AugmentedLagrangianStageMngPSO : public Plato::ParticleSwarmStageMng<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aFactory data factory
     * @param [in] aObjective gradient free objective function interface
     * @param [in] aConstraints list of gradient free constraint interfaces
    **********************************************************************************/
    explicit AugmentedLagrangianStageMngPSO(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory,
                                            const std::shared_ptr<Plato::GradFreeCriterion<ScalarType, OrdinalType>> & aObjective,
                                            const std::shared_ptr<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>> & aConstraints) :
            mNumAugLagFuncEval(0),
            mMeanCurrentBestObjFuncValue(0),
            mStdDevCurrentBestObjFuncValue(0),
            mCurrentGlobalBestObjFuncValue(std::numeric_limits<ScalarType>::max()),
            mCurrentGlobalBestAugLagFuncValue(std::numeric_limits<ScalarType>::max()),
            mPenaltyExpansionMultiplier(2),
            mPenaltyMultiplierUpperBound(100),
            mPenaltyContractionMultiplier(0.5),
            mFeasibilityInexactnessTolerance(1e-4),
            mCriteriaWorkVec(aFactory->objective().create()),
            mCurrentFeasibilityMeasure(aFactory->objective().create()),
            mPreviousFeasibilityMeasure(aFactory->objective().create()),
            mCurrentObjFuncValues(aFactory->objective().create()),
            mCurrentBestObjFuncValues(aFactory->objective().create()),
            mPreviousBestObjFuncValues(aFactory->objective().create()),
            mCurrentAugLagFuncValues(aFactory->objective().create()),
            mCurrentBestAugLagFuncValues(aFactory->objective().create()),
            mMeanBestConstraintValues(std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(aConstraints->size())),
            mStdDevBestConstraintValues(std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(aConstraints->size())),
            mCurrentGlobalBestConstraintValues(std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(aConstraints->size())),
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

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~AugmentedLagrangianStageMngPSO()
    {
    }

    /******************************************************************************//**
     * @brief Set expansion multiplier for penalty multipliers
     * @param [in] aInput expansion multiplier
    **********************************************************************************/
    void setPenaltyExpansionMultiplier(const ScalarType & aInput)
    {
        mPenaltyExpansionMultiplier = aInput;
    }

    /******************************************************************************//**
     * @brief Set contraction multiplier for penalty multipliers
     * @param [in] aInput contraction multiplier
    **********************************************************************************/
    void setPenaltyContractionMultiplier(const ScalarType & aInput)
    {
        mPenaltyContractionMultiplier = aInput;
    }

    /******************************************************************************//**
     * @brief Set upper bound on penalty multipliers
     * @param [in] aInput upper bound
    **********************************************************************************/
    void setPenaltyMultiplierUpperBound(const ScalarType & aInput)
    {
        mPenaltyMultiplierUpperBound = aInput;
    }

    /******************************************************************************//**
     * @brief Set feasibility inexactness tolerance
     * @param [in] aInput tolerance
    **********************************************************************************/
    void setFeasibilityInexactnessTolerance(const ScalarType & aInput)
    {
        mFeasibilityInexactnessTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Return number of constraints
     * @return number of constraints
    **********************************************************************************/
    OrdinalType getNumConstraints() const
    {
        return (mConstraints->size());
    }

    /******************************************************************************//**
     * @brief Return number of augmented Lagrangian function evaluations
     * @return number of function evaluations
    **********************************************************************************/
    OrdinalType getNumAugLagFuncEvaluations() const
    {
        return (mNumAugLagFuncEval);
    }

    /******************************************************************************//**
     * @brief Return mean of current best objective function values
     * @return mean value
    **********************************************************************************/
    ScalarType getMeanCurrentBestObjFuncValues() const
    {
        return (mMeanCurrentBestObjFuncValue);
    }

    /******************************************************************************//**
     * @brief Return standard deviation of current best objective function values
     * @return standard deviation value
    **********************************************************************************/
    ScalarType getStdDevCurrentBestObjFuncValues() const
    {
        return (mStdDevCurrentBestObjFuncValue);
    }

    /******************************************************************************//**
     * @brief Return current global best objective function value
     * @return global best objective function value
    **********************************************************************************/
    ScalarType getCurrentGlobalBestObjFuncValue() const
    {
        return (mCurrentGlobalBestObjFuncValue);
    }

    /******************************************************************************//**
     * @brief Return current objective function value
     * @param [in] aIndex particle index
    **********************************************************************************/
    ScalarType getCurrentObjFuncValue(const OrdinalType & aIndex) const
    {
        assert(mCurrentObjFuncValues.get() != nullptr);
        return((*mCurrentObjFuncValues)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Return current best objective function value
     * @param [in] aIndex particle index
    **********************************************************************************/
    ScalarType getCurrentBestObjFuncValue(const OrdinalType & aIndex) const
    {
        assert(mCurrentBestObjFuncValues.get() != nullptr);
        return((*mCurrentBestObjFuncValues)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Return previous best objective function value
     * @param [in] aIndex particle index
    **********************************************************************************/
    ScalarType getPreviousBestObjFuncValue(const OrdinalType & aIndex) const
    {
        assert(mPreviousBestObjFuncValues.get() != nullptr);
        return((*mPreviousBestObjFuncValues)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Return mean of current penalty multipliers
     * @param [in] aIndex particle index
    **********************************************************************************/
    ScalarType getMeanCurrentPenaltyMultipliers(const OrdinalType & aInput) const
    {
        assert(aInput < mMeanCurrentPenaltyMultipliers->size());
        return ((*mMeanCurrentPenaltyMultipliers)[aInput]);
    }

    /******************************************************************************//**
     * @brief Return standard deviation of current penalty multipliers
     * @param [in] aIndex particle index
    **********************************************************************************/
    ScalarType getStdDevCurrentPenaltyMultipliers(const OrdinalType & aInput) const
    {
        assert(aInput < mStdDevCurrentPenaltyMultipliers->size());
        return ((*mStdDevCurrentPenaltyMultipliers)[aInput]);
    }

    /******************************************************************************//**
     * @brief Return mean of current Lagrange multipliers
     * @param [in] aIndex particle index
    **********************************************************************************/
    ScalarType getMeanCurrentLagrangeMultipliers(const OrdinalType & aInput) const
    {
        assert(aInput < mMeanCurrentLagrangeMultipliers->size());
        return ((*mMeanCurrentLagrangeMultipliers)[aInput]);
    }

    /******************************************************************************//**
     * @brief Return standard deviation of current Lagrange multipliers
     * @param [in] aIndex particle index
    **********************************************************************************/
    ScalarType getStdDevCurrentLagrangeMultipliers(const OrdinalType & aInput) const
    {
        assert(aInput < mStdDevCurrentLagrangeMultipliers->size());
        return ((*mStdDevCurrentLagrangeMultipliers)[aInput]);
    }

    /******************************************************************************//**
     * @brief Return mean of current best constraint values
     * @param [in] aIndex particle index
    **********************************************************************************/
    ScalarType getMeanCurrentBestConstraintValues(const OrdinalType & aInput) const
    {
        assert(aInput < mMeanBestConstraintValues->size());
        return ((*mMeanBestConstraintValues)[aInput]);
    }

    /******************************************************************************//**
     * @brief Return standard deviation of current cconstraint values
     * @param [in] aIndex particle index
    **********************************************************************************/
    ScalarType getStdDevCurrentBestConstraintValues(const OrdinalType & aInput) const
    {
        assert(aInput < mStdDevBestConstraintValues->size());
        return ((*mStdDevBestConstraintValues)[aInput]);
    }

    /******************************************************************************//**
     * @brief Return current global constraint values
     * @param [in] aIndex particle index
    **********************************************************************************/
    ScalarType getCurrentGlobalBestConstraintValue(const OrdinalType & aInput) const
    {
        assert(aInput < mCurrentGlobalBestConstraintValues->size());
        return ((*mCurrentGlobalBestConstraintValues)[aInput]);
    }

    /******************************************************************************//**
     * @brief Return penalty multiplier
     * @param [in] aConstraintIndex constraint index
     * @param [in] aParticleIndex particle index
    **********************************************************************************/
    ScalarType getPenaltyMultiplier(const OrdinalType & aConstraintIndex, const OrdinalType & aParticleIndex) const
    {
        assert(mCurrentPenaltyMultipliers.get() != nullptr);
        assert(aConstraintIndex < mCurrentPenaltyMultipliers->getNumVectors());
        return((*mCurrentPenaltyMultipliers)(aConstraintIndex, aParticleIndex));
    }

    /******************************************************************************//**
     * @brief Return Lagrange multiplier
     * @param [in] aConstraintIndex constraint index
     * @param [in] aParticleIndex particle index
    **********************************************************************************/
    ScalarType getLagrangeMultiplier(const OrdinalType & aConstraintIndex, const OrdinalType & aParticleIndex) const
    {
        assert(mCurrentLagrangeMultipliers.get() != nullptr);
        assert(aConstraintIndex < mCurrentLagrangeMultipliers->getNumVectors());
        return((*mCurrentLagrangeMultipliers)(aConstraintIndex, aParticleIndex));
    }

    /******************************************************************************//**
     * @brief Return current constraint value
     * @param [in] aConstraintIndex constraint index
     * @param [in] aParticleIndex particle index
    **********************************************************************************/
    ScalarType getCurrentConstraintValue(const OrdinalType & aConstraintIndex, const OrdinalType & aParticleIndex) const
    {
        assert(mCurrentConstraintValues.get() != nullptr);
        assert(aConstraintIndex < mCurrentConstraintValues->getNumVectors());
        return((*mCurrentConstraintValues)(aConstraintIndex, aParticleIndex));
    }

    /******************************************************************************//**
     * @brief Return current best constraint value
     * @param [in] aConstraintIndex constraint index
     * @param [in] aParticleIndex particle index
    **********************************************************************************/
    ScalarType getCurrentBestConstraintValue(const OrdinalType & aConstraintIndex, const OrdinalType & aParticleIndex) const
    {
        assert(mCurrentBestConstraintValues.get() != nullptr);
        assert(aConstraintIndex < mCurrentBestConstraintValues->getNumVectors());
        return((*mCurrentBestConstraintValues)(aConstraintIndex, aParticleIndex));
    }

    /******************************************************************************//**
     * @brief Return previous best constraint value
     * @param [in] aConstraintIndex constraint index
     * @param [in] aParticleIndex particle index
    **********************************************************************************/
    ScalarType getPreviousBestConstraintValue(const OrdinalType & aConstraintIndex, const OrdinalType & aParticleIndex) const
    {
        assert(mPreviousBestConstraintValues.get() != nullptr);
        assert(aConstraintIndex < mPreviousBestConstraintValues->getNumVectors());
        return((*mPreviousBestConstraintValues)(aConstraintIndex, aParticleIndex));
    }

    /******************************************************************************//**
     * @brief Return 1D container of current best constraint means
     * @return const reference to the 1D container of current best constraint means
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getMeanCurrentBestConstraintValues() const
    {
        assert(mMeanBestConstraintValues.get() != nullptr);
        return(*mMeanBestConstraintValues);
    }

    /******************************************************************************//**
     * @brief Return 1D container of current best constraint standard deviations
     * @return const reference of the 1D container of current best constraint standard deviations
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getStdDevCurrentBestConstraintValues() const
    {
        assert(mStdDevBestConstraintValues.get() != nullptr);
        return(*mStdDevBestConstraintValues);
    }

    /******************************************************************************//**
     * @brief Return 1D container of current global best constraint values
     * @return const reference to the 1D container of current global best constraint values
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getCurrentGlobalBestConstraintValues() const
    {
        assert(mCurrentGlobalBestConstraintValues.get() != nullptr);
        return(*mCurrentGlobalBestConstraintValues);
    }

    /******************************************************************************//**
     * @brief Return 1D container of current objective function values
     * @return const reference to the 1D container of current objective function values
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getCurrentObjFuncValues() const
    {
        assert(mCurrentObjFuncValues.get() != nullptr);
        return(*mCurrentObjFuncValues);
    }

    /******************************************************************************//**
     * @brief Return 1D container of previous objective function values
     * @return const reference to the 1D container of previous objective function values
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getPreviousBestObjFuncValues() const
    {
        assert(mPreviousBestObjFuncValues.get() != nullptr);
        return(*mPreviousBestObjFuncValues);
    }

    /******************************************************************************//**
     * @brief Return 1D container of current best objective function values
     * @return const reference to the 1D container of current best objective function values
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getCurrentBestObjFuncValues() const
    {
        assert(mCurrentBestObjFuncValues.get() != nullptr);
        return(*mCurrentBestObjFuncValues);
    }

    /******************************************************************************//**
     * @brief Return 1D container of current constraint values
     * @param [in] aIndex constraint index
     * @return const reference to the 1D container of current constraint values
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getCurrentConstraintValues(const OrdinalType & aIndex) const
    {
        assert(mCurrentConstraintValues.get() != nullptr);
        assert(aIndex < mCurrentConstraintValues->getNumVectors());
        return((*mCurrentConstraintValues)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Return 1D container of current best constraint values
     * @param [in] aIndex constraint index
     * @return const reference to the 1D container of current best constraint values
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getCurrentBestConstraintValues(const OrdinalType & aIndex) const
    {
        assert(mCurrentBestConstraintValues.get() != nullptr);
        assert(aIndex < mCurrentBestConstraintValues->getNumVectors());
        return((*mCurrentBestConstraintValues)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Return 1D container of previous best constraint values
     * @param [in] aIndex constraint index
     * @return const reference to the 1D container of previous best constraint values
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getPreviousBestConstraintValues(const OrdinalType & aIndex) const
    {
        assert(mPreviousBestConstraintValues.get() != nullptr);
        assert(aIndex < mPreviousBestConstraintValues->getNumVectors());
        return((*mPreviousBestConstraintValues)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Return 1D container of current penalty multipliers
     * @param [in] aIndex constraint index
     * @return const reference to the 1D container of current penalty multipliers
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getPenaltyMultipliers(const OrdinalType & aIndex) const
    {
        assert(mCurrentPenaltyMultipliers.get() != nullptr);
        assert(aIndex < mCurrentPenaltyMultipliers->getNumVectors());
        return((*mCurrentPenaltyMultipliers)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Return 1D container of current Lagrange multipliers
     * @param [in] aIndex constraint index
     * @return const reference to the 1D container of current Lagrange multipliers
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getLagrangeMultipliers(const OrdinalType & aIndex) const
    {
        assert(mCurrentLagrangeMultipliers.get() != nullptr);
        assert(aIndex < mCurrentLagrangeMultipliers->getNumVectors());
        return((*mCurrentLagrangeMultipliers)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Find current best particle positions
     * @param [in] aDataMng PSO data manager
    **********************************************************************************/
    void findBestParticlePositions(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        aDataMng.cacheGlobalBestObjFunValue();
        this->computeFeasibilityMeasure();
        this->updateBestParticlesData(aDataMng);
        aDataMng.findGlobalBestParticle();
    }

    /******************************************************************************//**
     * @brief Evaluate augmented Lagrangian objective function
     * @param [in] aDataMng PSO data manager
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Update current set of Lagrange multipliers
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Update current set of penalty multipliers
    **********************************************************************************/
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
                tPenaltyMultipliers[tParticleIndex] = std::min(tPenaltyMultipliers[tParticleIndex], mPenaltyMultiplierUpperBound);
            }
        }
    }

    /******************************************************************************//**
     * @brief Compute objective and constraint statistics
    **********************************************************************************/
    void computeCriteriaStatistics()
    {
        this->computeObjFuncStatistics();
        this->computeDualDataStatistics();
        this->computeConstraintStatistics();
    }

    /******************************************************************************//**
     * @brief Re-set elements in objective and constraint containers to initial guess
    **********************************************************************************/
    void restart()
    {
        this->initializeObjFuncData();
        this->initializeConstraintData();
    }

private:
    /******************************************************************************//**
     * @brief Initialize objective and constraint class member containers
    **********************************************************************************/
    void initialize()
    {
        this->initializeObjFuncData();
        this->initializeMultipliersData();
        this->initializeConstraintData();
    }

    /******************************************************************************//**
     * @brief Initialize elements in objective function containers to initial guess
    **********************************************************************************/
    void initializeObjFuncData()
    {
        mCurrentObjFuncValues->fill(std::numeric_limits<ScalarType>::max());
        mCurrentBestObjFuncValues->fill(std::numeric_limits<ScalarType>::max());
        mCurrentAugLagFuncValues->fill(std::numeric_limits<ScalarType>::max());
        mCurrentBestAugLagFuncValues->fill(std::numeric_limits<ScalarType>::max());
    }

    /******************************************************************************//**
     * @brief Initialize elements in constraint containers to initial guess
    **********************************************************************************/
    void initializeConstraintData()
    {
        mCurrentFeasibilityMeasure->fill(std::numeric_limits<ScalarType>::max());
        mPreviousFeasibilityMeasure->fill(std::numeric_limits<ScalarType>::max());
        mCurrentGlobalBestConstraintValues->fill(std::numeric_limits<ScalarType>::max());

        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            (*mCurrentConstraintValues)[tIndex].fill(std::numeric_limits<ScalarType>::max());
            (*mCurrentBestConstraintValues)[tIndex].fill(std::numeric_limits<ScalarType>::max());
        }
    }

    /******************************************************************************//**
     * @brief Allocate penalty and Lagrange multipliers containers
    ***********************************************************************************/
    void initializeMultipliersData()
    {
        Plato::fill(static_cast<ScalarType>(1), *mCurrentPenaltyMultipliers);
        Plato::fill(static_cast<ScalarType>(0), *mCurrentLagrangeMultipliers);

        const OrdinalType tNumConstraints = mConstraints->size();
        mMeanCurrentPenaltyMultipliers = std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(tNumConstraints);
        mStdDevCurrentPenaltyMultipliers = std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(tNumConstraints);
        mMeanCurrentLagrangeMultipliers = std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(tNumConstraints);
        mStdDevCurrentLagrangeMultipliers = std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(tNumConstraints);
    }

    /******************************************************************************//**
     * @brief Cache current constraint and objective values
    ***********************************************************************************/
    void cacheCriteriaValues()
    {
        mPreviousBestObjFuncValues->update(static_cast<ScalarType>(1), *mCurrentBestObjFuncValues, static_cast<ScalarType>(0));
        Plato::update(static_cast<ScalarType>(1), *mCurrentBestConstraintValues, static_cast<ScalarType>(0), *mPreviousBestConstraintValues);
    }

    /******************************************************************************//**
     * @brief Compute feasibility measure \f$ \max(h_i)\ \forall i=1,\dots,N_c\f$,
     *        where \f$ h \f$ denotes the constraint and \f$ N_c \f$ is the total
     *        number of constraints. The feasibility measure is computed for each
     *        particle.
    ***********************************************************************************/
    void computeFeasibilityMeasure()
    {
        mPreviousFeasibilityMeasure->update(static_cast<ScalarType>(1), *mCurrentFeasibilityMeasure, static_cast<ScalarType>(0));
        mCurrentFeasibilityMeasure->fill(static_cast<ScalarType>(0));

        const OrdinalType tNumConstraints = mConstraints->size();
        const OrdinalType tNumParticles = mCurrentBestObjFuncValues->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyConstraint = (*mCurrentConstraintValues)[tConstraintIndex];
            mCriteriaWorkVec->update(static_cast<ScalarType>(1), tMyConstraint, static_cast<ScalarType>(0));
            mCriteriaWorkVec->modulus();
            for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
            {
                const ScalarType tMyConstraintValue = (*mCriteriaWorkVec)[tParticleIndex];
                const ScalarType tMyFeasibilityMeasure = (*mCurrentFeasibilityMeasure)[tParticleIndex];
                (*mCurrentFeasibilityMeasure)[tParticleIndex] = std::max(tMyConstraintValue, tMyFeasibilityMeasure);
            }
        }
    }

    /******************************************************************************//**
     * @brief Update best particle positions and criteria values
     * @param [in,out] aDataMng PSO algorithm data manager
    ***********************************************************************************/
    void updateBestParticlesData(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const OrdinalType tNumParticles = aDataMng.getNumParticles();
        for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
        {
            const bool tAugLagFuncReduced =
                    (*mCurrentAugLagFuncValues)[tParticleIndex] < (*mCurrentBestAugLagFuncValues)[tParticleIndex];
            const bool tFeasibilityImproved =
                    (*mCurrentFeasibilityMeasure)[tParticleIndex] < (*mPreviousFeasibilityMeasure)[tParticleIndex];
            if(tAugLagFuncReduced == true && tFeasibilityImproved == true)
            //if(tAugLagFuncReduced == true)
            {
                const Plato::Vector<ScalarType, OrdinalType> & tCurrentParticle = aDataMng.getCurrentParticle(tParticleIndex);
                aDataMng.setBestParticlePosition(tParticleIndex, tCurrentParticle);
                this->updateBestCriteriaValues(tParticleIndex, aDataMng);
            }
        }
    }

    /******************************************************************************//**
     * @brief Update best particle objective and constraint value containers
     * @param [in] aParticleIndex particle index
     * @param [in] aDataMng PSO algorithm data manager
    ***********************************************************************************/
    void updateBestCriteriaValues(const OrdinalType & aParticleIndex,
                                  Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        (*mCurrentBestAugLagFuncValues)[aParticleIndex] = (*mCurrentAugLagFuncValues)[aParticleIndex];
        aDataMng.setCurrentBestObjFuncValue(aParticleIndex, (*mCurrentBestAugLagFuncValues)[aParticleIndex]);
        (*mCurrentBestObjFuncValues)[aParticleIndex] = (*mCurrentObjFuncValues)[aParticleIndex];
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            (*mCurrentBestConstraintValues)(tConstraintIndex, aParticleIndex) =
                    (*mCurrentConstraintValues)(tConstraintIndex, aParticleIndex);
        }
    }

    /******************************************************************************//**
     * @brief Compute current best objective function statistics
    ***********************************************************************************/
    void computeObjFuncStatistics()
    {
        mMeanCurrentBestObjFuncValue = Plato::mean(*mCriteriaReductions, *mCurrentBestObjFuncValues);
        mStdDevCurrentBestObjFuncValue =
                Plato::standard_deviation(mMeanCurrentBestObjFuncValue, *mCurrentBestObjFuncValues, *mCriteriaReductions);
        const ScalarType tValue = mCriteriaReductions->min(*mCurrentBestObjFuncValues);
        const bool tFoundNewGlobalBest = tValue < mCurrentGlobalBestObjFuncValue;
        mCurrentGlobalBestObjFuncValue = tFoundNewGlobalBest == true ? tValue : mCurrentGlobalBestObjFuncValue;
    }

    /******************************************************************************//**
     * @brief Compute current penalty and Lagrange multipliers statistics
    ***********************************************************************************/
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

    /******************************************************************************//**
     * @brief Compute current best constraint statistics
    ***********************************************************************************/
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

            const ScalarType tValue = mCriteriaReductions->min(tMyBestConstraintValues);
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
            Plato::GradFreeCriterion<ScalarType, OrdinalType> & tConstraint = (*mConstraints)[tIndex];
            Plato::Vector<ScalarType, OrdinalType> & tMyValues = (*mCurrentConstraintValues)[tIndex];
            tMyValues.fill(static_cast<ScalarType>(0));
            tConstraint.value(aControl, tMyValues);
        }
    }

    /******************************************************************************//**
     * @brief Evaluate augmented Lagrangian criterion:
     *
     *        \f$ \ell(z, \lambda, \mu) = f(z) + sum_{i=1}^{N}\lambda_i \theta_i(z)
     *        + sum_{i=1}^{N}\mu_i \theta_i^2(z) \f$, where \f$ \theta_i =
     *        \max( h_i(x), \frac{-\lambda_i}{2\mu_i} ) \f$
     *
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
    OrdinalType mNumAugLagFuncEval; /*!< number of augmented Lagrangian function evaluations */

    ScalarType mMeanCurrentBestObjFuncValue; /*!< mean of current best objective function values */
    ScalarType mStdDevCurrentBestObjFuncValue; /*!< standard deviation of current best objective function values */
    ScalarType mCurrentGlobalBestObjFuncValue; /*!< current global best objective function value */
    ScalarType mCurrentGlobalBestAugLagFuncValue; /*!< current global best augmented Lagrangian function value */

    ScalarType mPenaltyExpansionMultiplier; /*!< expansion multiplier for penalty multipliers */
    ScalarType mPenaltyMultiplierUpperBound; /*!< upper bound on penalty multipliers */
    ScalarType mPenaltyContractionMultiplier; /*!< contraction multiplier for penalty multipliers */
    ScalarType mFeasibilityInexactnessTolerance; /*!< feasibility inexactness tolerance */

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCriteriaWorkVec; /*!< feasibility inexactness tolerance */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentObjFuncValues; /*!< current objective function values */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentBestObjFuncValues; /*!< current best objective function values */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mPreviousBestObjFuncValues; /*!< previous best objective function values */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentAugLagFuncValues; /*!< current augmented Lagrangian function values */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentBestAugLagFuncValues; /*!< current best augmented Lagrangian function values */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mMeanBestConstraintValues; /*!< mean of current best constraint values */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentFeasibilityMeasure; /*!< current feasibility measure */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mPreviousFeasibilityMeasure; /*!< previous feasibility measure */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mStdDevBestConstraintValues; /*!< standard deviation of current best constraint values */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mMeanCurrentPenaltyMultipliers; /*!< mean of current penalty multipliers */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mStdDevCurrentPenaltyMultipliers; /*!< standard deviation of current penalty multipliers */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mMeanCurrentLagrangeMultipliers; /*!< mean of current Lagrange multipliers */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mStdDevCurrentLagrangeMultipliers; /*!< standard deviation of current Lagrange multipliers */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentGlobalBestConstraintValues;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentConstraintValues; /*!< current constraint values */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentPenaltyMultipliers; /*!< current penalty multipliers */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentLagrangeMultipliers; /*!< current Lagrange multipliers */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentBestConstraintValues; /*!< current best constraint values */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousBestConstraintValues; /*!< previous best constraint values */

    std::shared_ptr<Plato::GradFreeCriterion<ScalarType, OrdinalType>> mObjective; /*!< objective criterion interface */
    std::shared_ptr<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>> mConstraints; /*!< constraint criterion interface */
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mCriteriaReductions; /*!< interface to parallel reduction operations associated with a criterion */

private:
    AugmentedLagrangianStageMngPSO(const Plato::AugmentedLagrangianStageMngPSO<ScalarType, OrdinalType>&);
    Plato::AugmentedLagrangianStageMngPSO<ScalarType, OrdinalType> & operator=(const Plato::AugmentedLagrangianStageMngPSO<ScalarType, OrdinalType>&);
};
// class AugmentedLagrangianStageMngPSO

/******************************************************************************//**
 * @brief Core operations for the Particle Swarm Optimization (PSO) algorithms
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class ParticleSwarmOperations
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aFactory data factory for class member data
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~ParticleSwarmOperations()
    {
    }

    /******************************************************************************//**
     * @brief Return inertia multiplier
     * @return inertia multiplier
    **********************************************************************************/
    ScalarType getInertiaMultiplier() const
    {
        return (mInertiaMultiplier);
    }

    /******************************************************************************//**
     * @brief Return trust region multiplier
     * @return trust region multiplier
    **********************************************************************************/
    ScalarType getTrustRegionMultiplier() const
    {
        return (mTrustRegionMultiplier);
    }

    /******************************************************************************//**
     * @brief Set inertia multiplier
     * @param [in] aInput inertia multiplier
    **********************************************************************************/
    void setInertiaMultiplier(const ScalarType & aInput)
    {
        mInertiaMultiplier = aInput;
    }

    /******************************************************************************//**
     * @brief Set cognitive behavior multiplier
     * @param [in] aInput cognitive behavior multiplier
    **********************************************************************************/
    void setCognitiveBehaviorMultiplier(const ScalarType & aInput)
    {
        mCognitiveBehaviorMultiplier = aInput;
    }

    /******************************************************************************//**
     * @brief Set social behavior multiplier
     * @param [in] aInput social behavior multiplier
    **********************************************************************************/
    void setSocialBehaviorMultiplier(const ScalarType & aInput)
    {
        mSocialBehaviorMultiplier = aInput;
    }

    /******************************************************************************//**
     * @brief Set trust region expansion multiplier
     * @param [in] aInput trust region expansion multiplier
    **********************************************************************************/
    void setTrustRegionExpansionMultiplier(const ScalarType & aInput)
    {
        mTrustRegionExpansionMultiplier = aInput;
    }

    /******************************************************************************//**
     * @brief Set trust region contraction multiplier
     * @param [in] aInput trust region contraction multiplier
    **********************************************************************************/
    void setTrustRegionContractionMultiplier(const ScalarType & aInput)
    {
        mTrustRegionContractionMultiplier = aInput;
    }

    /******************************************************************************//**
     * @brief Set number of consecutive failures. A failure is defined as \f$ f(x_i)
     *        = f(x_{i-1}) \f$, where i denotes the optimization iteration and \f$ f
     *        \f$ is the objective function.
     * @param [in] aInput number of consecutive failures
    **********************************************************************************/
    void setNumConsecutiveFailures(const OrdinalType & aInput)
    {
        mNumConsecutiveFailures = aInput;
    }

    /******************************************************************************//**
     * @brief Set number of consecutive successes. A failure is defined as \f$ f(x_i)
     *        < f(x_{i-1}) \f$, where i denotes the optimization iteration and \f$ f
     *        \f$ is the objective function.
     * @param [in] aInput number of consecutive successes
    **********************************************************************************/
    void setNumConsecutiveSuccesses(const OrdinalType & aInput)
    {
        mNumConsecutiveSuccesses = aInput;
    }

    /******************************************************************************//**
     * @brief Set maximum number of consecutive failures. A failure is defined as
     *        \f$ f(x_i) = f(x_{i-1}) \f$, where i denotes the optimization iteration
     *        and \f$ f \f$ is the objective function.
     * @param [in] aInput maximum number of consecutive failures
    **********************************************************************************/
    void setMaxNumConsecutiveFailures(const OrdinalType & aInput)
    {
        mMaxNumConsecutiveFailures = aInput;
    }

    /******************************************************************************//**
     * @brief Set maximum number of consecutive successes. A failure is defined as
     *        \f$ f(x_i) = f(x_{i-1}) \f$, where i denotes the optimization iteration
     *        and \f$ f \f$ is the objective function.
     * @param [in] aInput maximum number of consecutive successes
    **********************************************************************************/
    void setMaxNumConsecutiveSuccesses(const OrdinalType & aInput)
    {
        mMaxNumConsecutiveSuccesses = aInput;
    }

    /******************************************************************************//**
     * @brief Check if inertia multiplier is between theoretical bounds
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Update trust region multiplier
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Update particle velocities and find global best particle velocity
     * @param [in] aDataMng PSO data manager
    **********************************************************************************/
    void updateParticleVelocities(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        aDataMng.cachePreviousVelocities();
        std::uniform_real_distribution<ScalarType> tDistribution(0.0 /* lower bound */, 1.0 /* upper bound */);

        const OrdinalType tMyProcID = aDataMng.getCommWrapper().myProcID();
        const OrdinalType tCurrentGlobalBestParticleRank = aDataMng.getCurrentGlobalBestParticleRank();
        const OrdinalType tCurrentGlobalBestParticleIndex = aDataMng.getCurrentGlobalBestParticleIndex();

        const OrdinalType tNumParticles = aDataMng.getNumParticles();
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            if( (tIndex == tCurrentGlobalBestParticleIndex) && (tMyProcID == tCurrentGlobalBestParticleRank) )
            {
                this->updateGlobalBestParticleVelocity(tDistribution, aDataMng);
            }
            else
            {
                this->updateParticleVelocity(tIndex, tDistribution, aDataMng);
            }
        }
    }

    /******************************************************************************//**
     * @brief Update particle positions and find global best particle position
     * @param [in] aDataMng PSO data manager
    **********************************************************************************/
    void updateParticlePositions(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const OrdinalType tMyProcID = aDataMng.getCommWrapper().myProcID();
        const OrdinalType tCurrentGlobalBestParticleRank = aDataMng.getCurrentGlobalBestParticleRank();
        const OrdinalType tCurrentGlobalBestParticleIndex = aDataMng.getCurrentGlobalBestParticleIndex();

        const OrdinalType tNumParticles = aDataMng.getNumParticles();
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            if( (tIndex == tCurrentGlobalBestParticleIndex) && (tMyProcID == tCurrentGlobalBestParticleRank) )
            {
                this->updateGlobalBestParticlePosition(aDataMng);
            }
            else
            {
                this->updateParticlePosition(tIndex, aDataMng);
            }
        }
    }

    /******************************************************************************//**
     * @brief Check success rate: Did the PSO algorithm find a new minimum in the last iteration?
     * @param [in] aDataMng PSO data manager
    **********************************************************************************/
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
    /******************************************************************************//**
     * @brief Allocate class member containers
     * @param [in] aFactory PSO data factory
    **********************************************************************************/
    void initialize(const Plato::DataFactory<ScalarType, OrdinalType> & aFactory)
    {
        const OrdinalType tPARTICLE_INDEX = 0;
        mControlWorkVector = aFactory.control(tPARTICLE_INDEX).create();
    }

    /******************************************************************************//**
     * @brief Update particle velocity
     * @param [in] aParticleIndex particle index
     * @param [in] aDistribution uniform distribution sample generator
     * @param [in] aDataMng PSO data manager
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Update global best particle velocity
     * @param [in] aDistribution uniform distribution sample generator
     * @param [in] aDataMng PSO data manager
    **********************************************************************************/
    void updateGlobalBestParticleVelocity(std::uniform_real_distribution<ScalarType> & aDistribution,
                                          Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
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

    /******************************************************************************//**
     * @brief Update particle position
     * @param [in] aParticleIndex particle index
     * @param [in] aDataMng PSO data manager
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Update global best particle position
     * @param [in] aDataMng PSO data manager
    **********************************************************************************/
    void updateGlobalBestParticlePosition(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const Plato::Vector<ScalarType, OrdinalType> & tLowerBounds = aDataMng.getLowerBounds();
        const Plato::Vector<ScalarType, OrdinalType> & tUpperBounds = aDataMng.getUpperBounds();
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
            (*mControlWorkVector)[tIndex] = std::max((*mControlWorkVector)[tIndex], tLowerBounds[tIndex]);
            (*mControlWorkVector)[tIndex] = std::min((*mControlWorkVector)[tIndex], tUpperBounds[tIndex]);
        }

        aDataMng.setCurrentParticle(tCurrentGlobalBestParticleIndex, *mControlWorkVector);
    }

private:
    std::default_random_engine mGenerator; /*!< random number generator */

    OrdinalType mNumConsecutiveFailures; /*!< number of consecutive failures, \f$ F(x_i) = F(x_{i-1}) \f$ */
    OrdinalType mNumConsecutiveSuccesses; /*!< number of consecutive successes, \f$ F(x_i) < F(x_{i-1}) \f$ */
    OrdinalType mMaxNumConsecutiveFailures; /*!< maximum number of consecutive failures, \f$ F(x_i) = F(x_{i-1}) \f$ */
    OrdinalType mMaxNumConsecutiveSuccesses; /*!< maximum number of consecutive successes, \f$ F(x_i) < F(x_{i-1}) \f$ */

    ScalarType mInertiaMultiplier; /*!< inertia multiplier */
    ScalarType mSocialBehaviorMultiplier; /*!< social behavior multiplier */
    ScalarType mCognitiveBehaviorMultiplier; /*!< cognitive behavior multiplier */

    ScalarType mTrustRegionMultiplier; /*!< trust region multiplier */
    ScalarType mTrustRegionExpansionMultiplier; /*!< trust region expansion multiplier */
    ScalarType mTrustRegionContractionMultiplier; /*!< trust region contraction multiplier */

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWorkVector; /*!< controls/particles work vector */

private:
    ParticleSwarmOperations(const Plato::ParticleSwarmOperations<ScalarType, OrdinalType>&);
    Plato::ParticleSwarmOperations<ScalarType, OrdinalType> & operator=(const Plato::ParticleSwarmOperations<ScalarType, OrdinalType>&);
};
// class ParticleSwarmOperations

/******************************************************************************//**
 * @brief Main interface to Bound Constrained Particle Swarm Optimization (BCPSO) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class BoundConstrainedPSO
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aFactory PSO algorithm data factory
     * @param [in] aObjective gradient free objective function interface
    **********************************************************************************/
    explicit BoundConstrainedPSO(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory,
                                 const std::shared_ptr<Plato::GradFreeCriterion<ScalarType, OrdinalType>> & aObjective) :
            mOutputDiagnostics(false),
            mNumIterations(0),
            mNumObjFuncEvals(0),
            mMaxNumIterations(1000),
            mMeanBestObjFuncTolerance(5e-4),
            mStdDevBestObjFuncTolerance(1e-6),
            mGlobalBestObjFuncTolerance(1e-10),
            mStopCriterion(Plato::particle_swarm::DID_NOT_CONVERGE),
            mDataMng(std::make_shared<Plato::ParticleSwarmDataMng<ScalarType, OrdinalType>>(aFactory)),
            mOperations(std::make_shared<Plato::ParticleSwarmOperations<ScalarType, OrdinalType>>(aFactory)),
            mStageMng(std::make_shared<Plato::BoundConstrainedStageMngPSO<ScalarType, OrdinalType>>(aFactory, aObjective))
    {
    }

    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aFactory PSO algorithm data factory
     * @param [in] aStageMng stage manager (i.e. core interface to criteria evaluations)
    **********************************************************************************/
    explicit BoundConstrainedPSO(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory,
                                 const std::shared_ptr<Plato::ParticleSwarmStageMng<ScalarType, OrdinalType>> & aStageMng) :
            mOutputDiagnostics(false),
            mNumIterations(0),
            mNumObjFuncEvals(0),
            mMaxNumIterations(1000),
            mMeanBestObjFuncTolerance(5e-4),
            mStdDevBestObjFuncTolerance(1e-6),
            mGlobalBestObjFuncTolerance(1e-10),
            mStopCriterion(Plato::particle_swarm::DID_NOT_CONVERGE),
            mDataMng(std::make_shared<Plato::ParticleSwarmDataMng<ScalarType, OrdinalType>>(aFactory)),
            mOperations(std::make_shared<Plato::ParticleSwarmOperations<ScalarType, OrdinalType>>(aFactory)),
            mStageMng(aStageMng)
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
        mOutputDiagnostics = true;
    }

    /******************************************************************************//**
     * @brief Compute best particle positions statistics
    **********************************************************************************/
    void computeCurrentBestParticlesStatistics()
    {
        mDataMng->computeCurrentBestParticlesStatistics();
    }

    /******************************************************************************//**
     * @brief Set maximum number of iterations
     * @param [in] aInput maximum number of iterations
    **********************************************************************************/
    void setMaxNumIterations(const OrdinalType & aInput)
    {
        mMaxNumIterations = aInput;
    }

    /******************************************************************************//**
     * @brief Set maximum number of consecutive failures
     * @param [in] aInput maximum number of consecutive failures
    **********************************************************************************/
    void setMaxNumConsecutiveFailures(const OrdinalType & aInput)
    {
        mOperations->setMaxNumConsecutiveFailures(aInput);
    }

    /******************************************************************************//**
     * @brief Set maximum number of consecutive successes
     * @param [in] aInput maximum number of consecutive successes
    **********************************************************************************/
    void setMaxNumConsecutiveSuccesses(const OrdinalType & aInput)
    {
        mOperations->setMaxNumConsecutiveSuccesses(aInput);
    }

    /******************************************************************************//**
     * @brief Set time step used to compute particle velocities
     * @param [in] aInput time step
    **********************************************************************************/
    void setTimeStep(const ScalarType & aInput)
    {
        mDataMng->setTimeStep(aInput);
    }

    /******************************************************************************//**
     * @brief Set stopping tolerance based on the mean of the best objective function values
     * @param [in] aInput stopping tolerance
    **********************************************************************************/
    void setMeanBestObjFuncTolerance(const ScalarType & aInput)
    {
        mMeanBestObjFuncTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set stopping tolerance based on the standard deviation of the best objective function values
     * @param [in] aInput stopping tolerance
    **********************************************************************************/
    void setStdDevBestObjFuncTolerance(const ScalarType & aInput)
    {
        mStdDevBestObjFuncTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set stopping tolerance based on global best objective function value
     * @param [in] aInput stopping tolerance
    **********************************************************************************/
    void setGlobalBestObjFuncTolerance(const ScalarType & aInput)
    {
        mGlobalBestObjFuncTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set inertia multiplier
     * @param [in] aInput inertia multiplier
    **********************************************************************************/
    void setInertiaMultiplier(const ScalarType & aInput)
    {
        mOperations->setInertiaMultiplier(aInput);
    }

    /******************************************************************************//**
     * @brief Set inertia multiplier
     * @param [in] aInput inertia multiplier
    **********************************************************************************/
    void setCognitiveBehaviorMultiplier(const ScalarType & aInput)
    {
        mOperations->setCognitiveBehaviorMultiplier(aInput);
    }

    /******************************************************************************//**
     * @brief Set social behavior multiplier
     * @param [in] aInput social behavior multiplier
    **********************************************************************************/
    void setSocialBehaviorMultiplier(const ScalarType & aInput)
    {
        mOperations->setSocialBehaviorMultiplier(aInput);
    }

    /******************************************************************************//**
     * @brief Set trust region expansion multiplier
     * @param [in] aInput trust region expansion multiplier
    **********************************************************************************/
    void setTrustRegionExpansionMultiplier(const ScalarType & aInput)
    {
        mOperations->setTrustRegionExpansionMultiplier(aInput);
    }

    /******************************************************************************//**
     * @brief Set trust region contraction multiplier
     * @param [in] aInput trust region contraction multiplier
    **********************************************************************************/
    void setTrustRegionContractionMultiplier(const ScalarType & aInput)
    {
        mOperations->setTrustRegionContractionMultiplier(aInput);
    }

    /******************************************************************************//**
     * @brief Set all particle position's upper bounds to input scalar
     * @param [in] aInput upper bounds
    **********************************************************************************/
    void setUpperBounds(const ScalarType & aInput)
    {
        mDataMng->setUpperBounds(aInput);
    }

    /******************************************************************************//**
     * @brief Set all particle position's lower bounds to input scalar
     * @param [in] aInput lower bounds
    **********************************************************************************/
    void setLowerBounds(const ScalarType & aInput)
    {
        mDataMng->setLowerBounds(aInput);
    }

    /******************************************************************************//**
     * @brief Set particle position's upper bounds
     * @param [in] aInput upper bounds
    **********************************************************************************/
    void setUpperBounds(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        mDataMng->setUpperBounds(aInput);
    }

    /******************************************************************************//**
     * @brief Set particle position's lower bounds
     * @param [in] aInput lower bounds
    **********************************************************************************/
    void setLowerBounds(const Plato::Vector<ScalarType, OrdinalType> & aInput)
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

    /******************************************************************************//**
     * @brief Return current number of iterations
     * @return current number of iterations
    **********************************************************************************/
    OrdinalType getNumIterations() const
    {
        return (mNumIterations);
    }

    /******************************************************************************//**
     * @brief Return number of objective function evaluations
     * @return number of objective function evaluations
    **********************************************************************************/
    OrdinalType getNumObjFuncEvals() const
    {
        return (mNumObjFuncEvals);
    }

    /******************************************************************************//**
     * @brief Return mean of current best objective function values
     * @return mean of best objective function values
    **********************************************************************************/
    ScalarType getMeanCurrentBestObjFuncValues() const
    {
        return (mDataMng->getMeanCurrentBestObjFuncValues());
    }

    /******************************************************************************//**
     * @brief Return mean of previous best objective function values
     * @return mean of previous best objective function values
    **********************************************************************************/
    ScalarType getMeanPreviousBestObjFuncValues() const
    {
        return (mDataMng->getMeanPreviousBestObjFuncValues());
    }

    /******************************************************************************//**
     * @brief Return standard deviation of current best objective function values
     * @return standard deviation of current best objective function values
    **********************************************************************************/
    ScalarType getStdDevCurrentBestObjFuncValues() const
    {
        return (mDataMng->getStdDevCurrentBestObjFuncValues());
    }

    /******************************************************************************//**
     * @brief Return current global best objective function value
     * @return current global best objective function value
    **********************************************************************************/
    ScalarType getCurrentGlobalBestObjFuncValue() const
    {
        return (mDataMng->getCurrentGlobalBestObjFuncValue());
    }

    /******************************************************************************//**
     * @brief Get current best particle positions
     * @param [out] aInput 1D container of current best particle positions
    **********************************************************************************/
    void getMeanCurrentBestParticlePositions(std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> & aInput) const
    {
        const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & tDataMng = this->getDataMng();
        const Plato::Vector<ScalarType, OrdinalType> & tMeanParticlePositions = tDataMng.getMeanParticlePositions();
        if(aInput.get() == nullptr)
        {
            aInput = tMeanParticlePositions.create();
        }
        assert(aInput->size() == tMeanParticlePositions.size());
        aInput->update(static_cast<ScalarType>(1), tMeanParticlePositions, static_cast<ScalarType>(0));
    }

    /******************************************************************************//**
     * @brief Get standard deviation associated with the current best particle positions
     * @param [out] aInput 1D container of the standard deviation of the current best particle positions
    **********************************************************************************/
    void getStdDevCurrentBestParticlePositions(std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> & aInput) const
    {
        const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & tDataMng = this->getDataMng();
        const Plato::Vector<ScalarType, OrdinalType> & tStdDevParticlePositions = tDataMng.getStdDevParticlePositions();
        if(aInput.get() == nullptr)
        {
            aInput = tStdDevParticlePositions.create();
        }
        assert(aInput->size() == tStdDevParticlePositions.size());
        aInput->update(static_cast<ScalarType>(1), tStdDevParticlePositions, static_cast<ScalarType>(0));
    }

    /******************************************************************************//**
     * @brief Get current global best particle positions
     * @param [out] aInput 1D container of the current global best particle positions
    **********************************************************************************/
    void getCurrentGlobalBestParticlePosition(std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> & aInput) const
    {
        const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & tDataMng = this->getDataMng();
        const Plato::Vector<ScalarType, OrdinalType> & tGlobalBestParticlePosition = tDataMng.getGlobalBestParticlePosition();
        if(aInput.get() == nullptr)
        {
            aInput = tGlobalBestParticlePosition.create();
        }
        assert(aInput->size() == tGlobalBestParticlePosition.size());
        aInput->update(static_cast<ScalarType>(1), tGlobalBestParticlePosition, static_cast<ScalarType>(0));
    }

    /******************************************************************************//**
     * @brief Return stopping criterion
     * @return stopping criterion string
    **********************************************************************************/
    std::string getStoppingCriterion() const
    {
        std::string tReason;
        Plato::pso::get_stop_criterion(mStopCriterion, tReason);
        return (tReason);
    }

    /******************************************************************************//**
     * @brief Return PSO data manager
     * @return const reference to PSO data manager
    **********************************************************************************/
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
                mDataMng->computeCurrentBestParticlesStatistics();
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

    /******************************************************************************//**
     * @brief Set initial particle positions and check user inputs
    **********************************************************************************/
    void initialize()
    {
        mOperations->checkInertiaMultiplier();
        mDataMng->setInitialParticles();
    }

private:
    /******************************************************************************//**
     * @brief Check stopping criteria
    **********************************************************************************/
    bool checkStoppingCriteria()
    {
        bool tStop = false;
        const ScalarType tMeanCurrentBestObjFunValue = mDataMng->getMeanCurrentBestObjFuncValues();
        const ScalarType tCurrentGlobalBestObjFunValue = mDataMng->getCurrentGlobalBestObjFuncValue();
        const ScalarType tStdDevCurrentBestObjFunValue = mDataMng->getStdDevCurrentBestObjFuncValues();

        if(mNumIterations >= mMaxNumIterations)
        {
            tStop = true;
            mStopCriterion = Plato::particle_swarm::MAX_NUMBER_ITERATIONS;
        }
        else if(tCurrentGlobalBestObjFunValue < mGlobalBestObjFuncTolerance)
        {
            tStop = true;
            mStopCriterion = Plato::particle_swarm::TRUE_OBJECTIVE_TOLERANCE;
        }
        else if(tMeanCurrentBestObjFunValue < mMeanBestObjFuncTolerance)
        {
            tStop = true;
            mStopCriterion = Plato::particle_swarm::MEAN_OBJECTIVE_TOLERANCE;
        }
        else if(tStdDevCurrentBestObjFunValue < mStdDevBestObjFuncTolerance)
        {
            tStop = true;
            mStopCriterion = Plato::particle_swarm::STDDEV_OBJECTIVE_TOLERANCE;
        }

        return (tStop);
    }

    /******************************************************************************//**
     * @brief Open output file (i.e. diagnostics file)
    **********************************************************************************/
    void openOutputFile()
    {
        if(mOutputDiagnostics == true)
        {
            const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
            if(tMyCommWrapper.myProcID() == 0)
            {
                mOutputStream.open("plato_pso_algorithm_diagnostics.txt");
                Plato::pso::print_bcpso_diagnostics_header(mOutputData, mOutputStream, mOutputDiagnostics);
            }
        }
    }

    /******************************************************************************//**
     * @brief Close output file (i.e. diagnostics file)
    **********************************************************************************/
    void closeOutputFile()
    {
        if(mOutputDiagnostics == true)
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
        if(mOutputDiagnostics == true)
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
        if(mOutputDiagnostics == false)
        {
            return;
        }

        const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
        if(tMyCommWrapper.myProcID() == 0)
        {
            this->cacheOutputData();
            Plato::pso::print_bcpso_diagnostics(mOutputData, mOutputStream, mOutputDiagnostics);
        }
    }

    /******************************************************************************//**
     * @brief Print diagnostics particle swarm constrained optimization algorithm.
    **********************************************************************************/
    void outputDiagnostics(std::ofstream & aOutputStream)
    {
        if(mOutputDiagnostics == false)
        {
            return;
        }

        const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
        if(tMyCommWrapper.myProcID() == 0)
        {
            this->cacheOutputData();
            Plato::pso::print_alpso_inner_diagnostics(mOutputData, aOutputStream, mOutputDiagnostics);
        }
    }

    /******************************************************************************//**
     * @brief Cache output/diagnostics data.
    **********************************************************************************/
    void cacheOutputData()
    {
        mNumObjFuncEvals = mNumIterations * mDataMng->getNumParticles();
        mOutputData.mNumIter = mNumIterations;
        mOutputData.mObjFuncCount = mNumObjFuncEvals;
        mOutputData.mCurrentGlobalBestObjFuncValue = mDataMng->getCurrentGlobalBestObjFuncValue();
        mOutputData.mMeanCurrentBestObjFuncValues = mDataMng->getMeanCurrentBestObjFuncValues();
        mOutputData.mStdDevCurrentBestObjFuncValues = mDataMng->getStdDevCurrentBestObjFuncValues();
        mOutputData.mTrustRegionMultiplier = mOperations->getTrustRegionMultiplier();
    }

private:
    bool mOutputDiagnostics; /*!< flag - print diagnostics (default = false) */
    std::ofstream mOutputStream; /*!< output stream for the algorithm's diagnostics */

    OrdinalType mNumIterations; /*!< current number of iterations */
    OrdinalType mNumObjFuncEvals; /*!< current number of objective function values */
    OrdinalType mMaxNumIterations; /*!< maximum number of iterations */

    ScalarType mMeanBestObjFuncTolerance; /*!< stopping tolerance on the mean of the best objective function values */
    ScalarType mStdDevBestObjFuncTolerance; /*!< stopping tolerance on the standard deviation of the best objective function values */
    ScalarType mGlobalBestObjFuncTolerance; /*!< stopping tolerance on global best objective function value */

    Plato::particle_swarm::stop_t mStopCriterion; /*!< stopping criterion enum */
    Plato::OutputDataPSO<ScalarType, OrdinalType> mOutputData; /*!< PSO algorithm output/diagnostics data structure */
    std::shared_ptr<Plato::ParticleSwarmDataMng<ScalarType, OrdinalType>> mDataMng; /*!< PSO algorithm data manager */
    std::shared_ptr<Plato::ParticleSwarmOperations<ScalarType, OrdinalType>> mOperations; /*!< interface to core PSO operations */
    std::shared_ptr<Plato::ParticleSwarmStageMng<ScalarType, OrdinalType>> mStageMng; /*!< interface to criteria evaluations/calls */

private:
    BoundConstrainedPSO(const Plato::BoundConstrainedPSO<ScalarType, OrdinalType>&);
    Plato::BoundConstrainedPSO<ScalarType, OrdinalType> & operator=(const Plato::BoundConstrainedPSO<ScalarType, OrdinalType>&);
};
// class BoundConstrainedPSO

/******************************************************************************//**
 * @brief Interface for Augmented Lagrangian Particle Swarm Optimization (ALPSO) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class AugmentedLagrangianPSO
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aFactory PSO algorithm data factory
     * @param [in] aObjective gradient free objective function interface
     * @param [in] aConstraints list of gradient free constraint interface
    **********************************************************************************/
    AugmentedLagrangianPSO(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory,
                           const std::shared_ptr<Plato::GradFreeCriterion<ScalarType, OrdinalType>> & aObjective,
                           const std::shared_ptr<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>> & aConstraints) :
            mOutputDiagnostics(false),
            mNumIterations(0),
            mMaxNumAugLagOuterIterations(1e3),
            mMeanBestAugLagFuncTolerance(5e-4),
            mStdDevBestAugLagFuncTolerance(1e-6),
            mGlobalBestAugLagFuncTolerance(1e-10),
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
        mOutputDiagnostics = true;
        mOptimizer->enableDiagnostics();
    }

    /******************************************************************************//**
     * @brief Set maximum number of outer iterations
     * @param [in] aInput maximum number of outer iterations
    **********************************************************************************/
    void setMaxNumOuterIterations(const OrdinalType & aInput)
    {
        mMaxNumAugLagOuterIterations = aInput;
    }

    /******************************************************************************//**
     * @brief Set maximum number of inner iterations
     * @param [in] aInput maximum number of inner iterations
    **********************************************************************************/
    void setMaxNumInnerIterations(const OrdinalType & aInput)
    {
        mOptimizer->setMaxNumIterations(aInput);
    }

    /******************************************************************************//**
     * @brief Set maximum number of consecutive failures
     * @param [in] aInput maximum number of consecutive failures
    **********************************************************************************/
    void setMaxNumConsecutiveFailures(const OrdinalType & aInput)
    {
        mOptimizer->setMaxNumConsecutiveFailures(aInput);
    }

    /******************************************************************************//**
     * @brief Set maximum number of consecutive successes
     * @param [in] aInput maximum number of consecutive successes
    **********************************************************************************/
    void setMaxNumConsecutiveSuccesses(const OrdinalType & aInput)
    {
        mOptimizer->setMaxNumConsecutiveSuccesses(aInput);
    }

    /******************************************************************************//**
     * @brief Set time step used to compute particle velocities
     * @param [in] aInput time step used to compute particle velocities
    **********************************************************************************/
    void setTimeStep(const ScalarType & aInput)
    {
        mOptimizer->setTimeStep(aInput);
    }

    /******************************************************************************//**
     * @brief Set inertia multiplier
     * @param [in] aInput inertia multiplier
    **********************************************************************************/
    void setInertiaMultiplier(const ScalarType & aInput)
    {
        mOptimizer->setInertiaMultiplier(aInput);
    }

    /******************************************************************************//**
     * @brief Set cognitive behavior multiplier
     * @param [in] aInput cognitive behavior multiplier
    **********************************************************************************/
    void setCognitiveBehaviorMultiplier(const ScalarType & aInput)
    {
        mOptimizer->setCognitiveBehaviorMultiplier(aInput);
    }

    /******************************************************************************//**
     * @brief Set social behavior multiplier
     * @param [in] aInput social behavior multiplier
    **********************************************************************************/
    void setSocialBehaviorMultiplier(const ScalarType & aInput)
    {
        mOptimizer->setSocialBehaviorMultiplier(aInput);
    }

    /******************************************************************************//**
     * @brief Set penalty expansion multiplier
     * @param [in] aInput penalty expansion multiplier
    **********************************************************************************/
    void setPenaltyExpansionMultiplier(const ScalarType & aInput)
    {
        mStageMng->setPenaltyExpansionMultiplier(aInput);
    }

    /******************************************************************************//**
     * @brief Set penalty contraction multiplier
     * @param [in] aInput penalty contraction multiplier
    **********************************************************************************/
    void setPenaltyContractionMultiplier(const ScalarType & aInput)
    {
        mStageMng->setPenaltyContractionMultiplier(aInput);
    }

    /******************************************************************************//**
     * @brief Set upper bound on penalty multipliers
     * @param [in] aInput upper bound on penalty multipliers
    **********************************************************************************/
    void setPenaltyMultiplierUpperBound(const ScalarType & aInput)
    {
        mStageMng->setPenaltyMultiplierUpperBound(aInput);
    }

    /******************************************************************************//**
     * @brief Set feasibility inexactness tolerance
     * @param [in] aInput feasibility inexactness tolerance
    **********************************************************************************/
    void setFeasibilityInexactnessTolerance(const ScalarType & aInput)
    {
        mStageMng->setFeasibilityInexactnessTolerance(aInput);
    }

    /******************************************************************************//**
     * @brief Set stopping tolerance on the mean of the best augmented Lagrangian function value
     * @param [in] aInput stopping tolerance
    **********************************************************************************/
    void setMeanAugLagFuncTolerance(const ScalarType & aInput)
    {
        mMeanBestAugLagFuncTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set stopping tolerance on the standard deviation of the best augmented Lagrangian function value
     * @param [in] aInput stopping tolerance
    **********************************************************************************/
    void setStdDevAugLagFuncTolerance(const ScalarType & aInput)
    {
        mStdDevBestAugLagFuncTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set stopping tolerance on the global best augmented Lagrangian function value
     * @param [in] aInput stopping tolerance
    **********************************************************************************/
    void setBestAugLagFuncTolerance(const ScalarType & aInput)
    {
        mGlobalBestAugLagFuncTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set trust region expansion multiplier
     * @param [in] aInput trust region expansion multiplier
    **********************************************************************************/
    void setTrustRegionExpansionMultiplier(const ScalarType & aInput)
    {
        mOptimizer->setTrustRegionExpansionMultiplier(aInput);
    }

    /******************************************************************************//**
     * @brief Set trust region contraction multiplier
     * @param [in] aInput trust region contraction multiplier
    **********************************************************************************/
    void setTrustRegionContractionMultiplier(const ScalarType & aInput)
    {
        mOptimizer->setTrustRegionContractionMultiplier(aInput);
    }

    /******************************************************************************//**
     * @brief Set all upper bounds on particle positions to input scalar
     * @param [in] aInput upper bound
    **********************************************************************************/
    void setUpperBounds(const ScalarType & aInput)
    {
        mOptimizer->setUpperBounds(aInput);
    }

    /******************************************************************************//**
     * @brief Set all lower bounds on particle positions to input scalar
     * @param [in] aInput lower bound
    **********************************************************************************/
    void setLowerBounds(const ScalarType & aInput)
    {
        mOptimizer->setLowerBounds(aInput);
    }

    /******************************************************************************//**
     * @brief Set upper bounds on particle positions
     * @param [in] aInput upper bounds
    **********************************************************************************/
    void setUpperBounds(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        mOptimizer->setUpperBounds(aInput);
    }

    /******************************************************************************//**
     * @brief Set lower bounds on particle positions
     * @param [in] aInput lower bounds
    **********************************************************************************/
    void setLowerBounds(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        mOptimizer->setLowerBounds(aInput);
    }

    /******************************************************************************//**
     * @brief Return current number of iterations
     * @return current number of iterations
    **********************************************************************************/
    OrdinalType getNumIterations() const
    {
        return (mNumIterations);
    }

    /******************************************************************************//**
     * @brief Return number of constraints
     * @return number of constraints
    **********************************************************************************/
    OrdinalType getNumConstraints() const
    {
        return (mStageMng->getNumConstraints());
    }

    /******************************************************************************//**
     * @brief Return current number of augmented Lagrangian function evaluations
     * @return current number of augmented Lagrangian function evaluations
    **********************************************************************************/
    OrdinalType getNumAugLagFuncEvaluations() const
    {
        return (mStageMng->getNumAugLagFuncEvaluations());
    }

    /******************************************************************************//**
     * @brief Return mean of current best augmented Lagrangian function values
     * @return mean of current best augmented Lagrangian function values
    **********************************************************************************/
    ScalarType getMeanCurrentBestAugLagValues() const
    {
        return (mOptimizer->getMeanCurrentBestObjFuncValues());
    }

    /******************************************************************************//**
     * @brief Return standard deviation of current best augmented Lagrangian function values
     * @return standard deviation of current best augmented Lagrangian function values
    **********************************************************************************/
    ScalarType getStdDevCurrentBestAugLagValues() const
    {
        return (mOptimizer->getStdDevCurrentBestObjFuncValues());
    }

    /******************************************************************************//**
     * @brief Return current global best augmented Lagrangian function value
     * @return current global best augmented Lagrangian function value
    **********************************************************************************/
    ScalarType getCurrentGlobalBestAugLagValue() const
    {
        return (mOptimizer->getCurrentGlobalBestObjFuncValue());
    }

    /******************************************************************************//**
     * @brief Return mean of current best values for this constraint
     * @param [in] constraint index
     * @return mean of current best values for this constraint
    **********************************************************************************/
    ScalarType getMeanCurrentBestConstraintValues(const OrdinalType & aIndex) const
    {
        return (mStageMng->getMeanCurrentBestConstraintValues(aIndex));
    }

    /******************************************************************************//**
     * @brief Return standard deviation of current best values for this constraint
     * @param [in] constraint index
     * @return standard deviation of current best values for this constraint
    **********************************************************************************/
    ScalarType getStdDevCurrentBestConstraintValues(const OrdinalType & aIndex) const
    {
        return (mStageMng->getStdDevCurrentBestConstraintValues(aIndex));
    }

    /******************************************************************************//**
     * @brief Return current global best values for this constraint
     * @param [in] constraint index
     * @return current global best values for this constraint
    **********************************************************************************/
    ScalarType getCurrentGlobalBestConstraintValue(const OrdinalType & aIndex) const
    {
        return (mStageMng->getCurrentGlobalBestConstraintValue(aIndex));
    }

    /******************************************************************************//**
     * @brief Get mean current best particle positions
     * @param [out] aInput mean current best particle positions
    **********************************************************************************/
    void getMeanCurrentBestParticlePositions(std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> & aInput) const
    {
        const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & tDataMng = this->getDataMng();
        const Plato::Vector<ScalarType, OrdinalType> & tMeanParticlePositions = tDataMng.getMeanParticlePositions();
        if(aInput.get() == nullptr)
        {
            aInput = tMeanParticlePositions.create();
        }
        assert(aInput->size() == tMeanParticlePositions.size());
        aInput->update(static_cast<ScalarType>(1), tMeanParticlePositions, static_cast<ScalarType>(0));
    }

    /******************************************************************************//**
     * @brief Get standard deviation current best particle positions
     * @param [out] aInput standard deviation current best particle positions
    **********************************************************************************/
    void getStdDevCurrentBestParticlePositions(std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> & aInput) const
    {
        const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & tDataMng = this->getDataMng();
        const Plato::Vector<ScalarType, OrdinalType> & tStdDevParticlePositions = tDataMng.getStdDevParticlePositions();
        if(aInput.get() == nullptr)
        {
            aInput = tStdDevParticlePositions.create();
        }
        assert(aInput->size() == tStdDevParticlePositions.size());
        aInput->update(static_cast<ScalarType>(1), tStdDevParticlePositions, static_cast<ScalarType>(0));
    }

    /******************************************************************************//**
     * @brief Get current global best particle positions
     * @param [out] aInput global best particle positions
    **********************************************************************************/
    void getCurrentGlobalBestParticlePosition(std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> & aInput) const
    {
        const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & tDataMng = this->getDataMng();
        const Plato::Vector<ScalarType, OrdinalType> & tGlobalBestParticlePosition = tDataMng.getGlobalBestParticlePosition();
        if(aInput.get() == nullptr)
        {
            aInput = tGlobalBestParticlePosition.create();
        }
        assert(aInput->size() == tGlobalBestParticlePosition.size());
        aInput->update(static_cast<ScalarType>(1), tGlobalBestParticlePosition, static_cast<ScalarType>(0));
    }

    /******************************************************************************//**
     * @brief Get mean current best constraint values
     * @param [out] aInput mean current best constraint values
    **********************************************************************************/
    void getMeanCurrentBestConstraintValues(std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> & aInput) const
    {
        const Plato::Vector<ScalarType, OrdinalType> & tMeanConstraintValues = mStageMng->getMeanCurrentBestConstraintValues();
        if(aInput.get() == nullptr)
        {
            aInput = tMeanConstraintValues.create();
        }
        assert(aInput->size() == tMeanConstraintValues.size());
        aInput->update(static_cast<ScalarType>(1), tMeanConstraintValues, static_cast<ScalarType>(0));
    }

    /******************************************************************************//**
     * @brief Get standard deviation current best constraint values
     * @param [out] aInput standard deviation current best constraint values
    **********************************************************************************/
    void getStdDevCurrentBestConstraintValues(std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> & aInput) const
    {
        const Plato::Vector<ScalarType, OrdinalType> & tStdDevConstraintValues = mStageMng->getStdDevCurrentBestConstraintValues();
        if(aInput.get() == nullptr)
        {
            aInput = tStdDevConstraintValues.create();
        }
        assert(aInput->size() == tStdDevConstraintValues.size());
        aInput->update(static_cast<ScalarType>(1), tStdDevConstraintValues, static_cast<ScalarType>(0));
    }

    /******************************************************************************//**
     * @brief Get current global best constraint values
     * @param [out] aInput current global best constraint values
    **********************************************************************************/
    void getCurrentGlobalBestConstraintValues(std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> & aInput) const
    {
        const Plato::Vector<ScalarType, OrdinalType> & tGlobalBestConstraintValues = mStageMng->getCurrentGlobalBestConstraintValues();
        if(aInput.get() == nullptr)
        {
            aInput = tGlobalBestConstraintValues.create();
        }
        assert(aInput->size() == tGlobalBestConstraintValues.size());
        aInput->update(static_cast<ScalarType>(1), tGlobalBestConstraintValues, static_cast<ScalarType>(0));
    }


    /******************************************************************************//**
     * @brief Return stopping criterion
     * @return stopping criterion string
    **********************************************************************************/
    std::string getStoppingCriterion() const
    {
        std::string tReason;
        Plato::pso::get_stop_criterion(mStopCriterion, tReason);
        return (tReason);
    }


    /******************************************************************************//**
     * @brief Return PSO stage manager
     * @return non-const reference to PSO stage manager
    **********************************************************************************/
    Plato::ParticleSwarmStageMng<ScalarType, OrdinalType> & getStageMng()
    {
        return (*mStageMng);
    }


    /******************************************************************************//**
     * @brief Return PSO data manager
     * @return const reference to PSO data manager
    **********************************************************************************/
    const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & getDataMng() const
    {
        return (mOptimizer->getDataMng());
    }

    /******************************************************************************//**
     * @brief Solve constrained particle swarm optimization problem
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
                mOptimizer->computeCurrentBestParticlesStatistics();
                this->outputStoppingCriterion();
                this->closeOutputFile();
                break;
            }
            mStageMng->restart();
        }
    }

private:
    /******************************************************************************//**
     * @brief Initialize and allocate class member data
    **********************************************************************************/
    void initialize()
    {
        mOptimizer->initialize();
        const OrdinalType tNumConstraints = mStageMng->getNumConstraints();
        mOptimizer->setNumConstraints(tNumConstraints);
    }

    /******************************************************************************//**
     * @brief Check stopping criteria
     * @return flag (true = stop : false = continue)
    **********************************************************************************/
    bool checkStoppingCriteria()
    {
        bool tStop = false;
        const ScalarType tMeanCurrentBestObjFuncValue = mOptimizer->getMeanCurrentBestObjFuncValues();
        const ScalarType tCurrentGlobalBestAugLagFuncValue = mOptimizer->getCurrentGlobalBestObjFuncValue();
        const ScalarType tStdDevCurrentBestAugLagFuncValue = mOptimizer->getStdDevCurrentBestObjFuncValues();

        if(mNumIterations >= mMaxNumAugLagOuterIterations)
        {
            tStop = true;
            mStopCriterion = Plato::particle_swarm::MAX_NUMBER_ITERATIONS;
        }
        else if(tCurrentGlobalBestAugLagFuncValue < mGlobalBestAugLagFuncTolerance)
        {
            tStop = true;
            mStopCriterion = Plato::particle_swarm::TRUE_OBJECTIVE_TOLERANCE;
        }
        else if(tMeanCurrentBestObjFuncValue < mMeanBestAugLagFuncTolerance)
        {
            tStop = true;
            mStopCriterion = Plato::particle_swarm::MEAN_OBJECTIVE_TOLERANCE;
        }
        else if(tStdDevCurrentBestAugLagFuncValue < mStdDevBestAugLagFuncTolerance)
        {
            tStop = true;
            mStopCriterion = Plato::particle_swarm::STDDEV_OBJECTIVE_TOLERANCE;
        }

        return (tStop);
    }

    /******************************************************************************//**
     * @brief Open output file (i.e. diagnostics file)
    **********************************************************************************/
    void openOutputFile()
    {
        if(mOutputDiagnostics == true)
        {
            const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & tDataMng = mOptimizer->getDataMng();
            const Plato::CommWrapper& tMyCommWrapper = tDataMng.getCommWrapper();
            if(tMyCommWrapper.myProcID() == 0)
            {
                mOutputStream.open("plato_alpso_algorithm_diagnostics.txt");
                Plato::pso::print_alpso_diagnostics_header(mOutputData, mOutputStream, mOutputDiagnostics);
            }
        }
    }

    /******************************************************************************//**
     * @brief Close output file (i.e. diagnostics file)
    **********************************************************************************/
    void closeOutputFile()
    {
        if(mOutputDiagnostics == true)
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
        if(mOutputDiagnostics == true)
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
        if(mOutputDiagnostics == false)
        {
            return;
        }

        const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & tDataMng = mOptimizer->getDataMng();
        const Plato::CommWrapper& tMyCommWrapper = tDataMng.getCommWrapper();
        if(tMyCommWrapper.myProcID() == 0)
        {
            this->cacheObjFuncOutputData();
            this->cacheConstraintOutputData();
            Plato::pso::print_alpso_outer_diagnostics(mOutputData, mOutputStream, mOutputDiagnostics);
        }
    }

    /******************************************************************************//**
     * @brief Cache diagnostic data for the objective function
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Cache diagnostic data for all constraints
    **********************************************************************************/
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
    bool mOutputDiagnostics; /*!< flag - print diagnostics (default = false) */
    std::ofstream mOutputStream; /*!< output stream for the algorithm's diagnostics */

    OrdinalType mNumIterations; /*!< current number of iterations */
    OrdinalType mMaxNumAugLagOuterIterations; /*!< maximum number of outer iterations */

    ScalarType mMeanBestAugLagFuncTolerance; /*!< stopping tolerance on the mean of the best augmented Lagrangian function values */
    ScalarType mStdDevBestAugLagFuncTolerance; /*!< stopping tolerance on the standard deviation of the best augmented Lagrangian function values */
    ScalarType mGlobalBestAugLagFuncTolerance; /*!< stopping tolerance on global best augmented Lagrangian function value */

    Plato::particle_swarm::stop_t mStopCriterion; /*!< stopping criterion enum */
    Plato::OutputDataALPSO<ScalarType, OrdinalType> mOutputData; /*!< ALPSO algorithm output/diagnostics data structure */

    std::shared_ptr<Plato::AugmentedLagrangianStageMngPSO<ScalarType, OrdinalType>> mStageMng; /*!< stage manager interface for ALPSO */
    std::shared_ptr<Plato::BoundConstrainedPSO<ScalarType, OrdinalType>> mOptimizer; /*!< interface to BCPSO algorithm */

private:
    AugmentedLagrangianPSO(const Plato::AugmentedLagrangianPSO<ScalarType, OrdinalType>&);
    Plato::AugmentedLagrangianPSO<ScalarType, OrdinalType> & operator=(const Plato::AugmentedLagrangianPSO<ScalarType, OrdinalType>&);
};
// class AugmentedLagrangianPSO

/******************************************************************************//**
 * @brief Inputs from augmented Lagrangian Particle Swarm Optimization (ALPSO) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct AlgorithmOutputsALPSO
{
    std::string mStopCriterion; /*!< stopping criterion */

    OrdinalType mNumOuterIter; /*!< number of outer iterations */
    OrdinalType mNumAugLagFuncEval; /*!< number of augmented Lagrangian function evaluations */

    ScalarType mMeanBestAugLagFuncValue; /*!< augmented Lagrangian function mean from best particle set */
    ScalarType mStdDevBestAugLagFuncValue; /*!< augmented Lagrangian function standard deviation from best particle set */
    ScalarType mGlobalBestAugLagFuncValue; /*!< global best augmented Lagrangian function value */

    std::shared_ptr<Plato::Vector<ScalarType,OrdinalType>> mMeanBestParticles; /*!< particle dimension's standard deviation values from best particle set */
    std::shared_ptr<Plato::Vector<ScalarType,OrdinalType>> mStdDevBestParticles; /*!< particle dimension's mean values from best particle set */
    std::shared_ptr<Plato::Vector<ScalarType,OrdinalType>> mGlobalBestParticles; /*!< global best particle dimension's */

    std::shared_ptr<Plato::Vector<ScalarType,OrdinalType>> mMeanBestConstraintValues; /*!< constraint mean values from best particle set */
    std::shared_ptr<Plato::Vector<ScalarType,OrdinalType>> mStdDevBestConstraintValues; /*!< constraint standard deviation values from best particle set */
    std::shared_ptr<Plato::Vector<ScalarType,OrdinalType>> mGlobalBestConstraintValues; /*!< global best constraint values */
};
// struct AlgorithmOutputsALPSO

/******************************************************************************//**
 * @brief Inputs for augmented Lagrangian Particle Swarm Optimization (ALPSO) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct AlgorithmInputsALPSO
{
    /******************************************************************************//**
     * @brief Default constructor
     **********************************************************************************/
    AlgorithmInputsALPSO() :
            mOutputDiagnostics(false),
            mMaxNumOuterIter(1e3),
            mMaxNumInnerIter(5),
            mMaxNumConsecutiveFailures(10),
            mMaxNumConsecutiveSuccesses(10),
            mTimeStep(1),
            mMeanBestAugLagFuncTolerance(5e-4),
            mStdDevBestAugLagFuncTolerance(1e-6),
            mGlobalBestAugLagFuncTolerance(1e-10),
            mInertiaMultiplier(0.9),
            mSocialBehaviorMultiplier(0.8),
            mCognitiveBehaviorMultiplier(0.8),
            mPenaltyExpansionMultiplier(2),
            mPenaltyMultiplierUpperBound(100),
            mPenaltyContractionMultiplier(0.5),
            mFeasibilityInexactnessTolerance(1e-4),
            mTrustRegionExpansionMultiplier(4.0),
            mTrustRegionContractionMultiplier(0.75),
            mMemorySpace(Plato::MemorySpace::HOST),
            mCriteriaEvals(),
            mParticlesLowerBounds(),
            mParticlesUpperBounds(),
            mDual(),
            mParticles(),
            mControlReductions(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>()),
            mCriteriaReductions(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>())
    {
        mCommWrapper.useDefaultComm();
    }

    /******************************************************************************//**
     * @brief Default destructor
     **********************************************************************************/
    ~AlgorithmInputsALPSO()
    {
    }

    bool mOutputDiagnostics; /*!< flag to enable problem statistics output (default=false) */

    OrdinalType mMaxNumOuterIter; /*!< maximum number of outer iterations */
    OrdinalType mMaxNumInnerIter; /*!< maximum number of augmented Lagrangian subproblem iterations */
    OrdinalType mMaxNumConsecutiveFailures; /*!< maximum number of consecutive failures, global best F(x_{i+1}) == F(x_{i}) */
    OrdinalType mMaxNumConsecutiveSuccesses; /*!< maximum number of consecutive successes, global best F(x_{i+1}) < F(x_{i}) */

    ScalarType mTimeStep; /*!< time step \Delta{t} */
    ScalarType mMeanBestAugLagFuncTolerance; /*!< mean augmented Lagrangian function stopping tolerance */
    ScalarType mStdDevBestAugLagFuncTolerance; /*!< standard deviation stopping tolerance */
    ScalarType mGlobalBestAugLagFuncTolerance; /*!< best augmented Lagrangian function stopping tolerance */
    ScalarType mInertiaMultiplier; /*!< inertia multiplier */
    ScalarType mSocialBehaviorMultiplier; /*!< social behavior multiplier */
    ScalarType mCognitiveBehaviorMultiplier; /*!< cognite behavior multiplier */
    ScalarType mPenaltyExpansionMultiplier; /*!< penalty expansion multiplier */
    ScalarType mPenaltyMultiplierUpperBound; /*!< upper bound on penalty multiplier */
    ScalarType mPenaltyContractionMultiplier; /*!< penalty contraction multiplier */
    ScalarType mFeasibilityInexactnessTolerance; /*!< feasibility inexactness tolerance */
    ScalarType mTrustRegionExpansionMultiplier; /*!< trust region expansion multiplier */
    ScalarType mTrustRegionContractionMultiplier; /*!< trust region contraction multiplier */

    Plato::CommWrapper mCommWrapper; /*!< distributed memory communication wrapper */
    Plato::MemorySpace::type_t mMemorySpace; /*!< memory space: HOST (default) OR DEVICE */

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCriteriaEvals; /*!< criteria evaluations */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mParticlesLowerBounds; /*!< particles' lower bounds */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mParticlesUpperBounds; /*!< particles' upper bounds */

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mDual; /*!< Lagrange multipliers */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mParticles; /*!< particles */

    /*!< operations which require communication across processors, e.g. max, min, global sum */
    std::shared_ptr<Plato::ReductionOperations<ScalarType,OrdinalType>> mControlReductions;
    std::shared_ptr<Plato::ReductionOperations<ScalarType,OrdinalType>> mCriteriaReductions;
};
// AlgorithmInputsALPSO

/******************************************************************************//**
 * @brief Set augmented Lagrangian Particle Swarm Optimization (ALPSO) algorithm inputs
 * @param [in] aInputs inputs for ALPSO algorithm
 * @param [in,out] aAlgorithm ALPSO algorithm interface
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void set_alpso_algorithm_inputs(const Plato::AlgorithmInputsALPSO<ScalarType,OrdinalType> & aInputs,
                                       Plato::AugmentedLagrangianPSO<ScalarType,OrdinalType> & aAlgorithm)
{
    if(aInputs.mOutputDiagnostics == true)
    {
        aAlgorithm.enableDiagnostics();
    }

    aAlgorithm.setMaxNumOuterIterations(aInputs.mMaxNumOuterIter);
    aAlgorithm.setMaxNumInnerIterations(aInputs.mMaxNumInnerIter);
    aAlgorithm.setMaxNumConsecutiveFailures(aInputs.mMaxNumConsecutiveFailures);
    aAlgorithm.setMaxNumConsecutiveSuccesses(aInputs.mMaxNumConsecutiveSuccesses);

    aAlgorithm.setTimeStep(aInputs.mTimeStep);
    aAlgorithm.setLowerBounds(*aInputs.mParticlesLowerBounds);
    aAlgorithm.setUpperBounds(*aInputs.mParticlesUpperBounds);
    aAlgorithm.setInertiaMultiplier(aInputs.mInertiaMultiplier);
    aAlgorithm.setMeanAugLagFuncTolerance(aInputs.mMeanBestAugLagFuncTolerance);
    aAlgorithm.setSocialBehaviorMultiplier(aInputs.mSocialBehaviorMultiplier);
    aAlgorithm.setStdDevAugLagFuncTolerance(aInputs.mStdDevBestAugLagFuncTolerance);
    aAlgorithm.setBestAugLagFuncTolerance(aInputs.mGlobalBestAugLagFuncTolerance);
    aAlgorithm.setPenaltyExpansionMultiplier(aInputs.mPenaltyExpansionMultiplier);
    aAlgorithm.setPenaltyMultiplierUpperBound(aInputs.mPenaltyMultiplierUpperBound);
    aAlgorithm.setCognitiveBehaviorMultiplier(aInputs.mCognitiveBehaviorMultiplier);
    aAlgorithm.setPenaltyContractionMultiplier(aInputs.mPenaltyContractionMultiplier);
    aAlgorithm.setTrustRegionExpansionMultiplier(aInputs.mTrustRegionExpansionMultiplier);
    aAlgorithm.setFeasibilityInexactnessTolerance(aInputs.mFeasibilityInexactnessTolerance);
    aAlgorithm.setTrustRegionContractionMultiplier(aInputs.mTrustRegionContractionMultiplier);
}
// function set_alpso_algorithm_inputs

/******************************************************************************//**
 * @brief Set Augmented Lagrangian Particle Swarm Optimization (ALPSO) algorithm outputs
 * @param [in] aAlgorithm ALPSO algorithm interface
 * @param [in,out] aOutputs outputs from ALPSO algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void set_alpso_algorithm_outputs(const Plato::AugmentedLagrangianPSO<ScalarType, OrdinalType> & aAlgorithm,
                                        Plato::AlgorithmOutputsALPSO<ScalarType, OrdinalType> & aOutputs)
{
    aOutputs.mStopCriterion = aAlgorithm.getStoppingCriterion();

    aOutputs.mNumOuterIter = aAlgorithm.getNumIterations();
    aOutputs.mNumAugLagFuncEval = aAlgorithm.getNumAugLagFuncEvaluations();

    // AUGMENTED LAGRANGIAN FUNCTION DIAGNOSTICS
    aOutputs.mMeanBestAugLagFuncValue = aAlgorithm.getMeanCurrentBestAugLagValues();
    aOutputs.mGlobalBestAugLagFuncValue = aAlgorithm.getCurrentGlobalBestAugLagValue();
    aOutputs.mStdDevBestAugLagFuncValue = aAlgorithm.getStdDevCurrentBestAugLagValues();

    // PARTILCES DIAGNOSTICS
    aAlgorithm.getMeanCurrentBestParticlePositions(aOutputs.mMeanBestParticles);
    aAlgorithm.getStdDevCurrentBestParticlePositions(aOutputs.mStdDevBestParticles);
    aAlgorithm.getCurrentGlobalBestParticlePosition(aOutputs.mGlobalBestParticles);

    // CONSTRAINTS DIAGNOSTICS
    aAlgorithm.getMeanCurrentBestConstraintValues(aOutputs.mMeanBestConstraintValues);
    aAlgorithm.getStdDevCurrentBestConstraintValues(aOutputs.mStdDevBestConstraintValues);
    aAlgorithm.getCurrentGlobalBestConstraintValues(aOutputs.mGlobalBestConstraintValues);
}
// function set_alpso_algorithm_outputs

/******************************************************************************//**
 * @brief Augmented Lagrangian Particle Swarm Optimization (ALPSO) algorithm light interface
 * @param [in] aObjective user-defined grad-free objective function
 * @param [in] aConstraints user-defined list of grad-free constraints
 * @param [in] aInputs ALPSO algorithm inputs
 * @param [in,out] aOutputs ALPSO algorithm outputs
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void solve_alpso(const std::shared_ptr<Plato::GradFreeCriterion<ScalarType, OrdinalType>> & aObjective,
                        const std::shared_ptr<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>> & aConstraints,
                        const Plato::AlgorithmInputsALPSO<ScalarType, OrdinalType> & aInputs,
                        Plato::AlgorithmOutputsALPSO<ScalarType, OrdinalType> & aOutputs)
{
    // ********* ALLOCATE DATA FACTORY *********
    std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> tFactory;
    tFactory = std::make_shared<Plato::DataFactory<ScalarType, OrdinalType>>(aInputs.mMemorySpace);
    tFactory->setCommWrapper(aInputs.mCommWrapper);
    tFactory->allocateDual(*aInputs.mDual);
    tFactory->allocateControl(*aInputs.mParticles);
    tFactory->allocateObjFuncValues(*aInputs.mCriteriaEvals);
    tFactory->allocateControlReductionOperations(*aInputs.mControlReductions);
    tFactory->allocateObjFuncReductionOperations(*aInputs.mCriteriaReductions);

    // ********* ALLOCATE AUGMENTED LAGRANGIAN ALGORITHM *********
    Plato::AugmentedLagrangianPSO<ScalarType, OrdinalType> tAlgorithm(tFactory, aObjective, aConstraints);

    // ********* SOLVE OPTIMIZATION PROBLEM AND SAVE SOLUTION *********
    Plato::set_alpso_algorithm_inputs(aInputs, tAlgorithm);
    tAlgorithm.solve();
    Plato::set_alpso_algorithm_outputs(tAlgorithm, aOutputs);
}
// function solve_alpso

/******************************************************************************//**
 * @brief Inputs from Bound Constrained Particle Swarm Optimization (BCPSO) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct AlgorithmOutputsBCPSO
{
    std::string mStopCriterion; /*!< stopping criterion */

    OrdinalType mNumOuterIter; /*!< number of outer iterations */
    OrdinalType mNumObjFuncEval; /*!< number of objective function evaluations */

    ScalarType mMeanBestObjFuncValue; /*!< objective function mean from best particle set */
    ScalarType mStdDevBestObjFuncValue; /*!< objective function standard deviation from best particle set */
    ScalarType mGlobalBestObjFuncValue; /*!< global best objective function value */

    std::shared_ptr<Plato::Vector<ScalarType,OrdinalType>> mMeanBestParticles; /*!< particle dimension's standard deviation values from best particle set */
    std::shared_ptr<Plato::Vector<ScalarType,OrdinalType>> mStdDevBestParticles; /*!< particle dimension's mean values from best particle set */
    std::shared_ptr<Plato::Vector<ScalarType,OrdinalType>> mGlobalBestParticles; /*!< global best particle dimension's */
};
// struct AlgorithmOutputsBCPSO

/******************************************************************************//**
 * @brief Inputs for Bound Constrained Particle Swarm Optimization (BCPSO) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct AlgorithmInputsBCPSO
{
    /******************************************************************************//**
     * @brief Default constructor
     **********************************************************************************/
    AlgorithmInputsBCPSO() :
            mOutputDiagnostics(false),
            mMaxNumIterations(1e3),
            mMaxNumConsecutiveFailures(10),
            mMaxNumConsecutiveSuccesses(10),
            mTimeStep(1),
            mMeanBestObjFuncTolerance(5e-4),
            mStdDevBestObjFuncTolerance(1e-6),
            mGlobalBestObjFuncTolerance(1e-10),
            mInertiaMultiplier(0.9),
            mSocialBehaviorMultiplier(0.8),
            mCognitiveBehaviorMultiplier(0.8),
            mTrustRegionExpansionMultiplier(4.0),
            mTrustRegionContractionMultiplier(0.75),
            mMemorySpace(Plato::MemorySpace::HOST),
            mCriteriaEvals(),
            mParticlesLowerBounds(),
            mParticlesUpperBounds(),
            mParticles(),
            mControlReductions(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>()),
            mCriteriaReductions(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>())
    {
        mCommWrapper.useDefaultComm();
    }

    /******************************************************************************//**
     * @brief Default destructor
     **********************************************************************************/
    ~AlgorithmInputsBCPSO()
    {
    }

    bool mOutputDiagnostics; /*!< flag to enable problem statistics output (default=false) */

    OrdinalType mMaxNumIterations; /*!< maximum number of iterations */
    OrdinalType mMaxNumConsecutiveFailures; /*!< maximum number of consecutive failures, global best F(x_{i+1}) == F(x_{i}) */
    OrdinalType mMaxNumConsecutiveSuccesses; /*!< maximum number of consecutive successes, global best F(x_{i+1}) < F(x_{i}) */

    ScalarType mTimeStep; /*!< time step \Delta{t} */
    ScalarType mMeanBestObjFuncTolerance; /*!< mean objective function stopping tolerance */
    ScalarType mGlobalBestObjFuncTolerance; /*!< best objective function stopping tolerance */
    ScalarType mStdDevBestObjFuncTolerance; /*!< standard deviation stopping tolerance */
    ScalarType mInertiaMultiplier; /*!< inertia multiplier */
    ScalarType mSocialBehaviorMultiplier; /*!< social behavior multiplier */
    ScalarType mCognitiveBehaviorMultiplier; /*!< cognite behavior multiplier */
    ScalarType mTrustRegionExpansionMultiplier; /*!< trust region expansion multiplier */
    ScalarType mTrustRegionContractionMultiplier; /*!< trust region contraction multiplier */

    Plato::CommWrapper mCommWrapper; /*!< distributed memory communication wrapper */
    Plato::MemorySpace::type_t mMemorySpace; /*!< memory space: HOST (default) OR DEVICE */

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCriteriaEvals; /*!< criteria evaluations */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mParticlesLowerBounds; /*!< particles' lower bounds */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mParticlesUpperBounds; /*!< particles' upper bounds */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mParticles; /*!< particles */

    /*!< operations which require communication across processors, e.g. max, min, global sum */
    std::shared_ptr<Plato::ReductionOperations<ScalarType,OrdinalType>> mControlReductions;
    std::shared_ptr<Plato::ReductionOperations<ScalarType,OrdinalType>> mCriteriaReductions;
};
// struct AlgorithmInputsBCPSO

/******************************************************************************//**
 * @brief Set Bound Constrained Particle Swarm Optimization (BCPSO) algorithm inputs
 * @param [in] aInputs inputs for BCPSO algorithm
 * @param [in,out] aAlgorithm BCPSO algorithm interface
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void set_bcpso_algorithm_inputs(const Plato::AlgorithmInputsBCPSO<ScalarType,OrdinalType> & aInputs,
                                       Plato::BoundConstrainedPSO<ScalarType,OrdinalType> & aAlgorithm)
{
    if(aInputs.mOutputDiagnostics == true)
    {
        aAlgorithm.enableDiagnostics();
    }

    aAlgorithm.setMaxNumIterations(aInputs.mMaxNumIterations);
    aAlgorithm.setMaxNumConsecutiveFailures(aInputs.mMaxNumConsecutiveFailures);
    aAlgorithm.setMaxNumConsecutiveSuccesses(aInputs.mMaxNumConsecutiveSuccesses);

    aAlgorithm.setTimeStep(aInputs.mTimeStep);
    aAlgorithm.setLowerBounds(*aInputs.mParticlesLowerBounds);
    aAlgorithm.setUpperBounds(*aInputs.mParticlesUpperBounds);
    aAlgorithm.setInertiaMultiplier(aInputs.mInertiaMultiplier);
    aAlgorithm.setMeanBestObjFuncTolerance(aInputs.mMeanBestObjFuncTolerance);
    aAlgorithm.setStdDevBestObjFuncTolerance(aInputs.mStdDevBestObjFuncTolerance);
    aAlgorithm.setSocialBehaviorMultiplier(aInputs.mSocialBehaviorMultiplier);
    aAlgorithm.setGlobalBestObjFuncTolerance(aInputs.mGlobalBestObjFuncTolerance);
    aAlgorithm.setCognitiveBehaviorMultiplier(aInputs.mCognitiveBehaviorMultiplier);
    aAlgorithm.setTrustRegionExpansionMultiplier(aInputs.mTrustRegionExpansionMultiplier);
    aAlgorithm.setTrustRegionContractionMultiplier(aInputs.mTrustRegionContractionMultiplier);
}
// function set_bcpso_algorithm_inputs

/******************************************************************************//**
 * @brief Set Bound Constrained Particle Swarm Optimization (BCPSO) algorithm outputs
 * @param [in] aAlgorithm BCPSO algorithm interface
 * @param [in,out] aOutputs outputs from BCPSO algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void set_bcpso_algorithm_outputs(const Plato::BoundConstrainedPSO<ScalarType, OrdinalType> & aAlgorithm,
                                        Plato::AlgorithmOutputsBCPSO<ScalarType, OrdinalType> & aOutputs)
{
    aOutputs.mStopCriterion = aAlgorithm.getStoppingCriterion();

    aOutputs.mNumOuterIter = aAlgorithm.getNumIterations();
    aOutputs.mNumObjFuncEval = aAlgorithm.getNumObjFuncEvals();

    // AUGMENTED LAGRANGIAN FUNCTION DIAGNOSTICS
    aOutputs.mMeanBestObjFuncValue = aAlgorithm.getMeanCurrentBestObjFuncValues();
    aOutputs.mGlobalBestObjFuncValue = aAlgorithm.getCurrentGlobalBestObjFuncValue();
    aOutputs.mStdDevBestObjFuncValue = aAlgorithm.getStdDevCurrentBestObjFuncValues();

    // PARTILCES DIAGNOSTICS
    aAlgorithm.getMeanCurrentBestParticlePositions(aOutputs.mMeanBestParticles);
    aAlgorithm.getStdDevCurrentBestParticlePositions(aOutputs.mStdDevBestParticles);
    aAlgorithm.getCurrentGlobalBestParticlePosition(aOutputs.mGlobalBestParticles);
}
// function set_bcpso_algorithm_outputs

/******************************************************************************//**
 * @brief Bound Constrained Particle Swarm Optimization (BCPSO) algorithm light interface
 * @param [in] aObjective user-defined grad-free objective function
 * @param [in] aInputs BCPSO algorithm inputs
 * @param [in,out] aOutputs BCPSO algorithm outputs
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void solve_bcpso(const std::shared_ptr<Plato::GradFreeCriterion<ScalarType, OrdinalType>> & aObjective,
                        const Plato::AlgorithmInputsBCPSO<ScalarType, OrdinalType> & aInputs,
                        Plato::AlgorithmOutputsBCPSO<ScalarType, OrdinalType> & aOutputs)
{
    // ********* ALLOCATE DATA FACTORY *********
    std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> tFactory;
    tFactory = std::make_shared<Plato::DataFactory<ScalarType, OrdinalType>>(aInputs.mMemorySpace);
    tFactory->setCommWrapper(aInputs.mCommWrapper);
    tFactory->allocateControl(*aInputs.mParticles);
    tFactory->allocateObjFuncValues(*aInputs.mCriteriaEvals);
    tFactory->allocateControlReductionOperations(*aInputs.mControlReductions);
    tFactory->allocateObjFuncReductionOperations(*aInputs.mCriteriaReductions);

    // ********* ALLOCATE AUGMENTED LAGRANGIAN ALGORITHM *********
    Plato::BoundConstrainedPSO<ScalarType, OrdinalType> tAlgorithm(tFactory, aObjective);

    // ********* SOLVE OPTIMIZATION PROBLEM AND SAVE SOLUTION *********
    Plato::set_bcpso_algorithm_inputs(aInputs, tAlgorithm);
    tAlgorithm.solve();
    Plato::set_bcpso_algorithm_outputs(tAlgorithm, aOutputs);
}
// function solve_bcpso

/******************************************************************************//**
 * @brief Parser for Particle Swarm Optimization (PSO) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class ParserPSO
{
public:
    /******************************************************************************//**
     * @brief Constructor
    **********************************************************************************/
    ParserPSO()
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~ParserPSO()
    {
    }

    /******************************************************************************//**
     * @brief Parse options for bound constrained PSO algorithm
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @param [out] aData data structure with bound constrained PSO algorithm options
    **********************************************************************************/
    void parse(const Plato::InputData & aOptimizerNode, Plato::AlgorithmInputsBCPSO<ScalarType, OrdinalType> & aData)
    {
        if(aOptimizerNode.size<Plato::InputData>("Options"))
        {
            Plato::InputData tOptionsNode = aOptimizerNode.get<Plato::InputData>("Options");
            aData.mOutputDiagnostics = this->outputDiagnostics(tOptionsNode);

            aData.mMaxNumIterations = this->maxNumOuterIterations(tOptionsNode);
            aData.mMaxNumConsecutiveFailures = this->maxNumConsecutiveFailures(tOptionsNode);
            aData.mMaxNumConsecutiveSuccesses = this->maxNumConsecutiveSuccesses(tOptionsNode);

            aData.mTimeStep = this->particleVelocityTimeStep(tOptionsNode);
            aData.mInertiaMultiplier = this->inertiaMultiplier(tOptionsNode);
            aData.mSocialBehaviorMultiplier = this->socialBehaviorMultiplier(tOptionsNode);
            aData.mMeanBestObjFuncTolerance = this->meanBestObjFuncTolerance(tOptionsNode);
            aData.mGlobalBestObjFuncTolerance = this->globalBestObjFuncTolerance(tOptionsNode);
            aData.mStdDevBestObjFuncTolerance = this->stdDevBestObjFuncTolerance(tOptionsNode);
            aData.mCognitiveBehaviorMultiplier = this->cognitiveBehaviorMultiplier(tOptionsNode);
            aData.mTrustRegionExpansionMultiplier = this->trustRegionExpansionMultiplier(tOptionsNode);
            aData.mTrustRegionContractionMultiplier = this->trustRegionContractionMultiplier(tOptionsNode);
        }
    }

    /******************************************************************************//**
     * @brief Parse options for augmented Lagrangian PSO algorithm
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @param [out] aData data structure with augmented Lagrangian PSO algorithm options
    **********************************************************************************/
    void parse(const Plato::InputData & aOptimizerNode, Plato::AlgorithmInputsALPSO<ScalarType, OrdinalType> & aData)
    {
        if(aOptimizerNode.size<Plato::InputData>("Options"))
        {
            Plato::InputData tOptionsNode = aOptimizerNode.get<Plato::InputData>("Options");
            aData.mOutputDiagnostics = this->outputDiagnostics(tOptionsNode);

            aData.mMaxNumOuterIter = this->maxNumOuterIterations(tOptionsNode);
            aData.mMaxNumInnerIter = this->maxNumInnerIterations(tOptionsNode);
            aData.mMaxNumConsecutiveFailures = this->maxNumConsecutiveFailures(tOptionsNode);
            aData.mMaxNumConsecutiveSuccesses = this->maxNumConsecutiveSuccesses(tOptionsNode);

            aData.mTimeStep = this->particleVelocityTimeStep(tOptionsNode);
            aData.mInertiaMultiplier = this->inertiaMultiplier(tOptionsNode);
            aData.mSocialBehaviorMultiplier = this->socialBehaviorMultiplier(tOptionsNode);
            aData.mMeanBestAugLagFuncTolerance = this->meanBestAugLagFuncTolerance(tOptionsNode);
            aData.mPenaltyExpansionMultiplier = this->penaltyExpansionMultiplier(tOptionsNode);
            aData.mCognitiveBehaviorMultiplier = this->cognitiveBehaviorMultiplier(tOptionsNode);
            aData.mPenaltyMultiplierUpperBound = this->penaltyMultiplierUpperBound(tOptionsNode);
            aData.mPenaltyContractionMultiplier = this->penaltyContractionMultiplier(tOptionsNode);
            aData.mStdDevBestAugLagFuncTolerance = this->stdDevBestAugLagFuncTolerance(tOptionsNode);
            aData.mGlobalBestAugLagFuncTolerance = this->globalBestAugLagFuncTolerance(tOptionsNode);
            aData.mTrustRegionExpansionMultiplier = this->trustRegionExpansionMultiplier(tOptionsNode);
            aData.mFeasibilityInexactnessTolerance = this->feasibilityInexactnessTolerance(tOptionsNode);
            aData.mTrustRegionContractionMultiplier = this->trustRegionContractionMultiplier(tOptionsNode);
        }
    }

private:
    /******************************************************************************//**
     * @brief Parse output diagnostics keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    bool outputDiagnostics(const Plato::InputData & aOptionsNode)
    {
        bool tOuput = false;
        if(aOptionsNode.size<std::string>("OutputDiagnosticsToFile"))
        {
            tOuput = Plato::Get::Bool(aOptionsNode, "OutputDiagnosticsToFile");
        }
        return (tOuput);
    }

    /******************************************************************************//**
     * @brief Parse maximum number of outer iterations keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    OrdinalType maxNumOuterIterations(const Plato::InputData & aOptionsNode)
    {
        OrdinalType tOutput = 1e3;
        if(aOptionsNode.size<std::string>("MaxNumOuterIterations"))
        {
            tOutput = Plato::Get::Int(aOptionsNode, "MaxNumOuterIterations");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse maximum number of inner iterations keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    OrdinalType maxNumInnerIterations(const Plato::InputData & aOptionsNode)
    {
        OrdinalType tOutput = 5;
        if(aOptionsNode.size<std::string>("MaxNumInnerIterations"))
        {
            tOutput = Plato::Get::Int(aOptionsNode, "MaxNumInnerIterations");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse maximum number of consecutive failures keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    OrdinalType maxNumConsecutiveFailures(const Plato::InputData & aOptionsNode)
    {
        OrdinalType tOutput = 10;
        if(aOptionsNode.size<std::string>("MaxNumConsecutiveFailures"))
        {
            tOutput = Plato::Get::Int(aOptionsNode, "MaxNumConsecutiveFailures");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse maximum number of consecutive successes keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    OrdinalType maxNumConsecutiveSuccesses(const Plato::InputData & aOptionsNode)
    {
        OrdinalType tOutput = 10;
        if(aOptionsNode.size<std::string>("MaxNumConsecutiveSuccesses"))
        {
            tOutput = Plato::Get::Int(aOptionsNode, "MaxNumConsecutiveSuccesses");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse particle's velocity time step keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType particleVelocityTimeStep(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 1.0;
        if(aOptionsNode.size<std::string>("ParticleVelocityTimeStep"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "ParticleVelocityTimeStep");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse upper bound on penalty multiplier keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType penaltyMultiplierUpperBound(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 100;
        if(aOptionsNode.size<std::string>("PenaltyMultiplierUpperBound"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "PenaltyMultiplierUpperBound");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse expansion multiplier keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType penaltyExpansionMultiplier(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 2.0;
        if(aOptionsNode.size<std::string>("PenaltyExpansionMultiplier"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "PenaltyExpansionMultiplier");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse contraction multiplier keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType penaltyContractionMultiplier(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 0.5;
        if(aOptionsNode.size<std::string>("PenaltyContractionMultiplier"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "PenaltyContractionMultiplier");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse tolerance on the global best objective function
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType globalBestObjFuncTolerance(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 1e-10;
        if(aOptionsNode.size<std::string>("GlobalBestObjFuncTolerance"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "GlobalBestObjFuncTolerance");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse tolerance on the mean of the best objective function values
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType meanBestObjFuncTolerance(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 5e-4;
        if(aOptionsNode.size<std::string>("MeanBestObjFuncTolerance"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "MeanBestObjFuncTolerance");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse tolerance on the standard deviation of the best objective function values
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType stdDevBestObjFuncTolerance(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 1e-6;
        if(aOptionsNode.size<std::string>("StdDevBestObjFuncTolerance"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "StdDevBestObjFuncTolerance");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse tolerance on the mean of the best augmented Lagrangian function values
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType meanBestAugLagFuncTolerance(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 5e-4;
        if(aOptionsNode.size<std::string>("MeanBestAugLagFuncTolerance"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "MeanBestAugLagFuncTolerance");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse tolerance on the standard deviation of the best augmented Lagrangian function values
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType stdDevBestAugLagFuncTolerance(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 1e-6;
        if(aOptionsNode.size<std::string>("StdDevBestAugLagFuncTolerance"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "StdDevBestAugLagFuncTolerance");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse tolerance on the global best augmented Lagrangian function
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType globalBestAugLagFuncTolerance(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 1e-10;
        if(aOptionsNode.size<std::string>("GlobalBestAugLagFuncTolerance"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "GlobalBestAugLagFuncTolerance");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse tolerance on the global best augmented Lagrangian function
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType inertiaMultiplier(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 0.9;
        if(aOptionsNode.size<std::string>("InertiaMultiplier"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "InertiaMultiplier");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse social behavior multiplier keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType socialBehaviorMultiplier(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 0.8;
        if(aOptionsNode.size<std::string>("SocialBehaviorMultiplier"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "SocialBehaviorMultiplier");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse cognitive behavior multiplier keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType cognitiveBehaviorMultiplier(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 0.8;
        if(aOptionsNode.size<std::string>("CognitiveBehaviorMultiplier"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "CognitiveBehaviorMultiplier");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse trust region expansion multiplier keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType trustRegionExpansionMultiplier(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 4;
        if(aOptionsNode.size<std::string>("TrustRegionExpansionMultiplier"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "TrustRegionExpansionMultiplier");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse trust region contraction multiplier keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType trustRegionContractionMultiplier(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 0.75;
        if(aOptionsNode.size<std::string>("TrustRegionContractionMultiplier"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "TrustRegionContractionMultiplier");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse tolerance on feasibility inexactness
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType feasibilityInexactnessTolerance(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 1e-4;
        if(aOptionsNode.size<std::string>("FeasibilityInexactnessTolerance"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "FeasibilityInexactnessTolerance");
        }
        return (tOutput);
    }

private:
    ParserPSO(const Plato::ParserPSO<ScalarType, OrdinalType>&);
    Plato::ParserPSO<ScalarType, OrdinalType> & operator=(const Plato::ParserPSO<ScalarType, OrdinalType>&);
};
// class ParserPSO

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

    tFlag = Plato::particle_swarm::STDDEV_OBJECTIVE_TOLERANCE;
    Plato::pso::get_stop_criterion(tFlag, tDescription);
    tGold = "\n\n****** Optimization stopping due to standard deviation tolerance being met. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());
}

TEST(PlatoTest, PSO_PrintDiagnosticsInvalidArgumentsPSO)
{
    std::ofstream tFile1;
    Plato::OutputDataPSO<double> tData;
    ASSERT_THROW(Plato::pso::print_bcpso_diagnostics_header(tData, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::pso::print_bcpso_diagnostics_header(tData, tFile1, true /* print message */), std::invalid_argument);
    ASSERT_THROW(Plato::pso::print_bcpso_diagnostics(tData, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::pso::print_bcpso_diagnostics(tData, tFile1, true /* print message */), std::invalid_argument);
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
        ASSERT_NO_THROW(Plato::pso::print_bcpso_diagnostics_header(tData, tWriteFile));
        ASSERT_NO_THROW(Plato::pso::print_bcpso_diagnostics(tData, tWriteFile));

        tData.mNumIter = 2;
        tData.mObjFuncCount = 40;
        tData.mCurrentGlobalBestObjFuncValue = 2.2321;
        tData.mMeanCurrentBestObjFuncValues = 7.2321;
        tData.mStdDevCurrentBestObjFuncValues = 2.4321;
        tData.mTrustRegionMultiplier = 1.0;
        ASSERT_NO_THROW(Plato::pso::print_bcpso_diagnostics(tData, tWriteFile));
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
    EXPECT_NEAR(-0.00595, tStageMng.getCurrentGlobalBestConstraintValue(0  /* constraint index */), tTolerance);

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
    EXPECT_NEAR(-0.00595, tStageMng.getCurrentGlobalBestConstraintValue(0  /* constraint index */), tTolerance);
}

TEST(PlatoTest, PSO_ParserBCPSO)
{
    Plato::InputData tInputData;
    EXPECT_TRUE(tInputData.empty());
    ASSERT_STREQ("Input Data", tInputData.name().c_str());

    Plato::ParserPSO<double> tParserPSO;
    Plato::AlgorithmInputsBCPSO<double> tInputsPSO;
    tParserPSO.parse(tInputData, tInputsPSO);

    // ********* TEST: OPTIONS NODE NOT DEFINE -> USE DEFAULT PARAMETERS *********
    EXPECT_FALSE(tInputsPSO.mOutputDiagnostics);
    EXPECT_EQ(1000u, tInputsPSO.mMaxNumIterations);
    EXPECT_EQ(10u, tInputsPSO.mMaxNumConsecutiveFailures);
    EXPECT_EQ(10u, tInputsPSO.mMaxNumConsecutiveSuccesses);

    const double tTolerance = 1e-6;
    EXPECT_NEAR(1.0, tInputsPSO.mTimeStep, tTolerance);
    EXPECT_NEAR(0.9, tInputsPSO.mInertiaMultiplier, tTolerance);
    EXPECT_NEAR(0.8, tInputsPSO.mSocialBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(5e-4, tInputsPSO.mMeanBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(0.8, tInputsPSO.mCognitiveBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(1e-6, tInputsPSO.mStdDevBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(1e-10, tInputsPSO.mGlobalBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(4.0, tInputsPSO.mTrustRegionExpansionMultiplier, tTolerance);
    EXPECT_NEAR(0.75, tInputsPSO.mTrustRegionContractionMultiplier, tTolerance);

    // ********* TEST: OPTIONS NODE DEFINED, BUT PARAMETERS NOT SPECIFIED -> USE DEFAULT PARAMETERS *********
    Plato::InputData tOptions("Options");
    EXPECT_TRUE(tOptions.empty());
    tParserPSO.parse(tOptions, tInputsPSO);
    EXPECT_FALSE(tInputsPSO.mOutputDiagnostics);
    EXPECT_EQ(1000u, tInputsPSO.mMaxNumIterations);
    EXPECT_EQ(10u, tInputsPSO.mMaxNumConsecutiveFailures);
    EXPECT_EQ(10u, tInputsPSO.mMaxNumConsecutiveSuccesses);

    EXPECT_NEAR(1.0, tInputsPSO.mTimeStep, tTolerance);
    EXPECT_NEAR(0.9, tInputsPSO.mInertiaMultiplier, tTolerance);
    EXPECT_NEAR(0.8, tInputsPSO.mSocialBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(5e-4, tInputsPSO.mMeanBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(0.8, tInputsPSO.mCognitiveBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(1e-6, tInputsPSO.mStdDevBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(1e-10, tInputsPSO.mGlobalBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(4.0, tInputsPSO.mTrustRegionExpansionMultiplier, tTolerance);
    EXPECT_NEAR(0.75, tInputsPSO.mTrustRegionContractionMultiplier, tTolerance);

    // ********* TEST: SET PARAMETERS AND ADD OPTIONS NODE TO OPTIMIZER'S NODE -> NON-DEFAULT VALUES ARE EXPECTED *********
    tOptions.add<std::string>("MaxNumOuterIterations", "100");
    tOptions.add<std::string>("MaxNumConsecutiveFailures", "7");
    tOptions.add<std::string>("MaxNumConsecutiveSuccesses", "8");
    tOptions.add<std::string>("OutputDiagnosticsToFile", "true");
    tOptions.add<std::string>("InertiaMultiplier", "0.55");
    tOptions.add<std::string>("ParticleVelocityTimeStep", "0.75");
    tOptions.add<std::string>("SocialBehaviorMultiplier", "0.85");
    tOptions.add<std::string>("MeanBestObjFuncTolerance", "0.001");
    tOptions.add<std::string>("CognitiveBehaviorMultiplier", "0.6");
    tOptions.add<std::string>("StdDevBestObjFuncTolerance", "0.0001");
    tOptions.add<std::string>("GlobalBestObjFuncTolerance", "0.000001");
    tOptions.add<std::string>("TrustRegionExpansionMultiplier", "2");
    tOptions.add<std::string>("TrustRegionContractionMultiplier", "0.5");
    Plato::InputData tOptimizerNode("OptimizerNode");
    tOptimizerNode.add<Plato::InputData>("Options", tOptions);

    tParserPSO.parse(tOptimizerNode, tInputsPSO);
    EXPECT_TRUE(tInputsPSO.mOutputDiagnostics);
    EXPECT_EQ(100u, tInputsPSO.mMaxNumIterations);
    EXPECT_EQ(7u, tInputsPSO.mMaxNumConsecutiveFailures);
    EXPECT_EQ(8u, tInputsPSO.mMaxNumConsecutiveSuccesses);

    EXPECT_NEAR(0.75, tInputsPSO.mTimeStep, tTolerance);
    EXPECT_NEAR(0.55, tInputsPSO.mInertiaMultiplier, tTolerance);
    EXPECT_NEAR(0.85, tInputsPSO.mSocialBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(1e-3, tInputsPSO.mMeanBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(0.6, tInputsPSO.mCognitiveBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(1e-4, tInputsPSO.mStdDevBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(1e-6, tInputsPSO.mGlobalBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(2.0, tInputsPSO.mTrustRegionExpansionMultiplier, tTolerance);
    EXPECT_NEAR(0.5, tInputsPSO.mTrustRegionContractionMultiplier, tTolerance);
}

TEST(PlatoTest, PSO_ParserALPSO)
{
    Plato::InputData tInputData;
    EXPECT_TRUE(tInputData.empty());
    ASSERT_STREQ("Input Data", tInputData.name().c_str());

    Plato::ParserPSO<double> tParserPSO;
    Plato::AlgorithmInputsALPSO<double> tInputsPSO;
    tParserPSO.parse(tInputData, tInputsPSO);

    // ********* TEST: OPTIONS NODE NOT DEFINE -> USE DEFAULT PARAMETERS *********
    EXPECT_FALSE(tInputsPSO.mOutputDiagnostics);
    EXPECT_EQ(5, tInputsPSO.mMaxNumInnerIter);
    EXPECT_EQ(1000u, tInputsPSO.mMaxNumOuterIter);
    EXPECT_EQ(10u, tInputsPSO.mMaxNumConsecutiveFailures);
    EXPECT_EQ(10u, tInputsPSO.mMaxNumConsecutiveSuccesses);

    const double tTolerance = 1e-6;
    EXPECT_NEAR(1.0, tInputsPSO.mTimeStep, tTolerance);
    EXPECT_NEAR(0.9, tInputsPSO.mInertiaMultiplier, tTolerance);
    EXPECT_NEAR(0.8, tInputsPSO.mSocialBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(0.8, tInputsPSO.mCognitiveBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(5e-4, tInputsPSO.mMeanBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(2.0, tInputsPSO.mPenaltyExpansionMultiplier, tTolerance);
    EXPECT_NEAR(100, tInputsPSO.mPenaltyMultiplierUpperBound, tTolerance);
    EXPECT_NEAR(0.5, tInputsPSO.mPenaltyContractionMultiplier, tTolerance);
    EXPECT_NEAR(1e-4, tInputsPSO.mFeasibilityInexactnessTolerance, tTolerance);
    EXPECT_NEAR(1e-6, tInputsPSO.mStdDevBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(1e-10, tInputsPSO.mGlobalBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(4.0, tInputsPSO.mTrustRegionExpansionMultiplier, tTolerance);
    EXPECT_NEAR(0.75, tInputsPSO.mTrustRegionContractionMultiplier, tTolerance);

    // ********* TEST: OPTIONS NODE DEFINED, BUT PARAMETERS NOT SPECIFIED -> USE DEFAULT PARAMETERS *********
    Plato::InputData tOptions("Options");
    EXPECT_TRUE(tOptions.empty());
    tParserPSO.parse(tOptions, tInputsPSO);
    EXPECT_FALSE(tInputsPSO.mOutputDiagnostics);
    EXPECT_EQ(5, tInputsPSO.mMaxNumInnerIter);
    EXPECT_EQ(1000u, tInputsPSO.mMaxNumOuterIter);
    EXPECT_EQ(10u, tInputsPSO.mMaxNumConsecutiveFailures);
    EXPECT_EQ(10u, tInputsPSO.mMaxNumConsecutiveSuccesses);

    EXPECT_NEAR(1.0, tInputsPSO.mTimeStep, tTolerance);
    EXPECT_NEAR(0.9, tInputsPSO.mInertiaMultiplier, tTolerance);
    EXPECT_NEAR(0.8, tInputsPSO.mSocialBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(0.8, tInputsPSO.mCognitiveBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(5e-4, tInputsPSO.mMeanBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(2.0, tInputsPSO.mPenaltyExpansionMultiplier, tTolerance);
    EXPECT_NEAR(100, tInputsPSO.mPenaltyMultiplierUpperBound, tTolerance);
    EXPECT_NEAR(0.5, tInputsPSO.mPenaltyContractionMultiplier, tTolerance);
    EXPECT_NEAR(1e-4, tInputsPSO.mFeasibilityInexactnessTolerance, tTolerance);
    EXPECT_NEAR(1e-6, tInputsPSO.mStdDevBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(1e-10, tInputsPSO.mGlobalBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(4.0, tInputsPSO.mTrustRegionExpansionMultiplier, tTolerance);
    EXPECT_NEAR(0.75, tInputsPSO.mTrustRegionContractionMultiplier, tTolerance);

    // ********* TEST: SET PARAMETERS AND ADD OPTIONS NODE TO OPTIMIZER'S NODE -> NON-DEFAULT VALUES ARE EXPECTED *********
    tOptions.add<std::string>("MaxNumOuterIterations", "100");
    tOptions.add<std::string>("MaxNumConsecutiveFailures", "7");
    tOptions.add<std::string>("MaxNumConsecutiveSuccesses", "8");
    tOptions.add<std::string>("OutputDiagnosticsToFile", "true");
    tOptions.add<std::string>("InertiaMultiplier", "0.55");
    tOptions.add<std::string>("ParticleVelocityTimeStep", "0.75");
    tOptions.add<std::string>("SocialBehaviorMultiplier", "0.85");
    tOptions.add<std::string>("MeanBestAugLagFuncTolerance", "0.001");
    tOptions.add<std::string>("CognitiveBehaviorMultiplier", "0.6");
    tOptions.add<std::string>("StdDevBestAugLagFuncTolerance", "0.0001");
    tOptions.add<std::string>("GlobalBestAugLagFuncTolerance", "0.000001");
    tOptions.add<std::string>("TrustRegionExpansionMultiplier", "2");
    tOptions.add<std::string>("TrustRegionContractionMultiplier", "0.5");
    tOptions.add<std::string>("PenaltyExpansionMultiplier", "4");
    tOptions.add<std::string>("PenaltyMultiplierUpperBound", "10");
    tOptions.add<std::string>("PenaltyContractionMultiplier", "0.25");
    tOptions.add<std::string>("FeasibilityInexactnessTolerance", "0.01");
    Plato::InputData tOptimizerNode("OptimizerNode");
    tOptimizerNode.add<Plato::InputData>("Options", tOptions);

    tParserPSO.parse(tOptimizerNode, tInputsPSO);
    EXPECT_TRUE(tInputsPSO.mOutputDiagnostics);
    EXPECT_EQ(100u, tInputsPSO.mMaxNumOuterIter);
    EXPECT_EQ(7u, tInputsPSO.mMaxNumConsecutiveFailures);
    EXPECT_EQ(8u, tInputsPSO.mMaxNumConsecutiveSuccesses);

    EXPECT_NEAR(0.75, tInputsPSO.mTimeStep, tTolerance);
    EXPECT_NEAR(0.55, tInputsPSO.mInertiaMultiplier, tTolerance);
    EXPECT_NEAR(0.85, tInputsPSO.mSocialBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(1e-3, tInputsPSO.mMeanBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(0.6, tInputsPSO.mCognitiveBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(1e-4, tInputsPSO.mStdDevBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(1e-6, tInputsPSO.mGlobalBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(2.0, tInputsPSO.mTrustRegionExpansionMultiplier, tTolerance);
    EXPECT_NEAR(0.5, tInputsPSO.mTrustRegionContractionMultiplier, tTolerance);
    EXPECT_NEAR(4.0, tInputsPSO.mPenaltyExpansionMultiplier, tTolerance);
    EXPECT_NEAR(10, tInputsPSO.mPenaltyMultiplierUpperBound, tTolerance);
    EXPECT_NEAR(0.25, tInputsPSO.mPenaltyContractionMultiplier, tTolerance);
    EXPECT_NEAR(1e-2, tInputsPSO.mFeasibilityInexactnessTolerance, tTolerance);
}

TEST(PlatoTest, PSO_SolveBCPSO_Rosenbrock)
{
    // ********* ALLOCATE CRITERION *********
    std::shared_ptr<Plato::GradFreeCriterion<double>> tObjective = std::make_shared<Plato::GradFreeRosenbrock<double>>();

    // ********* ALLOCATE CORE DATA STRUCTURES *********
    const size_t tNumControls = 2;
    const size_t tNumParticles = 10;
    Plato::AlgorithmInputsBCPSO<double> tInputs;
    tInputs.mCriteriaEvals = std::make_shared<Plato::StandardVector<double>>(tNumParticles);
    tInputs.mParticlesLowerBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesLowerBounds->fill(-5);
    tInputs.mParticlesUpperBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesUpperBounds->fill(5);
    tInputs.mParticles = std::make_shared<Plato::StandardMultiVector<double>>(tNumParticles, tNumControls);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::AlgorithmOutputsBCPSO<double> tOutputs;
    Plato::solve_bcpso<double>(tObjective, tInputs, tOutputs);

    // ********* DIAGNOSTICS *********
    const double tTolerance = 1e-2;
    EXPECT_NEAR(0, tOutputs.mGlobalBestObjFuncValue, tTolerance);

    std::cout << "\nNUM ITERATIONS = " << tOutputs.mNumOuterIter << "\n";
    std::cout << "\nOBJECTIVE: BEST = " << tOutputs.mGlobalBestObjFuncValue << ", MEAN = "
            << tOutputs.mMeanBestObjFuncValue << ", STDDEV = " << tOutputs.mStdDevBestObjFuncValue << "\n";

    std::cout << tOutputs.mStopCriterion << "\n";

    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = " << (*tOutputs.mGlobalBestParticles)[tIndex] <<
                ", MEAN = " << (*tOutputs.mMeanBestParticles)[tIndex] << ", STDDEV = "
                << (*tOutputs.mStdDevBestParticles)[tIndex] << "\n";
    }
}

TEST(PlatoTest, PSO_SolveBCPSO_GoldsteinPrice)
{
    // ********* ALLOCATE CRITERION *********
    std::shared_ptr<Plato::GradFreeCriterion<double>> tObjective =
            std::make_shared<Plato::GradFreeGoldsteinPrice<double>>();

    // ********* ALLOCATE CORE DATA STRUCTURES *********
    const size_t tNumControls = 2;
    const size_t tNumParticles = 20;
    Plato::AlgorithmInputsBCPSO<double> tInputs;
    tInputs.mStdDevBestObjFuncTolerance = 1e-4;
    tInputs.mCriteriaEvals = std::make_shared<Plato::StandardVector<double>>(tNumParticles);
    tInputs.mParticlesLowerBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesLowerBounds->fill(-2);
    tInputs.mParticlesUpperBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesUpperBounds->fill(2);
    tInputs.mParticles = std::make_shared<Plato::StandardMultiVector<double>>(tNumParticles, tNumControls);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::AlgorithmOutputsBCPSO<double> tOutputs;
    Plato::solve_bcpso<double>(tObjective, tInputs, tOutputs);

    // ********* DIAGNOSTICS *********
    const double tTolerance = 1e-2;
    EXPECT_NEAR(3, tOutputs.mGlobalBestObjFuncValue, tTolerance);

    std::cout << "\nNUM ITERATIONS = " << tOutputs.mNumOuterIter << "\n";
    std::cout << "\nOBJECTIVE: BEST = " << tOutputs.mGlobalBestObjFuncValue << ", MEAN = "
            << tOutputs.mMeanBestObjFuncValue << ", STDDEV = " << tOutputs.mStdDevBestObjFuncValue << "\n";

    std::cout << tOutputs.mStopCriterion << "\n";

    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = " << (*tOutputs.mGlobalBestParticles)[tIndex] <<
                ", MEAN = " << (*tOutputs.mMeanBestParticles)[tIndex] << ", STDDEV = "
                << (*tOutputs.mStdDevBestParticles)[tIndex] << "\n";
    }
}

TEST(PlatoTest, PSO_SolveBCPSO_Himmelblau)
{
    // ********* ALLOCATE CRITERION *********
    std::shared_ptr<Plato::GradFreeCriterion<double>> tObjective =
            std::make_shared<Plato::GradFreeHimmelblau<double>>();

    // ********* ALLOCATE CORE DATA STRUCTURES *********
    const size_t tNumControls = 2;
    const size_t tNumParticles = 10;
    Plato::AlgorithmInputsBCPSO<double> tInputs;
    tInputs.mCriteriaEvals = std::make_shared<Plato::StandardVector<double>>(tNumParticles);
    tInputs.mParticlesLowerBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesLowerBounds->fill(-6);
    tInputs.mParticlesUpperBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesUpperBounds->fill(6);
    tInputs.mParticles = std::make_shared<Plato::StandardMultiVector<double>>(tNumParticles, tNumControls);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::AlgorithmOutputsBCPSO<double> tOutputs;
    Plato::solve_bcpso<double>(tObjective, tInputs, tOutputs);

    // ********* DIAGNOSTICS *********
    const double tTolerance = 1e-3;
    EXPECT_NEAR(0, tOutputs.mGlobalBestObjFuncValue, tTolerance);

    std::cout << "\nNUM ITERATIONS = " << tOutputs.mNumOuterIter << "\n";
    std::cout << "\nOBJECTIVE: BEST = " << tOutputs.mGlobalBestObjFuncValue << ", MEAN = "
            << tOutputs.mMeanBestObjFuncValue << ", STDDEV = " << tOutputs.mStdDevBestObjFuncValue << "\n";

    std::cout << tOutputs.mStopCriterion << "\n";

    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = " << (*tOutputs.mGlobalBestParticles)[tIndex] <<
                ", MEAN = " << (*tOutputs.mMeanBestParticles)[tIndex] << ", STDDEV = "
                << (*tOutputs.mStdDevBestParticles)[tIndex] << "\n";
    }
}

TEST(PlatoTest, PSO_SolveBCPSO_Circle)
{
    // ********* ALLOCATE CRITERION *********
    std::shared_ptr<Plato::GradFreeCriterion<double>> tObjective =
            std::make_shared<Plato::GradFreeCircle<double>>();

    // ********* ALLOCATE CORE DATA STRUCTURES *********
    const size_t tNumControls = 2;
    const size_t tNumParticles = 10;
    Plato::AlgorithmInputsBCPSO<double> tInputs;
    tInputs.mCriteriaEvals = std::make_shared<Plato::StandardVector<double>>(tNumParticles);
    tInputs.mParticlesLowerBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesLowerBounds->fill(-6);
    tInputs.mParticlesUpperBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesUpperBounds->fill(6);
    tInputs.mParticles = std::make_shared<Plato::StandardMultiVector<double>>(tNumParticles, tNumControls);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::AlgorithmOutputsBCPSO<double> tOutputs;
    Plato::solve_bcpso<double>(tObjective, tInputs, tOutputs);

    // ********* DIAGNOSTICS *********
    const double tTolerance = 1e-3;
    EXPECT_NEAR(0, tOutputs.mGlobalBestObjFuncValue, tTolerance);

    std::cout << "\nNUM ITERATIONS = " << tOutputs.mNumOuterIter << "\n";
    std::cout << "\nOBJECTIVE: BEST = " << tOutputs.mGlobalBestObjFuncValue << ", MEAN = "
            << tOutputs.mMeanBestObjFuncValue << ", STDDEV = " << tOutputs.mStdDevBestObjFuncValue << "\n";

    std::cout << tOutputs.mStopCriterion << "\n";

    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = " << (*tOutputs.mGlobalBestParticles)[tIndex] <<
                ", MEAN = " << (*tOutputs.mMeanBestParticles)[tIndex] << ", STDDEV = "
                << (*tOutputs.mStdDevBestParticles)[tIndex] << "\n";
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
    tAlgorithm.setMeanBestObjFuncTolerance(1e-6);
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
                << tAlgorithm.getDataMng().getMeanParticlePositions()[tIndex]
                        * tObjective->getNormalizationConstants()[tIndex]
                << ", STDDEV = "
                << tAlgorithm.getDataMng().getStdDevParticlePositions()[tIndex]
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

TEST(PlatoTest, PSO_SolveALPSO_RosenbrockObj_RadiusConstr)
{
    // ********* DEFINE CRITERIA *********
    const double tBound = 2;
    std::shared_ptr<Plato::GradFreeCriterion<double>> tConstraintOne = std::make_shared<Plato::GradFreeRadius<double>>(tBound);
    std::shared_ptr<Plato::GradFreeCriteriaList<double>> tConstraints = std::make_shared<Plato::GradFreeCriteriaList<double>>();
    tConstraints->add(tConstraintOne);
    std::shared_ptr<Plato::GradFreeCriterion<double>> tObjective = std::make_shared<Plato::GradFreeRosenbrock<double>>();

    // ********* ALLOCATE CORE DATA STRUCTURES *********
    const size_t tNumControls = 2;
    const size_t tNumParticles = 20;
    const size_t tNumConstraints = 1;
    Plato::AlgorithmInputsALPSO<double> tInputs;
    tInputs.mPenaltyMultiplierUpperBound = 1e3;
    tInputs.mMeanBestAugLagFuncTolerance = 1e-6;
    tInputs.mCriteriaEvals = std::make_shared<Plato::StandardVector<double>>(tNumParticles);
    tInputs.mParticlesLowerBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesLowerBounds->fill(-5);
    tInputs.mParticlesUpperBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesUpperBounds->fill(5);
    tInputs.mDual = std::make_shared<Plato::StandardMultiVector<double>>(tNumConstraints, tNumParticles);
    tInputs.mParticles = std::make_shared<Plato::StandardMultiVector<double>>(tNumParticles, tNumControls);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::AlgorithmOutputsALPSO<double> tOutputs;
    Plato::solve_alpso<double>(tObjective, tConstraints, tInputs, tOutputs);

    // ********* DIAGNOSTICS *********
    const double tTolerance = 1e-2;
    EXPECT_NEAR(0, tOutputs.mGlobalBestAugLagFuncValue, tTolerance);

    std::cout << "\nNUM ITERATIONS = " << tOutputs.mNumOuterIter << "\n";
    std::cout << "\nOBJECTIVE: BEST = " << tOutputs.mGlobalBestAugLagFuncValue << ", MEAN = "
            << tOutputs.mMeanBestAugLagFuncValue << ", STDDEV = " << tOutputs.mStdDevBestAugLagFuncValue << "\n";

    std::cout << "\nCONSTRAINT #0: BEST = " << (*tOutputs.mGlobalBestConstraintValues)[0] << ", MEAN = "
            << (*tOutputs.mMeanBestConstraintValues)[0] << ", STDDEV = " << (*tOutputs.mStdDevBestConstraintValues)[0]
            << "\n";

    std::cout << tOutputs.mStopCriterion << "\n";

    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = " << (*tOutputs.mGlobalBestParticles)[tIndex] <<
                ", MEAN = " << (*tOutputs.mMeanBestParticles)[tIndex] << ", STDDEV = "
                << (*tOutputs.mStdDevBestParticles)[tIndex] << "\n";
    }
}

TEST(PlatoTest, PSO_SolveALPSO_HimmelblauObj_ShiftedEllipseConstr)
{
    // ********* DEFINE CRITERIA *********
    std::shared_ptr<Plato::GradFreeShiftedEllipse<double>> tMyConstraint =
            std::make_shared<Plato::GradFreeShiftedEllipse<double>>();
    tMyConstraint->define(-2., 2., -3., 3.);
    std::shared_ptr<Plato::GradFreeCriteriaList<double>> tConstraints =
            std::make_shared<Plato::GradFreeCriteriaList<double>>();
    tConstraints->add(tMyConstraint);
    std::shared_ptr<Plato::GradFreeCriterion<double>> tObjective =
            std::make_shared<Plato::GradFreeHimmelblau<double>>();

    // ********* ALLOCATE CORE DATA STRUCTURES *********
    const size_t tNumControls = 2;
    const size_t tNumParticles = 20;
    const size_t tNumConstraints = 1;
    Plato::AlgorithmInputsALPSO<double> tInputs;
    tInputs.mCriteriaEvals = std::make_shared<Plato::StandardVector<double>>(tNumParticles);
    tInputs.mParticlesLowerBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesLowerBounds->fill(-5);
    tInputs.mParticlesUpperBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesUpperBounds->fill(5);
    tInputs.mDual = std::make_shared<Plato::StandardMultiVector<double>>(tNumConstraints, tNumParticles);
    tInputs.mParticles = std::make_shared<Plato::StandardMultiVector<double>>(tNumParticles, tNumControls);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::AlgorithmOutputsALPSO<double> tOutputs;
    Plato::solve_alpso<double>(tObjective, tConstraints, tInputs, tOutputs);

    // ********* DIAGNOSTICS *********
    const double tTolerance = 1e-2;
    EXPECT_NEAR(0, tOutputs.mGlobalBestAugLagFuncValue, tTolerance);

    std::cout << "\nNUM ITERATIONS = " << tOutputs.mNumOuterIter << "\n";
    std::cout << "\nOBJECTIVE: BEST = " << tOutputs.mGlobalBestAugLagFuncValue << ", MEAN = "
            << tOutputs.mMeanBestAugLagFuncValue << ", STDDEV = " << tOutputs.mStdDevBestAugLagFuncValue << "\n";

    std::cout << "\nCONSTRAINT #0: BEST = " << (*tOutputs.mGlobalBestConstraintValues)[0] << ", MEAN = "
            << (*tOutputs.mMeanBestConstraintValues)[0] << ", STDDEV = " << (*tOutputs.mStdDevBestConstraintValues)[0]
            << "\n";

    std::cout << tOutputs.mStopCriterion << "\n";

    std::vector<double> tGold = { -3.77931, -3.28319 };
    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = " << (*tOutputs.mGlobalBestParticles)[tIndex] <<
                ", MEAN = " << (*tOutputs.mMeanBestParticles)[tIndex] << ", STDDEV = "
                << (*tOutputs.mStdDevBestParticles)[tIndex] << "\n";
        EXPECT_NEAR(tGold[tIndex], (*tOutputs.mGlobalBestParticles)[tIndex], tTolerance);
    }
}

TEST(PlatoTest, PSO_SolveALPSO_CircleObj_RadiusConstr)
{
    // ********* DEFINE CRITERIA *********
    std::shared_ptr<Plato::GradFreeRadius<double>> tMyConstraint = std::make_shared<Plato::GradFreeRadius<double>>();
    std::shared_ptr<Plato::GradFreeCriteriaList<double>> tConstraints =
            std::make_shared<Plato::GradFreeCriteriaList<double>>();
    tConstraints->add(tMyConstraint);
    std::shared_ptr<Plato::GradFreeCriterion<double>> tObjective = std::make_shared<Plato::GradFreeCircle<double>>();

    // ********* ALLOCATE CORE DATA STRUCTURES *********
    const size_t tNumControls = 2;
    const size_t tNumParticles = 10;
    const size_t tNumConstraints = 1;
    Plato::AlgorithmInputsALPSO<double> tInputs;
    tInputs.mCriteriaEvals = std::make_shared<Plato::StandardVector<double>>(tNumParticles);
    tInputs.mParticlesLowerBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesLowerBounds->fill(0);
    tInputs.mParticlesUpperBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesUpperBounds->fill(1);
    tInputs.mDual = std::make_shared<Plato::StandardMultiVector<double>>(tNumConstraints, tNumParticles);
    tInputs.mParticles = std::make_shared<Plato::StandardMultiVector<double>>(tNumParticles, tNumControls);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::AlgorithmOutputsALPSO<double> tOutputs;
    Plato::solve_alpso<double>(tObjective, tConstraints, tInputs, tOutputs);

    // ********* DIAGNOSTICS *********
    const double tTolerance = 1e-2;
    EXPECT_NEAR(0, tOutputs.mGlobalBestAugLagFuncValue, tTolerance);

    std::cout << "\nNUM ITERATIONS = " << tOutputs.mNumOuterIter << "\n";
    std::cout << "\nOBJECTIVE: BEST = " << tOutputs.mGlobalBestAugLagFuncValue << ", MEAN = "
            << tOutputs.mMeanBestAugLagFuncValue << ", STDDEV = " << tOutputs.mStdDevBestAugLagFuncValue << "\n";

    std::cout << "\nCONSTRAINT #0: BEST = " << (*tOutputs.mGlobalBestConstraintValues)[0] << ", MEAN = "
            << (*tOutputs.mMeanBestConstraintValues)[0] << ", STDDEV = " << (*tOutputs.mStdDevBestConstraintValues)[0]
            << "\n";

    std::cout << tOutputs.mStopCriterion << "\n";

    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = " << (*tOutputs.mGlobalBestParticles)[tIndex] <<
                ", MEAN = " << (*tOutputs.mMeanBestParticles)[tIndex] << ", STDDEV = "
                << (*tOutputs.mStdDevBestParticles)[tIndex] << "\n";
    }
}

} // ParticleSwarmTest
