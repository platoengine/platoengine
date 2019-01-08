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

namespace Plato
{

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
    OrdinalType mNumIter;  /*!< number of outer iterations */
    OrdinalType mObjFuncCount;  /*!< number of objective function evaluations */
    OrdinalType mNumConstraints;  /*!< number of constraints - only needed for ALPSO output */

    ScalarType mBestObjFuncValue;  /*!< best - objective function value */
    ScalarType mMeanObjFuncValue;  /*!< mean - objective function value */
    ScalarType mStdDevObjFuncValue;  /*!< standard deviation - objective function value */
    ScalarType mTrustRegionMultiplier;  /*!< trust region multiplier */
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
        mAugLagFuncCount(0),
        mBestObjFuncValue(0),
        mMeanObjFuncValue(0),
        mStdDevObjFuncValue(0),
        mBestAugLagFuncValue(0),
        mMeanAugLagFuncValue(0),
        mStdDevAugLagFuncValue(0),
        mTrustRegionMultiplier(0),
        mBestConstraintValues(),
        mMeanConstraintValues(),
        mStdDevConstraintValues(),
        mMeanPenaltyMultipliers(),
        mStdDevPenaltyMultipliers(),
        mMeanLagrangeMultipliers(),
        mStdDevLagrangeMultipliers()
    {
    }

    explicit OutputDataALPSO(const OrdinalType & aNumConstraints) :
        mNumIter(0),
        mAugLagFuncCount(0),
        mBestObjFuncValue(0),
        mMeanObjFuncValue(0),
        mStdDevObjFuncValue(0),
        mBestAugLagFuncValue(0),
        mMeanAugLagFuncValue(0),
        mStdDevAugLagFuncValue(0),
        mTrustRegionMultiplier(0),
        mBestConstraintValues(std::vector<ScalarType>(aNumConstraints, 0)),
        mMeanConstraintValues(std::vector<ScalarType>(aNumConstraints, 0)),
        mStdDevConstraintValues(std::vector<ScalarType>(aNumConstraints, 0)),
        mMeanPenaltyMultipliers(std::vector<ScalarType>(aNumConstraints, 0)),
        mStdDevPenaltyMultipliers(std::vector<ScalarType>(aNumConstraints, 0)),
        mMeanLagrangeMultipliers(std::vector<ScalarType>(aNumConstraints, 0)),
        mStdDevLagrangeMultipliers(std::vector<ScalarType>(aNumConstraints, 0))
    {
    }

    OrdinalType mNumIter;  /*!< number of outer iterations */
    OrdinalType mAugLagFuncCount;  /*!< number of augmented Lagrangian function evaluations */

    ScalarType mBestObjFuncValue;  /*!< best objective function value */
    ScalarType mMeanObjFuncValue;  /*!< mean objective function value */
    ScalarType mStdDevObjFuncValue;  /*!< standard deviation for objective function value */
    ScalarType mBestAugLagFuncValue;  /*!< best augmented Lagrangian function */
    ScalarType mMeanAugLagFuncValue;  /*!< mean augmented Lagrangian function */
    ScalarType mStdDevAugLagFuncValue;  /*!< standard deviation for augmented Lagrangian function */
    ScalarType mTrustRegionMultiplier;  /*!< trust region multiplier */

    std::vector<ScalarType> mBestConstraintValues;  /*!< constraint values at global best particle location */
    std::vector<ScalarType> mMeanConstraintValues;  /*!< mean constraint values */
    std::vector<ScalarType> mStdDevConstraintValues;  /*!< standard deviation for constraint values */
    std::vector<ScalarType> mMeanPenaltyMultipliers;  /*!< mean penalty multipliers */
    std::vector<ScalarType> mStdDevPenaltyMultipliers;  /*!< standard deviation for penalty multipliers */
    std::vector<ScalarType> mMeanLagrangeMultipliers;  /*!< mean Lagrange multipliers */
    std::vector<ScalarType> mStdDevLagrangeMultipliers;  /*!< standard deviation for Lagrange multipliers */
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
            << aData.mObjFuncCount << std::setw(20) << aData.mBestObjFuncValue << std::setw(15)
            << aData.mMeanObjFuncValue << std::setw(15) << aData.mStdDevObjFuncValue << std::setw(15)
            << aData.mTrustRegionMultiplier << "\n" << std::flush;
}

template<typename ScalarType, typename OrdinalType>
inline void print_alpso_constraint_headers(const Plato::OutputDataALPSO<ScalarType, OrdinalType>& aData,
                                           std::ofstream& aOutputFile)
{
    const OrdinalType tNumConstraints = aData.mBestConstraintValues.size();
    for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
    {
        const OrdinalType tConstraintIndex = tIndex + static_cast<OrdinalType>(1);
        const OrdinalType tWidth = tIndex == static_cast<OrdinalType>(0) ? 13 : 12;
//        if(tIndex != static_cast<OrdinalType>(0))
//        {
        aOutputFile << std::setw(tWidth) << "Best(H" << tConstraintIndex << ")" << std::setw(13)
                << "Mean(H" << tConstraintIndex << ")" << std::setw(14) << "StdDev(H" << tConstraintIndex << ")"
                << std::setw(12) << "Mean(P" << tConstraintIndex << ")" << std::setw(14) << "StdDev(P"
                << tConstraintIndex << ")" << std::setw(12) << "Mean(l" << tConstraintIndex << ")" << std::setw(14)
                << "StdDev(l" << tConstraintIndex << ")";
//        }
//        else
//        {
//            const OrdinalType tWidth = tNumConstraints > static_cast<OrdinalType>(1) ? 10 : 13;
//            aOutputFile << std::scientific << std::setprecision(6) << "Mean(H" << tIndex + static_cast<OrdinalType>(1)
//                    << ")" << std::setw(tWidth);
//        }
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
        Plato::pso::is_vector_empty(aData.mBestConstraintValues);
        Plato::pso::is_vector_empty(aData.mMeanConstraintValues);
        Plato::pso::is_vector_empty(aData.mStdDevConstraintValues);
        Plato::pso::is_vector_empty(aData.mMeanPenaltyMultipliers);
        Plato::pso::is_vector_empty(aData.mStdDevPenaltyMultipliers);
        Plato::pso::is_vector_empty(aData.mMeanLagrangeMultipliers);
        Plato::pso::is_vector_empty(aData.mStdDevLagrangeMultipliers);
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
    const OrdinalType tNumConstraints = aData.mBestConstraintValues.size();
    for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
    {
        aOutputFile << std::setw(15) << aData.mBestConstraintValues[tIndex] << std::setw(15)
                << aData.mMeanConstraintValues[tIndex] << std::setw(15) << aData.mStdDevConstraintValues[tIndex]
                << std::setw(15) << aData.mMeanPenaltyMultipliers[tIndex] << std::setw(15)
                << aData.mStdDevPenaltyMultipliers[tIndex] << std::setw(15) << aData.mMeanLagrangeMultipliers[tIndex]
                << std::setw(15) << aData.mStdDevLagrangeMultipliers[tIndex];
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
        Plato::pso::is_vector_empty(aData.mBestConstraintValues);
        Plato::pso::is_vector_empty(aData.mMeanConstraintValues);
        Plato::pso::is_vector_empty(aData.mStdDevConstraintValues);
        Plato::pso::is_vector_empty(aData.mMeanPenaltyMultipliers);
        Plato::pso::is_vector_empty(aData.mStdDevPenaltyMultipliers);
        Plato::pso::is_vector_empty(aData.mMeanLagrangeMultipliers);
        Plato::pso::is_vector_empty(aData.mStdDevLagrangeMultipliers);
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
            << aData.mAugLagFuncCount << std::setw(20) << aData.mBestAugLagFuncValue << std::setw(15)
            << aData.mMeanAugLagFuncValue << std::setw(15) << aData.mStdDevAugLagFuncValue << std::setw(15)
            << aData.mBestObjFuncValue << std::setw(15) << aData.mMeanObjFuncValue << std::setw(15)
            << aData.mStdDevObjFuncValue << std::setw(15) << "*";
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
            << "*" << std::setw(20) << aData.mBestObjFuncValue << std::setw(15)
            << aData.mMeanObjFuncValue << std::setw(15) << aData.mStdDevObjFuncValue << std::setw(15) << "*"
            << std::setw(15) << "*" << std::setw(15) << "*" << std::setw(15) << aData.mTrustRegionMultiplier;
    Plato::pso::print_alpso_inner_constraint_diagnostics(aData, aOutputFile);
    aOutputFile << "\n" << std::flush;
}

} // namespace pso

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
    **********************************************************************************/
    GradFreeRadius() :
        mLimit(1)
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
            mTimeStep(1),
            mLowerBounds(),
            mUpperBounds(),
            mMeanParticlePosition(),
            mStdDevParticlePosition(),
            mGlobalBestParticlePosition(),
            mCurrentParticles(aFactory->control().create()),
            mCurrentVelocities(aFactory->control().create()),
            mPreviousVelocities(aFactory->control().create()),
            mBestParticlePositions(aFactory->control().create()),
            mCommWrapper(aFactory->getCommWrapper().create())
    {
        this->initialize();
    }

    ~ParticleSwarmDataMng()
    {
    }

    OrdinalType getNumParticles() const
    {
        assert(mCurrentParticles.use_count() > static_cast<OrdinalType>(0));
        return (mCurrentParticles->getNumVectors());
    }

    ScalarType getTimeStep() const
    {
        return (mTimeStep);
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentParticles() const
    {
        assert(mCurrentParticles.use_count() > static_cast<OrdinalType>(0));
        return (*mCurrentParticles);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getCurrentParticle(const OrdinalType & aIndex) const
    {
        assert(mCurrentParticles.use_count() > static_cast<OrdinalType>(0));
        return ((*mCurrentParticles)[aIndex]);
    }

    void setCurrentParticle(const OrdinalType & aIndex, const Plato::Vector<ScalarType, OrdinalType> & aParticle)
    {
        assert(mCurrentParticles.use_count() > static_cast<OrdinalType>(0));
        assert(aParticle.size() > static_cast<OrdinalType>(0));
        (*mCurrentParticles)[aIndex].update(static_cast<ScalarType>(1), aParticle, static_cast<ScalarType>(0));
    }

    void setCurrentParticles(const Plato::MultiVector<ScalarType, OrdinalType> & aParticles)
    {
        assert(aParticles.getNumVectors() > static_cast<OrdinalType>(0));
        assert(mCurrentParticles.use_count() > static_cast<OrdinalType>(0));
        Plato::update(static_cast<ScalarType>(1), aParticles, static_cast<ScalarType>(0), *mCurrentParticles);
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentVelocities() const
    {
        assert(mCurrentVelocities.use_count() > static_cast<OrdinalType>(0));
        return (*mCurrentVelocities);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getCurrentVelocity(const OrdinalType & aIndex) const
    {
        assert(mCurrentVelocities.use_count() > static_cast<OrdinalType>(0));
        return ((*mCurrentVelocities)[aIndex]);
    }

    void setCurrentVelocity(const OrdinalType & aIndex, const Plato::Vector<ScalarType, OrdinalType> & aParticleVel)
    {
        assert(mCurrentVelocities.use_count() > static_cast<OrdinalType>(0));
        assert(aParticleVel.size() > static_cast<OrdinalType>(0));
        (*mCurrentVelocities)[aIndex].update(static_cast<ScalarType>(1), aParticleVel, static_cast<ScalarType>(0));
    }

    void setCurrentVelocities(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() > static_cast<OrdinalType>(0));
        assert(mCurrentVelocities.use_count() > static_cast<OrdinalType>(0));
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mCurrentVelocities);
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getPreviousVelocities() const
    {
        assert(mPreviousVelocities.use_count() > static_cast<OrdinalType>(0));
        return (*mPreviousVelocities);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getPreviousVelocity(const OrdinalType & aIndex) const
    {
        assert(mPreviousVelocities.use_count() > static_cast<OrdinalType>(0));
        return ((*mPreviousVelocities)[aIndex]);
    }

    void setPreviousVelocities(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() > static_cast<OrdinalType>(0));
        assert(mPreviousVelocities.use_count() > static_cast<OrdinalType>(0));
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mPreviousVelocities);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getLowerBounds() const
    {
        assert(mLowerBounds.use_count() > static_cast<OrdinalType>(0));
        return (*mLowerBounds);
    }

    void setLowerBounds(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mLowerBounds.use_count() > static_cast<OrdinalType>(0));
        assert(aInput.size() > static_cast<OrdinalType>(0));
        mLowerBounds->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    void setLowerBounds(const ScalarType & aInput)
    {
        assert(mLowerBounds.use_count() > static_cast<OrdinalType>(0));
        mLowerBounds->fill(aInput);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getUpperBounds() const
    {
        assert(mUpperBounds.use_count() > static_cast<OrdinalType>(0));
        return (*mUpperBounds);
    }

    void setUpperBounds(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mUpperBounds.use_count() > static_cast<OrdinalType>(0));
        assert(aInput.size() > static_cast<OrdinalType>(0));
        mUpperBounds->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    void setUpperBounds(const ScalarType & aInput)
    {
        assert(mUpperBounds.use_count() > static_cast<OrdinalType>(0));
        mUpperBounds->fill(aInput);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getParticlePositionMean() const
    {
        assert(mMeanParticlePosition.use_count() > static_cast<OrdinalType>(0));
        return (*mMeanParticlePosition);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getParticlePositionStdDev() const
    {
        assert(mStdDevParticlePosition.use_count() > static_cast<OrdinalType>(0));
        return (*mStdDevParticlePosition);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getGlobalBestParticlePosition() const
    {
        assert(mGlobalBestParticlePosition.use_count() > static_cast<OrdinalType>(0));
        return (*mGlobalBestParticlePosition);
    }

    void setGlobalBestParticlePosition(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));
        assert(mGlobalBestParticlePosition.use_count() > static_cast<OrdinalType>(0));
        mGlobalBestParticlePosition->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    const Plato::Vector<ScalarType, OrdinalType> & getBestParticlePosition(const OrdinalType & aIndex) const
    {
        assert(mBestParticlePositions.use_count() > static_cast<OrdinalType>(0));
        return ((*mBestParticlePositions)[aIndex]);
    }

    void setBestParticlePosition(const OrdinalType & aIndex, const Plato::Vector<ScalarType, OrdinalType> & aInput) const
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));
        assert(mBestParticlePositions.use_count() > static_cast<OrdinalType>(0));
        (*mBestParticlePositions)[aIndex].update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    void setBestParticlePositions(const Plato::MultiVector<ScalarType, OrdinalType> & aInput) const
    {
        assert(aInput.getNumVectors() > static_cast<OrdinalType>(0));
        assert(mBestParticlePositions.use_count() > static_cast<OrdinalType>(0));
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

    /******************************************************************************//**
     * @brief Return a const reference to the distributed memory communication wrapper
     * @return const reference to the distributed memory communication wrapper
    **********************************************************************************/
    const Plato::CommWrapper& getCommWrapper() const
    {
        return (mCommWrapper.operator*());
    }

    void computeBestParticlePositionStatistics()
    {
        this->computeBestParticlePositionMean();
        this->computeBestParticlePositionStdDev();
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

        mGlobalBestParticlePosition = tMyParticle.create();
        mGlobalBestParticlePosition->fill(std::numeric_limits<ScalarType>::max());
        Plato::fill(std::numeric_limits<ScalarType>::max(), *mBestParticlePositions);
    }

    void computeBestParticlePositionMean()
    {
        mMeanParticlePosition->fill(static_cast<ScalarType>(0));
        const OrdinalType tNumParticles = this->getNumParticles();
        for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyParticle = (*mBestParticlePositions)[tParticleIndex];
            const OrdinalType tMyParticleDims = tMyParticle.size();
            for(OrdinalType tDim = 0; tDim < tMyParticleDims; tDim++)
            {
                (*mMeanParticlePosition)[tDim] += tMyParticle[tDim] / static_cast<ScalarType>(tNumParticles);
            }
        }
    }

    void computeBestParticlePositionStdDev()
    {
        mStdDevParticlePosition->fill(static_cast<ScalarType>(0));
        const OrdinalType tNumParticles = this->getNumParticles();
        const OrdinalType tNumControls = mStdDevParticlePosition->size();
        for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyBestParticlePosition = (*mBestParticlePositions)[tParticleIndex];
            for(OrdinalType tDim = 0; tDim < tNumControls; tDim++)
            {
                const ScalarType tMisfit = tMyBestParticlePosition[tDim] - (*mMeanParticlePosition)[tDim];
                (*mStdDevParticlePosition)[tDim] += tMisfit * tMisfit;
            }
        }

        for(OrdinalType tDim = 0; tDim < tNumControls; tDim++)
        {
            const ScalarType tValue = (*mStdDevParticlePosition)[tDim] / static_cast<ScalarType>(tNumParticles - 1);
            (*mStdDevParticlePosition)[tDim] = std::pow(tValue, static_cast<ScalarType>(0.5));
        }
    }

private:
    ScalarType mTimeStep;

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

    virtual void evaluateObjective(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                   Plato::Vector<ScalarType, OrdinalType> & aOutput) = 0;
};
// class ParticleSwarmStageMng

template<typename ScalarType, typename OrdinalType = size_t>
class BoundConstrainedStageMngPSO : public Plato::ParticleSwarmStageMng<ScalarType, OrdinalType>
{
public:
    explicit BoundConstrainedStageMngPSO(const std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> & aObjective) :
            mObjective(aObjective)
    {
    }

    virtual ~BoundConstrainedStageMngPSO()
    {
    }

    void evaluateObjective(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                           Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {
        mObjective->value(aControl, aOutput);
    }

private:
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
            mPenaltyExpansionMultiplier(2),
            mPenaltyContractionMultiplier(0.5),
            mFeasibilityInexactnessTolerance(1e-4),
            mCurrentObjFuncValues(aFactory->objective().create()),
            mPreviousObjFuncValues(aFactory->objective().create()),
            mPenaltyMultipliers(aFactory->dual().create()),
            mLagrangeMultipliers(aFactory->dual().create()),
            mCurrentConstraintValues(aFactory->dual().create()),
            mPreviousConstraintValues(aFactory->dual().create()),
            mObjective(aObjective),
            mConstraints(aConstraints)
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

    const Plato::Vector<ScalarType, OrdinalType> & getCurrentObjFuncValues() const
    {
        assert(mCurrentObjFuncValues.get() != nullptr);
        return(*mCurrentObjFuncValues);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getPreviousObjFuncValues() const
    {
        assert(mPreviousObjFuncValues.get() != nullptr);
        return(*mPreviousObjFuncValues);
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentConstraintValues() const
    {
        assert(mCurrentConstraintValues.get() != nullptr);
        return(*mCurrentConstraintValues);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getCurrentConstraintValue(const OrdinalType & aIndex) const
    {
        assert(mCurrentConstraintValues.get() != nullptr);
        assert(aIndex < mCurrentConstraintValues->getNumVectors());
        return((*mCurrentConstraintValues)[aIndex]);
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getPreviousConstraintValues() const
    {
        assert(mPreviousConstraintValues.get() != nullptr);
        return(*mPreviousConstraintValues);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getPreviousConstraintValue(const OrdinalType & aIndex) const
    {
        assert(mPreviousConstraintValues.get() != nullptr);
        assert(aIndex < mPreviousConstraintValues->getNumVectors());
        return((*mPreviousConstraintValues)[aIndex]);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getPenaltyMultipliers(const OrdinalType & aIndex) const
    {
        assert(mPenaltyMultipliers.get() != nullptr);
        assert(aIndex < mPenaltyMultipliers->getNumVectors());
        return((*mPenaltyMultipliers)[aIndex]);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getLagrangeMultipliers(const OrdinalType & aIndex) const
    {
        assert(mLagrangeMultipliers.get() != nullptr);
        assert(aIndex < mLagrangeMultipliers->getNumVectors());
        return((*mLagrangeMultipliers)[aIndex]);
    }

    void evaluateObjective(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                           Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {
        mNumAugLagFuncEval += aOutput.size();
        // Evaluate objective function
        mPreviousObjFuncValues->update(static_cast<ScalarType>(1), *mCurrentObjFuncValues, static_cast<ScalarType>(0));
        mCurrentObjFuncValues->fill(static_cast<ScalarType>(0));
        mObjective->value(aControl, *mCurrentObjFuncValues);
        aOutput.update(static_cast<ScalarType>(1), *mCurrentObjFuncValues, static_cast<ScalarType>(0));
        // Evaluate constraints
        this->evaluateConstraints(aControl);
        // Evaluate augmented Lagrangian
        this->evaluateAugmentedLagrangian(aOutput);
    }

    void updateLagrangeMultipliers()
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tPenaltyValues = (*mPenaltyMultipliers)[tConstraintIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tConstraintValues = (*mCurrentConstraintValues)[tConstraintIndex];
            Plato::Vector<ScalarType, OrdinalType> & tLagrangeMultipliers = (*mLagrangeMultipliers)[tConstraintIndex];

            const OrdinalType tNumParticles = tConstraintValues.size();
            for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
            {
                const ScalarType tLagMultiplierOverPenalty = -tLagrangeMultipliers[tParticleIndex]
                        / (static_cast<ScalarType>(2) * tPenaltyValues[tParticleIndex]);
                const ScalarType tSuggestedConstraintValue = std::max(tConstraintValues[tParticleIndex], tLagMultiplierOverPenalty);
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
            const Plato::Vector<ScalarType, OrdinalType> & tCurrentConstraint = (*mCurrentConstraintValues)[tConstraintIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tPreviousConstraint = (*mPreviousConstraintValues)[tConstraintIndex];
            Plato::Vector<ScalarType, OrdinalType> & tPenaltyMultipliers = (*mPenaltyMultipliers)[tConstraintIndex];

            const OrdinalType tNumParticles = tPenaltyMultipliers.size();
            for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
            {
                const bool tIsConstraintGreaterThanPrevious = tCurrentConstraint[tParticleIndex] > tPreviousConstraint[tParticleIndex];
                const bool tIsConstraintGreaterThanFeasibilityTol = tCurrentConstraint[tParticleIndex] > mFeasibilityInexactnessTolerance;
                tPenaltyMultipliers[tParticleIndex] = tIsConstraintGreaterThanPrevious || tIsConstraintGreaterThanFeasibilityTol ?
                        mPenaltyExpansionMultiplier * tPenaltyMultipliers[tParticleIndex] : tPenaltyMultipliers[tParticleIndex];

                const bool tIsConstraintLessThanFeasibilityTol = tCurrentConstraint[tParticleIndex] <= mFeasibilityInexactnessTolerance;
                tPenaltyMultipliers[tParticleIndex] = tIsConstraintLessThanFeasibilityTol ?
                        mPenaltyContractionMultiplier * tPenaltyMultipliers[tParticleIndex] : tPenaltyMultipliers[tParticleIndex];
            }
        }
    }

private:
    void initialize()
    {
        Plato::fill(static_cast<ScalarType>(1), *mPenaltyMultipliers);
        Plato::fill(static_cast<ScalarType>(0), *mLagrangeMultipliers);
    }

    /******************************************************************************//**
     * @brief Evaluate inequality constraints
     * @param [in] aControl control multi-vector
    **********************************************************************************/
    void evaluateConstraints(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        Plato::update(static_cast<ScalarType>(1), *mCurrentConstraintValues, static_cast<ScalarType>(0), *mPreviousConstraintValues);

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
    void evaluateAugmentedLagrangian(Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tPenaltyMultipliers = (*mPenaltyMultipliers)[tConstraintIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tLagrangeMultipliers = (*mLagrangeMultipliers)[tConstraintIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tConstraintValues = (*mCurrentConstraintValues)[tConstraintIndex];

            const OrdinalType tNumParticles = mCurrentObjFuncValues->size();
            for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
            {
                const ScalarType tLagMultiplierOverPenalty = -tLagrangeMultipliers[tParticleIndex]
                        / (static_cast<ScalarType>(2) * tPenaltyMultipliers[tParticleIndex]);
                const ScalarType tSuggestedConstraintValue = std::max(tConstraintValues[tParticleIndex], tLagMultiplierOverPenalty);
                const ScalarType tLagrangeMultipliersTimesConstraint = tLagrangeMultipliers[tParticleIndex] * tSuggestedConstraintValue;
                const ScalarType tConstraintTimesConstraint = tSuggestedConstraintValue * tSuggestedConstraintValue;
                const ScalarType tMyConstraintContribution = tLagrangeMultipliersTimesConstraint
                        + tPenaltyMultipliers[tParticleIndex] * tConstraintTimesConstraint;
                aOutput[tParticleIndex] += tMyConstraintContribution;
            }
        }
    }

private:
    OrdinalType mNumAugLagFuncEval;
    ScalarType mPenaltyExpansionMultiplier;
    ScalarType mPenaltyContractionMultiplier;
    ScalarType mFeasibilityInexactnessTolerance;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentObjFuncValues;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mPreviousObjFuncValues;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPenaltyMultipliers;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mLagrangeMultipliers;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentConstraintValues;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousConstraintValues;

    std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> mObjective;
    std::shared_ptr<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>> mConstraints;
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mDualReductionOperations;

private:
    AugmentedLagrangianStageMngPSO(const Plato::AugmentedLagrangianStageMngPSO<ScalarType, OrdinalType>&);
    Plato::AugmentedLagrangianStageMngPSO<ScalarType, OrdinalType> & operator=(const Plato::AugmentedLagrangianStageMngPSO<ScalarType, OrdinalType>&);
};
// class AugmentedLagrangianStageMngPSO

template<typename ScalarType, typename OrdinalType>
inline ScalarType mean(const Plato::ReductionOperations<ScalarType, OrdinalType> & aReductions,
                       const Plato::Vector<ScalarType, OrdinalType> & aInput)
{
    ScalarType tOutput = aReductions.sum(aInput);
    const OrdinalType tNumElements = aInput.size();
    tOutput = tOutput / tNumElements;
    return (tOutput);
}

template<typename ScalarType, typename OrdinalType>
inline ScalarType standard_deviation(const ScalarType & aMean, const Plato::Vector<ScalarType, OrdinalType> & aInput)
{
    ScalarType tOutput = 0;
    const OrdinalType tNumParticles = aInput.size();
    for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
    {
        const ScalarType tMisfit = aInput[tIndex] - aMean;
        tOutput += tMisfit * tMisfit;
    }
    tOutput = tOutput / (tNumParticles - static_cast<OrdinalType>(1));
    tOutput = std::pow(tOutput, static_cast<ScalarType>(0.5));
    return (tOutput);
}

template<typename ScalarType, typename OrdinalType = size_t>
class ParticleSwarmOperations
{
public:
    explicit ParticleSwarmOperations(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory) :
        mNumConsecutiveFailures(0),
        mNumConsecutiveSuccesses(0),
        mMaxNumConsecutiveFailures(10),
        mMaxNumConsecutiveSuccesses(10),
        mGlobalBestParticleIndex(0),
        mInertiaMultiplier(0.9),
        mSocialBehaviorMultiplier(0.8),
        mCognitiveBehaviorMultiplier(0.8),
        mBestObjFunValueMean(std::numeric_limits<ScalarType>::max()),
        mBestObjFunValueStdDev(std::numeric_limits<ScalarType>::max()),
        mCurrentGlobalBestObjFunValue(std::numeric_limits<ScalarType>::max()),
        mPreviousGlobalBestObjFunValue(std::numeric_limits<ScalarType>::max()),
        mTrustRegionMultiplier(1),
        mTrustRegionExpansionMultiplier(4.0),
        mTrustRegionContractionMultiplier(0.75),
        mWorkVector(aFactory->objective().create()),
        mCurrentBestObjFuncValues(aFactory->objective().create()),
        mCurrentObjFuncValues(aFactory->objective().create()),
        mReductions(aFactory->getObjFuncReductionOperations().create())
    {
        this->initialize();
    }

    virtual ~ParticleSwarmOperations()
    {
    }

    ScalarType getInertiaMultiplier() const
    {
        return (mInertiaMultiplier);
    }

    ScalarType getBestObjFuncValueMean() const
    {
        return (mBestObjFunValueMean);
    }

    ScalarType getBestObjFuncValueStdDev() const
    {
        return (mBestObjFunValueStdDev);
    }

    ScalarType getTrustRegionMultiplier() const
    {
        return (mTrustRegionMultiplier);
    }

    ScalarType getCurrentGlobalBestObjFunValue() const
    {
        return (mCurrentGlobalBestObjFunValue);
    }

    ScalarType getPreviousGlobalBestObjFunValue() const
    {
        return (mPreviousGlobalBestObjFunValue);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getCurrentBestObjFuncValues() const
    {
        return (*mCurrentBestObjFuncValues);
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

    void setCurrentGlobalBestObjFunValue(const ScalarType & aInput)
    {
        mCurrentGlobalBestObjFunValue = aInput;
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

    void setGlobalBestParticleIndex(const OrdinalType & aInput)
    {
        mGlobalBestParticleIndex = aInput;
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

    void setCurrentBestObjFuncValues(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        mCurrentBestObjFuncValues->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    void setCurrentObjFuncValues(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        mCurrentObjFuncValues->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
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

    void computeBestObjFuncStatistics()
    {
        mBestObjFunValueMean = Plato::mean(*mReductions, *mCurrentBestObjFuncValues);
        mBestObjFunValueStdDev = Plato::standard_deviation(mBestObjFunValueMean, *mCurrentBestObjFuncValues);
    }

    void findBestParticlePositions(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        mPreviousGlobalBestObjFunValue = mCurrentGlobalBestObjFunValue;
        const OrdinalType tNumParticles = aDataMng.getNumParticles();
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            this->checkBestParticlePosition(tIndex, aDataMng);
            this->checkGlobalBestParticlePosition(tIndex, aDataMng);
        }
        this->checkGlobalBestParticleUpdateSuccessRate();
    }

    void updateParticleVelocities(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        aDataMng.cachePreviousVelocities();
        std::default_random_engine tGenerator;
        std::uniform_real_distribution<ScalarType> tDistribution(0.0 /* lower bound */, 1.0 /* upper bound */);
        const OrdinalType tNumParticles = aDataMng.getNumParticles();
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            if(tIndex != mGlobalBestParticleIndex)
            {
                this->updateParticleVelocity(tIndex, tGenerator, tDistribution, aDataMng);
            }
            else
            {
                assert(tIndex == mGlobalBestParticleIndex);
                this->updateGlobalBestParticleVelocity(tGenerator, tDistribution, aDataMng);
            }
        }
    }

    void updateParticlePositions(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const OrdinalType tNumParticles = aDataMng.getNumParticles();
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            if(tIndex != mGlobalBestParticleIndex)
            {
                this->updateParticlePosition(tIndex, aDataMng);
            }
            else
            {
                assert(tIndex == mGlobalBestParticleIndex);
                this->updateGlobalBestParticlePosition(aDataMng);
            }
        }
    }

private:
    void initialize()
    {
        mCurrentBestObjFuncValues->fill(std::numeric_limits<ScalarType>::max());
        mCurrentObjFuncValues->fill(std::numeric_limits<ScalarType>::max());
    }

    void checkGlobalBestParticleUpdateSuccessRate()
    {
        if(mCurrentGlobalBestObjFunValue < mPreviousGlobalBestObjFunValue)
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

    void checkBestParticlePosition(const OrdinalType & aParticleIndex,
                                   Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        if((*mCurrentObjFuncValues)[aParticleIndex] < (*mCurrentBestObjFuncValues)[aParticleIndex])
        {
            const Plato::Vector<ScalarType, OrdinalType> & tCurrentParticle = aDataMng.getCurrentParticle(aParticleIndex);
            aDataMng.setBestParticlePosition(aParticleIndex, tCurrentParticle);
            (*mCurrentBestObjFuncValues)[aParticleIndex] = (*mCurrentObjFuncValues)[aParticleIndex];
        }
    }

    void checkGlobalBestParticlePosition(const OrdinalType & aParticleIndex,
                                         Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const ScalarType tCurrentParticleObjFuncValue = (*mCurrentObjFuncValues)[aParticleIndex];
        if(tCurrentParticleObjFuncValue < mCurrentGlobalBestObjFunValue)
        {
            mCurrentGlobalBestObjFunValue = tCurrentParticleObjFuncValue;
            const Plato::Vector<ScalarType, OrdinalType> & tCurrentParticle = aDataMng.getCurrentParticle(aParticleIndex);
            aDataMng.setGlobalBestParticlePosition(tCurrentParticle);
            mGlobalBestParticleIndex = aParticleIndex;
        }
    }

    void updateParticleVelocity(const OrdinalType & aParticleIndex,
                                std::default_random_engine & aGenerator,
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
            const ScalarType tRandomNumOne = aDistribution(aGenerator);
            const ScalarType tCognitiveMultiplier = tRandomNumOne * mCognitiveBehaviorMultiplier;
            const ScalarType tCognitiveValue = tCognitiveMultiplier * (tBestParticlePosition[tIndex] - tCurrentParticle[tIndex]);
            // social behavior contribution
            const ScalarType tRandomNumTwo = aDistribution(aGenerator);
            const ScalarType tSocialMultiplier = tRandomNumTwo * mSocialBehaviorMultiplier;
            const ScalarType tSocialValue = tSocialMultiplier * (tGlobalBestParticlePosition[tIndex] - tCurrentParticle[tIndex]);
            // set new velocity
            (*mWorkVector)[tIndex] = tInertiaValue + tCognitiveValue + tSocialValue;
        }

        aDataMng.setCurrentVelocity(aParticleIndex, *mWorkVector);
    }

    void updateGlobalBestParticleVelocity(std::default_random_engine & aGenerator,
                                          std::uniform_real_distribution<ScalarType> & aDistribution,
                                          Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const Plato::Vector<ScalarType, OrdinalType> & tPreviousVel = aDataMng.getPreviousVelocity(mGlobalBestParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tGlobalBestParticlePosition = aDataMng.getGlobalBestParticlePosition();
        const Plato::Vector<ScalarType, OrdinalType> & tCurrentParticlePosition = aDataMng.getCurrentParticle(mGlobalBestParticleIndex);

        const OrdinalType tNumControls = tPreviousVel.size();
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            const ScalarType tRandomNum = aDistribution(aGenerator);
            const ScalarType tStochasticTrustRegionMultiplier = mTrustRegionMultiplier
                    * (static_cast<ScalarType>(1) - static_cast<ScalarType>(2) * tRandomNum);
            (*mWorkVector)[tIndex] = (static_cast<ScalarType>(-1) * tCurrentParticlePosition[tIndex]) + tGlobalBestParticlePosition[tIndex]
                    + (mInertiaMultiplier * tPreviousVel[tIndex]) + tStochasticTrustRegionMultiplier;
        }

        aDataMng.setCurrentVelocity(mGlobalBestParticleIndex, *mWorkVector);
    }

    void updateParticlePosition(const OrdinalType & aParticleIndex,
                                Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const ScalarType tTimeStep = aDataMng.getTimeStep();
        const Plato::Vector<ScalarType, OrdinalType> & tLowerBounds = aDataMng.getLowerBounds();
        const Plato::Vector<ScalarType, OrdinalType> & tUpperBounds = aDataMng.getUpperBounds();
        const Plato::Vector<ScalarType, OrdinalType> & tParticleVel = aDataMng.getCurrentVelocity(aParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tParticlePosition = aDataMng.getCurrentParticle(aParticleIndex);
        mWorkVector->update(static_cast<ScalarType>(1), tParticlePosition, static_cast<ScalarType>(0));

        const OrdinalType tNumControls = tParticleVel.size();
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            (*mWorkVector)[tIndex] = (*mWorkVector)[tIndex] + (tTimeStep * tParticleVel[tIndex]);
            (*mWorkVector)[tIndex] = std::max((*mWorkVector)[tIndex], tLowerBounds[tIndex]);
            (*mWorkVector)[tIndex] = std::min((*mWorkVector)[tIndex], tUpperBounds[tIndex]);
        }

        aDataMng.setCurrentParticle(aParticleIndex, *mWorkVector);
    }

    void updateGlobalBestParticlePosition(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const Plato::Vector<ScalarType, OrdinalType> & tLowerBounds = aDataMng.getLowerBounds();
        const Plato::Vector<ScalarType, OrdinalType> & tUpperBounds = aDataMng.getUpperBounds();
        const Plato::Vector<ScalarType, OrdinalType> & tGlobalBestParticleVel = aDataMng.getCurrentVelocity(mGlobalBestParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tGlobalBestParticlePosition = aDataMng.getGlobalBestParticlePosition();

        std::default_random_engine tGenerator;
        std::uniform_real_distribution<ScalarType> tDistribution(0.0 /* lower bound */, 1.0 /* upper bound */);
        const ScalarType tRandomNum = tDistribution(tGenerator);
        const ScalarType tStochasticTrustRegionMultiplier = mTrustRegionMultiplier
                * (static_cast<ScalarType>(1) - static_cast<ScalarType>(2) * tRandomNum);

        const OrdinalType tNumControls = tGlobalBestParticlePosition.size();
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            (*mWorkVector)[tIndex] = tGlobalBestParticlePosition[tIndex]
                    + (mInertiaMultiplier * tGlobalBestParticleVel[tIndex]) + tStochasticTrustRegionMultiplier;
        }

        aDataMng.setCurrentParticle(mGlobalBestParticleIndex, *mWorkVector);
    }

private:
    OrdinalType mNumConsecutiveFailures;
    OrdinalType mNumConsecutiveSuccesses;
    OrdinalType mMaxNumConsecutiveFailures;
    OrdinalType mMaxNumConsecutiveSuccesses;

    OrdinalType mGlobalBestParticleIndex;

    ScalarType mInertiaMultiplier;
    ScalarType mSocialBehaviorMultiplier;
    ScalarType mCognitiveBehaviorMultiplier;

    ScalarType mBestObjFunValueMean;
    ScalarType mBestObjFunValueStdDev;

    ScalarType mCurrentGlobalBestObjFunValue;
    ScalarType mPreviousGlobalBestObjFunValue;

    ScalarType mTrustRegionMultiplier;
    ScalarType mTrustRegionExpansionMultiplier;
    ScalarType mTrustRegionContractionMultiplier;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mWorkVector;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentBestObjFuncValues;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentObjFuncValues;

    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mReductions;

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
            mWorkVector(aFactory->objective().create()),
            mDataMng(std::make_shared<Plato::ParticleSwarmDataMng<ScalarType, OrdinalType>>(aFactory)),
            mStageMng(std::make_shared<Plato::BoundConstrainedStageMngPSO<ScalarType, OrdinalType>>(aObjective)),
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
            mWorkVector(aFactory->objective().create()),
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

    OrdinalType getNumIterations() const
    {
        return (mNumIterations);
    }

    ScalarType getBestObjFuncValueMean() const
    {
        return (mOperations->getBestObjFuncValueMean());
    }

    ScalarType getBestObjFuncValueStdDev() const
    {
        return (mOperations->getBestObjFuncValueStdDev());
    }

    ScalarType getBestObjFuncValue() const
    {
        return (mOperations->getCurrentGlobalBestObjFunValue());
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
        assert(mDataMng.use_count() > static_cast<OrdinalType>(0));
        assert(mStageMng.use_count() > static_cast<OrdinalType>(0));

        this->openOutputFile();
        this->initialize();

        mNumIterations = 0;
        while(1)
        {
            mNumIterations++;
            mStageMng->evaluateObjective(mDataMng->getCurrentParticles(), *mWorkVector);
            mOperations->setCurrentObjFuncValues(*mWorkVector);
            mOperations->findBestParticlePositions(*mDataMng);
            mOperations->updateParticleVelocities(*mDataMng);
            mOperations->updateParticlePositions(*mDataMng);
            mOperations->updateTrustRegionMultiplier();
            mOperations->computeBestObjFuncStatistics();
            this->outputDiagnostics();

            if(this->checkStoppingCriteria())
            {
                mDataMng->computeBestParticlePositionStatistics();
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
            mStageMng->evaluateObjective(mDataMng->getCurrentParticles(), *mWorkVector);
            mOperations->setCurrentObjFuncValues(*mWorkVector);
            mOperations->findBestParticlePositions(*mDataMng);
            mOperations->updateParticleVelocities(*mDataMng);
            mOperations->updateParticlePositions(*mDataMng);
            mOperations->updateTrustRegionMultiplier();
            mOperations->computeBestObjFuncStatistics();
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
        const ScalarType tBestObjFunValueMean = mOperations->getBestObjFuncValueMean();
        const ScalarType tCurrentGlobalBestObjFunValue = mOperations->getCurrentGlobalBestObjFunValue();

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
        mOutputData.mBestObjFuncValue = mOperations->getCurrentGlobalBestObjFunValue();
        mOutputData.mMeanObjFuncValue = mOperations->getBestObjFuncValueMean();
        mOutputData.mStdDevObjFuncValue = mOperations->getBestObjFuncValueStdDev();
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
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mWorkVector;
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
            mMaxNumIterations(1000),
            mBestObjFuncTolerance(1e-10),
            mMeanObjFuncTolerance(5e-4),
            mStopCriterion(Plato::particle_swarm::DID_NOT_CONVERGE),
            mStageMng(std::make_shared<Plato::AugmentedLagrangianStageMngPSO<ScalarType, OrdinalType>>(aFactory, aObjective, aConstraints)),
            mOptimizer(std::make_shared<Plato::BoundConstrainedPSO<ScalarType, OrdinalType>>(aFactory, mStageMng))
    {
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
    }

    void setMaxNumIterations(const OrdinalType & aInput)
    {
        mMaxNumIterations = aInput;
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

    ScalarType getBestObjFuncValueMean() const
    {
        return (mOptimizer->getBestObjFuncValueMean());
    }

    ScalarType getBestObjFuncValueStdDev() const
    {
        return (mOptimizer->getBestObjFuncValueStdDev());
    }

    ScalarType getBestObjFuncValue() const
    {
        return (mOptimizer->getCurrentGlobalBestObjFunValue());
    }

    const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & getDataMng() const
    {
        return (mOptimizer->getDataMng());
    }

    /******************************************************************************//**
     * @brief Solve particle swarm optimization problem
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
            mOptimizer->solve();
            mStageMng->updatePenaltyMultipliers();
            mStageMng->updateLagrangeMultipliers();

            this->outputDiagnostics();
            if(this->checkStoppingCriteria())
            {
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
        const ScalarType tBestObjFunValueMean = mOptimizer->getBestObjFuncValueMean();
        const ScalarType tCurrentGlobalBestObjFunValue = mOptimizer->getCurrentGlobalBestObjFunValue();

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
            const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & tDataMng = mOptimizer->getDataMng();
            const Plato::CommWrapper& tMyCommWrapper = tDataMng->getCommWrapper();
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
            const Plato::CommWrapper& tMyCommWrapper = tDataMng->getCommWrapper();
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
            const Plato::CommWrapper& tMyCommWrapper = tDataMng->getCommWrapper();
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
        const Plato::CommWrapper& tMyCommWrapper = tDataMng->getCommWrapper();
        if(tMyCommWrapper.myProcID() == 0)
        {
            mOutputData.mNumIter = mNumIterations;
            mOutputData.mAugLagFuncEval = mStageMng->getNumAugLagFuncEvaluations();
            mOutputData.mBestObjFuncValue = mOptimizer->getCurrentGlobalBestObjFunValue();
            mOutputData.mMeanObjFuncValue = mOptimizer->getBestObjFuncValueMean();
            mOutputData.mStdDevObjFuncValue = mOptimizer->getBestObjFuncValueStdDev();
            mOutputData.mTrustRegionMultiplier = mOptimizer->getTrustRegionMultiplier();

            Plato::pso::print_alpso_outer_diagnostics(mOutputData, mOutputStream, mPrintDiagnostics);
        }
    }

private:
    bool mPrintDiagnostics;
    std::ofstream mOutputStream;

    OrdinalType mNumIterations;
    OrdinalType mMaxNumIterations;

    ScalarType mBestObjFuncTolerance;
    ScalarType mMeanObjFuncTolerance;

    Plato::particle_swarm::stop_t mStopCriterion;
    Plato::OutputDataALPSO<ScalarType, OrdinalType> mOutputData;

    std::shared_ptr<Plato::AugmentedLagrangianStageMngPSO<ScalarType, OrdinalType>> mStageMng;
    std::shared_ptr<Plato::BoundConstrainedPSO<ScalarType, OrdinalType>> mOptimizer;

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
    std::ofstream tFile;
    ASSERT_THROW(Plato::pso::is_file_open(tFile), std::invalid_argument);

    tFile.open("MyFile.txt");
    ASSERT_NO_THROW(Plato::pso::is_file_open(tFile));
    tFile.close();
    std::system("rm -f MyFile.txt");
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
    std::ofstream tWriteFile;
    tWriteFile.open("MyFile1.txt");
    Plato::OutputDataPSO<double> tData;
    tData.mNumIter = 1;
    tData.mObjFuncCount = 20;
    tData.mBestObjFuncValue = 4.2321;
    tData.mMeanObjFuncValue = 8.2321;
    tData.mStdDevObjFuncValue = 2.2321;
    tData.mTrustRegionMultiplier = 1.0;
    ASSERT_NO_THROW(Plato::pso::print_pso_diagnostics_header(tData, tWriteFile));
    ASSERT_NO_THROW(Plato::pso::print_pso_diagnostics(tData, tWriteFile));

    tData.mNumIter = 2;
    tData.mObjFuncCount = 40;
    tData.mBestObjFuncValue = 2.2321;
    tData.mMeanObjFuncValue = 7.2321;
    tData.mStdDevObjFuncValue = 2.4321;
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
    std::ofstream tWriteFile;
    tWriteFile.open("MyFile1.txt");
    Plato::OutputDataPSO<double> tDataPSO;
    const size_t tNumConstraints = 2;
    Plato::OutputDataALPSO<double> tDataALPSO(tNumConstraints);
    tDataPSO.mNumConstraints = tNumConstraints;

    // **** AUGMENTED LAGRANGIAN OUTPUT ****
    tDataPSO.mNumIter = 0;
    tDataPSO.mObjFuncCount = 1;
    tDataPSO.mBestObjFuncValue = 1;
    tDataPSO.mMeanObjFuncValue = 1.5;
    tDataPSO.mStdDevObjFuncValue = 2.34e-2;
    tDataPSO.mTrustRegionMultiplier = 0.5;
    ASSERT_NO_THROW(Plato::pso::print_alpso_diagnostics_header(tDataALPSO, tWriteFile));
    ASSERT_NO_THROW(Plato::pso::print_alpso_outer_diagnostics(tDataALPSO, tWriteFile));

    tDataPSO.mNumIter = 1;
    tDataPSO.mObjFuncCount = 10;
    tDataPSO.mBestObjFuncValue = 0.1435;
    tDataPSO.mMeanObjFuncValue = 0.78;
    tDataPSO.mStdDevObjFuncValue = 0.298736;
    tDataPSO.mTrustRegionMultiplier = 3.45656e-1;
    ASSERT_NO_THROW(Plato::pso::print_alpso_inner_diagnostics(tDataPSO, tWriteFile));

    // **** AUGMENTED LAGRANGIAN OUTPUT ****
    tDataALPSO.mNumIter = 1;
    tDataALPSO.mAugLagFuncCount = 10;
    tDataALPSO.mBestAugLagFuncValue = 1.2359e-1;
    tDataALPSO.mMeanAugLagFuncValue = 3.2359e-1;
    tDataALPSO.mStdDevAugLagFuncValue = 3.2359e-2;
    tDataALPSO.mBestObjFuncValue = 8.2359e-2;
    tDataALPSO.mMeanObjFuncValue = 9.2359e-2;
    tDataALPSO.mStdDevObjFuncValue = 2.2359e-2;
    tDataALPSO.mBestConstraintValues[0] = 1.23e-5;
    tDataALPSO.mBestConstraintValues[1] = 3.65e-3;
    tDataALPSO.mMeanConstraintValues[0] = 4.23e-5;
    tDataALPSO.mMeanConstraintValues[1] = 6.65e-3;
    tDataALPSO.mStdDevConstraintValues[0] = 1.23e-5;
    tDataALPSO.mStdDevConstraintValues[1] = 8.65e-4;
    tDataALPSO.mMeanPenaltyMultipliers[0] = 1;
    tDataALPSO.mMeanPenaltyMultipliers[1] = 2;
    tDataALPSO.mStdDevPenaltyMultipliers[0] = 0.25;
    tDataALPSO.mStdDevPenaltyMultipliers[1] = 0.1;
    tDataALPSO.mMeanLagrangeMultipliers[0] = 1.23e-2;
    tDataALPSO.mMeanLagrangeMultipliers[1] = 8.65e-1;
    tDataALPSO.mStdDevLagrangeMultipliers[0] = 9.23e-3;
    tDataALPSO.mStdDevLagrangeMultipliers[1] = 5.65e-1;
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
    Plato::ParticleSwarmOperations<double> tOperations(tFactory);
    tOperations.setCurrentBestObjFuncValues(tBestObjFuncValues);
    tOperations.computeBestObjFuncStatistics();

    const double tTolerance = 1e-6;
    EXPECT_NEAR(0.00010692, tOperations.getBestObjFuncValueMean(), tTolerance);
    EXPECT_NEAR(0.000354175, tOperations.getBestObjFuncValueStdDev(), tTolerance);
}

TEST(PlatoTest, PSO_findBestParticlePositions)
{
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 5;
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateControl(tNumControls, tNumParticles);

    Plato::ParticleSwarmOperations<double> tOperations(tFactory);

    tOperations.setCurrentGlobalBestObjFunValue(0.000423009);
    std::vector<double> tData = { 0.00044607, 0.0639247, 3.9283e-05, 0.0318453, 0.000420515 };
    Plato::StandardVector<double> tCurrentObjFuncValues(tData);
    tOperations.setCurrentObjFuncValues(tCurrentObjFuncValues);

    tData = { 0.000423009, 0.0008654, 0.00174032, 0.000871822, 0.000426448 };
    Plato::StandardVector<double> tBestObjFuncValues(tData);
    tOperations.setCurrentBestObjFuncValues(tBestObjFuncValues);

    Plato::ParticleSwarmDataMng<double> tDataMng(tFactory);
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

    tOperations.findBestParticlePositions(tDataMng);

    const double tTolerance = 1e-6;
    EXPECT_NEAR(3.9283e-05, tOperations.getCurrentGlobalBestObjFunValue(), tTolerance);
    EXPECT_NEAR(0.000423009, tOperations.getPreviousGlobalBestObjFunValue(), tTolerance);
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
    EXPECT_NEAR(0.000423009, tOperations.getCurrentBestObjFuncValues()[0], tTolerance);
    EXPECT_NEAR(0.0008654, tOperations.getCurrentBestObjFuncValues()[1], tTolerance);
    EXPECT_NEAR(3.9283e-05, tOperations.getCurrentBestObjFuncValues()[2], tTolerance);
    EXPECT_NEAR(0.000871822, tOperations.getCurrentBestObjFuncValues()[3], tTolerance);
    EXPECT_NEAR(0.000420515, tOperations.getCurrentBestObjFuncValues()[4], tTolerance);
}

TEST(PlatoTest, PSO_BoundConstrainedStageMng)
{
    const size_t tNumControls = 2;
    const size_t tNumParticles = 1;
    Plato::StandardVector<double> tObjFuncValues(tNumParticles);
    Plato::StandardMultiVector<double> tControl(tNumParticles, tNumControls, 2.0 /* initial guess */);

    std::shared_ptr<Plato::GradFreeRosenbrock<double>> tObjective = std::make_shared<Plato::GradFreeRosenbrock<double>>();
    Plato::BoundConstrainedStageMngPSO<double> tStageMng(tObjective);
    tStageMng.evaluateObjective(tControl, tObjFuncValues);

    const double tTolerance = 1e-6;
    EXPECT_NEAR(401, tObjFuncValues[0], tTolerance);
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

    // FIRST CALL
    Plato::StandardVector<double> tLagrangianValues(tNumParticles);
    Plato::StandardMultiVector<double> tControl(tNumParticles, tNumControls);
    tControl[0].fill(0.725);
    tControl[1].fill(0.705);
    tStageMng.evaluateObjective(tControl, tLagrangianValues);

    const double tTolerance = 1e-6;
    EXPECT_NEAR(4.053290625, tLagrangianValues[0], tTolerance);
    EXPECT_NEAR(4.4123850625, tLagrangianValues[1], tTolerance);
    EXPECT_NEAR(4.0506640625, tStageMng.getCurrentObjFuncValues()[0], tTolerance);
    EXPECT_NEAR(4.4123850625, tStageMng.getCurrentObjFuncValues()[1], tTolerance);

    tStageMng.updatePenaltyMultipliers();
    EXPECT_NEAR(2, tStageMng.getPenaltyMultipliers(0 /* constraint index */)[0], tTolerance);
    EXPECT_NEAR(0.5, tStageMng.getPenaltyMultipliers(0 /* constraint index */)[1], tTolerance);

    tStageMng.updateLagrangeMultipliers();
    EXPECT_NEAR(0.205, tStageMng.getLagrangeMultipliers(0 /* constraint index */)[0], tTolerance);
    EXPECT_NEAR(0, tStageMng.getLagrangeMultipliers(0 /* constraint index */)[1], tTolerance);

    // SECOND CALL
    tControl[0].fill(0.715);
    tControl[1].fill(0.695);
    tStageMng.evaluateObjective(tControl, tLagrangianValues);
    EXPECT_NEAR(4.2392603175, tLagrangianValues[0], tTolerance);
    EXPECT_NEAR(4.5863650625, tLagrangianValues[1], tTolerance);
    EXPECT_NEAR(4.2336500625, tStageMng.getCurrentObjFuncValues()[0], tTolerance);
    EXPECT_NEAR(4.5863650625, tStageMng.getCurrentObjFuncValues()[1], tTolerance);
    EXPECT_NEAR(4.0506640625, tStageMng.getPreviousObjFuncValues()[0], tTolerance);
    EXPECT_NEAR(4.4123850625, tStageMng.getPreviousObjFuncValues()[1], tTolerance);
    EXPECT_NEAR(0.02245, tStageMng.getCurrentConstraintValue(0 /* constraint index */)[0], tTolerance);
    EXPECT_NEAR(-0.03395, tStageMng.getCurrentConstraintValue(0 /* constraint index */)[1], tTolerance);
    EXPECT_NEAR(0.05125, tStageMng.getPreviousConstraintValue(0 /* constraint index */)[0], tTolerance);
    EXPECT_NEAR(-0.00595, tStageMng.getPreviousConstraintValue(0 /* constraint index */)[1], tTolerance);

    tStageMng.updatePenaltyMultipliers();
    EXPECT_NEAR(4, tStageMng.getPenaltyMultipliers(0 /* constraint index */)[0], tTolerance);
    EXPECT_NEAR(0.25, tStageMng.getPenaltyMultipliers(0 /* constraint index */)[1], tTolerance);

    tStageMng.updateLagrangeMultipliers();
    EXPECT_NEAR(0.3846, tStageMng.getLagrangeMultipliers(0 /* constraint index */)[0], tTolerance);
    EXPECT_NEAR(0, tStageMng.getLagrangeMultipliers(0 /* constraint index */)[1], tTolerance);
}

TEST(PlatoTest, PSO_Solve)
{
    // ********* Allocate Core Optimization Data Templates *********
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 20;
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateControl(tNumControls, tNumParticles);

    // TEST ALGORITHM
    std::shared_ptr<Plato::GradFreeRosenbrock<double>> tObjective = std::make_shared<Plato::GradFreeRosenbrock<double>>();
    Plato::BoundConstrainedPSO<double> tAlgorithm(tFactory, tObjective);
    tAlgorithm.setLowerBounds(-6);
    tAlgorithm.setUpperBounds(6);
    tAlgorithm.solve();

    const double tTolerance = 1e-4;
    EXPECT_NEAR(0, tAlgorithm.getBestObjFuncValue(), tTolerance);

    std::cout << "NUM ITERATIONS = " << tAlgorithm.getNumIterations() << "\n";
    std::cout << "OBJECTIVE: BEST = " << tAlgorithm.getBestObjFuncValue() << ", MEAN = "
            << tAlgorithm.getBestObjFuncValueMean() << ", STDDEV = " << tAlgorithm.getBestObjFuncValueStdDev() << "\n";
    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = "
                << tAlgorithm.getDataMng().getGlobalBestParticlePosition()[tIndex] << ", MEAN = "
                << tAlgorithm.getDataMng().getParticlePositionMean()[tIndex] << ", STDDEV = "
                << tAlgorithm.getDataMng().getParticlePositionStdDev()[tIndex] << "\n";
    }
}

} // ParticleSwarmTest
