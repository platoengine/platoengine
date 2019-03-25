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
 //
 // *************************************************************************
 //@HEADER
*/

/*
 * Plato_ParticleSwarmIO_Utilities.hpp
 *
 *  Created on: Jan 24, 2019
 */

#pragma once

#include <string>
#include <iomanip>
#include <fstream>

#include "Plato_ParticleSwarmIO_BCPSO.hpp"
#include "Plato_ParticleSwarmIO_ALPSO.hpp"

namespace Plato
{

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
        case Plato::particle_swarm::stop_t::TRUST_REGION_MULTIPLIER_TOLERANCE:
        {
            aOutput = "\n\n****** Optimization stopping due to trust region multiplier tolerance being met. ******\n\n";
            break;
        }
    }
}
// function print_bcpso_diagnostics_header

/******************************************************************************//**
 * @brief Print header for bound constrained Particle Swarm Optimization (BCPSO) diagnostics file
 * @param [in,out] aOutputFile output file
 * @param [in] aPrint flag use to enable/disable output (default = disabled)
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void print_bcpso_diagnostics_header(const Plato::DiagnosticsBCPSO<ScalarType, OrdinalType>& aData,
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
inline void print_bcpso_diagnostics(const Plato::DiagnosticsBCPSO<ScalarType, OrdinalType>& aData,
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
inline void print_alpso_constraint_headers(const Plato::DiagnosticsALPSO<ScalarType, OrdinalType>& aData,
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
inline void print_alpso_diagnostics_header(const Plato::DiagnosticsALPSO<ScalarType, OrdinalType>& aData,
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
inline void print_alpso_outer_constraint_diagnostics(const Plato::DiagnosticsALPSO<ScalarType, OrdinalType>& aData,
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
inline void print_alpso_outer_diagnostics(const Plato::DiagnosticsALPSO<ScalarType, OrdinalType>& aData,
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
inline void print_alpso_inner_constraint_diagnostics(const Plato::DiagnosticsBCPSO<ScalarType, OrdinalType>& aData,
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
inline void print_alpso_inner_diagnostics(const Plato::DiagnosticsBCPSO<ScalarType, OrdinalType>& aData,
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

/******************************************************************************//**
 * @brief Output solution (i.e. optimization variables) to file.
 * @param [in] aBest 1D container with global best solution (i.e. optimization variables)
 * @param [in] aMean 1D container with the mean of the best set of solutions
 * @param [in] aStdDev 1D container with the standard deviation of the best set of solutions
 * @return indicate if solution was print to file
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline bool print_solution(const Plato::Vector<ScalarType, OrdinalType> & aBest,
                           const Plato::Vector<ScalarType, OrdinalType> & aMean,
                           const Plato::Vector<ScalarType, OrdinalType> & aStdDev)
{
    bool tPrintSolution = false;
    const OrdinalType tCheck = aBest.size() * aMean.size() * aStdDev.size();

    if(tCheck > static_cast<OrdinalType>(0))
    {
        std::ofstream tOutputStream;
        tOutputStream.open("plato_pso_solution.txt");

        tOutputStream << std::scientific << std::setprecision(6) << std::right << "Best(X)" << std::setw(18) << "Mean(X)"
                    << std::setw(15) << "StdDev(X)" << "\n" << std::flush;

        const OrdinalType tNumControls = aBest.size();
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            tOutputStream << std::scientific << std::setprecision(6) << std::right << aBest[tIndex] << std::setw(15)
            << aMean[tIndex] << std::setw(15) << aStdDev[tIndex] << "\n" << std::flush;
        }

        tOutputStream.close();
        tPrintSolution = true;
    }

    return (tPrintSolution);
}
// function print_solution

/******************************************************************************//**
 * @brief Print header for diagnostic file with particle data history
 * @param [in,out] aOutputFile output file
**********************************************************************************/
inline void print_particle_data_header(std::ofstream & aOutputFile)
{
    aOutputFile << "OUTPUT FORMAT: (F_i(X), X_i^j, ..., X_i^J) ... (F_I(X), X_I^j, ..., X_I^J)\n";
    aOutputFile << "The subscript i denotes the particle index and the superscript j denotes the design variable index.\n";
    aOutputFile << "Each particle is associated with a criterion (F_i(X)) and a set of design variables (X). The total\n";
    aOutputFile << "number of particles and design variables is denoted by uppercase letters I and J, respectively.\n\n\n\n";
    aOutputFile << std::right << "Iter    " << "(F_i(X), X_i^j, ..., X_i^J) ... (F_I(X), X_I^j, ..., X_I^J)\n";
}
// function print_particle_data_header

/******************************************************************************//**
 * @brief Print particle data and append information to existing file.
 * @param [in] aIteration current outer optimization iteration
 * @param [in] aCriteriaValues 1D container with criteria values
 * @param [in] aParticlePositions 2D container with particle positions
 * @param [in,out] aOutputFile output file
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void print_particle_data(const OrdinalType & aIteration,
                                const Plato::Vector<ScalarType, OrdinalType> & aCriteriaValues,
                                const Plato::MultiVector<ScalarType, OrdinalType> & aParticlePositions,
                                std::ofstream & aOutputFile)
{
    aOutputFile << std::scientific << std::setprecision(6) << std::right << aIteration << std::setw(8);
    const OrdinalType tNumParticles = aParticlePositions.getNumVectors();
    for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
    {
        aOutputFile << "(" << aCriteriaValues[tParticleIndex] << ", ";
        const OrdinalType tNumControls = aParticlePositions[tParticleIndex].size();
        for(OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
        {
            aOutputFile << aParticlePositions(tParticleIndex, tControlIndex);
            std::string tChar = tControlIndex < tNumControls - static_cast<OrdinalType>(1) ? ", " : ")";
            aOutputFile << tChar.c_str();
        }
        aOutputFile << "  ";
    }
    aOutputFile << "\n" << std::flush;
}
// function print_particle_data

/******************************************************************************//**
 * @brief Print header for diagnostic file with global best particle data history
 * @param [in,out] aOutputFile output file
**********************************************************************************/
inline void print_global_best_particle_data_header(std::ofstream & aOutputFile)
{
    aOutputFile << "OUTPUT FORMAT: (F(X), X^j, ..., X^J)\n";
    aOutputFile << "The superscript j denotes the design variable index. Each particle is associated with a set of\n";
    aOutputFile << "design variables (X). The total number of design variables is denoted by the uppercase letter J.";
    aOutputFile << "\n\n\n\n";
    aOutputFile << std::right << "Iter" << std::setw(18) <<  "Particle Index" << std::setw(28) << "(F(X), X^j, ..., X^J)\n";
}
// function print_global_best_particle_data_header

/******************************************************************************//**
 * @brief Print global best particle data and append information to existing file.
 * @param [in] aIteration current outer optimization iteration
 * @param [in] aParticleIndex global best particle index
 * @param [in] aCriteriaValue global best criteria value
 * @param [in] aParticlePositions 1D container with global best particle positions
 * @param [in,out] aOutputFile output file
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void print_global_best_particle_data(const OrdinalType & aIteration,
                                            const OrdinalType & aParticleIndex,
                                            const ScalarType & aCriterionValue,
                                            const Plato::Vector<ScalarType, OrdinalType> & aParticlePositions,
                                            std::ofstream & aOutputFile)
{
    aOutputFile << std::scientific << std::setprecision(6) << std::right << aIteration << std::setw(12) <<
            aParticleIndex << std::setw(14);
    aOutputFile << "(" << aCriterionValue << ", ";
    const OrdinalType tNumControls = aParticlePositions.size();
    for(OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
    {
        aOutputFile << aParticlePositions[tControlIndex];
        std::string tChar = tControlIndex < tNumControls - static_cast<OrdinalType>(1) ? ", " : ")";
        aOutputFile << tChar.c_str();
    }
    aOutputFile << "  ";
    aOutputFile << "\n" << std::flush;
}
// function print_global_best_particle_data

} // namespace pso

} // namespace Plato
