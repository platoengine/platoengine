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
 * Plato_Test_OptimizersIO.cpp
 *
 *  Created on: Sep 15, 2018
 */

#include "gtest/gtest.h"

#include <sstream>
#include <fstream>
#include <exception>
#include <stdexcept>

namespace Plato
{

namespace error
{

/******************************************************************************//**
 *
 * @brief Check if output file is open. An error is thrown if output file is not open.
 * @param [in] aOutputFile output file
 *
**********************************************************************************/
void is_file_open(const std::ofstream& aOutputFile)
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
 *
 * @brief Check if vector is empty. An error is thrown if vector is empty.
 * @param [in] aInput a vector
 *
**********************************************************************************/
template<typename ScalarType>
void is_vector_empty(const std::vector<ScalarType>& aInput)
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

} // namespace error

/******************************************************************************//**
 *
 * @brief Diagnostic data for Conservative Convex Separable Approximation algorithms (CCSA)
 *
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct OutputDataCCSA
{
    OrdinalType mNumIter;  /*!< number of outer iterations */
    OrdinalType mObjFuncCount;  /*!< number of objective function evaluations */

    ScalarType mKKTMeasure;  /*!< measures inexactness in first-order optimality conditions */
    ScalarType mObjFuncValue;  /*!< objective function value */
    ScalarType mNormObjFuncGrad;  /*!< norm of the objective function gradient */
    ScalarType mStationarityMeasure;  /*!< norm of the descent direction */
    ScalarType mControlStagnationMeasure;  /*!< norm of the difference between two subsequent control fields */
    ScalarType mObjectiveStagnationMeasure;  /*!< measures stagnation in two subsequent objective function evaluations */
    std::vector<ScalarType> mConstraints;  /*!< residual value for each constraint */
};
// struct OutputDataCCSA

/******************************************************************************//**
 *
 * @brief Diagnostic data for Optimality Criteria (OC) algorithm
 *
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct OutputDataOC
{
    OrdinalType mNumIter;  /*!< number of outer iterations */
    OrdinalType mObjFuncCount;  /*!< number of objective function evaluations */

    ScalarType mObjFuncValue;  /*!< objective function value */
    ScalarType mNormObjFuncGrad;  /*!< norm of the objective function gradient */
    ScalarType mControlStagnationMeasure;  /*!< norm of the difference between two subsequent control fields */
    ScalarType mObjectiveStagnationMeasure;  /*!< measures stagnation in two subsequent objective function evaluations */
    std::vector<ScalarType> mConstraints;  /*!< residual value for each constraint */
};
// struct OutputDataOC

/******************************************************************************//**
 *
 * @brief Diagnostic data for Kelley-Sachs algorithm
 *
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct OutputDataKelleySachs
{
    OrdinalType mNumIter;  /*!< number of outer iterations */
    OrdinalType mNumIterPCG;  /*!< number preconditioned conjugate gradient iterations */
    OrdinalType mObjFuncCount;  /*!< number of objective function evaluations */
    OrdinalType mNumTrustRegionIter;  /*!< number of trust region (inner-loop) iterations */

    ScalarType mActualRed;  /*!< actual reduction */
    ScalarType mAredOverPred;  /*!< actual over predicted reduction ratio */
    ScalarType mObjFuncValue;  /*!< objective function value */
    ScalarType mNormObjFuncGrad;  /*!< norm of the objective function gradient */
    ScalarType mTrustRegionRadius;  /*!< trust region radius */
    ScalarType mStationarityMeasure;  /*!< norm of the descent direction */
    ScalarType mControlStagnationMeasure;  /*!< norm of the difference between two subsequent control fields */
    ScalarType mObjectiveStagnationMeasure;  /*!< measures stagnation in two subsequent objective function evaluations */
    std::vector<ScalarType> mConstraints;  /*!< residual value for each constraint */
};
// struct OutputDataKelleySachs

/******************************************************************************//**
 *
 * @brief Check for errors in CCSA algorithm diagnostics
 * @param [in] aData diagnostic data for ccsa algorithm
 * @param [in] aOutputFile output file
 *
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
void check_for_ccsa_diagnostics_errors(const Plato::OutputDataCCSA<ScalarType, OrdinalType>& aData,
                                       const std::ofstream& aOutputFile)
{
    try
    {
        Plato::error::is_file_open(aOutputFile);
        Plato::error::is_vector_empty(aData.mConstraints);
    }
    catch(const std::invalid_argument& tError)
    {
        throw tError;
    }
}

/******************************************************************************//**
 *
 * @brief Print header for CCSA diagnostics file
 * @param [in] aData diagnostic data for ccsa algorithm
 * @param [in,out] aOutputFile output file
 * @param [in] aPrint flag use to enable/disable output (default = disable)
 *
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
void print_ccsa_diagnostics_header(const Plato::OutputDataCCSA<ScalarType, OrdinalType>& aData,
                                   std::ofstream& aOutputFile,
                                   bool aPrint = false)
{
    try
    {
        Plato::check_for_ccsa_diagnostics_errors(aData, aOutputFile);
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
    assert(aData.mConstraints.size() > static_cast<OrdinalType>(0));

    aOutputFile << std::scientific << std::setprecision(6) << std::right << "Iter" << std::setw(10) << "F-count"
            << std::setw(14) << "F(X)" << std::setw(16) << "Norm(F')" << std::setw(10);

    const OrdinalType tNumConstraints = aData.mConstraints.size();
    for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
    {
        if(tIndex != static_cast<OrdinalType>(0))
        {
            aOutputFile << "H" << tIndex + static_cast<OrdinalType>(1) << "(X)" << std::setw(13);
        }
        else
        {
            const OrdinalType tWidth = tNumConstraints > static_cast<OrdinalType>(1) ? 11 : 13;
            aOutputFile << "H" << tIndex + static_cast<OrdinalType>(1) << "(X)" << std::setw(tWidth);
        }
    }

    aOutputFile << "KKT" << std::setw(18) << "Norm(S)" << std::setw(15) << "abs(dX)" << std::setw(15) << "abs(dF)"
            << "\n" << std::flush;
}

/******************************************************************************//**
 *
 * @brief Print diagnostics for CCSA algorithm
 * @param [in] aData diagnostic data for ccsa algorithm
 * @param [in,out] aOutputFile output file
 * @param [in] aPrint flag use to enable/disable output (default = disable)
 *
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
void print_ccsa_diagnostics(const Plato::OutputDataCCSA<ScalarType, OrdinalType>& aData,
                            std::ofstream& aOutputFile,
                            bool aPrint = false)
{
    try
    {
        Plato::check_for_ccsa_diagnostics_errors(aData, aOutputFile);
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
    assert(aData.mConstraints.size() > static_cast<OrdinalType>(0));

    aOutputFile << std::scientific << std::setprecision(6) << std::right << aData.mNumIter << std::setw(10)
            << aData.mObjFuncCount << std::setw(20) << aData.mObjFuncValue << std::setw(15) << aData.mNormObjFuncGrad
            << std::setw(15);

    const OrdinalType tNumConstraints = aData.mConstraints.size();
    for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
    {
        aOutputFile << aData.mConstraints[tIndex] << std::setw(15);
    }

    aOutputFile << aData.mKKTMeasure << std::setw(15) << aData.mStationarityMeasure << std::setw(15)
            << aData.mControlStagnationMeasure << std::setw(15) << aData.mObjectiveStagnationMeasure << "\n"
            << std::flush;
}

/******************************************************************************//**
 *
 * @brief Print header for Optimality Criteria (OC) diagnostics file
 * @param [in] aData diagnostic data for optimality criteria algorithm
 * @param [in,out] aOutputFile output file
 * @param [in] aPrint flag use to enable/disable output (default = disable)
 *
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
void print_oc_diagnostics_header(const Plato::OutputDataOC<ScalarType, OrdinalType>& aData,
                                 std::ofstream& aOutputFile,
                                 bool aPrint = false)
{
    try
    {
        Plato::error::is_file_open(aOutputFile);
        Plato::error::is_vector_empty(aData.mConstraints);
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
    assert(aData.mConstraints.size() > static_cast<OrdinalType>(0));

    aOutputFile << std::scientific << std::setprecision(6) << std::right << "Iter" << std::setw(10) << "F-count"
            << std::setw(14) << "F(X)" << std::setw(16) << "Norm(F')" << std::setw(10);

    const OrdinalType tNumConstraints = aData.mConstraints.size();
    for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
    {
        if(tIndex != static_cast<OrdinalType>(0))
        {
            aOutputFile << "H" << tIndex + static_cast<OrdinalType>(1) << "(X)" << std::setw(16);
        }
        else
        {
            const OrdinalType tWidth = tNumConstraints > static_cast<OrdinalType>(1) ? 11 : 16;
            aOutputFile << "H" << tIndex + static_cast<OrdinalType>(1) << "(X)" << std::setw(tWidth);
        }
    }

    aOutputFile << "abs(dX)" << std::setw(15) << "abs(dF)" << "\n" << std::flush;
}

/******************************************************************************//**
 *
 * @brief Print diagnostics for Optimality Criteria (OC) algorithm
 * @param [in] aData diagnostic data for optimality criteria algorithm
 * @param [in,out] aOutputFile output file
 * @param [in] aPrint flag use to enable/disable output (default = disable)
 *
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
void print_oc_diagnostics(const Plato::OutputDataOC<ScalarType, OrdinalType>& aData,
                          std::ofstream& aOutputFile,
                          bool aPrint = false)
{
    try
    {
        Plato::error::is_file_open(aOutputFile);
        Plato::error::is_vector_empty(aData.mConstraints);
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
    assert(aData.mConstraints.size() > static_cast<OrdinalType>(0));

    aOutputFile << std::scientific << std::setprecision(6) << std::right << aData.mNumIter << std::setw(10)
            << aData.mObjFuncCount << std::setw(20) << aData.mObjFuncValue << std::setw(15) << aData.mNormObjFuncGrad
            << std::setw(15);

    const OrdinalType tNumConstraints = aData.mConstraints.size();
    for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
    {
        aOutputFile << aData.mConstraints[tIndex] << std::setw(15);
    }

    aOutputFile << aData.mControlStagnationMeasure << std::setw(15) << aData.mObjectiveStagnationMeasure << "\n"
            << std::flush;
}

/******************************************************************************//**
 *
 * @brief Print header for Kelley-Sachs-Bound-Constrained (KSBC) diagnostics file
 * @param [in,out] aOutputFile output file
 * @param [in] aPrint flag use to enable/disable output (default = disable)
 *
**********************************************************************************/
void print_ksbc_diagnostics_header(std::ofstream& aOutputFile, bool aPrint = false)
{
    try
    {
        Plato::error::is_file_open(aOutputFile);
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

    // PRIMARY DIAGNOSTICS  (OUTER-LOOP)
    aOutputFile << std::scientific << std::setprecision(6) << std::right << "Iter" << std::setw(10) << "F-count"
            << std::setw(14) << "F(X)" << std::setw(16) << "Norm(F')" << std::setw(15) << "Norm(S)" << std::setw(12);

    // TRUST REGION PROBLEM DATA (INNER-LOOP)
    aOutputFile << "TR-Iter" << std::setw(14) << "TR-Radius" << std::setw(12) << "ARed" << std::setw(17) << "TR-Ratio"
            << std::setw(13) << "PCG-Iter" << std::setw(13) << "abs(dX)" << std::setw(15) << "abs(dF)" << "\n"
            << std::flush;
}

/******************************************************************************//**
 *
 * @brief Print diagnostics for Kelley-Sachs-Bound-Constrained (KSBC) algorithm
 * @param [in] aData diagnostic data for Kelley-Sachs-Bound-Constrained algorithm
 * @param [in,out] aOutputFile output file
 * @param [in] aPrint flag use to enable/disable output (default = disable)
 *
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
void print_ksbc_diagnostics(const Plato::OutputDataKelleySachs<ScalarType, OrdinalType>& aData,
                            std::ofstream& aOutputFile,
                            bool aPrint = false)
{
    try
    {
        Plato::error::is_file_open(aOutputFile);
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

    // PRIMARY DIAGNOSTICS  (OUTER-LOOP)
    aOutputFile << std::scientific << std::setprecision(6) << std::right << aData.mNumIter << std::setw(10)
            << aData.mObjFuncCount << std::setw(20) << aData.mObjFuncValue << std::setw(15) << aData.mNormObjFuncGrad
            << std::setw(15) << aData.mStationarityMeasure << std::setw(7);

    // TRUST REGION PROBLEM DATA (INNER-LOOP)
    aOutputFile << aData.mNumTrustRegionIter << std::setw(18) << aData.mTrustRegionRadius << std::setw(15)
            << aData.mActualRed << std::setw(15) << aData.mAredOverPred << std::setw(7) << aData.mNumIterPCG
            << std::setw(19) << aData.mControlStagnationMeasure << std::setw(16) << aData.mObjectiveStagnationMeasure
            << "\n" << std::flush;
}

} // namespace Plato

namespace PlatoTest
{

TEST(PlatoTest, IsFileOpenExeption)
{
    std::ofstream tFile;
    ASSERT_THROW(Plato::error::is_file_open(tFile), std::invalid_argument);

    tFile.open("MyFile.txt");
    ASSERT_NO_THROW(Plato::error::is_file_open(tFile));
    tFile.close();
    std::system("rm -f MyFile.txt");
}

TEST(PlatoTest, IsVectorEmpty)
{
    std::vector<double> tVector;
    ASSERT_THROW(Plato::error::is_vector_empty(tVector), std::invalid_argument);

    const size_t tLength = 1;
    tVector.resize(tLength);
    ASSERT_NO_THROW(Plato::error::is_vector_empty(tVector));
}

TEST(PlatoTest, CheckForCCSAdiagnosticsErrors)
{
    std::ofstream tFile1;
    Plato::OutputDataCCSA<double> tData;
    ASSERT_THROW(Plato::check_for_ccsa_diagnostics_errors(tData, tFile1), std::invalid_argument);

    tFile1.open("MyFile1.txt");
    ASSERT_THROW(Plato::check_for_ccsa_diagnostics_errors(tData, tFile1), std::invalid_argument);

    const size_t tNumConstraints = 1;
    tData.mConstraints.resize(tNumConstraints);
    ASSERT_NO_THROW(Plato::check_for_ccsa_diagnostics_errors(tData, tFile1));
    tFile1.close();
    std::system("rm -f MyFile1.txt");

    std::ofstream tFile2;
    ASSERT_THROW(Plato::check_for_ccsa_diagnostics_errors(tData, tFile2), std::invalid_argument);

    tFile2.open("MyFile2.txt");
    ASSERT_NO_THROW(Plato::check_for_ccsa_diagnostics_errors(tData, tFile2));
    tFile2.close();
    std::system("rm -f MyFile2.txt");
}

TEST(PlatoTest, PrintCCSAdiagnosticsInvalidArguments)
{
    std::ofstream tFile1;
    Plato::OutputDataCCSA<double> tData;
    ASSERT_THROW(Plato::print_ccsa_diagnostics_header(tData, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::print_ccsa_diagnostics_header(tData, tFile1, true /* print message */), std::invalid_argument);
    ASSERT_THROW(Plato::print_ccsa_diagnostics(tData, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::print_ccsa_diagnostics(tData, tFile1, true /* print message */), std::invalid_argument);

    tFile1.open("MyFile1.txt");
    ASSERT_THROW(Plato::print_ccsa_diagnostics_header(tData, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::print_ccsa_diagnostics_header(tData, tFile1, true /* print message */), std::invalid_argument);
    ASSERT_THROW(Plato::print_ccsa_diagnostics(tData, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::print_ccsa_diagnostics(tData, tFile1, true /* print message */), std::invalid_argument);
    tFile1.close();
    std::system("rm -f MyFile1.txt");
}

TEST(PlatoTest, PrintCCSAdiagnosticsHeader)
{
    std::ofstream tWriteFile;
    tWriteFile.open("MyFile1.txt");
    Plato::OutputDataCCSA<double> tData;
    const size_t tNumConstraints = 1;
    tData.mConstraints.resize(tNumConstraints);
    ASSERT_NO_THROW(Plato::print_ccsa_diagnostics_header(tData, tWriteFile));
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

    std::stringstream tGold("IterF-countF(X)Norm(F')H1(X)KKTNorm(S)abs(dX)abs(dF)");
    ASSERT_STREQ(tReadData.str().c_str(), tGold.str().c_str());
}

TEST(PlatoTest, PrintDiagnosticsOneConstraintsCCSA)
{
    std::ofstream tWriteFile;
    tWriteFile.open("MyFile1.txt");
    Plato::OutputDataCCSA<double> tData;
    tData.mNumIter = 0;
    tData.mObjFuncCount = 1;
    tData.mObjFuncValue = 1.0;
    tData.mKKTMeasure = 1.2345678e-6;
    tData.mNormObjFuncGrad = 4.5656e-3;
    tData.mStationarityMeasure = 2.34e-2;
    tData.mControlStagnationMeasure = 1.2345678e6;
    tData.mObjectiveStagnationMeasure = std::numeric_limits<double>::max();
    const size_t tNumConstraints = 1;
    tData.mConstraints.resize(tNumConstraints);
    tData.mConstraints[0] = 1.23e-5;

    ASSERT_NO_THROW(Plato::print_ccsa_diagnostics_header(tData, tWriteFile));
    ASSERT_NO_THROW(Plato::print_ccsa_diagnostics(tData, tWriteFile));
    tData.mNumIter = 1;
    tData.mObjFuncCount = 3;
    tData.mObjFuncValue = 0.298736;
    tData.mKKTMeasure = 1.2345678e-2;
    tData.mNormObjFuncGrad = 3.45656e-1;
    tData.mStationarityMeasure = 2.34e-1;
    tData.mControlStagnationMeasure = 0.18743;
    tData.mObjectiveStagnationMeasure = 0.7109;
    tData.mConstraints[0] = 8.23e-2;
    ASSERT_NO_THROW(Plato::print_ccsa_diagnostics(tData, tWriteFile));
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
    tGold << "IterF-countF(X)Norm(F')H1(X)KKTNorm(S)abs(dX)abs(dF)";
    tGold << "011.000000e+004.565600e-031.230000e-051.234568e-062.340000e-021.234568e+061.797693e+308";
    tGold << "132.987360e-013.456560e-018.230000e-021.234568e-022.340000e-011.874300e-017.109000e-01";
    ASSERT_STREQ(tReadData.str().c_str(), tGold.str().c_str());
}

TEST(PlatoTest, PrintDiagnosticsTwoConstraintsCCSA)
{
    std::ofstream tWriteFile;
    tWriteFile.open("MyFile1.txt");
    Plato::OutputDataCCSA<double> tData;
    tData.mNumIter = 0;
    tData.mObjFuncCount = 1;
    tData.mObjFuncValue = 1.0;
    tData.mKKTMeasure = 1.2345678e-6;
    tData.mNormObjFuncGrad = 4.5656e-3;
    tData.mStationarityMeasure = 2.34e-2;
    tData.mControlStagnationMeasure = 1.2345678e6;
    tData.mObjectiveStagnationMeasure = std::numeric_limits<double>::max();
    const size_t tNumConstraints = 2;
    tData.mConstraints.resize(tNumConstraints);
    tData.mConstraints[0] = 1.23e-5;
    tData.mConstraints[1] = 3.33e-3;

    ASSERT_NO_THROW(Plato::print_ccsa_diagnostics_header(tData, tWriteFile));
    ASSERT_NO_THROW(Plato::print_ccsa_diagnostics(tData, tWriteFile));
    tData.mNumIter = 1;
    tData.mObjFuncCount = 3;
    tData.mObjFuncValue = 0.298736;
    tData.mKKTMeasure = 1.2345678e-2;
    tData.mNormObjFuncGrad = 3.45656e-1;
    tData.mStationarityMeasure = 2.34e-1;
    tData.mControlStagnationMeasure = 0.18743;
    tData.mObjectiveStagnationMeasure = 0.7109;
    tData.mConstraints[0] = 8.23e-2;
    tData.mConstraints[1] = 8.33e-5;
    ASSERT_NO_THROW(Plato::print_ccsa_diagnostics(tData, tWriteFile));
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
    tGold << "IterF-countF(X)Norm(F')H1(X)H2(X)KKTNorm(S)abs(dX)abs(dF)";
    tGold << "011.000000e+004.565600e-031.230000e-053.330000e-031.234568e-062.340000e-021.234568e+061.797693e+308";
    tGold << "132.987360e-013.456560e-018.230000e-028.330000e-051.234568e-022.340000e-011.874300e-017.109000e-01";
    ASSERT_STREQ(tReadData.str().c_str(), tGold.str().c_str());
}

TEST(PlatoTest, PrintOptimalityCriteriaDiagnosticsInvalidArguments)
{
    std::ofstream tFile1;
    Plato::OutputDataOC<double> tData;
    ASSERT_THROW(Plato::print_oc_diagnostics_header(tData, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::print_oc_diagnostics_header(tData, tFile1, true /* print message */), std::invalid_argument);
    ASSERT_THROW(Plato::print_oc_diagnostics(tData, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::print_oc_diagnostics(tData, tFile1, true /* print message */), std::invalid_argument);

    tFile1.open("MyFile1.txt");
    ASSERT_THROW(Plato::print_oc_diagnostics_header(tData, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::print_oc_diagnostics_header(tData, tFile1, true /* print message */), std::invalid_argument);
    ASSERT_THROW(Plato::print_oc_diagnostics(tData, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::print_oc_diagnostics(tData, tFile1, true /* print message */), std::invalid_argument);
    tFile1.close();
    std::system("rm -f MyFile1.txt");

    std::ofstream tFile2;
    ASSERT_THROW(Plato::print_oc_diagnostics_header(tData, tFile2), std::invalid_argument);
    ASSERT_THROW(Plato::print_oc_diagnostics_header(tData, tFile2, true /* print message */), std::invalid_argument);
    ASSERT_THROW(Plato::print_oc_diagnostics(tData, tFile2), std::invalid_argument);
    ASSERT_THROW(Plato::print_oc_diagnostics(tData, tFile2, true /* print message */), std::invalid_argument);
}

TEST(PlatoTest, PrintDiagnosticsOneConstraintsOC)
{
    std::ofstream tWriteFile;
    tWriteFile.open("MyFile1.txt");
    Plato::OutputDataOC<double> tData;
    tData.mNumIter = 0;
    tData.mObjFuncCount = 1;
    tData.mObjFuncValue = 1.0;
    tData.mNormObjFuncGrad = 4.5656e-3;
    tData.mControlStagnationMeasure = 1.2345678e6;
    tData.mObjectiveStagnationMeasure = std::numeric_limits<double>::max();
    const size_t tNumConstraints = 1;
    tData.mConstraints.resize(tNumConstraints);
    tData.mConstraints[0] = 1.23e-5;

    ASSERT_NO_THROW(Plato::print_oc_diagnostics_header(tData, tWriteFile));
    ASSERT_NO_THROW(Plato::print_oc_diagnostics(tData, tWriteFile));
    tData.mNumIter = 1;
    tData.mObjFuncCount = 3;
    tData.mObjFuncValue = 0.298736;
    tData.mNormObjFuncGrad = 3.45656e-1;
    tData.mControlStagnationMeasure = 0.18743;
    tData.mObjectiveStagnationMeasure = 0.7109;
    tData.mConstraints[0] = 8.23e-2;
    ASSERT_NO_THROW(Plato::print_oc_diagnostics(tData, tWriteFile));
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

    std::stringstream tGold("");
    tGold << "IterF-countF(X)Norm(F')H1(X)abs(dX)abs(dF)";
    tGold << "011.000000e+004.565600e-031.230000e-051.234568e+061.797693e+308";
    tGold << "132.987360e-013.456560e-018.230000e-021.874300e-017.109000e-01";
    ASSERT_STREQ(tReadData.str().c_str(), tGold.str().c_str());
}

TEST(PlatoTest, PrintDiagnosticsTwoConstraintsOC)
{
    std::ofstream tWriteFile;
    tWriteFile.open("MyFile1.txt");
    Plato::OutputDataOC<double> tData;
    tData.mNumIter = 0;
    tData.mObjFuncCount = 1;
    tData.mObjFuncValue = 1.0;
    tData.mNormObjFuncGrad = 4.5656e-3;
    tData.mControlStagnationMeasure = 1.2345678e6;
    tData.mObjectiveStagnationMeasure = std::numeric_limits<double>::max();
    const size_t tNumConstraints = 2;
    tData.mConstraints.resize(tNumConstraints);
    tData.mConstraints[0] = 1.23e-5;
    tData.mConstraints[1] = 3.33e-3;

    ASSERT_NO_THROW(Plato::print_oc_diagnostics_header(tData, tWriteFile));
    ASSERT_NO_THROW(Plato::print_oc_diagnostics(tData, tWriteFile));
    tData.mNumIter = 1;
    tData.mObjFuncCount = 3;
    tData.mObjFuncValue = 0.298736;
    tData.mNormObjFuncGrad = 3.45656e-1;
    tData.mControlStagnationMeasure = 0.18743;
    tData.mObjectiveStagnationMeasure = 0.7109;
    tData.mConstraints[0] = 8.23e-2;
    tData.mConstraints[1] = 8.33e-5;
    ASSERT_NO_THROW(Plato::print_oc_diagnostics(tData, tWriteFile));
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
    tGold << "IterF-countF(X)Norm(F')H1(X)H2(X)abs(dX)abs(dF)";
    tGold << "011.000000e+004.565600e-031.230000e-053.330000e-031.234568e+061.797693e+308";
    tGold << "132.987360e-013.456560e-018.230000e-028.330000e-051.874300e-017.109000e-01";
    ASSERT_STREQ(tReadData.str().c_str(), tGold.str().c_str());
}

TEST(PlatoTest, PrintKsbcDiagnosticsInvalidArguments)
{
    std::ofstream tFile1;
    Plato::OutputDataKelleySachs<double> tData;
    ASSERT_THROW(Plato::print_ksbc_diagnostics_header(tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::print_ksbc_diagnostics_header(tFile1, true /* print message */), std::invalid_argument);
    ASSERT_THROW(Plato::print_ksbc_diagnostics(tData, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::print_ksbc_diagnostics(tData, tFile1, true /* print message */), std::invalid_argument);
}

TEST(PlatoTest, PrintDiagnosticsKSBC)
{
    std::ofstream tWriteFile;
    tWriteFile.open("MyFile1.txt");
    Plato::OutputDataKelleySachs<double> tData;
    tData.mNumIter = 0;
    tData.mNumIterPCG = 0;
    tData.mObjFuncCount = 1;
    tData.mNumTrustRegionIter = 0;

    tData.mActualRed = 0;
    tData.mAredOverPred = 0;
    tData.mObjFuncValue = 1.0;
    tData.mNormObjFuncGrad = 4.5656e-3;
    tData.mTrustRegionRadius = 1e2;
    tData.mStationarityMeasure = 2.34e-2;
    tData.mControlStagnationMeasure = 1.2345678e6;
    tData.mObjectiveStagnationMeasure = std::numeric_limits<double>::max();

    ASSERT_NO_THROW(Plato::print_ksbc_diagnostics_header(tWriteFile));
    ASSERT_NO_THROW(Plato::print_ksbc_diagnostics(tData, tWriteFile));
    tData.mNumIter = 1;
    tData.mNumIterPCG = 34;
    tData.mObjFuncCount = 3;
    tData.mNumTrustRegionIter = 22;

    tData.mActualRed = 0.1435;
    tData.mAredOverPred = 0.78;
    tData.mObjFuncValue = 0.298736;
    tData.mNormObjFuncGrad = 3.45656e-1;
    tData.mTrustRegionRadius = 4;
    tData.mStationarityMeasure = 2.34e-1;
    tData.mControlStagnationMeasure = 0.18743;
    tData.mObjectiveStagnationMeasure = 0.7109;
    ASSERT_NO_THROW(Plato::print_ksbc_diagnostics(tData, tWriteFile));
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
    tGold << "IterF-countF(X)Norm(F')Norm(S)TR-IterTR-RadiusARedTR-RatioPCG-Iterabs(dX)abs(dF)";
    tGold << "011.000000e+004.565600e-032.340000e-0201.000000e+020.000000e+000.000000e+0001.234568e+061.797693e+308";
    tGold << "132.987360e-013.456560e-012.340000e-01224.000000e+001.435000e-017.800000e-01341.874300e-017.109000e-01";
    ASSERT_STREQ(tReadData.str().c_str(), tGold.str().c_str());
}

} // namespace PlatoTest
