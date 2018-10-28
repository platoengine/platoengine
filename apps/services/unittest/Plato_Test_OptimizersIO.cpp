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
 * Plato_Test_OptimizersIO.cpp
 *
 *  Created on: Sep 15, 2018
 */

#include "gtest/gtest.h"

#include "Plato_Types.hpp"
#include "Plato_OptimizersIO.hpp"

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
    Plato::OutputDataKSBC<double> tData;
    ASSERT_THROW(Plato::print_ksbc_diagnostics_header(tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::print_ksbc_diagnostics_header(tFile1, true /* print message */), std::invalid_argument);
    ASSERT_THROW(Plato::print_ksbc_diagnostics(tData, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::print_ksbc_diagnostics(tData, tFile1, true /* print message */), std::invalid_argument);
}

TEST(PlatoTest, PrintDiagnosticsKSBC)
{
    std::ofstream tWriteFile;
    tWriteFile.open("MyFile1.txt");
    Plato::OutputDataKSBC<double> tData;
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

TEST(PlatoTest, PrintDiagnosticsKSAL_1)
{
    std::ofstream tWriteFile;
    tWriteFile.open("MyFile1.txt");
    Plato::OutputDataKSAL<double> tData;
    tData.mNumIter = 0;
    tData.mNumIterPCG = 0;
    tData.mObjFuncCount = 1;
    tData.mNumTrustRegionIter = 0;
    tData.mNumLineSearchIter = 0;

    tData.mPenalty = 0.1;
    tData.mActualRed = 0;
    tData.mAredOverPred = 0;
    tData.mObjFuncValue = 1.0;
    tData.mNormObjFuncGrad = 4.5656e-3;
    tData.mTrustRegionRadius = 1e2;
    tData.mStationarityMeasure = 2.34e-2;
    tData.mControlStagnationMeasure = 1.2345678e6;
    tData.mObjectiveStagnationMeasure = std::numeric_limits<double>::max();

    const size_t tNumConstraints = 1;
    tData.mConstraintValues.resize(tNumConstraints);
    tData.mConstraintValues[0] = 1.23e-5;

    ASSERT_NO_THROW(Plato::print_ksal_diagnostics_header(tData, tWriteFile));
    ASSERT_NO_THROW(Plato::print_ksal_diagnostics(tData, tWriteFile));
    tData.mNumIter = 1;
    tData.mNumIterPCG = 34;
    tData.mObjFuncCount = 3;
    tData.mNumTrustRegionIter = 22;
    tData.mNumLineSearchIter = 4;

    tData.mPenalty = 0.075;
    tData.mActualRed = 0.1435;
    tData.mAredOverPred = 0.78;
    tData.mObjFuncValue = 0.298736;
    tData.mNormObjFuncGrad = 3.45656e-1;
    tData.mTrustRegionRadius = 4;
    tData.mStationarityMeasure = 2.34e-1;
    tData.mControlStagnationMeasure = 0.18743;
    tData.mObjectiveStagnationMeasure = 0.7109;
    tData.mConstraintValues[0] = 2.45e-7;
    ASSERT_NO_THROW(Plato::print_ksal_diagnostics(tData, tWriteFile));
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
    tGold << "IterF-countF(X)Norm(F')Norm(S)H1(X)TR-IterLS-IterTR-RadiusARedTR-RatioPCG-Iterabs(dX)abs(dF)Penalty";
    tGold << "011.000000e+004.565600e-032.340000e-021.230000e-05001.000000e+020.000000e+000.000000e+0001.234568e+061.797693e+3081.000000e-01";
    tGold << "132.987360e-013.456560e-012.340000e-012.450000e-072244.000000e+001.435000e-017.800000e-01341.874300e-017.109000e-017.500000e-02";
    ASSERT_STREQ(tReadData.str().c_str(), tGold.str().c_str());
}

TEST(PlatoTest, PrintDiagnosticsKSAL_2)
{
    std::ofstream tWriteFile;
    tWriteFile.open("MyFile1.txt");
    Plato::OutputDataKSAL<double> tData;
    tData.mNumIter = 0;
    tData.mNumIterPCG = 0;
    tData.mObjFuncCount = 1;
    tData.mNumTrustRegionIter = 0;
    tData.mNumLineSearchIter = 0;

    tData.mPenalty = 0.1;
    tData.mActualRed = 0;
    tData.mAredOverPred = 0;
    tData.mObjFuncValue = 1.0;
    tData.mNormObjFuncGrad = 4.5656e-3;
    tData.mTrustRegionRadius = 1e2;
    tData.mStationarityMeasure = 2.34e-2;
    tData.mControlStagnationMeasure = 1.2345678e6;
    tData.mObjectiveStagnationMeasure = std::numeric_limits<double>::max();

    const size_t tNumConstraints = 2;
    tData.mConstraintValues.resize(tNumConstraints);
    tData.mConstraintValues[0] = 1.23e-5;
    tData.mConstraintValues[1] = 3.33e-3;

    ASSERT_NO_THROW(Plato::print_ksal_diagnostics_header(tData, tWriteFile));
    ASSERT_NO_THROW(Plato::print_ksal_diagnostics(tData, tWriteFile));
    tData.mNumIter = 1;
    tData.mNumIterPCG = 34;
    tData.mObjFuncCount = 3;
    tData.mNumTrustRegionIter = 22;
    tData.mNumLineSearchIter = 1;

    tData.mPenalty = 0.075;
    tData.mActualRed = 0.1435;
    tData.mAredOverPred = 0.78;
    tData.mObjFuncValue = 0.298736;
    tData.mNormObjFuncGrad = 3.45656e-1;
    tData.mTrustRegionRadius = 4;
    tData.mStationarityMeasure = 2.34e-1;
    tData.mControlStagnationMeasure = 0.18743;
    tData.mObjectiveStagnationMeasure = 0.7109;
    tData.mConstraintValues[0] = 2.45e-7;
    tData.mConstraintValues[1] = 4.23e-8;
    ASSERT_NO_THROW(Plato::print_ksal_diagnostics(tData, tWriteFile));
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
    tGold << "IterF-countF(X)Norm(F')Norm(S)H1(X)H2(X)TR-IterLS-IterTR-RadiusARedTR-RatioPCG-Iterabs(dX)abs(dF)Penalty";
    tGold << "011.000000e+004.565600e-032.340000e-021.230000e-053.330000e-03001.000000e+020.000000e+000.000000e+0001.234568e+061.797693e+3081.000000e-01";
    tGold << "132.987360e-013.456560e-012.340000e-012.450000e-074.230000e-082214.000000e+001.435000e-017.800000e-01341.874300e-017.109000e-017.500000e-02";
    ASSERT_STREQ(tReadData.str().c_str(), tGold.str().c_str());
}

TEST(PlatoTest, PrintKsalDiagnosticsInvalidArguments)
{
    std::ofstream tFile1;
    Plato::OutputDataKSAL<double> tData;
    ASSERT_THROW(Plato::print_ksal_diagnostics_header(tData, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::print_ksal_diagnostics_header(tData, tFile1, true /* print message */), std::invalid_argument);
    ASSERT_THROW(Plato::print_ksal_diagnostics(tData, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::print_ksal_diagnostics(tData, tFile1, true /* print message */), std::invalid_argument);
}

TEST(PlatoTest, TestStoppingCriterionOutputAlgorithm)
{
    std::string tDescription;
    Plato::algorithm::stop_t tFlag = Plato::algorithm::NOT_CONVERGED;
    Plato::get_stop_criterion(tFlag, tDescription);
    std::string tGold("\n\n****** Optimization algorithm did not converge. ******\n\n");
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::NaN_ACTUAL_REDUCTION;
    Plato::get_stop_criterion(tFlag, tDescription);
    tGold = "\n\n****** Optimization stopping due to NaN actual reduction. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::NaN_STATIONARITY_MEASURE;
    Plato::get_stop_criterion(tFlag, tDescription);
    tGold = "\n\n****** Optimization stopping due to NaN stationary measure. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::NaN_OBJECTIVE_STAGNATION;
    Plato::get_stop_criterion(tFlag, tDescription);
    tGold = "\n\n****** Optimization stopping due to NaN objective stagnation. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::NaN_CONTROL_STAGNATION;
    Plato::get_stop_criterion(tFlag, tDescription);
    tGold = "\n\n****** Optimization stopping due to NaN control stagnation. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::NaN_OBJECTIVE_GRADIENT;
    Plato::get_stop_criterion(tFlag, tDescription);
    tGold = "\n\n****** Optimization stopping due to NaN norm of objective gradient vector. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::SMALL_PENALTY_PARAMETER;
    Plato::get_stop_criterion(tFlag, tDescription);
    tGold = "\n\n****** Optimization stopping due to small penalty parameter in augmented Lagrangian being met. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::OPTIMALITY_AND_FEASIBILITY;
    Plato::get_stop_criterion(tFlag, tDescription);
    tGold.clear();
    tGold = "\n\n****** Optimization stopping due to optimality and feasibility tolerance being met. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::NORM_STEP;
    Plato::get_stop_criterion(tFlag, tDescription);
    tGold.clear();
    tGold = "\n\n****** Optimization stopping due to norm of trial step tolerance being met. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::NaN_NORM_GRADIENT;
    Plato::get_stop_criterion(tFlag, tDescription);
    tGold.clear();
    tGold = "\n\n****** Optimization stopping due to NaN norm of gradient vector. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::NaN_NORM_TRIAL_STEP;
    Plato::get_stop_criterion(tFlag, tDescription);
    tGold.clear();
    tGold = "\n\n****** Optimization stopping due to NaN norm of trial step vector. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::MAX_NUMBER_ITERATIONS;
    Plato::get_stop_criterion(tFlag, tDescription);
    tGold.clear();
    tGold = "\n\n****** Optimization stopping due to exceeding maximum number of iterations. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::CONTROL_STAGNATION;
    Plato::get_stop_criterion(tFlag, tDescription);
    tGold.clear();
    tGold = "\n\n****** Optimization stopping due to control (i.e. design variable) stagnation. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::SMALL_TRUST_REGION_RADIUS;
    Plato::get_stop_criterion(tFlag, tDescription);
    tGold.clear();
    tGold = "\n\n****** Optimization stopping due to small trust region radius. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::OBJECTIVE_STAGNATION;
    Plato::get_stop_criterion(tFlag, tDescription);
    tGold.clear();
    tGold = "\n\n****** Optimization stopping due to objective stagnation. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::NORM_GRADIENT;
    Plato::get_stop_criterion(tFlag, tDescription);
    tGold.clear();
    tGold = "\n\n****** Optimization stopping due to norm of gradient tolerance being met. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::STATIONARITY_MEASURE;
    Plato::get_stop_criterion(tFlag, tDescription);
    tGold.clear();
    tGold = "\n\n****** Optimization stopping due to stationary measure being met. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::ACTUAL_REDUCTION_TOLERANCE;
    Plato::get_stop_criterion(tFlag, tDescription);
    tGold.clear();
    tGold = "\n\n****** Optimization stopping due to actual reduction tolerance being met. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());
}

} // namespace PlatoTest
