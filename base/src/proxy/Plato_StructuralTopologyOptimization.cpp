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
 * Plato_StructuralTopologyOptimization.cpp
 *
 *  Created on: Nov 16, 2017
 */

#include <cmath>
#include <cstdio>
#include <algorithm>

#include "Plato_StructuralTopologyOptimization.hpp"

namespace Plato
{

StructuralTopologyOptimization::StructuralTopologyOptimization(const double & aPoissonRatio,
                                                               const double & aElasticModulus,
                                                               const int & aNumElementsXdirection,
                                                               const int & aNumElementsYdirection) :
        mNumElemDOFs(8),
        mGlobalNumDofs(0),
        mNumDesignVariables(0),
        mNumElemXDirection(aNumElementsXdirection),
        mNumElemYDirection(aNumElementsYdirection),
        mPenalty(3),
        mFilterRadius(1.5),
        mPoissonRatio(aPoissonRatio),
        mElasticModulus(aElasticModulus),
        mVolumeFraction(0.4),
        mStiffnessMatrixOneNorm(0),
        mIsForceSet(false),
        mIsStateCached(true),
        mIsDirichletConditionsSet(false),
        mSolver(),
        mForce(),
        mFixedDOFs(),
        mWorkVector(),
        mElemDisplacements(),
        mTrialDisplacements(),
        mCurrentDisplacements(),
        mElemStiffTimesElemDisplacements(),
        mStiffnessMatrix(),
        mElementStiffnessMatrix()
{
    this->initialize();
}

StructuralTopologyOptimization::~StructuralTopologyOptimization()
{
}

int StructuralTopologyOptimization::getGlobalNumDofs() const
{
    return (mGlobalNumDofs);
}

int StructuralTopologyOptimization::getNumDesignVariables() const
{
    return (mNumDesignVariables);
}

double StructuralTopologyOptimization::getVolumeFraction() const
{
    return (mVolumeFraction);
}

double StructuralTopologyOptimization::getStiffnessMatrixOneNorm() const
{
    return (mStiffnessMatrixOneNorm);
}

const Epetra_SerialDenseVector & StructuralTopologyOptimization::getFixedDOFs() const
{
    return (mFixedDOFs);
}

const Epetra_SerialDenseVector & StructuralTopologyOptimization::getForceVector() const
{
    return (mForce);
}

const Epetra_SerialDenseVector & StructuralTopologyOptimization::getDisplacements() const
{
    return (mCurrentDisplacements);
}

const Epetra_SerialDenseMatrix & StructuralTopologyOptimization::getStiffnessMatrix() const
{
    return (mStiffnessMatrix);
}

const Epetra_SerialDenseMatrix & StructuralTopologyOptimization::getElementStiffnessMatrix() const
{
    return (mElementStiffnessMatrix);
}

void StructuralTopologyOptimization::setPenaltyFactor(const double & aInput)
{
    mPenalty = aInput;
}

void StructuralTopologyOptimization::setFilterRadius(const double & aInput)
{
    mFilterRadius = aInput;
}

void StructuralTopologyOptimization::setVolumeFraction(const double & aInput)
{
    mVolumeFraction = aInput;
}

void StructuralTopologyOptimization::setNumElements(const int & aNumElementsXdirection, const int & aNumElementsYdirection)
{
    mNumElemXDirection = aNumElementsXdirection;
    mNumElemYDirection = aNumElementsYdirection;
}

void StructuralTopologyOptimization::setFixedDOFs(const Epetra_SerialDenseVector & aInput)
{
    mIsDirichletConditionsSet = true;
    assert(aInput.Length() > static_cast<int>(0));
    const int tLength = aInput.Length();
    int tWasResized = mFixedDOFs.Resize(tLength);
    tWasResized += 0;
    assert(tWasResized == static_cast<int>(0));
    aInput.COPY(aInput.Length(), aInput.A(), mFixedDOFs.A());
}

void StructuralTopologyOptimization::setForceVector(const Epetra_SerialDenseVector & aInput)
{
    mIsForceSet = true;
    assert(aInput.Length() == mForce.Length());
    aInput.COPY(aInput.Length(), aInput.A(), mForce.A());
}

void StructuralTopologyOptimization::cacheState()
{
    int tLength = mTrialDisplacements.Length();
    assert(tLength == mGlobalNumDofs);
    mTrialDisplacements.COPY(tLength, mTrialDisplacements.A(), mCurrentDisplacements.A());
}

bool StructuralTopologyOptimization::isStateCached() const
{
    return (mIsStateCached);
}

void StructuralTopologyOptimization::disableCacheState()
{
    mIsStateCached = false;
}

void StructuralTopologyOptimization::solve(const Epetra_SerialDenseVector & aControl)
{
    assert(aControl.Length() == mNumDesignVariables);
    assert(mFixedDOFs.Length() > static_cast<int>(0));

    this->checkInputs();

    this->assembleStiffnessMatrix(aControl);

    double tNormForce = mForce.Norm2();
    tNormForce += 0.0;
    assert(tNormForce > static_cast<double>(0));
    mStiffnessMatrixOneNorm = mStiffnessMatrix.NormOne();
    assert(mStiffnessMatrixOneNorm > static_cast<double>(0));

    mSolver.SetMatrix(mStiffnessMatrix);
    mWorkVector.COPY(mWorkVector.Length(), mForce.A(), mWorkVector.A());
    if(mIsStateCached == true)
    {
        mTrialDisplacements.Scale(0);
        mSolver.SetVectors(mTrialDisplacements, mWorkVector);
    }
    else
    {
        mCurrentDisplacements.Scale(0);
        mSolver.SetVectors(mCurrentDisplacements, mWorkVector);
    }
    mSolver.FactorWithEquilibration(true);
    mSolver.Factor();
    mSolver.Solve();
}

double StructuralTopologyOptimization::computeVolumeMisfit(const Epetra_SerialDenseVector & aControl)
{
    int tNumControls = aControl.Length();
    double tCurrentVolume = aControl.ASUM(tNumControls, aControl.A());
    double tNormalizedVolume = tCurrentVolume / static_cast<double>(mNumDesignVariables);
    double tOutput = tNormalizedVolume - mVolumeFraction;
    return (tOutput);
}

double StructuralTopologyOptimization::computeCompliance(const Epetra_SerialDenseVector & aControl)
{
    double tCompliance = 0;
    std::vector<int> tElemDofs(mNumElemDOFs);
    for(int tElemIndexI = 0; tElemIndexI < mNumElemXDirection; tElemIndexI++)
    {
        for(int tElemIndexJ = 0; tElemIndexJ < mNumElemYDirection; tElemIndexJ++)
        {
            int tIndexOne = ((mNumElemYDirection + 1) * tElemIndexI) + tElemIndexJ;
            int tIndexTwo = ((mNumElemYDirection + 1) * (tElemIndexI + 1)) + tElemIndexJ;
            tElemDofs[0] = 2 * tIndexOne;
            tElemDofs[1] = 2 * tIndexOne + 1;
            tElemDofs[2] = 2 * tIndexTwo;
            tElemDofs[3] = 2 * tIndexTwo + 1;
            tElemDofs[4] = 2 * tIndexTwo + 2;
            tElemDofs[5] = 2 * tIndexTwo + 3;
            tElemDofs[6] = 2 * tIndexOne + 2;
            tElemDofs[7] = 2 * tIndexOne + 3;
            if(mIsStateCached == true)
            {
                this->setElementDisplacements(tElemDofs, mTrialDisplacements);
            }
            else
            {
                this->setElementDisplacements(tElemDofs, mCurrentDisplacements);
            }
            int tControlIndex = (tElemIndexI * mNumElemYDirection) + tElemIndexJ;
            mElementStiffnessMatrix.GEMV('N',
                                         mElementStiffnessMatrix.M(),
                                         mElementStiffnessMatrix.N(),
                                         1,
                                         mElementStiffnessMatrix.A(),
                                         mElementStiffnessMatrix.LDA(),
                                         mElemDisplacements.A(),
                                         0,
                                         mElemStiffTimesElemDisplacements.A());
            double tElemPotentialEnergy = 0;
            tElemPotentialEnergy = mElemDisplacements.DOT(mElemDisplacements.Length(),
                                                          mElemStiffTimesElemDisplacements.A(),
                                                          mElemDisplacements.A());
            tCompliance += std::pow(aControl[tControlIndex], mPenalty) * tElemPotentialEnergy;
        }
    }

    return (tCompliance);
}

void StructuralTopologyOptimization::computeVolumeGradient(const Epetra_SerialDenseVector & aControl,
                                                           Epetra_SerialDenseVector & aOutput)
{
    assert(aOutput.Length() == aControl.Length());
    int tNumControls = aOutput.Length();
    for(int tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        aOutput.A()[tIndex] = 1. / static_cast<double>(mNumDesignVariables);
    }
}

void StructuralTopologyOptimization::computeComplianceGradient(const Epetra_SerialDenseVector & aControl,
                                                               Epetra_SerialDenseVector & aOutput)
{
    aOutput.Scale(0);
    std::vector<int> tElemDofs(mNumElemDOFs);
    for(int tElemIndexI = 0; tElemIndexI < mNumElemXDirection; tElemIndexI++)
    {
        for(int tElemIndexJ = 0; tElemIndexJ < mNumElemYDirection; tElemIndexJ++)
        {
            int tIndexOne = ((mNumElemYDirection + 1) * tElemIndexI) + tElemIndexJ;
            int tIndexTwo = ((mNumElemYDirection + 1) * (tElemIndexI + 1)) + tElemIndexJ;
            tElemDofs[0] = 2 * tIndexOne;
            tElemDofs[1] = 2 * tIndexOne + 1;
            tElemDofs[2] = 2 * tIndexTwo;
            tElemDofs[3] = 2 * tIndexTwo + 1;
            tElemDofs[4] = 2 * tIndexTwo + 2;
            tElemDofs[5] = 2 * tIndexTwo + 3;
            tElemDofs[6] = 2 * tIndexOne + 2;
            tElemDofs[7] = 2 * tIndexOne + 3;
            this->setElementDisplacements(tElemDofs, mCurrentDisplacements);
            int tControlIndex = (tElemIndexI * mNumElemYDirection) + tElemIndexJ;
            mElementStiffnessMatrix.GEMV('N',
                                         mElementStiffnessMatrix.M(),
                                         mElementStiffnessMatrix.N(),
                                         1,
                                         mElementStiffnessMatrix.A(),
                                         mElementStiffnessMatrix.LDA(),
                                         mElemDisplacements.A(),
                                         0,
                                         mElemStiffTimesElemDisplacements.A());
            double tElemPotentialEnergy = 0;
            tElemPotentialEnergy = mElemDisplacements.DOT(mElemDisplacements.Length(),
                                                          mElemStiffTimesElemDisplacements.A(),
                                                          mElemDisplacements.A());
            aOutput[tControlIndex] = -mPenalty * std::pow(aControl[tControlIndex], mPenalty - 1) * tElemPotentialEnergy;
        }
    }
}

void StructuralTopologyOptimization::computeComplianceHessianTimesVector(const Epetra_SerialDenseVector & aControl,
                                                                         const Epetra_SerialDenseVector & aVector,
                                                                         Epetra_SerialDenseVector & aOutput)
{
    assert(aControl.Length() == aVector.Length());
    assert(aControl.Length() == aOutput.Length());
    // ******** GAUSS-NEWTON APPROXIMATION ********
    aOutput.Scale(0);
    std::vector<int> tElemDofs(mNumElemDOFs);
    for(int tElemIndexI = 0; tElemIndexI < mNumElemXDirection; tElemIndexI++)
    {
        for(int tElemIndexJ = 0; tElemIndexJ < mNumElemYDirection; tElemIndexJ++)
        {
            int tIndexOne = ((mNumElemYDirection + 1) * tElemIndexI) + tElemIndexJ;
            int tIndexTwo = ((mNumElemYDirection + 1) * (tElemIndexI + 1)) + tElemIndexJ;
            tElemDofs[0] = 2 * tIndexOne;
            tElemDofs[1] = 2 * tIndexOne + 1;
            tElemDofs[2] = 2 * tIndexTwo;
            tElemDofs[3] = 2 * tIndexTwo + 1;
            tElemDofs[4] = 2 * tIndexTwo + 2;
            tElemDofs[5] = 2 * tIndexTwo + 3;
            tElemDofs[6] = 2 * tIndexOne + 2;
            tElemDofs[7] = 2 * tIndexOne + 3;
            this->setElementDisplacements(tElemDofs, mCurrentDisplacements);
            int tControlIndex = (tElemIndexI * mNumElemYDirection) + tElemIndexJ;
            mElementStiffnessMatrix.GEMV('N',
                                         mElementStiffnessMatrix.M(),
                                         mElementStiffnessMatrix.N(),
                                         1,
                                         mElementStiffnessMatrix.A(),
                                         mElementStiffnessMatrix.LDA(),
                                         mElemDisplacements.A(),
                                         0,
                                         mElemStiffTimesElemDisplacements.A());
            double tElemPotentialEnergy = 0;
            tElemPotentialEnergy = mElemDisplacements.DOT(mElemDisplacements.Length(),
                                                          mElemStiffTimesElemDisplacements.A(),
                                                          mElemDisplacements.A());
            aOutput[tControlIndex] = -mPenalty * (mPenalty - 1.0) * std::pow(aControl[tControlIndex], mPenalty - 2.0)
                                     * tElemPotentialEnergy * aVector[tControlIndex];
        }
    }
}

void StructuralTopologyOptimization::applySensitivityFilter(const Epetra_SerialDenseVector & aControl,
                                                            const Epetra_SerialDenseVector & aUnfilteredGradient,
                                                            Epetra_SerialDenseVector & aFilteredGradient)
{
    assert(aControl.Length() == aFilteredGradient.Length());
    assert(aFilteredGradient.Length() == aUnfilteredGradient.Length());

    aFilteredGradient.Scale(0);
    for(int tElemIndexI = 0; tElemIndexI < mNumElemXDirection; tElemIndexI++)
    {
        for(int tElemIndexJ = 0; tElemIndexJ < mNumElemYDirection; tElemIndexJ++)
        {
            double tSum = 0;
            int tBeginOne = std::max(tElemIndexI - static_cast<int>(std::ceil(mFilterRadius)), 0);
            int tEndOne = std::min(tElemIndexI + static_cast<int>(std::ceil(mFilterRadius)), mNumElemXDirection);
            int tFilteredControlIndex = (tElemIndexI * mNumElemYDirection) + tElemIndexJ;
            for(int tFilterIndexI = tBeginOne; tFilterIndexI < tEndOne; tFilterIndexI++)
            {
                int tBeginTwo = std::max(tElemIndexJ - static_cast<int>(std::ceil(mFilterRadius)), 0);
                int tEndTwo = std::min(tElemIndexJ + static_cast<int>(std::ceil(mFilterRadius)), mNumElemYDirection);
                for(int tFilterIndexJ = tBeginTwo; tFilterIndexJ < tEndTwo; tFilterIndexJ++)
                {
                    double tValue = ((tElemIndexI - tFilterIndexI) * (tElemIndexI - tFilterIndexI))
                            + ((tElemIndexJ - tFilterIndexJ) * (tElemIndexJ - tFilterIndexJ));
                    double tFactor = mFilterRadius - std::sqrt(tValue);
                    tSum = tSum + std::max(0., tFactor);
                    int tUnfilteredControlIndex = (tFilterIndexI * mNumElemYDirection) + tFilterIndexJ;
                    aFilteredGradient[tFilteredControlIndex] = aFilteredGradient[tFilteredControlIndex]
                            + std::max(0., tFactor) * aControl[tUnfilteredControlIndex]
                              * aUnfilteredGradient[tUnfilteredControlIndex];
                }
            }
            aFilteredGradient[tFilteredControlIndex] = aFilteredGradient[tFilteredControlIndex]
                    / (aControl[tFilteredControlIndex] * tSum);
        }
    }
}

void StructuralTopologyOptimization::initialize()
{
    mNumDesignVariables = mNumElemXDirection * mNumElemYDirection;
    // **************** ALLOCATE GLOBAL FORCE AND STIFFNESS MATRIX ****************
    mGlobalNumDofs = 2 * (mNumElemXDirection + 1) * (mNumElemYDirection + 1);
    mWorkVector.Resize(mGlobalNumDofs);
    mForce.Resize(mGlobalNumDofs);
    mElemDisplacements.Resize(mNumElemDOFs);
    mTrialDisplacements.Resize(mGlobalNumDofs);
    mCurrentDisplacements.Resize(mGlobalNumDofs);
    mElemStiffTimesElemDisplacements.Resize(mNumElemDOFs);
    int tWasReshaped = mStiffnessMatrix.Reshape(mGlobalNumDofs, mGlobalNumDofs);
    tWasReshaped += 0;
    assert(tWasReshaped == static_cast<int>(0));

    // **************** ALLOCATE ELEMENT STIFFNESS MATRIX ****************
    std::vector<double> tStiffness(mNumElemDOFs);
    tStiffness[0] = static_cast<double>(0.5) - (mPoissonRatio / static_cast<double>(6));
    tStiffness[1] = static_cast<double>(0.125) + (mPoissonRatio / static_cast<double>(8));
    tStiffness[2] = static_cast<double>(-0.25) - (mPoissonRatio / static_cast<double>(12));
    tStiffness[3] = static_cast<double>(-0.125) + (static_cast<double>(3) * mPoissonRatio / static_cast<double>(8));
    tStiffness[4] = static_cast<double>(-0.25) + (mPoissonRatio / static_cast<double>(12));
    tStiffness[5] = static_cast<double>(-0.125) - (mPoissonRatio / static_cast<double>(8));
    tStiffness[6] = mPoissonRatio / static_cast<double>(6);
    tStiffness[7] = static_cast<double>(0.125) - (static_cast<double>(3) * mPoissonRatio / static_cast<double>(8));

    this->fillElementStiffnessMatrix(tStiffness);
}

void StructuralTopologyOptimization::checkInputs()
{
    if(mIsForceSet == false)
    {
        std::ostringstream tErrorMsg;
        tErrorMsg << "\n\n********* ERROR IN " << __FILE__ << ", FUNCTION: " << __func__ << ", LINE: " << __LINE__
                  << ", MESSAGE: FORCE VECTOR WAS NOT SET. ABORT! *********\n\n";
        std::perror(tErrorMsg.str().c_str());
        std::abort();
    }
    else if(mIsDirichletConditionsSet == false)
    {
        std::ostringstream tErrorMsg;
        tErrorMsg << "\n\n********* ERROR IN " << __FILE__ << ", FUNCTION: " << __func__ << ", LINE: " << __LINE__
                  << ", MESSAGE: DIRICHLET CONDITIONS WERE NOT SET. ABORT! *********\n\n";
        std::perror(tErrorMsg.str().c_str());
        std::abort();
    }
}

void StructuralTopologyOptimization::fillElementStiffnessMatrix(const std::vector<double> & aStiffness)
{
    mElementStiffnessMatrix.Reshape(mNumElemDOFs, mNumElemDOFs);

    const double tConstant = mElasticModulus / (static_cast<double>(1) - (mPoissonRatio * mPoissonRatio));
    //*********************** COLUMN ONE ***********************
    mElementStiffnessMatrix[0][0] = tConstant * aStiffness[0];
    mElementStiffnessMatrix[0][1] = tConstant * aStiffness[1];
    mElementStiffnessMatrix[0][2] = tConstant * aStiffness[2];
    mElementStiffnessMatrix[0][3] = tConstant * aStiffness[3];
    mElementStiffnessMatrix[0][4] = tConstant * aStiffness[4];
    mElementStiffnessMatrix[0][5] = tConstant * aStiffness[5];
    mElementStiffnessMatrix[0][6] = tConstant * aStiffness[6];
    mElementStiffnessMatrix[0][7] = tConstant * aStiffness[7];
    //*********************** COLUMN TWO ***********************
    mElementStiffnessMatrix[1][0] = tConstant * aStiffness[1];
    mElementStiffnessMatrix[1][1] = tConstant * aStiffness[0];
    mElementStiffnessMatrix[1][2] = tConstant * aStiffness[7];
    mElementStiffnessMatrix[1][3] = tConstant * aStiffness[6];
    mElementStiffnessMatrix[1][4] = tConstant * aStiffness[5];
    mElementStiffnessMatrix[1][5] = tConstant * aStiffness[4];
    mElementStiffnessMatrix[1][6] = tConstant * aStiffness[3];
    mElementStiffnessMatrix[1][7] = tConstant * aStiffness[2];
    //********************** COLUMN THREE **********************
    mElementStiffnessMatrix[2][0] = tConstant * aStiffness[2];
    mElementStiffnessMatrix[2][1] = tConstant * aStiffness[7];
    mElementStiffnessMatrix[2][2] = tConstant * aStiffness[0];
    mElementStiffnessMatrix[2][3] = tConstant * aStiffness[5];
    mElementStiffnessMatrix[2][4] = tConstant * aStiffness[6];
    mElementStiffnessMatrix[2][5] = tConstant * aStiffness[3];
    mElementStiffnessMatrix[2][6] = tConstant * aStiffness[4];
    mElementStiffnessMatrix[2][7] = tConstant * aStiffness[1];
    //*********************** COLUMN FOUR **********************
    mElementStiffnessMatrix[3][0] = tConstant * aStiffness[3];
    mElementStiffnessMatrix[3][1] = tConstant * aStiffness[6];
    mElementStiffnessMatrix[3][2] = tConstant * aStiffness[5];
    mElementStiffnessMatrix[3][3] = tConstant * aStiffness[0];
    mElementStiffnessMatrix[3][4] = tConstant * aStiffness[7];
    mElementStiffnessMatrix[3][5] = tConstant * aStiffness[2];
    mElementStiffnessMatrix[3][6] = tConstant * aStiffness[1];
    mElementStiffnessMatrix[3][7] = tConstant * aStiffness[4];
    //*********************** COLUMN FIVE **********************
    mElementStiffnessMatrix[4][0] = tConstant * aStiffness[4];
    mElementStiffnessMatrix[4][1] = tConstant * aStiffness[5];
    mElementStiffnessMatrix[4][2] = tConstant * aStiffness[6];
    mElementStiffnessMatrix[4][3] = tConstant * aStiffness[7];
    mElementStiffnessMatrix[4][4] = tConstant * aStiffness[0];
    mElementStiffnessMatrix[4][5] = tConstant * aStiffness[1];
    mElementStiffnessMatrix[4][6] = tConstant * aStiffness[2];
    mElementStiffnessMatrix[4][7] = tConstant * aStiffness[3];
    //*********************** COLUMN SIX ***********************
    mElementStiffnessMatrix[5][0] = tConstant * aStiffness[5];
    mElementStiffnessMatrix[5][1] = tConstant * aStiffness[4];
    mElementStiffnessMatrix[5][2] = tConstant * aStiffness[3];
    mElementStiffnessMatrix[5][3] = tConstant * aStiffness[2];
    mElementStiffnessMatrix[5][4] = tConstant * aStiffness[1];
    mElementStiffnessMatrix[5][5] = tConstant * aStiffness[0];
    mElementStiffnessMatrix[5][6] = tConstant * aStiffness[7];
    mElementStiffnessMatrix[5][7] = tConstant * aStiffness[6];
    //********************* COLUMN SEVEN ***********************
    mElementStiffnessMatrix[6][0] = tConstant * aStiffness[6];
    mElementStiffnessMatrix[6][1] = tConstant * aStiffness[3];
    mElementStiffnessMatrix[6][2] = tConstant * aStiffness[4];
    mElementStiffnessMatrix[6][3] = tConstant * aStiffness[1];
    mElementStiffnessMatrix[6][4] = tConstant * aStiffness[2];
    mElementStiffnessMatrix[6][5] = tConstant * aStiffness[7];
    mElementStiffnessMatrix[6][6] = tConstant * aStiffness[0];
    mElementStiffnessMatrix[6][7] = tConstant * aStiffness[5];
    //********************* COLUMN EIGHT **********************
    mElementStiffnessMatrix[7][0] = tConstant * aStiffness[7];
    mElementStiffnessMatrix[7][1] = tConstant * aStiffness[2];
    mElementStiffnessMatrix[7][2] = tConstant * aStiffness[1];
    mElementStiffnessMatrix[7][3] = tConstant * aStiffness[4];
    mElementStiffnessMatrix[7][4] = tConstant * aStiffness[3];
    mElementStiffnessMatrix[7][5] = tConstant * aStiffness[6];
    mElementStiffnessMatrix[7][6] = tConstant * aStiffness[5];
    mElementStiffnessMatrix[7][7] = tConstant * aStiffness[0];
}

void StructuralTopologyOptimization::assembleStiffnessMatrix(const Epetra_SerialDenseVector & aControl)
{
    const double tValue = 0;
    mStiffnessMatrix.Scale(tValue);

    const int tLength = mFixedDOFs.Length();
    std::vector<double> tFixedDofs(tLength);
    std::copy(mFixedDOFs.A(), mFixedDOFs.A() + tLength, tFixedDofs.begin());

    std::vector<double>::iterator tIterator;
    std::vector<int> tElemDOFs(mNumElemDOFs);
    for(int tElemeIndexI = 0; tElemeIndexI < mNumElemXDirection; tElemeIndexI++)
    {
        for(int tElemeIndexJ = 0; tElemeIndexJ < mNumElemYDirection; tElemeIndexJ++)
        {
            int tIndexOne = ((mNumElemYDirection + 1) * tElemeIndexI) + tElemeIndexJ;
            int tIndexTwo = ((mNumElemYDirection + 1) * (tElemeIndexI + 1)) + tElemeIndexJ;
            tElemDOFs[0] = 2 * tIndexOne;
            tElemDOFs[1] = 2 * tIndexOne + 1;
            tElemDOFs[2] = 2 * tIndexTwo;
            tElemDOFs[3] = 2 * tIndexTwo + 1;
            tElemDOFs[4] = 2 * tIndexTwo + 2;
            tElemDOFs[5] = 2 * tIndexTwo + 3;
            tElemDOFs[6] = 2 * tIndexOne + 2;
            tElemDOFs[7] = 2 * tIndexOne + 3;
            for(int tDOF_IndexJ = 0; tDOF_IndexJ < mNumElemDOFs; tDOF_IndexJ++)
            {
                int tColumnIndex = tElemDOFs[tDOF_IndexJ];
                tIterator = std::find(tFixedDofs.begin(), tFixedDofs.end(), tColumnIndex);
                if(tIterator != tFixedDofs.end())
                {
                    mStiffnessMatrix[tColumnIndex][tColumnIndex] = 1;
                }
                else
                {
                    for(int tDOF_IndexI = 0; tDOF_IndexI < mNumElemDOFs; tDOF_IndexI++)
                    {
                        int tRowIndex = tElemDOFs[tDOF_IndexI];
                        tIterator = std::find(tFixedDofs.begin(), tFixedDofs.end(), tRowIndex);
                        if(tIterator == tFixedDofs.end())
                        {
                            int tControlIndex = (tElemeIndexI * mNumElemYDirection) + tElemeIndexJ;
                            mStiffnessMatrix[tColumnIndex][tRowIndex] =
                                    mStiffnessMatrix[tColumnIndex][tRowIndex] + (std::pow(aControl[tControlIndex], mPenalty)
                                            * mElementStiffnessMatrix[tDOF_IndexJ][tDOF_IndexI]);
                        }
                    }
                }
            }
        }
    }
}

void StructuralTopologyOptimization::setElementDisplacements(const std::vector<int> & aElemDofs, const Epetra_SerialDenseVector & aDisplacements)
{
    assert(aElemDofs.size() == static_cast<size_t>(mNumElemDOFs));
    for(int tIndex = 0; tIndex < mNumElemDOFs; tIndex++)
    {
        int tElemDof = aElemDofs[tIndex];
        mElemDisplacements[tIndex] = aDisplacements[tElemDof];
    }
}

} // namespace Plato
