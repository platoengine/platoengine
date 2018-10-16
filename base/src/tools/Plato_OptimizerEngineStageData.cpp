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
 * Plato_OptimizerEngineStageData.cpp
 *
 *  Created on: Oct 27, 2017
 */

#include <cassert>

#include "Plato_OptimizerEngineStageData.hpp"

namespace Plato
{

/******************************************************************************/
OptimizerEngineStageData::OptimizerEngineStageData() :
        mCheckGradient(false),
        mCheckHessian(false),
        mUserInitialGuess(false),
        mOutputControlToFile(false),
        mOutputDiagnosticsToFile(false),
        mHaveHessian(true),
        mInitialMovingAsymptoteScaleFactor(0.5),
        mGCMMAInnerKKTTolerance(5e-4),
        mCCSAOuterKKTTolerance(1e-10),
        mCCSAOuterControlStagnationTolerance(1e-8),
        mGCMMAInnerControlStagnationTolerance(1e-8),
        mCCSAOuterObjectiveStagnationTolerance(1e-6),
        mCCSAOuterStationarityTolerance(1e-4),
        mKSTrustRegionExpansionFactor(2.0),
        mKSTrustRegionContractionFactor(0.75),
        mKSOuterGradientTolerance(1e-4),
        mKSOuterStationarityTolerance(1e-4),
        mKSOuterStagnationTolerance(1e-8),
        mKSOuterControlStagnationTolerance(1e-8),
        mKSOuterActualReductionTolerance(1e-8),
        mKSInitialRadiusScale(.1),
        mKSMaxRadiusScale(.5),
        mMaxNumIterations(500),
        mProblemUpdateFrequency(0),
        mDerivativeCheckerFinalSuperscript(8),
        mDerivativeCheckerInitialSuperscript(1),
        mGCMMAMaxInnerIterations(5),
        mKSMaxTrustRegionIterations(25),
        mAlgebra(),
        mStateName(),
        mInputFileName(),
        mCacheStageName(),
        mUpdateProblemStageName(),
        mOutputStageName(),
        mObjectiveValueOutputName(),
        mObjectiveHessianOutputName(),
        mObjectiveGradientOutputName(),
        mInitializationStageName(),
        mSetLowerBoundsStageName(),
        mSetUpperBoundsStageName(),
        mLowerBoundValueName(),
        mLowerBoundVectorName(),
        mUpperBoundValueName(),
        mUpperBoundVectorName(),
        mObjectiveValueStageName(),
        mObjectiveHessianStageName(),
        mObjectiveGradientStageName(),
        mInitialGuess(std::vector<double>(1, 0.5)),
        mLowerBoundValues(std::vector<double>(1, 0.)),
        mUpperBoundValues(std::vector<double>(1, 1.)),
        mConstraintNormalizedTargetValues(),
        mConstraintAbsoluteTargetValues(),
        mConstraintReferenceValues(),
        mControlNames(),
        mConstraintValueNames(),
        mDescentDirectionNames(),
        mConstraintHessianNames(),
        mConstraintGradientNames(),
        mConstraintReferenceValueNames()
/******************************************************************************/
{
}

/******************************************************************************/
OptimizerEngineStageData::~OptimizerEngineStageData()
/******************************************************************************/
{
}

/******************************************************************************/
std::vector<double> OptimizerEngineStageData::getLowerBoundValues() const
/******************************************************************************/
{
    return (mLowerBoundValues);
}

/******************************************************************************/
std::vector<double> OptimizerEngineStageData::getUpperBoundValues() const
/******************************************************************************/
{
    return (mUpperBoundValues);
}

/******************************************************************************/
void OptimizerEngineStageData::setLowerBoundValues(const std::vector<double> & aInput)
/******************************************************************************/
{
    mLowerBoundValues = aInput;
}

/******************************************************************************/
void OptimizerEngineStageData::setUpperBoundValues(const std::vector<double> & aInput)
/******************************************************************************/
{
    mUpperBoundValues = aInput;
}

/******************************************************************************/
void OptimizerEngineStageData::setOutputControlToFile(const bool & aInput)
/******************************************************************************/
{
    mOutputControlToFile = aInput;
}

/******************************************************************************/
void OptimizerEngineStageData::setOutputDiagnosticsToFile(const bool & aInput)
/******************************************************************************/
{
    mOutputDiagnosticsToFile = aInput;
}

void OptimizerEngineStageData::setHaveHessian(const bool & aInput)
{
    mHaveHessian = aInput;
}

/******************************************************************************/
void OptimizerEngineStageData::setCheckGradient(const bool & aInput)
/******************************************************************************/
{
    mCheckGradient = aInput;
}

/******************************************************************************/
void OptimizerEngineStageData::setCheckHessian(const bool & aInput)
/******************************************************************************/
{
    mCheckHessian = aInput;
}

/******************************************************************************/
void OptimizerEngineStageData::setUserInitialGuess(const bool & aInput)
/******************************************************************************/
{
    mUserInitialGuess = aInput;
}

/******************************************************************************/
bool OptimizerEngineStageData::getOutputControlToFile() const
/******************************************************************************/
{
    return (mOutputControlToFile);
}

/******************************************************************************/
bool OptimizerEngineStageData::getOutputDiagnosticsToFile() const
/******************************************************************************/
{
    return (mOutputDiagnosticsToFile);
}

bool OptimizerEngineStageData::getHaveHessian() const
{
    return mHaveHessian;
}

/******************************************************************************/
bool OptimizerEngineStageData::getCheckGradient() const
/******************************************************************************/
{
    return mCheckGradient;
}

/******************************************************************************/
bool OptimizerEngineStageData::getCheckHessian() const
/******************************************************************************/
{
    return mCheckHessian;
}

/******************************************************************************/
bool OptimizerEngineStageData::getUserInitialGuess() const
/******************************************************************************/
{
    return (mUserInitialGuess);
}

/******************************************************************************/
size_t OptimizerEngineStageData::getMaxNumIterations() const
/******************************************************************************/
{
    return (mMaxNumIterations);
}

/******************************************************************************/
void OptimizerEngineStageData::setMaxNumIterations(const size_t & aInput)
/******************************************************************************/
{
    assert(aInput >= static_cast<size_t>(0));
    mMaxNumIterations = aInput;
}

/******************************************************************************/
int OptimizerEngineStageData::getDerivativeCheckerFinalSuperscript() const
/******************************************************************************/
{
    return (mDerivativeCheckerFinalSuperscript);
}

/******************************************************************************/
void OptimizerEngineStageData::setDerivativeCheckerFinalSuperscript(const int & aInput)
/******************************************************************************/
{
    mDerivativeCheckerFinalSuperscript = aInput;
}

/******************************************************************************/
int OptimizerEngineStageData::getDerivativeCheckerInitialSuperscript() const
/******************************************************************************/
{
    return (mDerivativeCheckerInitialSuperscript);
}

/******************************************************************************/
void OptimizerEngineStageData::setDerivativeCheckerInitialSuperscript(const int & aInput)
/******************************************************************************/
{
    mDerivativeCheckerInitialSuperscript = aInput;
}

/******************************************************************************/
std::vector<double> OptimizerEngineStageData::getInitialGuess() const
/******************************************************************************/
{
    return (mInitialGuess);
}

/******************************************************************************/
void OptimizerEngineStageData::setInitialGuess(const std::vector<double> & aInput)
/******************************************************************************/
{
    mInitialGuess = aInput;
}

/******************************************************************************/
void OptimizerEngineStageData::setInitialGuess(const std::vector<std::string> & aInput)
/******************************************************************************/
{
    mInitialGuess.resize(aInput.size());
    for(unsigned int i=0; i<aInput.size(); i++)
    {
       mInitialGuess[i] = std::strtold(aInput[i].c_str(), nullptr);
    }
}

/******************************************************************************/
double OptimizerEngineStageData::getKSTrustRegionExpansionFactor() const
/******************************************************************************/
{
    return (mKSTrustRegionExpansionFactor);
}

/******************************************************************************/
void OptimizerEngineStageData::setKSTrustRegionExpansionFactor(const double & aInput)
/******************************************************************************/
{
    mKSTrustRegionExpansionFactor = aInput;
}

/******************************************************************************/
double OptimizerEngineStageData::getKSTrustRegionContractionFactor() const
/******************************************************************************/
{
    return (mKSTrustRegionContractionFactor);
}

/******************************************************************************/
void OptimizerEngineStageData::setKSTrustRegionContractionFactor(const double & aInput)
/******************************************************************************/
{
    mKSTrustRegionContractionFactor = aInput;
}

/******************************************************************************/
int OptimizerEngineStageData::getGCMMAMaxInnerIterations() const
/******************************************************************************/
{
    return (mGCMMAMaxInnerIterations);
}

/******************************************************************************/
void OptimizerEngineStageData::setGCMMAMaxInnerIterations(const int & aInput)
/******************************************************************************/
{
    mGCMMAMaxInnerIterations = aInput;
}

/******************************************************************************/
double OptimizerEngineStageData::getGCMMAInnerKKTTolerance() const
/******************************************************************************/
{
    return (mGCMMAInnerKKTTolerance);
}

/******************************************************************************/
void OptimizerEngineStageData::setGCMMAInnerKKTTolerance(const double & aInput)
/******************************************************************************/
{
    mGCMMAInnerKKTTolerance = aInput;
}

/******************************************************************************/
double OptimizerEngineStageData::getGCMMAInnerControlStagnationTolerance() const
/******************************************************************************/
{
    return (mGCMMAInnerControlStagnationTolerance);
}

/******************************************************************************/
void OptimizerEngineStageData::setGCMMAInnerControlStagnationTolerance(const double & aInput)
/******************************************************************************/
{
    mGCMMAInnerControlStagnationTolerance = aInput;
}

/******************************************************************************/
double OptimizerEngineStageData::getInitialMovingAsymptoteScaleFactor() const
/******************************************************************************/
{
    return (mInitialMovingAsymptoteScaleFactor);
}

/******************************************************************************/
void OptimizerEngineStageData::setInitialMovingAsymptoteScaleFactor(const double & aInput)
/******************************************************************************/
{
    mInitialMovingAsymptoteScaleFactor = aInput;
}

/******************************************************************************/
double OptimizerEngineStageData::getCCSAOuterKKTTolerance() const
/******************************************************************************/
{
    return (mCCSAOuterKKTTolerance);
}

/******************************************************************************/
void OptimizerEngineStageData::setCCSAOuterKKTTolerance(const double & aInput)
/******************************************************************************/
{
    mCCSAOuterKKTTolerance = aInput;
}

/******************************************************************************/
double OptimizerEngineStageData::getCCSAOuterControlStagnationTolerance() const
/******************************************************************************/
{
    return (mCCSAOuterControlStagnationTolerance);
}

/******************************************************************************/
void OptimizerEngineStageData::setCCSAOuterControlStagnationTolerance(const double & aInput)
/******************************************************************************/
{
    mCCSAOuterControlStagnationTolerance = aInput;
}

/******************************************************************************/
double OptimizerEngineStageData::getCCSAOuterObjectiveStagnationTolerance() const
/******************************************************************************/
{
    return (mCCSAOuterObjectiveStagnationTolerance);
}

/******************************************************************************/
void OptimizerEngineStageData::setCCSAOuterObjectiveStagnationTolerance(const double & aInput)
/******************************************************************************/
{
    mCCSAOuterObjectiveStagnationTolerance = aInput;
}

/******************************************************************************/
double OptimizerEngineStageData::getCCSAOuterStationarityTolerance() const
/******************************************************************************/
{
    return (mCCSAOuterStationarityTolerance);
}

/******************************************************************************/
void OptimizerEngineStageData::setCCSAOuterStationarityTolerance(const double & aInput)
/******************************************************************************/
{
    mCCSAOuterStationarityTolerance = aInput;
}

/******************************************************************************/
int OptimizerEngineStageData::getKSMaxTrustRegionIterations() const
/******************************************************************************/
{
    return (mKSMaxTrustRegionIterations);
}

/******************************************************************************/
void OptimizerEngineStageData::setKSMaxTrustRegionIterations(const int & aInput)
/******************************************************************************/
{
    mKSMaxTrustRegionIterations = aInput;
}

/******************************************************************************/
double OptimizerEngineStageData::getKSOuterGradientTolerance() const
/******************************************************************************/
{
    return (mKSOuterGradientTolerance);
}

/******************************************************************************/
void OptimizerEngineStageData::setKSOuterGradientTolerance(const double & aInput)
/******************************************************************************/
{
    mKSOuterGradientTolerance = aInput;
}

/******************************************************************************/
double OptimizerEngineStageData::getKSOuterStationarityTolerance() const
/******************************************************************************/
{
    return (mKSOuterStationarityTolerance);
}

/******************************************************************************/
void OptimizerEngineStageData::setKSOuterStationarityTolerance(const double & aInput)
/******************************************************************************/
{
    mKSOuterStationarityTolerance = aInput;
}

/******************************************************************************/
double OptimizerEngineStageData::getKSOuterStagnationTolerance() const
/******************************************************************************/
{
    return (mKSOuterStagnationTolerance);
}

/******************************************************************************/
void OptimizerEngineStageData::setKSOuterStagnationTolerance(const double & aInput)
/******************************************************************************/
{
    mKSOuterStagnationTolerance = aInput;
}

/******************************************************************************/
double OptimizerEngineStageData::getKSOuterControlStagnationTolerance() const
/******************************************************************************/
{
    return (mKSOuterControlStagnationTolerance);
}

/******************************************************************************/
void OptimizerEngineStageData::setKSOuterControlStagnationTolerance(const double & aInput)
/******************************************************************************/
{
    mKSOuterControlStagnationTolerance = aInput;
}

/******************************************************************************/
double OptimizerEngineStageData::getKSOuterActualReductionTolerance() const
/******************************************************************************/
{
    return (mKSOuterActualReductionTolerance);
}

/******************************************************************************/
void OptimizerEngineStageData::setKSOuterActualReductionTolerance(const double & aInput)
/******************************************************************************/
{
    mKSOuterActualReductionTolerance = aInput;
}

/******************************************************************************/
std::string OptimizerEngineStageData::getStateName() const
/******************************************************************************/
{
    assert(mStateName.empty() == false);
    return (mStateName);
}

/******************************************************************************/
void OptimizerEngineStageData::setStateNames(const std::string & aInput)
/******************************************************************************/
{
    assert(aInput.empty() == false);
    mStateName = aInput;
}

/******************************************************************************/
std::string OptimizerEngineStageData::getInputFileName() const
/******************************************************************************/
{
    return (mInputFileName);
}

/******************************************************************************/
void OptimizerEngineStageData::setInputFileName(const std::string & aInput)
/******************************************************************************/
{
    mInputFileName = aInput;
}

/******************************************************************************/
size_t OptimizerEngineStageData::getNumControlVectors() const
/******************************************************************************/
{
    assert(mControlNames.empty() == false);
    return (mControlNames.size());
}

/******************************************************************************/
std::string OptimizerEngineStageData::getLowerBoundVectorName() const
/******************************************************************************/
{
    return mLowerBoundVectorName;
}

/******************************************************************************/
std::string OptimizerEngineStageData::getLowerBoundValueName() const
/******************************************************************************/
{
    return mLowerBoundValueName;
}

/******************************************************************************/
void OptimizerEngineStageData::setLowerBoundValueName(const std::string & aInput)
/******************************************************************************/
{
    assert(aInput.empty() == false);
    mLowerBoundValueName = aInput;
}

/******************************************************************************/
void OptimizerEngineStageData::setLowerBoundVectorName(const std::string & aInput)
/******************************************************************************/
{
    assert(aInput.empty() == false);
    mLowerBoundVectorName = aInput;
}

/******************************************************************************/
std::string OptimizerEngineStageData::getUpperBoundVectorName() const
/******************************************************************************/
{
    return mUpperBoundVectorName;
}

/******************************************************************************/
std::string OptimizerEngineStageData::getUpperBoundValueName() const
/******************************************************************************/
{
    return mUpperBoundValueName;
}

/******************************************************************************/
void OptimizerEngineStageData::setUpperBoundValueName(const std::string & aInput)
/******************************************************************************/
{
    assert(aInput.empty() == false);
    mUpperBoundValueName = aInput;
}

/******************************************************************************/
void OptimizerEngineStageData::setUpperBoundVectorName(const std::string & aInput)
/******************************************************************************/
{
    assert(aInput.empty() == false);
    mUpperBoundVectorName = aInput;
}

/******************************************************************************/
std::vector<std::string> OptimizerEngineStageData::getControlNames() const
/******************************************************************************/
{
    assert(mControlNames.empty() == false);
    return (mControlNames);
}

/******************************************************************************/
std::string OptimizerEngineStageData::getControlName(const size_t & aInput) const
/******************************************************************************/
{
    assert(aInput >= static_cast<size_t>(0));
    assert(mControlNames.empty() == false);
    assert(aInput < mControlNames.size());
    return (mControlNames[aInput]);
}

/******************************************************************************/
void OptimizerEngineStageData::addControlName(const std::string & aInput)
/******************************************************************************/
{
    assert(aInput.empty() == false);
    mControlNames.push_back(aInput);
}

/******************************************************************************/
void OptimizerEngineStageData::setControlNames(const std::vector<std::string> & aInput)
/******************************************************************************/
{
    assert(aInput.empty() == false);
    mControlNames = aInput;
}

std::vector<std::string> OptimizerEngineStageData::getDescentDirectionNames() const
{
    assert(mDescentDirectionNames.empty() == false);
    return (mDescentDirectionNames);
}

/******************************************************************************/
std::string OptimizerEngineStageData::getDescentDirectionName(const size_t & aInput) const
/******************************************************************************/
{
    assert(aInput >= static_cast<size_t>(0));
    assert(mDescentDirectionNames.empty() == false);
    assert(aInput < mDescentDirectionNames.size());
    return (mDescentDirectionNames[aInput]);
}

/******************************************************************************/
void OptimizerEngineStageData::addDescentDirectionName(const std::string & aInput)
/******************************************************************************/
{
    assert(aInput.empty() == false);
    mDescentDirectionNames.push_back(aInput);
}

/******************************************************************************/
void OptimizerEngineStageData::setDescentDirectionNames(const std::vector<std::string> & aInput)
/******************************************************************************/
{
    assert(aInput.empty() == false);
    mDescentDirectionNames = aInput;
}

/******************************************************************************/
std::string OptimizerEngineStageData::getAlgebra() const
/******************************************************************************/
{
    return (mAlgebra);
}

/******************************************************************************/
void OptimizerEngineStageData::setAlgebra(const std::string & aInput)
/******************************************************************************/
{
    mAlgebra = aInput;
}

/******************************************************************************/
std::string OptimizerEngineStageData::getOutputStageName() const
/******************************************************************************/
{
    return (mOutputStageName);
}

/******************************************************************************/
void OptimizerEngineStageData::setOutputStageName(const std::string & aInput)
/******************************************************************************/
{
    mOutputStageName.clear();
    mOutputStageName = aInput;
}

/******************************************************************************/
std::string OptimizerEngineStageData::getCacheStageName() const
/******************************************************************************/
{
    return (mCacheStageName);
}
/******************************************************************************/
void OptimizerEngineStageData::setCacheStageName(const std::string & aInput)
/******************************************************************************/
{
    mCacheStageName.clear();
    mCacheStageName = aInput;
}

/******************************************************************************/
std::string OptimizerEngineStageData::getUpdateProblemStageName() const
/******************************************************************************/
{
    return (mUpdateProblemStageName);
}
/******************************************************************************/
void OptimizerEngineStageData::setUpdateProblemStageName(const std::string & aInput)
/******************************************************************************/
{
    mUpdateProblemStageName = aInput;
}

/******************************************************************************/
std::string OptimizerEngineStageData::getObjectiveValueOutputName() const
/******************************************************************************/
{
    assert(mObjectiveValueOutputName.empty() == false);
    return (mObjectiveValueOutputName);
}

/******************************************************************************/
void OptimizerEngineStageData::setObjectiveValueOutputName(const std::string & aInput)
/******************************************************************************/
{
    mObjectiveValueOutputName.clear();
    mObjectiveValueOutputName.assign(aInput.begin(), aInput.end());
}

/******************************************************************************/
std::string OptimizerEngineStageData::getObjectiveValueStageName() const
/******************************************************************************/
{
    return (mObjectiveValueStageName);
}

/******************************************************************************/
void OptimizerEngineStageData::setObjectiveValueStageName(const std::string & aInput)
/******************************************************************************/
{
    mObjectiveValueStageName.clear();
    mObjectiveValueStageName.assign(aInput.begin(), aInput.end());
}

/******************************************************************************/
std::string OptimizerEngineStageData::getObjectiveHessianOutputName() const
/******************************************************************************/
{
    return (mObjectiveHessianOutputName);
}

/******************************************************************************/
void OptimizerEngineStageData::setObjectiveHessianOutputName(const std::string & aInput)
/******************************************************************************/
{
    assert(aInput.empty() == false);
    mObjectiveHessianOutputName.clear();
    mObjectiveHessianOutputName.assign(aInput.begin(), aInput.end());
}

/******************************************************************************/
std::string OptimizerEngineStageData::getObjectiveHessianStageName() const
/******************************************************************************/
{
    return (mObjectiveHessianStageName);
}

/******************************************************************************/
void OptimizerEngineStageData::setObjectiveHessianStageName(const std::string & aInput)
/******************************************************************************/
{
    assert(aInput.empty() == false);
    mObjectiveHessianStageName.clear();
    mObjectiveHessianStageName.assign(aInput.begin(), aInput.end());
}

/******************************************************************************/
std::string OptimizerEngineStageData::getObjectiveGradientOutputName() const
/******************************************************************************/
{
    assert(mObjectiveGradientOutputName.empty() == false);
    return (mObjectiveGradientOutputName);
}

/******************************************************************************/
void OptimizerEngineStageData::setObjectiveGradientOutputName(const std::string & aInput)
/******************************************************************************/
{
    assert(aInput.empty() == false);
    mObjectiveGradientOutputName.clear();
    mObjectiveGradientOutputName.assign(aInput.begin(), aInput.end());
}

/******************************************************************************/
std::string OptimizerEngineStageData::getObjectiveGradientStageName() const
/******************************************************************************/
{
    return (mObjectiveGradientStageName);
}

/******************************************************************************/
void OptimizerEngineStageData::setObjectiveGradientStageName(const std::string & aInput)
/******************************************************************************/
{
    mObjectiveGradientStageName.clear();
    mObjectiveGradientStageName.assign(aInput.begin(), aInput.end());
}

/******************************************************************************/
std::string OptimizerEngineStageData::getInitializationStageName() const
/******************************************************************************/
{
    return (mInitializationStageName);
}

/******************************************************************************/
void OptimizerEngineStageData::setInitializationStageName(const std::string & aInput)
/******************************************************************************/
{
    assert(aInput.empty() == false);
    mInitializationStageName.clear();
    mInitializationStageName.assign(aInput.begin(), aInput.end());
}

/******************************************************************************/
std::string OptimizerEngineStageData::getSetLowerBoundsStageName() const
/******************************************************************************/
{
    return (mSetLowerBoundsStageName);
}

/******************************************************************************/
void OptimizerEngineStageData::setSetLowerBoundsStageName(const std::string & aInput)
/******************************************************************************/
{
    assert(aInput.empty() == false);
    mSetLowerBoundsStageName.clear();
    mSetLowerBoundsStageName.assign(aInput.begin(), aInput.end());
}

/******************************************************************************/
std::string OptimizerEngineStageData::getSetUpperBoundsStageName() const
/******************************************************************************/
{
    return (mSetUpperBoundsStageName);
}

/******************************************************************************/
void OptimizerEngineStageData::setSetUpperBoundsStageName(const std::string & aInput)
/******************************************************************************/
{
    assert(aInput.empty() == false);
    mSetUpperBoundsStageName.clear();
    mSetUpperBoundsStageName.assign(aInput.begin(), aInput.end());
}

/******************************************************************************/
size_t OptimizerEngineStageData::getNumConstraints() const
/******************************************************************************/
{
    assert(mConstraintValueNames.empty() == false);
    return (mConstraintValueNames.size());
}

/******************************************************************************/
double OptimizerEngineStageData::getConstraintNormalizedTargetValue(const size_t & aIndex) const
/******************************************************************************/
{
    assert(mConstraintNormalizedTargetValues.empty() == false);
    assert(aIndex >= static_cast<size_t>(0));
    assert(aIndex < static_cast<size_t>(mConstraintNormalizedTargetValues.size()));
    const std::string & tValueName = mConstraintValueNames[aIndex];
    std::map<std::string, double>::const_iterator tIterator;
    tIterator = mConstraintNormalizedTargetValues.find(tValueName);
    return (tIterator->second);
}

/******************************************************************************/
double OptimizerEngineStageData::getConstraintNormalizedTargetValue(const std::string & aValueName) const
/******************************************************************************/
{
    assert(mConstraintNormalizedTargetValues.empty() == false);
    std::map<std::string, double>::const_iterator tIterator;
    tIterator = mConstraintNormalizedTargetValues.find(aValueName);
    assert(tIterator != mConstraintNormalizedTargetValues.end());
    return (tIterator->second);
}

/******************************************************************************/
void OptimizerEngineStageData::addConstraintNormalizedTargetValue(const std::string & aValueName, const double & aInput)
/******************************************************************************/
{
    std::map<std::string, double>::iterator tIterator;
    tIterator = mConstraintNormalizedTargetValues.find(aValueName);
    if(tIterator != mConstraintNormalizedTargetValues.end())
    {
        tIterator->second = aInput;
    }
    else
    {
        mConstraintNormalizedTargetValues[aValueName] = aInput;
    }
}

/******************************************************************************/
bool OptimizerEngineStageData::constraintNormalizedTargetValueWasSet(const std::string & aValueName) const
/******************************************************************************/
{
    std::map<std::string, double>::const_iterator tIterator;
    tIterator = mConstraintNormalizedTargetValues.find(aValueName);
    return (tIterator != mConstraintNormalizedTargetValues.end());
}

/******************************************************************************/
double OptimizerEngineStageData::getConstraintAbsoluteTargetValue(const size_t & aIndex) const
/******************************************************************************/
{
    assert(aIndex >= static_cast<size_t>(0));
    assert(aIndex < static_cast<size_t>(mConstraintAbsoluteTargetValues.size()));
    const std::string & tValueName = mConstraintValueNames[aIndex];
    std::map<std::string, double>::const_iterator tIterator;
    tIterator = mConstraintAbsoluteTargetValues.find(tValueName);
    return (tIterator->second);
}

/******************************************************************************/
double OptimizerEngineStageData::getConstraintAbsoluteTargetValue(const std::string & aValueName) const
/******************************************************************************/
{
    std::map<std::string, double>::const_iterator tIterator;
    tIterator = mConstraintAbsoluteTargetValues.find(aValueName);
    if(tIterator == mConstraintAbsoluteTargetValues.end())
    {
        std::cout << "\n\nERROR: Trying to get a constraint's absolute value when it hasn't been set.\n\n";
        return 0.0;
    }
    else
        return (tIterator->second);
}

/******************************************************************************/
void OptimizerEngineStageData::addConstraintAbsoluteTargetValue(const std::string & aValueName, const double & aInput)
/******************************************************************************/
{
    std::map<std::string, double>::iterator tIterator;
    tIterator = mConstraintAbsoluteTargetValues.find(aValueName);
    if(tIterator != mConstraintAbsoluteTargetValues.end())
    {
        tIterator->second = aInput;
    }
    else
    {
        mConstraintAbsoluteTargetValues[aValueName] = aInput;
    }
}

/******************************************************************************/
bool OptimizerEngineStageData::constraintAbsoluteTargetValueWasSet(const std::string & aValueName) const
/******************************************************************************/
{
    std::map<std::string, double>::const_iterator tIterator;
    tIterator = mConstraintAbsoluteTargetValues.find(aValueName);
    return (tIterator != mConstraintAbsoluteTargetValues.end());
}

/******************************************************************************/
double OptimizerEngineStageData::getConstraintReferenceValue(const size_t & aIndex) const
/******************************************************************************/
{
    assert(mConstraintReferenceValues.empty() == false);
    assert(aIndex >= static_cast<size_t>(0));
    assert(aIndex < static_cast<size_t>(mConstraintReferenceValues.size()));
    const std::string & tValueName = mConstraintValueNames[aIndex];
    std::map<std::string, double>::const_iterator tIterator;
    tIterator = mConstraintReferenceValues.find(tValueName);
    return (tIterator->second);
}

/******************************************************************************/
double OptimizerEngineStageData::getConstraintReferenceValue(const std::string & aValueName) const
/******************************************************************************/
{
    assert(mConstraintReferenceValues.empty() == false);
    std::map<std::string, double>::const_iterator tIterator;
    tIterator = mConstraintReferenceValues.find(aValueName);
    assert(tIterator != mConstraintReferenceValues.end());
    return (tIterator->second);
}

/******************************************************************************/
void OptimizerEngineStageData::addConstraintReferenceValue(const std::string & aValueName, const double & aInput)
/******************************************************************************/
{
    std::map<std::string, double>::iterator tIterator;
    tIterator = mConstraintReferenceValues.find(aValueName);
    if(tIterator != mConstraintReferenceValues.end())
    {
        tIterator->second = aInput;
    }
    else
    {
        mConstraintReferenceValues[aValueName] = aInput;
    }
}

/******************************************************************************/
bool OptimizerEngineStageData::constraintReferenceValueWasSet(const std::string & aValueName) const
/******************************************************************************/
{
    std::map<std::string, double>::const_iterator tIterator;
    tIterator = mConstraintReferenceValues.find(aValueName);
    return (tIterator != mConstraintReferenceValues.end());
}

/******************************************************************************/
std::vector<std::string> OptimizerEngineStageData::getConstraintValueNames() const
/******************************************************************************/
{
    assert(mConstraintValueNames.empty() == false);
    return (mConstraintValueNames);
}

/******************************************************************************/
std::string OptimizerEngineStageData::getConstraintValueName(const size_t & aIndex) const
/******************************************************************************/
{
    assert(aIndex >= static_cast<size_t>(0));
    assert(mConstraintValueNames.empty() == false);
    assert(aIndex < mConstraintValueNames.size());
    return (mConstraintValueNames[aIndex]);
}

/******************************************************************************/
void OptimizerEngineStageData::addConstraintValueName(const std::string & aInput)
/******************************************************************************/
{
    assert(aInput.empty() == false);
    mConstraintValueNames.push_back(aInput);
}

/******************************************************************************/
void OptimizerEngineStageData::setConstraintValueNames(const std::vector<std::string> & aInput)
/******************************************************************************/
{
    assert(aInput.empty() == false);
    mConstraintValueNames = aInput;
}

/******************************************************************************/
std::string OptimizerEngineStageData::getConstraintReferenceValueName(const size_t & aIndex) const
/******************************************************************************/
{
    if(mConstraintReferenceValueNames.empty() == true)
        return "";
    assert(aIndex >= static_cast<size_t>(0));
    assert(aIndex < static_cast<size_t>(mConstraintReferenceValueNames.size()));
    const std::string & tValueName = mConstraintValueNames[aIndex];
    std::map<std::string, std::string>::const_iterator tIterator;
    tIterator = mConstraintReferenceValueNames.find(tValueName);
    return (tIterator->second);
}

/******************************************************************************/
std::string OptimizerEngineStageData::getConstraintReferenceValueName(const std::string & aValueName) const
/******************************************************************************/
{
    assert(mConstraintReferenceValueNames.empty() == false);
    std::map<std::string, std::string>::const_iterator tIterator;
    tIterator = mConstraintReferenceValueNames.find(aValueName);
    assert(tIterator != mConstraintReferenceValueNames.end());
    return (tIterator->second);
}

/******************************************************************************/
void OptimizerEngineStageData::addConstraintReferenceValueName(const std::string & aValueName,
                                                               const std::string & aReferenceValueName)
/******************************************************************************/
{
    std::map<std::string, std::string>::iterator tIterator;
    tIterator = mConstraintReferenceValueNames.find(aValueName);
    if(tIterator != mConstraintReferenceValueNames.end())
    {
        tIterator->second = aReferenceValueName;
    }
    else
    {
        mConstraintReferenceValueNames[aValueName] = aReferenceValueName;
    }
}

/******************************************************************************/
std::string OptimizerEngineStageData::getConstraintHessianName(const size_t & aIndex) const
/******************************************************************************/
{
    assert(mConstraintHessianNames.empty() == false);
    assert(aIndex >= static_cast<size_t>(0));
    assert(aIndex < static_cast<size_t>(mConstraintHessianNames.size()));
    const std::string & tValueName = mConstraintValueNames[aIndex];
    std::map<std::string, std::string>::const_iterator tIterator;
    tIterator = mConstraintHessianNames.find(tValueName);
    return (tIterator->second);
}

/******************************************************************************/
std::string OptimizerEngineStageData::getConstraintHessianName(const std::string & aValueName) const
/******************************************************************************/
{
    assert(mConstraintHessianNames.empty() == false);
    std::map<std::string, std::string>::const_iterator tIterator;
    tIterator = mConstraintHessianNames.find(aValueName);
    assert(tIterator != mConstraintHessianNames.end());
    return (tIterator->second);
}

/******************************************************************************/
void OptimizerEngineStageData::addConstraintHessianName(const std::string & aValueName, const std::string & aHessianName)
/******************************************************************************/
{
    std::map<std::string, std::string>::iterator tIterator;
    tIterator = mConstraintHessianNames.find(aValueName);
    if(tIterator != mConstraintHessianNames.end())
    {
        tIterator->second = aHessianName;
    }
    else
    {
        mConstraintHessianNames[aValueName] = aHessianName;
    }
}

/******************************************************************************/
std::string OptimizerEngineStageData::getConstraintGradientName(const size_t & aIndex) const
/******************************************************************************/
{
    assert(mConstraintGradientNames.empty() == false);
    assert(aIndex >= static_cast<size_t>(0));
    assert(aIndex < static_cast<size_t>(mConstraintGradientNames.size()));
    const std::string & tValueName = mConstraintValueNames[aIndex];
    std::map<std::string, std::string>::const_iterator tIterator;
    tIterator = mConstraintGradientNames.find(tValueName);
    return (tIterator->second);
}

/******************************************************************************/
std::string OptimizerEngineStageData::getConstraintGradientName(const std::string & aValueName) const
/******************************************************************************/
{
    assert(mConstraintGradientNames.empty() == false);
    std::map<std::string, std::string>::const_iterator tIterator;
    tIterator = mConstraintGradientNames.find(aValueName);
    assert(tIterator != mConstraintGradientNames.end());
    return (tIterator->second);
}

/******************************************************************************/
void OptimizerEngineStageData::addConstraintGradientName(const std::string & aValueName, const std::string & aGradientName)
/******************************************************************************/
{
    std::map<std::string, std::string>::iterator tIterator;
    tIterator = mConstraintGradientNames.find(aValueName);
    if(tIterator != mConstraintGradientNames.end())
    {
        tIterator->second = aGradientName;
    }
    else
    {
        mConstraintGradientNames[aValueName] = aGradientName;
    }
}

double OptimizerEngineStageData::getKSInitialRadiusScale() const
{
    return mKSInitialRadiusScale;
}
void OptimizerEngineStageData::setKSInitialRadiusScale(const double& aInput)
{
    mKSInitialRadiusScale = aInput;
}
double OptimizerEngineStageData::getKSMaxRadiusScale() const
{
    return mKSMaxRadiusScale;
}
void OptimizerEngineStageData::setKSMaxRadiusScale(const double& aInput)
{
    mKSMaxRadiusScale = aInput;
}

double OptimizerEngineStageData::getProblemUpdateFrequency() const
{
    return mProblemUpdateFrequency;
}
void OptimizerEngineStageData::setProblemUpdateFrequency(const double& aInput)
{
    mProblemUpdateFrequency = aInput;
}

} //namespace Plato
