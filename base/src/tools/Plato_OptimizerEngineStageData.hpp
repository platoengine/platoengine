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
 * Plato_OptimizerEngineStageData.hpp
 *
 *  Created on: Oct 27, 2017
 */

#ifndef PLATO_OPTIMIZERENGINESTAGEDATA_HPP_
#define PLATO_OPTIMIZERENGINESTAGEDATA_HPP_

#include <map>
#include <string>
#include <vector>

#include "Plato_InputData.hpp"

namespace Plato
{

class OptimizerEngineStageData : public Plato::InputData
{
public:
    OptimizerEngineStageData();
    ~OptimizerEngineStageData();

    void setCheckGradient(const bool & aInput);
    void setCheckHessian(const bool & aInput);
    void setUserInitialGuess(const bool & aInput);
    void setOutputControlToFile(const bool & aInput);
    void setOutputDiagnosticsToFile(const bool & aInput);
    void setHaveHessian(const bool & aInput);

    bool getCheckGradient() const;
    bool getCheckHessian() const;
    bool getUserInitialGuess() const;
    bool getOutputControlToFile() const;
    bool getOutputDiagnosticsToFile() const;
    bool getHaveHessian() const;

    std::vector<double> getLowerBoundValues() const;
    std::vector<double> getUpperBoundValues() const;
    void setLowerBoundValues(const std::vector<double> & aInput);
    void setUpperBoundValues(const std::vector<double> & aInput);

    size_t getMaxNumIterations() const;
    void setMaxNumIterations(const size_t & aInput);

    int getDerivativeCheckerFinalSuperscript() const;
    void setDerivativeCheckerFinalSuperscript(const int & aInput);
    int getDerivativeCheckerInitialSuperscript() const;
    void setDerivativeCheckerInitialSuperscript(const int & aInput);

    std::vector<double> getInitialGuess() const;
    void setInitialGuess(const std::vector<double> & aInput);
    void setInitialGuess(const std::vector<std::string> & aInput);

    double getInitialMovingAsymptoteScaleFactor() const;
    void setInitialMovingAsymptoteScaleFactor(const double & aInput);

    double getGCMMAInnerKKTTolerance() const;
    void setGCMMAInnerKKTTolerance(const double & aInput);

    int getGCMMAMaxInnerIterations() const;
    void setGCMMAMaxInnerIterations(const int & aInput);

    double getGCMMAInnerControlStagnationTolerance() const;
    void setGCMMAInnerControlStagnationTolerance(const double & aInput);

    double getCCSAOuterKKTTolerance() const;
    void setCCSAOuterKKTTolerance(const double & aInput);

    double getCCSAOuterControlStagnationTolerance() const;
    void setCCSAOuterControlStagnationTolerance(const double & aInput);

    double getCCSAOuterObjectiveStagnationTolerance() const;
    void setCCSAOuterObjectiveStagnationTolerance(const double & aInput);

    double getCCSAOuterStationarityTolerance() const;
    void setCCSAOuterStationarityTolerance(const double & aInput);

    double getKSTrustRegionExpansionFactor() const;
    void setKSTrustRegionExpansionFactor(const double & aInput);

    double getKSTrustRegionContractionFactor() const;
    void setKSTrustRegionContractionFactor(const double & aInput);

    int getKSMaxTrustRegionIterations() const;
    void setKSMaxTrustRegionIterations(const int & aInput);

    double getKSOuterGradientTolerance() const;
    void setKSOuterGradientTolerance(const double & aInput);

    double getKSOuterStationarityTolerance() const;
    void setKSOuterStationarityTolerance(const double & aInput);

    double getKSOuterStagnationTolerance() const;
    void setKSOuterStagnationTolerance(const double & aInput);

    double getKSOuterControlStagnationTolerance() const;
    void setKSOuterControlStagnationTolerance(const double & aInput);

    double getKSOuterActualReductionTolerance() const;
    void setKSOuterActualReductionTolerance(const double & aInput);

    std::string getStateName() const;
    void setStateNames(const std::string & aInput);

    std::string getInputFileName() const;
    void setInputFileName(const std::string & aInput);

    size_t getNumControlVectors() const;
    std::vector<std::string> getControlNames() const;
    std::string getControlName(const size_t & aIndex) const;
    void addControlName(const std::string & aInput);
    void setControlNames(const std::vector<std::string> & aInput);

    std::string getLowerBoundVectorName() const;
    std::string getLowerBoundValueName() const;
    void setLowerBoundValueName(const std::string & aInput);
    void setLowerBoundVectorName(const std::string & aInput);

    std::string getUpperBoundVectorName() const;
    std::string getUpperBoundValueName() const;
    void setUpperBoundValueName(const std::string & aInput);
    void setUpperBoundVectorName(const std::string & aInput);

    std::vector<std::string> getDescentDirectionNames() const;
    std::string getDescentDirectionName(const size_t & aIndex) const;
    void addDescentDirectionName(const std::string & aInput);
    void setDescentDirectionNames(const std::vector<std::string> & aInput);

    std::string getAlgebra() const;
    void setAlgebra(const std::string & aInput);

    std::string getOutputStageName() const;
    void setOutputStageName(const std::string & aInput);

    std::string getCacheStageName() const;
    void setCacheStageName(const std::string & aInput);

    std::string  getObjectiveValueOutputName() const;
    void setObjectiveValueOutputName(const std::string & aInput);
    std::string  getObjectiveValueStageName() const;
    void setObjectiveValueStageName(const std::string & aInput);

    std::string getObjectiveHessianOutputName() const;
    void setObjectiveHessianOutputName(const std::string & aInput);
    std::string  getObjectiveHessianStageName() const;
    void setObjectiveHessianStageName(const std::string & aInput);

    std::string getObjectiveGradientOutputName() const;
    void setObjectiveGradientOutputName(const std::string & aInput);
    std::string getObjectiveGradientStageName() const;
    void setObjectiveGradientStageName(const std::string & aInput);

    std::string getInitializationStageName() const;
    void setInitializationStageName(const std::string & aInput);

    std::string getSetLowerBoundsStageName() const;
    void setSetLowerBoundsStageName(const std::string & aInput);

    std::string getSetUpperBoundsStageName() const;
    void setSetUpperBoundsStageName(const std::string & aInput);

    size_t getNumConstraints() const;

    std::vector<std::string> getConstraintValueNames() const;
    std::string getConstraintValueName(const size_t & aIndex) const;
    void addConstraintValueName(const std::string & aValueName);
    void setConstraintValueNames(const std::vector<std::string> & aInput);

    double getConstraintNormalizedTargetValue(const size_t & aIndex) const;
    double getConstraintNormalizedTargetValue(const std::string & aValueName) const;
    void addConstraintNormalizedTargetValue(const std::string & aValueName, const double & aInput);
    bool constraintNormalizedTargetValueWasSet(const std::string & aValueName) const;

    double getConstraintAbsoluteTargetValue(const size_t & aIndex) const;
    double getConstraintAbsoluteTargetValue(const std::string & aValueName) const;
    void addConstraintAbsoluteTargetValue(const std::string & aValueName, const double & aInput);
    bool constraintAbsoluteTargetValueWasSet(const std::string & aValueName) const;

    double getConstraintReferenceValue(const size_t & aIndex) const;
    double getConstraintReferenceValue(const std::string & aValueName) const;
    void addConstraintReferenceValue(const std::string & aValueName, const double & aInput);
    bool constraintReferenceValueWasSet(const std::string & aValueName) const;

    std::string getConstraintReferenceValueName(const size_t & aIndex) const;
    std::string getConstraintReferenceValueName(const std::string & aValueName) const;
    void addConstraintReferenceValueName(const std::string & aValueName, const std::string & aReferenceValueName);

    std::string getConstraintHessianName(const size_t & aIndex) const;
    std::string getConstraintHessianName(const std::string & aValueName) const;
    void addConstraintHessianName(const std::string & aValueName, const std::string & aHessianName);

    std::string getConstraintGradientName(const size_t & aIndex) const;
    std::string getConstraintGradientName(const std::string & aValueName) const;
    void addConstraintGradientName(const std::string & aValueName, const std::string & aGradientName);

    double getKSInitialRadiusScale() const;
    void setKSInitialRadiusScale(const double& aInput);
    double getKSMaxRadiusScale() const;
    void setKSMaxRadiusScale(const double& aInput);

private:
    bool mCheckGradient;
    bool mCheckHessian;
    bool mUserInitialGuess;
    bool mOutputControlToFile;
    bool mOutputDiagnosticsToFile;
    bool mHaveHessian;

    double mInitialMovingAsymptoteScaleFactor;
    double mGCMMAInnerKKTTolerance;
    double mCCSAOuterKKTTolerance;
    double mCCSAOuterControlStagnationTolerance;
    double mGCMMAInnerControlStagnationTolerance;
    double mCCSAOuterObjectiveStagnationTolerance;
    double mCCSAOuterStationarityTolerance;
    double mKSTrustRegionExpansionFactor;
    double mKSTrustRegionContractionFactor;
    double mKSOuterGradientTolerance;
    double mKSOuterStationarityTolerance;
    double mKSOuterStagnationTolerance;
    double mKSOuterControlStagnationTolerance;
    double mKSOuterActualReductionTolerance;
    double mKSInitialRadiusScale;
    double mKSMaxRadiusScale;

    size_t mMaxNumIterations;

    int mDerivativeCheckerFinalSuperscript;
    int mDerivativeCheckerInitialSuperscript;
    int mGCMMAMaxInnerIterations;
    int mKSMaxTrustRegionIterations;

    std::string mAlgebra;
    std::string mStateName;
    std::string mInputFileName;
    std::string mCacheStageName;
    std::string mOutputStageName;
    std::string mObjectiveValueOutputName;
    std::string mObjectiveHessianOutputName;
    std::string mObjectiveGradientOutputName;
    std::string mInitializationStageName;
    std::string mSetLowerBoundsStageName;
    std::string mSetUpperBoundsStageName;
    std::string mLowerBoundValueName;
    std::string mLowerBoundVectorName;
    std::string mUpperBoundValueName;
    std::string mUpperBoundVectorName;
    std::string mObjectiveValueStageName;
    std::string mObjectiveHessianStageName;
    std::string mObjectiveGradientStageName;

    std::vector<double> mInitialGuess;
    std::vector<double> mLowerBoundValues;
    std::vector<double> mUpperBoundValues;
    std::map<std::string, double> mConstraintNormalizedTargetValues;
    std::map<std::string, double> mConstraintAbsoluteTargetValues;
    std::map<std::string, double> mConstraintReferenceValues;

    std::vector<std::string> mControlNames;
    std::vector<std::string> mConstraintValueNames;
    std::vector<std::string> mDescentDirectionNames;
    std::map<std::string, std::string> mConstraintHessianNames;
    std::map<std::string, std::string> mConstraintGradientNames;
    std::map<std::string, std::string> mConstraintReferenceValueNames;

    // USING DEFAULT COPY AND ASSIGNMENT CONSTRUCTORS
};

} // namespace Plato

#endif /* PLATO_OPTIMIZERENGINESTAGEDATA_HPP_ */
