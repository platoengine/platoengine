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
 * Plato_MeanPlusVarianceGradient.cpp
 *
 *  Created on: Jul 10, 2019
 */

#include "PlatoApp.hpp"
#include "Plato_Macros.hpp"
#include "Plato_InputData.hpp"
#include "Plato_OperationsUtilities.hpp"
#include "Plato_MeanPlusVarianceGradient.hpp"
#include "Plato_StatisticsOperationsUtilities.hpp"

namespace Plato
{

MeanPlusVarianceGradient::MeanPlusVarianceGradient(PlatoApp* aPlatoApp, const Plato::InputData& aOperationNode) :
        Plato::LocalOp(aPlatoApp),
        mStdDevMultiplier(-1),
        mCriterionValueDataLayout(Plato::data::UNDEFINED),
        mCriterionGradientDataLayout(Plato::data::UNDEFINED),
        mCriterionValueSamplesToProbability(),
        mCriterionGradSamplesToProbability()
{
    this->initialize(aOperationNode);
}

MeanPlusVarianceGradient::~MeanPlusVarianceGradient()
{
}

void MeanPlusVarianceGradient::operator()()
{
    this->setCriterionGradientSampleProbabilityPairs();

    bool tStandardDeviationMultiplierNotDefined = mStdDevMultiplier <= 0.0;
    if(tStandardDeviationMultiplierNotDefined == true)
    {
        this->computeMeanCriterionGradientSampleSet();
    }
    else
    {
        this->setCriterionValueSampleProbabilityPairs();
        this->computeMeanAndStdDevCriterionValueSampleSet();
        this->computeGradientMeanPlusStandardDeviationCriterion();
    }
}

void MeanPlusVarianceGradient::getArguments(std::vector<LocalArg>& aLocalArgs)
{
    aLocalArgs = mLocalArguments;
}

std::string MeanPlusVarianceGradient::getOperationName() const
{
    return (mOperationName);
}

std::string MeanPlusVarianceGradient::getFunctionIdentifier() const
{
    return (mFunctionIdentifier);
}

Plato::data::layout_t MeanPlusVarianceGradient::getCriterionValueDataLayout() const
{
    return (mCriterionValueDataLayout);
}

Plato::data::layout_t MeanPlusVarianceGradient::getCriterionGradientDataLayout() const
{
    return (mCriterionGradientDataLayout);
}

double MeanPlusVarianceGradient::getStandardDeviationMultipliers() const
{
    return (mStdDevMultiplier);
}

double MeanPlusVarianceGradient::getCriterionValueProbability(const std::string& aInput) const
{
    auto tIterator = mCriterionValueSamplesArgNameToProbability.find(aInput);
    if(tIterator != mCriterionValueSamplesArgNameToProbability.end())
    {
        return (tIterator->second);
    }
    else
    {
        const std::string tError = std::string("INPUT ARGUMENT NAME = ") + aInput + " IS NOT DEFINED.\n";
        THROWERR(tError)
    }
}

double MeanPlusVarianceGradient::getCriterionGradientProbability(const std::string& aInput) const
{
    auto tIterator = mCriterionGradSamplesArgNameToProbability.find(aInput);
    if(tIterator != mCriterionGradSamplesArgNameToProbability.end())
    {
        return (tIterator->second);
    }
    else
    {
        THROWERR("INPUT ARGUMENT NAME = " + aInput + " IS NOT DEFINED.\n")
    }
}

std::string MeanPlusVarianceGradient::getCriterionValueOutputArgument(const std::string& aInput) const
{
    auto tIterator = mCriterionValueStatisticsToOutputName.find(aInput);
    if(tIterator != mCriterionValueStatisticsToOutputName.end())
    {
        return (tIterator->second);
    }
    else
    {
        THROWERR("INPUT STATISTIC'S MEASURE NAME = " + aInput + " IS NOT DEFINED.\n")
    }
}

std::string MeanPlusVarianceGradient::getCriterionGradientOutputArgument(const std::string& aStatisticMeasure) const
{
    auto tIterator = mCriterionGradientStatisticsToOutputName.find(aStatisticMeasure);
    if(tIterator != mCriterionGradientStatisticsToOutputName.end())
    {
        return (tIterator->second);
    }
    else
    {
        THROWERR("INPUT STATISTIC'S MEASURE NAME = " + aStatisticMeasure + " IS NOT DEFINED.\n")
    }
}

void MeanPlusVarianceGradient::initialize(const Plato::InputData& aOperationNode)
{
    this->parseName(aOperationNode);
    this->parseFunction(aOperationNode);
    this->parseCriterionGradientArguments(aOperationNode);
    bool tStandardDeviationMultiplierIsDefined = mStdDevMultiplier > 0.0;
    if(tStandardDeviationMultiplierIsDefined == true)
    {
        this->parseCriterionValueArguments(aOperationNode);
        this->checkInputProbabilityValues();
    }
}

void MeanPlusVarianceGradient::checkInputProbabilityValues()
{
    if(mCriterionValueSamplesArgNameToProbability.size() != mCriterionGradSamplesArgNameToProbability.size())
    {
        const std::string tError = std::string("THE NUMBER OF CRITERION VALUE AND GRADIENT SAMPLES DON'T MATCH. ") +
                "THE NUMBER OF CRITERION VALUE SAMPLES IS " + std::to_string(mCriterionValueSamplesArgNameToProbability.size()) +
                " AND THE NUMBER OF CRITERION GRADIENT SAMPLES IS " + std::to_string(mCriterionGradSamplesArgNameToProbability.size()) + ".\n";
        THROWERR(tError)
    }

    auto tGradIterator = mCriterionGradSamplesArgNameToProbability.begin();
    for(auto tValIterator = mCriterionValueSamplesArgNameToProbability.begin(); tValIterator != mCriterionValueSamplesArgNameToProbability.end(); ++tValIterator)
    {
        auto tDiff = std::abs(tValIterator->second - tGradIterator->second);
        if(tDiff > std::double_t(1e-16))
        {
            const std::string tError = std::string("THE PROBABILITY ASSOCIATED WITH THE CRITERION VALUE SAMPLE, ") +
                    tValIterator->first + ", AND THE CRITERION GRADIENT SAMPLE, " + tGradIterator->first +
                    ", DON'T MATCH. " + "THE PROBAILITY FOR THE CRITERION VALUE SAMPLE IS " +
                    std::to_string(tValIterator->second) + " AND THE PROBABILITY FOR THE CRITERION GRADIENT SAMPLE IS " +
                    std::to_string(tGradIterator->second) + ". THESE TWO PROBABILITIES ARE EXPECTED TO HAVE THE SAME VALUE.\n";
            THROWERR(tError)
        }
        ++tGradIterator;
    }
}

void MeanPlusVarianceGradient::parseName(const Plato::InputData& aOperationNode)
{
    mOperationName = Plato::Get::String(aOperationNode, "Name");
    if(mOperationName.empty() == true)
    {
        mOperationName = std::string("Stochastic Criterion Gradient");
    }
}

void MeanPlusVarianceGradient::parseFunction(const Plato::InputData& aOperationNode)
{
    mFunctionIdentifier = Plato::Get::String(aOperationNode, "Function");
    if(mFunctionIdentifier.empty() == true)
    {
        const std::string tError = std::string("FUNCTION KEYWORD IS NOT DEFINED IN OPERATION = ") + mOperationName +
                ". PLATO EXPECTS THE FOLLOWING FUNCTION NAME, MeanPlusStdDevGradient, AS INPUT.\n";
        THROWERR(tError)
    }
}

Plato::data::layout_t MeanPlusVarianceGradient::parseDataLayout(const Plato::InputData& aOperationNode,
                                                                const std::string& aCriteriaName)
{
    const std::string tLayout = Plato::Get::String(aOperationNode, "Layout", true);
    if(tLayout.empty() == true)
    {
        const std::string tError = std::string("DATA LAYOUT IS NOT DEFINED IN OPERATION = ") +
                mOperationName + ". CHECK DATA LAYOUT INPUT IN " + aCriteriaName + ".\n";
        THROWERR(tError)
    }
    return(Plato::getLayout(tLayout));
}

void MeanPlusVarianceGradient::setCriterionValueSampleProbabilityPairs()
{
    if(mCriterionValueSamplesArgNameToProbability.size() != mCriterionValueSamplesToProbability.size())
    {
        THROWERR("EXPECTED THE CRITERIA VALUE SAMPLE-PROBABILITY SET AND SAMPLE-PROBABILITY MAP TO HAVE THE SAME SIZE.\n")
    }

    auto tVectorIterator = mCriterionValueSamplesToProbability.begin();
    // tIterator->first = Argument name & tIterator->second = Probability
    for(auto tIterator = mCriterionValueSamplesArgNameToProbability.begin();
            tIterator != mCriterionValueSamplesArgNameToProbability.end(); ++tIterator)
    {
        std::vector<double>* tInputValue = mPlatoApp->getValue(tIterator->first);
        tVectorIterator->mSample = (*tInputValue)[0];
        tVectorIterator->mProbability = tIterator->second;
    }
}

void MeanPlusVarianceGradient::setCriterionGradientSampleProbabilityPairs()
{
    if(mCriterionGradSamplesArgNameToProbability.size() != mCriterionGradSamplesToProbability.size())
    {
        THROWERR("EXPECTED THE CRITERIA GRADIENT SAMPLE-PROBABILITY SET AND SAMPLE-PROBABILITY MAP TO HAVE THE SAME SIZE.\n")
    }

    if(mCriterionGradientDataLayout == Plato::data::ELEMENT_FIELD)
    {
        this->setElementFieldGradientSampleProbabilityPairs();
    }
    else
    {
        this->setNodeFieldGradientSampleProbabilityPairs();
    }
}

void MeanPlusVarianceGradient::setNodeFieldGradientSampleProbabilityPairs()
{
    // tIterator->first = sample's argument name & tIterator->second = probability
    auto tVectorIterator = mCriterionGradSamplesToProbability.begin();
    for(auto tIterator = mCriterionGradSamplesArgNameToProbability.begin();
            tIterator != mCriterionGradSamplesArgNameToProbability.end(); ++tIterator)
    {
        tVectorIterator->mProbability = tIterator->second;
        tVectorIterator->mLength = mPlatoApp->getNodeFieldLength(tIterator->first);
        tVectorIterator->mSample = mPlatoApp->getNodeFieldData(tIterator->first);
    }
}

void MeanPlusVarianceGradient::setElementFieldGradientSampleProbabilityPairs()
{
    // tIterator->first = sample's argument name & tIterator->second = probability
    auto tVectorIterator = mCriterionGradSamplesToProbability.begin();
    for(auto tIterator = mCriterionGradSamplesArgNameToProbability.begin();
            tIterator != mCriterionGradSamplesArgNameToProbability.end(); ++tIterator)
    {
        tVectorIterator->mProbability = tIterator->second;
        tVectorIterator->mLength = mPlatoApp->getLocalNumElements();
        tVectorIterator->mSample = mPlatoApp->getElementFieldData(tIterator->first);
    }
}

void MeanPlusVarianceGradient::reserveCriterionValueSampleProbabilityPairs()
{
    for(auto tIterator = mCriterionValueSamplesArgNameToProbability.begin();
            tIterator != mCriterionValueSamplesArgNameToProbability.end(); ++tIterator)
    {
        mCriterionValueSamplesToProbability.push_back(Plato::SampleProbPair<double, double>{});
    }
}

void MeanPlusVarianceGradient::reserveCriterionGradSampleProbabilityPairs()
{
    for(auto tIterator = mCriterionGradSamplesArgNameToProbability.begin();
            tIterator != mCriterionGradSamplesArgNameToProbability.end(); ++tIterator)
    {
        mCriterionGradSamplesToProbability.push_back(Plato::SampleProbPair<double*, double>{});
    }
}

void MeanPlusVarianceGradient::parseInputs(const Plato::data::layout_t& aDataLayout,
                                           const Plato::InputData& aInput,
                                           std::map<std::string, double> & aOutput)
{
    for(auto tInputNode : aInput.getByName<Plato::InputData>("Input"))
    {
        std::string tInputArgumentName = Plato::Get::String(tInputNode, "ArgumentName");
        this->addLocalArgument(aDataLayout, tInputArgumentName);
        double tProbability = this->getMyProbability(tInputNode);
        aOutput[tInputArgumentName] = tProbability;
    }
}

void MeanPlusVarianceGradient::parseCriterionValueArguments(const Plato::InputData& aOperationNode)
{
    for(auto tCriteriaNode : aOperationNode.getByName<Plato::InputData>("CriterionValue"))
    {
        mCriterionValueDataLayout = this->parseDataLayout(tCriteriaNode, "CriterionValue");
        this->parseInputs(mCriterionValueDataLayout, tCriteriaNode, mCriterionValueSamplesArgNameToProbability);
        this->parseCriterionValueOutputs(tCriteriaNode);
        this->reserveCriterionValueSampleProbabilityPairs();
    }
}

void MeanPlusVarianceGradient::parseCriterionGradientArguments(const Plato::InputData& aOperationNode)
{
    for(auto tCriteriaNode : aOperationNode.getByName<Plato::InputData>("CriterionGradient"))
    {
        mCriterionGradientDataLayout = this->parseDataLayout(tCriteriaNode, "CriterionGradient");
        this->parseInputs(mCriterionGradientDataLayout, tCriteriaNode, mCriterionGradSamplesArgNameToProbability);
        this->parseCriterionGradientOutputs(tCriteriaNode);
        this->reserveCriterionGradSampleProbabilityPairs();
    }
}

std::string MeanPlusVarianceGradient::getStatisticMeasure(const Plato::InputData& aOutputNode,
                                                          const std::string& aOutputArgumentName) const
{
    std::string tStatisticMeasure = Plato::Get::String(aOutputNode, "Statistic", true);
    if(tStatisticMeasure.empty() == true)
    {
        THROWERR("THE STATISTIC KEYWORD FOR OUTPUT ARGUMENT " + aOutputArgumentName + " IS EMPTY.\n");
    }
    return (tStatisticMeasure);
}

std::string MeanPlusVarianceGradient::getOutputArgument(const Plato::InputData& aOutputNode,
                                                        const std::string& aCriterionName) const
{
    std::string tOutputArgumentName = Plato::Get::String(aOutputNode, "ArgumentName");
    if(tOutputArgumentName.empty() == true)
    {
        THROWERR("DETECTED AN EMPTY OUTPUT ARGUMENT KEYWORD IN THE " + aCriterionName + " BLOCK.\n");
    }
    return (tOutputArgumentName);
}

void MeanPlusVarianceGradient::parseCriterionValueOutputs(const Plato::InputData& aCriteriaNode)
{
    for(auto tOutputNode : aCriteriaNode.getByName<Plato::InputData>("Output"))
    {
        const std::string tOutputArgumentName = this->getOutputArgument(tOutputNode, "CriterionValue");
        const std::string tStatisticMeasure = this->getStatisticMeasure(tOutputNode, tOutputArgumentName);
        mCriterionValueStatisticsToOutputName[tStatisticMeasure] = tOutputArgumentName;
        this->addLocalArgument(mCriterionValueDataLayout, tOutputArgumentName);
    }
}

void MeanPlusVarianceGradient::parseCriterionGradientOutputs(const Plato::InputData& aCriteriaNode)
{
    for(auto tOutputNode : aCriteriaNode.getByName<Plato::InputData>("Output"))
    {
        const std::string tOutputArgumentName = this->getOutputArgument(tOutputNode, "CriterionGradient");
        const std::string tStatisticMeasure = this->getStatisticMeasure(tOutputNode, tOutputArgumentName);
        mCriterionGradientStatisticsToOutputName[tStatisticMeasure] = tOutputArgumentName;
        this->setMyStandardDeviationMultiplier(tStatisticMeasure);
        this->addLocalArgument(mCriterionGradientDataLayout, tOutputArgumentName);
    }
}

void MeanPlusVarianceGradient::setMyStandardDeviationMultiplier(const std::string& aStatisticMeasure)
{
    std::vector<std::string> tStringList;
    Plato::split(aStatisticMeasure, tStringList);
    if(tStringList.size() > 2u)
    {
        mStdDevMultiplier = this->getMyStandardDeviationMultiplier(tStringList[2]);
        mOutputGradientArgumentName = mCriterionGradientStatisticsToOutputName.find(aStatisticMeasure)->second;
    }
}

double MeanPlusVarianceGradient::getMyStandardDeviationMultiplier(const std::string& aInput)
{
    try
    {
        double tMySigmaValue = std::stod(aInput);
        return (tMySigmaValue);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        THROWERR(tErrorMsg.what())
    }
}

double MeanPlusVarianceGradient::getMyProbability(const Plato::InputData& aInputNode)
{
    const double tProbability = Plato::Get::Double(aInputNode, "Probability");
    if(tProbability <= 0.0)
    {
        const std::string tArgumentName = Plato::Get::String(aInputNode, "ArgumentName");
        const std::string tError = std::string("INVALID PROBABILITY SPECIFIED FOR INPUT ARGUMENT = ")
                + tArgumentName + ". " + "INPUT PROBABILITY WAS SET TO " + std::to_string(tProbability)
                + " AND IT SHOULD BE A POSITIVE NUMBER (I.E. GREATER THAN ZERO).\n";
        THROWERR(tError)
    }
    return (tProbability);
}

void MeanPlusVarianceGradient::addLocalArgument(const Plato::data::layout_t& aDataLayout, const std::string & aArgumentName)
{
    if(aArgumentName.empty() == true)
    {
        THROWERR("INPUT ARGUMENT NAME IS EMPTY, I.E. ARGUMENT'S NAME IS NOT DEFINED.\n");
    }
    mLocalArguments.push_back(Plato::LocalArg{ aDataLayout, aArgumentName });
}

std::vector<double>* MeanPlusVarianceGradient::getCriterionValueOutputData(const std::string& aStatisticMeasure)

{
    auto tIterator = mCriterionValueStatisticsToOutputName.find(aStatisticMeasure);
    if(tIterator == mCriterionValueStatisticsToOutputName.end())
    {
        THROWERR("UNDEFINED STATISTIC MEASURE " + aStatisticMeasure + ".\n")
    }
    const std::string& tOutputArgumentMean = tIterator->second;
    return (mPlatoApp->getValue(tOutputArgumentMean));
}

double* MeanPlusVarianceGradient::getCriterionGradientOutputData(const std::string& aStatisticMeasure)

{
    auto tIterator = mCriterionGradientStatisticsToOutputName.find(aStatisticMeasure);
    if(tIterator == mCriterionGradientStatisticsToOutputName.end())
    {
        THROWERR("UNDEFINED STATISTIC MEASURE " + aStatisticMeasure + ".\n")
    }

    const std::string& tOutputArgumentName = tIterator->second;
    if(mCriterionGradientDataLayout == Plato::data::ELEMENT_FIELD)
    {
        return (mPlatoApp->getElementFieldData(tOutputArgumentName));
    }
    else
    {
        return (mPlatoApp->getNodeFieldData(tOutputArgumentName));
    }
}

void MeanPlusVarianceGradient::computeMeanCriterionGradientSampleSet()
{
    double* tOutputData = this->getCriterionGradientOutputData("MEAN");
    Plato::compute_sample_set_mean(mCriterionGradSamplesToProbability, tOutputData);
}

void MeanPlusVarianceGradient::computeMeanAndStdDevCriterionValueSampleSet()
{
    std::vector<double>* tMean = this->getCriterionValueOutputData("MEAN");
    (*tMean)[0] = Plato::compute_sample_set_mean(mCriterionValueSamplesToProbability);
    std::vector<double>* tStdDev = this->getCriterionValueOutputData("STD_DEV");
    (*tStdDev)[0] = Plato::compute_sample_set_standard_deviation((*tMean)[0], mCriterionValueSamplesToProbability);
}

void MeanPlusVarianceGradient::computeGradientMeanPlusStandardDeviationCriterion()
{
    if(mCriterionGradientDataLayout == Plato::data::ELEMENT_FIELD)
    {
        this->computeGradientMeanPlusStdDevCriterionForElementField();
    }
    else if(mCriterionGradientDataLayout == Plato::data::SCALAR_FIELD)
    {
        this->computeGradientMeanPlusStdDevCriterionForNodeField();
    }
    else
    {
        const std::string tParsedLayout = Plato::getLayout(mCriterionGradientDataLayout);
        const std::string tError = std::string("MEAN PLUS STANDARD DEVIATION CRITERION GRADIENT CAN ONLY BE COMPUTED FOR ")
                + "NODAL AND ELEMENT FIELD QoIs. " + "INVALID INPUT DATA LAYOUT = " + tParsedLayout + ".\n";
        THROWERR(tError)
    }
}

void MeanPlusVarianceGradient::computeGradientMeanPlusStdDevCriterionForNodeField()
{
    std::vector<double>* tCriterionMean = this->getCriterionValueOutputData("MEAN");
    std::vector<double>* tCriterionStdDev = this->getCriterionValueOutputData("STD_DEV");
    double* tOutputGradient = mPlatoApp->getNodeFieldData(mOutputGradientArgumentName);
    Plato::compute_sample_set_mean_plus_std_dev_gradient((*tCriterionMean)[0],
                                                         (*tCriterionStdDev)[0],
                                                         mStdDevMultiplier,
                                                         mCriterionValueSamplesToProbability,
                                                         mCriterionGradSamplesToProbability,
                                                         tOutputGradient);
}

void MeanPlusVarianceGradient::computeGradientMeanPlusStdDevCriterionForElementField()
{
    std::vector<double>* tCriterionMean = this->getCriterionValueOutputData("MEAN");
    std::vector<double>* tCriterionStdDev = this->getCriterionValueOutputData("STD_DEV");
    double* tOutputGradient = mPlatoApp->getElementFieldData(mOutputGradientArgumentName);
    Plato::compute_sample_set_mean_plus_std_dev_gradient((*tCriterionMean)[0],
                                                         (*tCriterionStdDev)[0],
                                                         mStdDevMultiplier,
                                                         mCriterionValueSamplesToProbability,
                                                         mCriterionGradSamplesToProbability,
                                                         tOutputGradient);
}

}
// namespace Plato

