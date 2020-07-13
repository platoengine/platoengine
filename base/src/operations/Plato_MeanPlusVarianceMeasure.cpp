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
 * Plato_MeanPlusVarianceMeasure.cpp
 *
 *  Created on: Jul 5, 2019
 */

#include "PlatoApp.hpp"
#include "Plato_Macros.hpp"
#include "Plato_InputData.hpp"
#include "Plato_OperationsUtilities.hpp"
#include "Plato_MeanPlusVarianceMeasure.hpp"
#include "Plato_StatisticsOperationsUtilities.hpp"

namespace Plato
{

MeanPlusVarianceMeasure::MeanPlusVarianceMeasure(PlatoApp *aPlatoApp,
                                                 const Plato::InputData &aOperationNode) :
    Plato::LocalOp(aPlatoApp),
    mDataLayout(Plato::data::UNDEFINED)
{
    this->initialize(aOperationNode);
}

MeanPlusVarianceMeasure::~MeanPlusVarianceMeasure()
{
}

void MeanPlusVarianceMeasure::operator()()
{
    this->computeMeanQoI();
    bool tStandardDeviationMeasuresRequested = mOutArgumentToStdDevMultiplier.size() > 0u;
    if (tStandardDeviationMeasuresRequested == true)
    {
        this->computeStandardDeviationQoI();
        this->computeMeanPlusStdDev();
    }
}

void MeanPlusVarianceMeasure::getArguments(std::vector<LocalArg> &aLocalArgs)
{
    aLocalArgs = mLocalArguments;
}

std::string MeanPlusVarianceMeasure::getOperationName() const
{
    return (mOperationName);
}

std::string MeanPlusVarianceMeasure::getFunctionIdentifier() const
{
    return (mFunctionIdentifier);
}

Plato::data::layout_t MeanPlusVarianceMeasure::getDataLayout() const
{
    return (mDataLayout);
}

std::vector<double> MeanPlusVarianceMeasure::getStandardDeviationMultipliers() const
{
    std::vector<double> tOuput;
    for (auto tIterator = mOutArgumentToStdDevMultiplier.begin(); tIterator != mOutArgumentToStdDevMultiplier.end(); ++tIterator)
    {
        tOuput.push_back(tIterator->second);
    }
    std::sort(tOuput.begin(), tOuput.end());
    return (tOuput);
}

size_t MeanPlusVarianceMeasure::getLocalLength(const std::string &aArgumentName) const
{
    size_t tLenght = 0;
    if (mDataLayout == Plato::data::ELEMENT_FIELD)
    {
        tLenght = mPlatoApp->getLocalNumElements();
    }
    else if (mDataLayout == Plato::data::SCALAR_FIELD)
    {
        tLenght = mPlatoApp->getNodeFieldLength(aArgumentName);
    }
    return (tLenght);
}

double MeanPlusVarianceMeasure::getProbability(const std::string &aInput) const
{
    auto tIterator = mSampleArgumentNameToProbability.find(aInput);
    if (tIterator != mSampleArgumentNameToProbability.end())
    {
        return (tIterator->second);
    }
    else
    {
        const std::string tError = std::string("INPUT ARGUMENT NAME = ") + aInput + " IS NOT DEFINED.\n";
        THROWERR(tError);
    }
}

std::string MeanPlusVarianceMeasure::getOutputArgument(const std::string &aStatisticsMeasure) const
{
    auto tIterator = mStatisticsToOutArgument.find(aStatisticsMeasure);
    if (tIterator != mStatisticsToOutArgument.end())
    {
        return (tIterator->second);
    }
    else
    {
        std::string tError = std::string("INPUT STATISTIC'S MEASURE NAME = ") + aStatisticsMeasure + " IS NOT DEFINED.\n";
        THROWERR(tError);
    }
}

void MeanPlusVarianceMeasure::initialize(const Plato::InputData &aOperationNode)
{
    this->parseName(aOperationNode);
    this->parseFunction(aOperationNode);
    this->parseDataLayout(aOperationNode);
    this->parseInputs(aOperationNode);
    this->parseOutputs(aOperationNode);
}

void MeanPlusVarianceMeasure::parseName(const Plato::InputData &aOperationNode)
{
    mOperationName = Plato::Get::String(aOperationNode, "Name");
    if (mOperationName.empty() == true)
    {
        const std::string tOperationName = Plato::Get::String(aOperationNode, "Name");
        THROWERR("USER DEFINED FUNCTION NAME IS NOT.\n");
    }
}

void MeanPlusVarianceMeasure::parseFunction(const Plato::InputData &aOperationNode)
{
    mFunctionIdentifier = Plato::Get::String(aOperationNode, "Function");
    if (mFunctionIdentifier.empty() == true)
    {
        const std::string tError = std::string("FUNCTION IS NOT DEFINED IN OPERATION = ") + mOperationName + ".\n";
        THROWERR(tError);
    }
}

void MeanPlusVarianceMeasure::parseDataLayout(const Plato::InputData &aOperationNode)
{
    const std::string tLayout = Plato::Get::String(aOperationNode, "Layout", true);
    if (tLayout.empty() == true)
    {
        const std::string tOperationName = Plato::Get::String(aOperationNode, "Name");
        const std::string tError = std::string("DATA LAYOUT IS NOT DEFINED IN OPERATION = ") + tOperationName + ".\n";
        THROWERR(tError);
    }
    mDataLayout = Plato::getLayout(tLayout);
}

void MeanPlusVarianceMeasure::parseInputs(const Plato::InputData &aOperationNode)
{
    for (auto& tInputNode : aOperationNode.getByName<Plato::InputData>("Input"))
    {
        std::string tInputArgumentName = Plato::Get::String(tInputNode, "ArgumentName");
        this->setMyLocalArgument(tInputArgumentName);
        double tProbability = this->getMyProbability(tInputNode);
        mSampleArgumentNameToProbability[tInputArgumentName] = tProbability;
    }
}

void MeanPlusVarianceMeasure::parseOutputs(const Plato::InputData &aOperationNode)
{
    for (auto& tOutputNode : aOperationNode.getByName<Plato::InputData>("Output"))
    {
        std::string tStatisticMeasure = Plato::Get::String(tOutputNode, "Statistic", true);
        std::string tOutputArgumentName = Plato::Get::String(tOutputNode, "ArgumentName");
        this->setStatisticsToOutputArgumentNameMap(tStatisticMeasure, tOutputArgumentName);
        this->setMySigmaValue(tStatisticMeasure);
        this->setMyLocalArgument(tOutputArgumentName);
    }
}

void MeanPlusVarianceMeasure::setMySigmaValue(const std::string &aStatisticMeasure)
{
    if (aStatisticMeasure.empty() == true)
    {
        THROWERR("OUTPUT ARGUMENT NAME IS EMPTY, I.E. ARGUMENT'S NAME IS NOT DEFINED.\n");
    }

    std::vector<std::string> tTokens;
    Plato::split(aStatisticMeasure, tTokens);
    if (tTokens.size() > 2u)
    {
        const double tMySigmaValue = this->getMySigmaValue(tTokens[2]);
        const std::string &tOutputArgumentName = mStatisticsToOutArgument.find(aStatisticMeasure)->second;
        mOutArgumentToStdDevMultiplier[tOutputArgumentName] = tMySigmaValue;
    }
}

double MeanPlusVarianceMeasure::getMySigmaValue(const std::string &aInput)
{
    try
    {
        double tMySigmaValue = std::stod(aInput);
        return (tMySigmaValue);
    }
    catch (const std::invalid_argument &tErrorMsg)
    {
        THROWERR(tErrorMsg.what())
    }
}

void MeanPlusVarianceMeasure::setStatisticsToOutputArgumentNameMap(const std::string &aStatisticMeasure,
                                                                   const std::string &aOutputArgumentName)
{
    if (aStatisticMeasure.empty() == true)
    {
        THROWERR("STATISTIC KEYWORD IS EMPTY, I.E. STATISTIC MEASURE IS NOT DEFINED.\n");
    }
    if (aOutputArgumentName.empty() == true)
    {
        THROWERR("OUTPUT ARGUMENT NAME KEYWORD IS EMPTY, I.E. OUTPUT ARGUMENT NAME IS NOT DEFINED.\n");
    }
    mStatisticsToOutArgument[aStatisticMeasure] = aOutputArgumentName;
}

double MeanPlusVarianceMeasure::getMyProbability(const Plato::InputData &aInputNode)
{
    const double tProbability = Plato::Get::Double(aInputNode, "Probability");
    if (tProbability <= 0.0)
    {
        const std::string tArgumentName = Plato::Get::String(aInputNode, "ArgumentName");
        const std::string tError = std::string("INVALID PROBABILITY SPECIFIED FOR INPUT ARGUMENT = ") + tArgumentName + ". " + "INPUT PROBABILITY WAS SET TO "
            + std::to_string(tProbability) + ". INPUT PROBABILITY SHOULD BE A POSITIVE NUMBER (I.E. GREATER THAN ZERO).\n";
        THROWERR(tError);
    }
    return (tProbability);
}

void MeanPlusVarianceMeasure::setMyLocalArgument(const std::string &aArgumentName)
{
    if (aArgumentName.empty() == true)
    {
        THROWERR("INPUT ARGUMENT NAME IS EMPTY, I.E. ARGUMENT'S NAME IS NOT DEFINED.\n");
    }
    if (mDataLayout == Plato::data::SCALAR)
    {
        mLocalArguments.push_back(Plato::LocalArg { mDataLayout, aArgumentName, 1 /* number of scalar values, i.e. array length */});
    }
    else
    {
        mLocalArguments.push_back(Plato::LocalArg { mDataLayout, aArgumentName });
    }
}

void MeanPlusVarianceMeasure::computeMeanQoI()
{
    if (mDataLayout == Plato::data::SCALAR)
    {
        this->computeMeanGlobalQoI();
    }
    else if (mDataLayout == Plato::data::SCALAR_FIELD)
    {
        this->computeMeanNodeFieldQoI();
    }
    else if (mDataLayout == Plato::data::ELEMENT_FIELD)
    {
        this->computeMeanElementFieldQoI();
    }
    else
    {
        const std::string tParsedLayout = Plato::getLayout(mDataLayout);
        const std::string tError = std::string("MEAN CAN ONLY BE COMPUTED FOR NODAL FIELDS, ELEMENT FIELDS AND SCALAR VALUES QoIs.")
            + " INVALID INPUT DATA LAYOUT = " + tParsedLayout + ".\n";
        THROWERR(tError);
    }
}

double* MeanPlusVarianceMeasure::getOutputDataFieldQoI(const std::string &aStatisticMeasure)

{
    auto tIterator = mStatisticsToOutArgument.find(aStatisticMeasure);
    if (tIterator == mStatisticsToOutArgument.end())
    {
        THROWERR("UNDEFINED STATISTIC MEASURE " + aStatisticMeasure + ".\n")
    }

    const std::string &tOutputArgumentName = tIterator->second;
    if (mDataLayout == Plato::data::ELEMENT_FIELD)
    {
        return (mPlatoApp->getElementFieldData(tOutputArgumentName));
    }
    else
    {
        return (mPlatoApp->getNodeFieldData(tOutputArgumentName));
    }
}

std::vector<double>* MeanPlusVarianceMeasure::getOutputDataGlobalQoI(const std::string &aStatisticMeasure)

{
    auto tIterator = mStatisticsToOutArgument.find(aStatisticMeasure);
    if (tIterator == mStatisticsToOutArgument.end())
    {
        THROWERR("UNDEFINED STATISTIC MEASURE " + aStatisticMeasure + ".\n")
    }
    const std::string &tOutputArgumentMean = tIterator->second;
    return (mPlatoApp->getValue(tOutputArgumentMean));
}

void MeanPlusVarianceMeasure::setSampleProbabilityPairsNodeQoI(std::vector<Plato::SampleProbPair<double*, double>> &aPairs)
{
    for (auto tItr = mSampleArgumentNameToProbability.begin(); tItr != mSampleArgumentNameToProbability.end(); ++tItr)
    {
        aPairs.push_back(Plato::SampleProbPair<double*, double> { });
        aPairs.rbegin()->mProbability = tItr->second;
        aPairs.rbegin()->mLength = mPlatoApp->getNodeFieldLength(tItr->first);
        aPairs.rbegin()->mSample = mPlatoApp->getNodeFieldData(tItr->first);
    }
}

void MeanPlusVarianceMeasure::setSampleProbabilityPairsElementQoI(std::vector<Plato::SampleProbPair<double*, double>> &aPairs)
{
    for (auto tItr = mSampleArgumentNameToProbability.begin(); tItr != mSampleArgumentNameToProbability.end(); ++tItr)
    {
        aPairs.push_back(Plato::SampleProbPair<double*, double> { });
        aPairs.rbegin()->mProbability = tItr->second;
        aPairs.rbegin()->mLength = mPlatoApp->getLocalNumElements();
        aPairs.rbegin()->mSample = mPlatoApp->getElementFieldData(tItr->first);
    }
}

void MeanPlusVarianceMeasure::setSampleProbabilityPairsGlobalQoI(std::vector<Plato::SampleProbPair<double, double>> &aPairs)
{
    for (auto tItr = mSampleArgumentNameToProbability.begin(); tItr != mSampleArgumentNameToProbability.end(); ++tItr)
    {
        aPairs.push_back(Plato::SampleProbPair<double, double> { });
        aPairs.rbegin()->mProbability = tItr->second;
        aPairs.rbegin()->mLength = mPlatoApp->getValue(tItr->first)->size();
        aPairs.rbegin()->mSample = mPlatoApp->getValue(tItr->first)->operator [](0);
    }
}

void MeanPlusVarianceMeasure::computeMeanGlobalQoI()
{
    std::vector<Plato::SampleProbPair<double, double>> tSampleDataToProbabilityPairs;
    this->setSampleProbabilityPairsGlobalQoI(tSampleDataToProbabilityPairs);
    std::vector<double> *tOutput = this->getOutputDataGlobalQoI("MEAN");
    (*tOutput)[0] = Plato::compute_sample_set_mean(tSampleDataToProbabilityPairs);
}

void MeanPlusVarianceMeasure::computeMeanNodeFieldQoI()
{
    std::vector<Plato::SampleProbPair<double*, double>> tSampleDataToProbabilityPairs;
    this->setSampleProbabilityPairsNodeQoI(tSampleDataToProbabilityPairs);
    double *tOutput = this->getOutputDataFieldQoI("MEAN");
    Plato::compute_sample_set_mean(tSampleDataToProbabilityPairs, tOutput);
}

void MeanPlusVarianceMeasure::computeMeanElementFieldQoI()
{
    std::vector<Plato::SampleProbPair<double*, double>> tSampleDataToProbabilityPairs;
    this->setSampleProbabilityPairsElementQoI(tSampleDataToProbabilityPairs);
    double *tOutput = this->getOutputDataFieldQoI("MEAN");
    Plato::compute_sample_set_mean(tSampleDataToProbabilityPairs, tOutput);
}

void MeanPlusVarianceMeasure::computeStdDevGlobalQoI()
{
    std::vector<Plato::SampleProbPair<double, double>> tSampleDataToProbabilityPairs;
    this->setSampleProbabilityPairsGlobalQoI(tSampleDataToProbabilityPairs);
    std::vector<double> *tMean = this->getOutputDataGlobalQoI("MEAN");
    std::vector<double> *tStdDev = this->getOutputDataGlobalQoI("STD_DEV");
    (*tStdDev)[0] = Plato::compute_sample_set_standard_deviation((*tMean)[0], tSampleDataToProbabilityPairs);
}

void MeanPlusVarianceMeasure::computeStdDevNodeFieldQoI()
{
    std::vector<Plato::SampleProbPair<double*, double>> tSampleDataToProbabilityPairs;
    this->setSampleProbabilityPairsNodeQoI(tSampleDataToProbabilityPairs);
    double *tMean = this->getOutputDataFieldQoI("MEAN");
    double *tStdDev = this->getOutputDataFieldQoI("STD_DEV");
    Plato::compute_sample_set_standard_deviation(tMean, tSampleDataToProbabilityPairs, tStdDev);
}

void MeanPlusVarianceMeasure::computeStdDevElementFieldQoI()
{
    std::vector<Plato::SampleProbPair<double*, double>> tSampleDataToProbabilityPairs;
    this->setSampleProbabilityPairsElementQoI(tSampleDataToProbabilityPairs);
    double *tMean = this->getOutputDataFieldQoI("MEAN");
    double *tStdDev = this->getOutputDataFieldQoI("STD_DEV");
    Plato::compute_sample_set_standard_deviation(tMean, tSampleDataToProbabilityPairs, tStdDev);
}

void MeanPlusVarianceMeasure::computeStandardDeviationQoI()
{
    if (mDataLayout == Plato::data::SCALAR)
    {
        this->computeStdDevGlobalQoI();
    }
    else if (mDataLayout == Plato::data::SCALAR_FIELD)
    {
        this->computeStdDevNodeFieldQoI();
    }
    else if (mDataLayout == Plato::data::ELEMENT_FIELD)
    {
        this->computeStdDevElementFieldQoI();
    }
    else
    {
        const std::string tParsedLayout = Plato::getLayout(mDataLayout);
        const std::string tError = std::string("STANDARD DEVIATION CAN ONLY BE COMPUTED FOR NODAL FIELDS, ELEMENT FIELDS AND SCALAR VALUES QoIs.")
            + " INVALID INPUT DATA LAYOUT = " + tParsedLayout + ".\n";
        THROWERR(tError);
    }
}

void MeanPlusVarianceMeasure::computeMeanPlusStdDev()
{
    if (mDataLayout == Plato::data::ELEMENT_FIELD)
    {
        this->computeMeanPlusStdDevFieldQoI();
    }
    else if (mDataLayout == Plato::data::SCALAR_FIELD)
    {
        this->computeMeanPlusStdDevFieldQoI();
    }
    else if (mDataLayout == Plato::data::SCALAR)
    {
        this->computeMeanPlusStdDevGlobalQoI();
    }
    else
    {
        const std::string tParsedLayout = Plato::getLayout(mDataLayout);
        const std::string tError = std::string("MEAN PLUS STANDARD DEVIATION MEASURE CAN ONLY BE COMPUTED FOR ")
            + "NODAL FIELDS, ELEMENT FIELDS AND SCALAR VALUES QoIs. " + "INVALID INPUT DATA LAYOUT = " + tParsedLayout + ".\n";
        THROWERR(tError);
    }
}

void MeanPlusVarianceMeasure::computeMeanPlusStdDevGlobalQoI()
{
    std::vector<double> *tOutputMeanData = this->getOutputDataGlobalQoI("MEAN");
    std::vector<double> *tOutputSigmaData = this->getOutputDataGlobalQoI("STD_DEV");

    // tIterator->first = statistic's measure argument name AND tIterator->second = standard deviation multiplier
    for (auto tOuterIterator = mOutArgumentToStdDevMultiplier.begin(); 
         tOuterIterator != mOutArgumentToStdDevMultiplier.end(); ++tOuterIterator)
    {
        const std::string &tOutputArgumentMeanPlusSigma = tOuterIterator->first;
        std::vector<double> *tOutMeanPlusSigmaData = mPlatoApp->getValue(tOutputArgumentMeanPlusSigma);
/*
        (*tOutMeanPlusSigmaData)[0] = (*tOutputMeanData)[0] + (tOuterIterator->second * (*tOutputSigmaData)[0]); // tIterator->second = sigma multiplier
*/
        (*tOutMeanPlusSigmaData)[0] = (*tOutputMeanData)[0];
    }
}

void MeanPlusVarianceMeasure::computeMeanPlusStdDevFieldQoI()
{
    const auto tArgumentName = this->getOutputArgument("MEAN");
    const size_t tLocalLength = this->getLocalLength(tArgumentName);
    double *tOutputMeanData = this->getOutputDataFieldQoI("MEAN");
    double *tOutputSigmaData = this->getOutputDataFieldQoI("STD_DEV");

    // tIterator->first = statistic's measure argument name AND tIterator->second = standard deviation multiplier
    for (auto tOuterIterator = mOutArgumentToStdDevMultiplier.begin(); 
         tOuterIterator != mOutArgumentToStdDevMultiplier.end(); ++tOuterIterator)
    {
        const auto &tOutputArgumentMeanPlusSigma = tOuterIterator->first;
        double *tOutMeanPlusSigmaData = mPlatoApp->getNodeFieldData(tOutputArgumentMeanPlusSigma);
        for (size_t tIndex = 0; tIndex < tLocalLength; tIndex++)
        {
            tOutMeanPlusSigmaData[tIndex] = tOutputMeanData[tIndex] + (tOuterIterator->second * tOutputSigmaData[tIndex]);
        }
    }
}

}
// namespace Plato
