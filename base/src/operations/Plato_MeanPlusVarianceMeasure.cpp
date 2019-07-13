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

MeanPlusVarianceMeasure::MeanPlusVarianceMeasure(PlatoApp* aPlatoApp, const Plato::InputData& aOperationNode) :
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
    this->computeMean();
    bool tStandardDeviationMeasuresRequested = mOutArgumentToStdDevMultiplier.size() > 0u;
    if(tStandardDeviationMeasuresRequested == true)
    {
        this->computeStandardDeviation();
        this->computeMeanPlusStdDev();
    }
}

void MeanPlusVarianceMeasure::getArguments(std::vector<LocalArg>& aLocalArgs)
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
    for(auto tIterator = mOutArgumentToStdDevMultiplier.begin(); tIterator != mOutArgumentToStdDevMultiplier.end(); ++tIterator)
    {
        tOuput.push_back(tIterator->second);
    }
    std::sort(tOuput.begin(), tOuput.end());
    return (tOuput);
}

double MeanPlusVarianceMeasure::getProbability(const std::string& aInput) const
{
    auto tIterator = mSampleArgumentNameToProbability.find(aInput);
    if(tIterator != mSampleArgumentNameToProbability.end())
    {
        return(tIterator->second);
    }
    else
    {
        const std::string tError = std::string("INPUT ARGUMENT NAME = ") + aInput + " IS NOT DEFINED.\n";
        THROWERR(tError);
    }
}

std::string MeanPlusVarianceMeasure::getOutputArgument(const std::string& aInput) const
{
    auto tIterator = mStatisticsToOutArgument.find(aInput);
    if(tIterator != mStatisticsToOutArgument.end())
    {
        return(tIterator->second);
    }
    else
    {
        std::string tError = std::string("INPUT STATISTIC'S MEASURE NAME = ") + aInput
                + " IS NOT DEFINED.\n";
        THROWERR(tError);
    }
}

void MeanPlusVarianceMeasure::initialize(const Plato::InputData& aOperationNode)
{
    this->parseName(aOperationNode);
    this->parseFunction(aOperationNode);
    this->parseDataLayout(aOperationNode);
    this->parseInputs(aOperationNode);
    this->parseOutputs(aOperationNode);
}

void MeanPlusVarianceMeasure::parseName(const Plato::InputData& aOperationNode)
{
    mOperationName = Plato::Get::String(aOperationNode, "Name");
    if(mOperationName.empty() == true)
    {
        const std::string tOperationName = Plato::Get::String(aOperationNode, "Name");
        THROWERR("USER DEFINED FUNCTION NAME IS NOT.\n");
    }
}

void MeanPlusVarianceMeasure::parseFunction(const Plato::InputData& aOperationNode)
{
    mFunctionIdentifier = Plato::Get::String(aOperationNode, "Function");
    if(mFunctionIdentifier.empty() == true)
    {
        const std::string tError = std::string("FUNCTION IS NOT DEFINED IN OPERATION = ") + mOperationName + ".\n";
        THROWERR(tError);
    }
}

void MeanPlusVarianceMeasure::parseDataLayout(const Plato::InputData& aOperationNode)
{
    const std::string tLayout = Plato::Get::String(aOperationNode, "Layout", true);
    if(tLayout.empty() == true)
    {
        const std::string tOperationName = Plato::Get::String(aOperationNode, "Name");
        const std::string tError = std::string("DATA LAYOUT IS NOT DEFINED IN OPERATION = ") + tOperationName + ".\n";
        THROWERR(tError);
    }
    mDataLayout = Plato::getLayout(tLayout);
}

void MeanPlusVarianceMeasure::parseInputs(const Plato::InputData& aOperationNode)
{
    for(auto tInputNode : aOperationNode.getByName<Plato::InputData>("Input"))
    {
        std::string tInputArgumentName = Plato::Get::String(tInputNode, "ArgumentName");
        this->setMyLocalArgument(tInputArgumentName);
        double tProbability = this->getMyProbability(tInputNode);
        mSampleArgumentNameToProbability[tInputArgumentName] = tProbability;
    }
}

void MeanPlusVarianceMeasure::parseOutputs(const Plato::InputData& aOperationNode)
{
    for(auto tOutputNode : aOperationNode.getByName<Plato::InputData>("Output"))
    {
        std::string tStatisticMeasure = Plato::Get::String(tOutputNode, "Statistic", true);
        std::string tOutputArgumentName = Plato::Get::String(tOutputNode, "ArgumentName");
        this->setStatisticsToOutputArgumentNameMap(tStatisticMeasure, tOutputArgumentName);
        this->setMySigmaValue(tStatisticMeasure);
        this->setMyLocalArgument(tOutputArgumentName);
    }
}

void MeanPlusVarianceMeasure::setMySigmaValue(const std::string & aStatisticMeasure)
{
    if(aStatisticMeasure.empty() == true)
    {
        THROWERR("OUTPUT ARGUMENT NAME IS EMPTY, I.E. ARGUMENT'S NAME IS NOT DEFINED.\n");
    }

    std::vector<std::string> tStringList;
    Plato::split(aStatisticMeasure, tStringList);
    if(tStringList.size() > 2u)
    {
        const double tMySigmaValue = this->getMySigmaValue(tStringList[2]);
        const std::string& tOutputArgumentName = mStatisticsToOutArgument.find(aStatisticMeasure)->second;
        mOutArgumentToStdDevMultiplier[tOutputArgumentName] = tMySigmaValue;
    }
}

double MeanPlusVarianceMeasure::getMySigmaValue(const std::string& aInput)
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

void MeanPlusVarianceMeasure::setStatisticsToOutputArgumentNameMap
(const std::string & aStatisticMeasure, const std::string & aOutputArgumentName)
{
    if(aStatisticMeasure.empty() == true)
    {
        THROWERR("STATISTIC KEYWORD IS EMPTY, I.E. STATISTIC MEASURE IS NOT DEFINED.\n");
    }
    if(aOutputArgumentName.empty() == true)
    {
        THROWERR("OUTPUT ARGUMENT NAME KEYWORD IS EMPTY, I.E. OUTPUT ARGUMENT NAME IS NOT DEFINED.\n");
    }
    mStatisticsToOutArgument[aStatisticMeasure] = aOutputArgumentName;
}

double MeanPlusVarianceMeasure::getMyProbability(const Plato::InputData& aInputNode)
{
    const double tProbability = Plato::Get::Double(aInputNode, "Probability");
    if(tProbability <= 0.0)
    {
        const std::string tArgumentName = Plato::Get::String(aInputNode, "ArgumentName");
        const std::string tError = std::string("INVALID PROBABILITY SPECIFIED FOR INPUT ARGUMENT = ") + tArgumentName
                + ". " + "INPUT PROBABILITY WAS SET TO " + std::to_string(tProbability)
                + ". INPUT PROBABILITY SHOULD BE A POSITIVE NUMBER (I.E. GREATER THAN ZERO).\n";
        THROWERR(tError);
    }
    return (tProbability);
}

void MeanPlusVarianceMeasure::setMyLocalArgument(const std::string & aArgumentName)
{
    if(aArgumentName.empty() == true)
    {
        THROWERR("INPUT ARGUMENT NAME IS EMPTY, I.E. ARGUMENT'S NAME IS NOT DEFINED.\n");
    }
    mLocalArguments.push_back(Plato::LocalArg { mDataLayout, aArgumentName });
}

void MeanPlusVarianceMeasure::computeMean()
{
    if(mDataLayout == Plato::data::ELEMENT_FIELD)
    {
        Plato::compute_sample_set_mean(mSampleArgumentNameToProbability, mStatisticsToOutArgument, mPlatoApp);
    }
    else if(mDataLayout == Plato::data::SCALAR_FIELD)
    {
        Plato::compute_node_field_mean(mSampleArgumentNameToProbability, mStatisticsToOutArgument, mPlatoApp);
    }
    else if(mDataLayout == Plato::data::SCALAR)
    {
        Plato::compute_scalar_value_mean(mSampleArgumentNameToProbability, mStatisticsToOutArgument, mPlatoApp);
    }
    else
    {
        const std::string tParsedLayout = Plato::getLayout(mDataLayout);
        const std::string tError = std::string("MEAN CAN ONLY BE COMPUTED FOR NODAL FIELDS, ELEMENT FIELDS AND SCALAR VALUES QoIs.")
                + " INVALID INPUT DATA LAYOUT = " + tParsedLayout + ".\n";
        THROWERR(tError);
    }
}

void MeanPlusVarianceMeasure::setSampleDataToProbabilityPairs(std::vector<Plato::SampleProbPair<double*, double>>& aPairs)
{
    for(auto tItr = mSampleArgumentNameToProbability.begin(); tItr != mSampleArgumentNameToProbability.end(); ++tItr)
    {
        aPairs.push_back(Plato::SampleProbPair<double*, double>{});
        aPairs.rbegin()->mProbability = tItr->second;
        aPairs.rbegin()->mLength = mPlatoApp->getLocalNumElements();
        aPairs.rbegin()->mSample = mPlatoApp->getElementFieldData(tItr->first);
    }
}

void MeanPlusVarianceMeasure::computeMeanElementFieldQoI()
{
    std::vector<Plato::SampleProbPair<double*, double>> tSampleDataToProbabilityPairs;
    this->setSampleDataToProbabilityPairs(tSampleDataToProbabilityPairs);
    // tIterator->first = sample's argument name & tIterator->second = probability

}

void MeanPlusVarianceMeasure::computeStandardDeviation()
{
    if(mDataLayout == Plato::data::ELEMENT_FIELD)
    {
        Plato::compute_element_field_standard_deviation(mSampleArgumentNameToProbability, mStatisticsToOutArgument, mPlatoApp);
    }
    else if(mDataLayout == Plato::data::SCALAR_FIELD)
    {
        Plato::compute_node_field_standard_deviation(mSampleArgumentNameToProbability, mStatisticsToOutArgument, mPlatoApp);
    }
    else if(mDataLayout == Plato::data::SCALAR)
    {
        Plato::compute_scalar_value_standard_deviation(mSampleArgumentNameToProbability, mStatisticsToOutArgument, mPlatoApp);
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
    if(mDataLayout == Plato::data::ELEMENT_FIELD)
    {
        this->computeMeanPlusStdDevElementField();
    }
    else if(mDataLayout == Plato::data::SCALAR_FIELD)
    {
        this->computeMeanPlusStdDevNodeField();
    }
    else if(mDataLayout == Plato::data::SCALAR)
    {
        this->computeMeanPlusStdDevScalarValue();
    }
    else
    {
        const std::string tParsedLayout = Plato::getLayout(mDataLayout);
        const std::string tError = std::string("MEAN PLUS STANDARD DEVIATION MEASURE CAN ONLY BE COMPUTED FOR ") +
                "NODAL FIELDS, ELEMENT FIELDS AND SCALAR VALUES QoIs. " + "INVALID INPUT DATA LAYOUT = " + tParsedLayout + ".\n";
        THROWERR(tError);
    }
}

void MeanPlusVarianceMeasure::computeMeanPlusStdDevScalarValue()
{
    const std::string& tOutputArgumentMean = mStatisticsToOutArgument["MEAN"];
    std::vector<double>* tOutputMeanData = mPlatoApp->getValue(tOutputArgumentMean);
    const std::string& tOutputArgumentStdDev = mStatisticsToOutArgument["STD_DEV"];
    std::vector<double>* tOutputSigmaData = mPlatoApp->getValue(tOutputArgumentStdDev);

    for(auto tOuterIterator = mOutArgumentToStdDevMultiplier.begin(); tOuterIterator != mOutArgumentToStdDevMultiplier.end(); ++tOuterIterator)
    {
        const std::string& tOutputArgumentMeanPlusSigma = tOuterIterator->first;
        std::vector<double>* tOutMeanPlusSigmaData = mPlatoApp->getValue(tOutputArgumentMeanPlusSigma);
        (*tOutMeanPlusSigmaData)[0] = (*tOutputMeanData)[0] + (tOuterIterator->second * (*tOutputSigmaData)[0]); // tIterator->second = sigma multiplier
    }
}

void MeanPlusVarianceMeasure::computeMeanPlusStdDevNodeField()
{
    const std::string& tOutputArgumentMean = mStatisticsToOutArgument["MEAN"];
    double* tOutputMeanData = mPlatoApp->getNodeFieldData(tOutputArgumentMean);
    const std::string& tOutputArgumentStdDev = mStatisticsToOutArgument["STD_DEV"];
    double* tOutputSigmaData = mPlatoApp->getNodeFieldData(tOutputArgumentStdDev);

    const size_t tLocalLength = mPlatoApp->getNodeFieldLength(tOutputArgumentStdDev);
    for(auto tOuterIterator = mOutArgumentToStdDevMultiplier.begin(); tOuterIterator != mOutArgumentToStdDevMultiplier.end(); ++tOuterIterator)
    {
        const std::string& tOutputArgumentMeanPlusSigma = tOuterIterator->first;
        double* tOutMeanPlusSigmaData = mPlatoApp->getNodeFieldData(tOutputArgumentMeanPlusSigma);
        for(size_t tIndex = 0; tIndex < tLocalLength; tIndex++)
        {
            tOutMeanPlusSigmaData[tIndex] = tOutputMeanData[tIndex]
                    + (tOuterIterator->second * tOutputSigmaData[tIndex]); // tIterator->second = sigma multiplier
        }

        mPlatoApp->compressAndUpdateNodeField(tOutputArgumentMeanPlusSigma);
    }
}

void MeanPlusVarianceMeasure::computeMeanPlusStdDevElementField()
{
    const std::string& tOutputArgumentMean = mStatisticsToOutArgument["MEAN"];
    double* tOutputMeanData = mPlatoApp->getElementFieldData(tOutputArgumentMean);
    const std::string& tOutputArgumentStdDev = mStatisticsToOutArgument["STD_DEV"];
    double* tOutputSigmaData = mPlatoApp->getElementFieldData(tOutputArgumentStdDev);

    const size_t tLocalNumElements = mPlatoApp->getLocalNumElements();
    for(auto tOuterIterator = mOutArgumentToStdDevMultiplier.begin(); tOuterIterator != mOutArgumentToStdDevMultiplier.end(); ++tOuterIterator)
    {
        const std::string& tOutputArgumentMeanPlusSigma = tOuterIterator->first;
        double* tOutMeanPlusSigmaData = mPlatoApp->getElementFieldData(tOutputArgumentMeanPlusSigma);
        for(size_t tIndex = 0; tIndex < tLocalNumElements; tIndex++)
        {
            tOutMeanPlusSigmaData[tIndex] = tOutputMeanData[tIndex]
                    + (tOuterIterator->second * tOutputSigmaData[tIndex]); // tIterator->second = sigma multiplier
        }
    }
}

}
// namespace Plato
