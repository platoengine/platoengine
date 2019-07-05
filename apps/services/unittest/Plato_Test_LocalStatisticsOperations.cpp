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
 * PLATO_Test_LocalStatisticsOperations.cpp
 *
 *  Created on: Jun 27, 2019
 */

#include "PlatoApp.hpp"
#include "Plato_Macros.hpp"
#include "Plato_InputData.hpp"
#include "Plato_LocalOperation.hpp"
#include "Plato_OperationsUtilities.hpp"

namespace Plato
{

class MeanPlusVarianceMeasure : public Plato::LocalOp
{
public:
    MeanPlusVarianceMeasure(PlatoApp* aPlatoApp, const Plato::InputData& aStatisticsNode) :
            Plato::LocalOp(aPlatoApp),
            mDataLayout(Plato::data::UNDEFINED)
    {
        this->initialize(aStatisticsNode);
    }

    virtual ~MeanPlusVarianceMeasure()
    {
    }

    void operator()()
    {
        this->computeMean();
        bool tStandardDeviationMeasuresRequested = mOutArgumentToSigma.size() > 0u;
        if(tStandardDeviationMeasuresRequested == true)
        {
            this->computeStandardDeviation();
            this->computeMeanPlusStdDev();
        }
    }

    void getArguments(std::vector<LocalArg>& aLocalArgs)
    {
        aLocalArgs = mLocalArguments;
    }

private:
    void split(const std::string & aInput, std::vector<std::string> & aOutput)
    {
        std::string tSegment;
        std::stringstream tArgument(aInput);
        while(std::getline(tArgument, tSegment, '_'))
        {
           aOutput.push_back(tSegment);
        }
    }

    void initialize(const Plato::InputData& aStatisticsNode)
    {
        this->parseAlias(aStatisticsNode);
        this->parseDataLayout(aStatisticsNode);
        this->parseInputs(aStatisticsNode);
        this->parseOutputs(aStatisticsNode);
    }

    void parseAlias(const Plato::InputData& aStatisticsNode)
    {
        mAlias = Plato::Get::String(aStatisticsNode, "Alias");
        if(mAlias.empty() == true)
        {
            const std::string tOperationName = Plato::Get::String(aStatisticsNode, "Name");
            const std::string tError = std::string("ALIAS IS NOT DEFINED IN OPERATION = ") + tOperationName + ".\n";
            THROWERR(tError);
        }
    }

    void parseDataLayout(const Plato::InputData& aStatisticsNode)
    {
        const std::string tLayout = Plato::Get::String(aStatisticsNode, "Layout");
        if(tLayout.empty() == true)
        {
            const std::string tOperationName = Plato::Get::String(aStatisticsNode, "Name");
            const std::string tError = std::string("DATA LAYOUT IS NOT DEFINED IN OPERATION = ") + tOperationName + ".\n";
            THROWERR(tError);
        }
        mDataLayout = Plato::getLayout(tLayout);
    }

    void parseInputs(const Plato::InputData& aStatisticsNode)
    {
        for(auto tInputNode : aStatisticsNode.getByName<Plato::InputData>("Input"))
        {
            std::string tInputArgumentName = Plato::Get::String(tInputNode, "ArgumentName");
            this->setMyLocalArgument(tInputArgumentName);
            double tProbability = this->getMyProbability(tInputNode);
            mInArgumentToProbability[tInputArgumentName] = tProbability;
        }
    }

    void parseOutputs(const Plato::InputData& aStatisticsNode)
    {
        for(auto tOutputNode : aStatisticsNode.getByName<Plato::InputData>("Output"))
        {
            std::string tStatisticMeasure = Plato::Get::String(tOutputNode, "ArgumentName", true);
            this->setOutputArgumentName(tStatisticMeasure);
            this->setMySigmaValue(tStatisticMeasure);
            const std::string& tOutputArgumentName = mStatisticsToOutArgument.find(tStatisticMeasure)->second;
            this->setMyLocalArgument(tOutputArgumentName);
        }
    }

    void setMySigmaValue(const std::string & aStatisticMeasure)
    {
        if(aStatisticMeasure.empty() == true)
        {
            THROWERR("OUTPUT ARGUMENT NAME IS EMPTY, I.E. ARGUMENT'S NAME IS NOT DEFINED.\n");
        }

        std::vector<std::string> tStringList;
        this->split(aStatisticMeasure, tStringList);
        if(tStringList.size() > 1u)
        {
            if(tStringList.size() < 3u)
            {
                const std::string tError = std::string("STATISTIC MEASURE ARGUMENT IS NOT PROPERLY FORMATED. ")
                        + "THE STATISTIC'S MEASURE FORMAT IS EXPECTED TO BE: MEAN_PLUS_#_STDDEV. "
                        + "THE USER PROVIDED THE FOLLOWING FORMAT INSTEAD: " + aStatisticMeasure + ".\n";
            }
            const double tMySigmaValue = std::stod(tStringList[2]);
            const std::string& tOutputArgumentName = mStatisticsToOutArgument.find(aStatisticMeasure)->second;
            mOutArgumentToSigma[tOutputArgumentName] = tMySigmaValue;
        }
    }

    void setOutputArgumentName(const std::string & aStatisticMeasure)
    {
        if(aStatisticMeasure.empty() == true)
        {
            THROWERR("OUTPUT ARGUMENT NAME IS EMPTY, I.E. ARGUMENT'S NAME IS NOT DEFINED.\n");
        }
        std::string tOutputArgumentName = mAlias + "_" + aStatisticMeasure;
        mStatisticsToOutArgument[aStatisticMeasure] = tOutputArgumentName;
    }

    double getMyProbability(const Plato::InputData& aInputNode)
    {
        const double tProbability = Plato::Get::Double(aInputNode, "Probability");
        if(tProbability > 0.0)
        {
            const std::string tArgumentName = Plato::Get::String(aInputNode, "ArgumentName");
            const std::string tError = std::string("INVALID PROBABILITY SPECIFIED FOR INPUT ARGUMENT = ") + tArgumentName
                    + ". " + "INPUT PROBABILITY WAS SET TO " + std::to_string(tProbability)
                    + ". INPUT PROBABILITY SHOULD BE A POSITIVE NUMBER (I.E. GREATER THAN ZERO).\n";
            THROWERR(tError);
        }
        return (tProbability);
    }

    void setMyLocalArgument(const std::string & aArgumentName)
    {
        if(aArgumentName.empty() == true)
        {
            THROWERR("INPUT ARGUMENT NAME IS EMPTY, I.E. ARGUMENT'S NAME IS NOT DEFINED.\n");
        }
        mLocalArguments.push_back(Plato::LocalArg { mDataLayout, aArgumentName });
    }

    void computeMean()
    {
        if(mDataLayout == Plato::data::ELEMENT_FIELD)
        {
            this->computeMeanElementField();
        }
        else if(mDataLayout == Plato::data::SCALAR_FIELD)
        {
            this->computeMeanNodeField();
        }
        else if(mDataLayout == Plato::data::SCALAR)
        {
            this->computeMeanScalarValue();
        }
        else
        {
            const std::string tParsedLayout = Plato::getLayout(mDataLayout);
            const std::string tError = std::string("MEAN CAN ONLY BE COMPUTED FOR NODAL FIELDS, ELEMENT FIELDS AND SCALAR VALUES QoIs.")
                    + " INVALID INPUT DATA LAYOUT = " + tParsedLayout + ".\n";
            THROWERR(tError);
        }
    }

    void computeMeanScalarValue()
    {
        double tLocalValue = 0;
        for(auto tIterator = mInArgumentToProbability.begin(); tIterator != mInArgumentToProbability.end(); ++tIterator)
        {
            const std::string& tInputArgumentName = tIterator->first;
            std::vector<double>* tInputValue = mPlatoApp->getValue(tInputArgumentName);
            tLocalValue += tIterator->second * (*tInputValue)[0];  // tIterator->second = Probability
        }

        double tGlobalValue = 0.;
        mPlatoApp->reduceScalarValue(tLocalValue, tGlobalValue);
        const std::string& tOutputArgumentName = mStatisticsToOutArgument["MEAN"];
        std::vector<double>* tOutputValue = mPlatoApp->getValue(tOutputArgumentName);
        (*tOutputValue)[0] = tGlobalValue;
    }

    void computeMeanNodeField()
    {
        const std::string& tOutputArgumentName = mStatisticsToOutArgument["MEAN"];
        const size_t tLength = mPlatoApp->getNodeFieldLength(tOutputArgumentName);
        double* tOutputData = mPlatoApp->getNodeFieldData(tOutputArgumentName);
        this->zero(tLength, tOutputData);

        for(auto tIterator = mInArgumentToProbability.begin(); tIterator != mInArgumentToProbability.end(); ++tIterator)
        {
            const std::string& tInputArgumentName = tIterator->first;
            double* tInputData = mPlatoApp->getNodeFieldData(tInputArgumentName);
            for(size_t tIndex = 0; tIndex < tLength; tIndex++)
            {
                tOutputData[tIndex] += tIterator->second * tInputData[tIndex];  // tIterator->second = Probability
            }
        }

        mPlatoApp->compressAndUpdateNodeField(tOutputArgumentName);
    }

    void computeMeanElementField()
    {
        const size_t tLocalNumElements = mPlatoApp->getLocalNumElements();
        const std::string& tOutputArgumentName = mStatisticsToOutArgument["MEAN"];
        double* tOutputData = mPlatoApp->getElementFieldData(tOutputArgumentName);
        this->zero(tLocalNumElements, tOutputData);

        for(auto tIterator = mInArgumentToProbability.begin(); tIterator != mInArgumentToProbability.end(); ++tIterator)
        {
            const std::string& tInputArgumentName = tIterator->first;
            double* tInputDataView = mPlatoApp->getElementFieldData(tInputArgumentName);
            for(size_t tIndex = 0; tIndex < tLocalNumElements; tIndex++)
            {
                tOutputData[tIndex] += tIterator->second * tInputDataView[tIndex]; // tIterator->second = Probability
            }
        }
    }

    void computeStandardDeviation()
    {
        if(mDataLayout == Plato::data::ELEMENT_FIELD)
        {
            this->computeStandardDeviationElementField();
        }
        else if(mDataLayout == Plato::data::SCALAR_FIELD)
        {
            this->computeStandardDeviationNodeField();
        }
        else if(mDataLayout == Plato::data::SCALAR)
        {
            this->computeStandardDeviationScalarValue();
        }
        else
        {
            const std::string tParsedLayout = Plato::getLayout(mDataLayout);
            const std::string tError = std::string("STANDARD DEVIATION CAN ONLY BE COMPUTED FOR NODAL FIELDS, ELEMENT FIELDS AND SCALAR VALUES QoIs.")
                    + " INVALID INPUT DATA LAYOUT = " + tParsedLayout + ".\n";
            THROWERR(tError);
        }
    }

    void computeStandardDeviationScalarValue()
    {
        const std::string& tOutputArgumentMean = mStatisticsToOutArgument["MEAN"];
        std::vector<double>* tOutputMeanData = mPlatoApp->getValue(tOutputArgumentMean);

        double tLocalValue = 0;
        for(auto tIterator = mInArgumentToProbability.begin(); tIterator != mInArgumentToProbability.end(); ++tIterator)
        {
            const std::string& tInputArgumentName = tIterator->first;
            std::vector<double>* tInputData = mPlatoApp->getValue(tInputArgumentName);
            double tSampleMinusMean = tInputData[0] - tOutputMeanData[0];
            tLocalValue += tIterator->second * std::pow(tSampleMinusMean, 2.0); // tIterator->second = Probability
        }

        double tGlobalValue = 0.;
        mPlatoApp->reduceScalarValue(tLocalValue, tGlobalValue);
        const std::string& tOutputArgumentStdDev = mStatisticsToOutArgument["STDDEV"];
        std::vector<double>* tOutputSigmaData = mPlatoApp->getValue(tOutputArgumentStdDev);
        (*tOutputSigmaData)[0] = tGlobalValue;
    }

    void computeStandardDeviationNodeField()
    {
        const std::string& tOutputArgumentMean = mStatisticsToOutArgument["MEAN"];
        double* tOutputMeanData = mPlatoApp->getNodeFieldData(tOutputArgumentMean);
        const std::string& tOutputArgumentStdDev = mStatisticsToOutArgument["STDDEV"];
        double* tOutputSigmaData = mPlatoApp->getNodeFieldData(tOutputArgumentStdDev);

        const size_t tLocalLength = mPlatoApp->getNodeFieldLength(tOutputArgumentMean);
        this->zero(tLocalLength, tOutputSigmaData);

        for(auto tIterator = mInArgumentToProbability.begin(); tIterator != mInArgumentToProbability.end(); ++tIterator)
        {
            const std::string& tInputArgumentName = tIterator->first;
            double* tInputData = mPlatoApp->getNodeFieldData(tInputArgumentName);
            for(size_t tIndex = 0; tIndex < tLocalLength; tIndex++)
            {
                double tSampleMinusMean = tInputData[tIndex] - tOutputMeanData[tIndex];
                tOutputSigmaData[tIndex] += tIterator->second * std::pow(tSampleMinusMean, 2.0); // tIterator->second = Probability
            }
        }

        mPlatoApp->compressAndUpdateNodeField(tOutputArgumentStdDev);
    }

    void computeStandardDeviationElementField()
    {
        const size_t tLocalNumElements = mPlatoApp->getLocalNumElements();
        const std::string& tOutputArgumentMean = mStatisticsToOutArgument["MEAN"];
        double* tOutputMeanData = mPlatoApp->getElementFieldData(tOutputArgumentMean);
        const std::string& tOutputArgumentStdDev = mStatisticsToOutArgument["STDDEV"];
        double* tOutputSigmaData = mPlatoApp->getElementFieldData(tOutputArgumentStdDev);
        this->zero(tLocalNumElements, tOutputSigmaData);

        for(auto tIterator = mInArgumentToProbability.begin(); tIterator != mInArgumentToProbability.end(); ++tIterator)
        {
            const std::string& tInputArgumentName = tIterator->first;
            double* tInputData = mPlatoApp->getElementFieldData(tInputArgumentName);
            for(size_t tIndex = 0; tIndex < tLocalNumElements; tIndex++)
            {
                double tSampleMinusMean = tInputData[tIndex] - tOutputMeanData[tIndex];
                tOutputSigmaData[tIndex] += tIterator->second * std::pow(tSampleMinusMean, 2.0); // tIterator->second = Probability
            }
        }
    }

    void computeMeanPlusStdDev()
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

    void computeMeanPlusStdDevScalarValue()
    {
        const std::string& tOutputArgumentMean = mStatisticsToOutArgument["MEAN"];
        std::vector<double>* tOutputMeanData = mPlatoApp->getValue(tOutputArgumentMean);
        const std::string& tOutputArgumentStdDev = mStatisticsToOutArgument["STDDEV"];
        std::vector<double>* tOutputSigmaData = mPlatoApp->getValue(tOutputArgumentStdDev);

        for(auto tOuterIterator = mOutArgumentToSigma.begin(); tOuterIterator != mOutArgumentToSigma.end(); ++tOuterIterator)
        {
            const std::string& tOutputArgumentMeanPlusSigma = tOuterIterator->first;
            std::vector<double>* tOutMeanPlusSigmaData = mPlatoApp->getValue(tOutputArgumentMeanPlusSigma);
            (*tOutMeanPlusSigmaData)[0] = (*tOutputMeanData)[0] + (tOuterIterator->second * (*tOutputSigmaData)[0]); // tIterator->second = sigma multiplier
        }
    }

    void computeMeanPlusStdDevNodeField()
    {
        const std::string& tOutputArgumentMean = mStatisticsToOutArgument["MEAN"];
        double* tOutputMeanData = mPlatoApp->getNodeFieldData(tOutputArgumentMean);
        const std::string& tOutputArgumentStdDev = mStatisticsToOutArgument["STDDEV"];
        double* tOutputSigmaData = mPlatoApp->getNodeFieldData(tOutputArgumentStdDev);

        const size_t tLocalLength = mPlatoApp->getNodeFieldLength(tOutputArgumentStdDev);
        for(auto tOuterIterator = mOutArgumentToSigma.begin(); tOuterIterator != mOutArgumentToSigma.end(); ++tOuterIterator)
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

    void computeMeanPlusStdDevElementField()
    {
        const std::string& tOutputArgumentMean = mStatisticsToOutArgument["MEAN"];
        double* tOutputMeanData = mPlatoApp->getElementFieldData(tOutputArgumentMean);
        const std::string& tOutputArgumentStdDev = mStatisticsToOutArgument["STDDEV"];
        double* tOutputSigmaData = mPlatoApp->getElementFieldData(tOutputArgumentStdDev);

        const size_t tLocalNumElements = mPlatoApp->getLocalNumElements();
        for(auto tOuterIterator = mOutArgumentToSigma.begin(); tOuterIterator != mOutArgumentToSigma.end(); ++tOuterIterator)
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

    void zero(const size_t& aLength, double* aData)
    {
        for(size_t tIndex = 0; tIndex < aLength; tIndex++)
        {
            aData[tIndex] = 0.0;
        }
    }

private:
    std::string mAlias; /*!< alias used for output QoI */
    Plato::data::layout_t mDataLayout; /*!< output/input data layout */
    std::vector<Plato::LocalArg> mLocalArguments; /*!< input/output shared data set */

    std::map<std::string, double> mInArgumentToProbability; /*!< sample to probability map */
    std::map<std::string, double> mOutArgumentToSigma; /*!< standard deviation multiplier to output argument name map */
    std::map<std::string, std::string> mStatisticsToOutArgument; /*!< statistics to output argument name map */

};
// class MeanPlusVarianceMeasure

}
// namespace Plato

