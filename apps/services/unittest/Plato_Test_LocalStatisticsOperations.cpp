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
 * Plato_Test_LocalStatisticsOperations.cpp
 *
 *  Created on: Jun 27, 2019
 */

#include <gtest/gtest.h>

#include "PlatoApp.hpp"
#include "Plato_Macros.hpp"
#include "Plato_InputData.hpp"
#include "Plato_LocalOperation.hpp"
#include "Plato_OperationsUtilities.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Compute mean, standard deviation and mean plus standard deviation measures
**********************************************************************************/
class MeanPlusVarianceMeasure : public Plato::LocalOp
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aPlatoApp PLATO application interface
     * @param [in] aOperationNode input data
    **********************************************************************************/
    MeanPlusVarianceMeasure(PlatoApp* aPlatoApp, const Plato::InputData& aOperationNode) :
            Plato::LocalOp(aPlatoApp),
            mDataLayout(Plato::data::UNDEFINED)
    {
        this->initialize(aOperationNode);
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~MeanPlusVarianceMeasure()
    {
    }

    /******************************************************************************//**
     * @brief Compute statistics
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Get input and output arguments associated with the local operation
     * @param [in/out] aLocalArgs local input and output arguments
    **********************************************************************************/
    void getArguments(std::vector<LocalArg>& aLocalArgs)
    {
        aLocalArgs = mLocalArguments;
    }

    /******************************************************************************//**
     * @brief Return output argument's alias
     * @return output argument's alias
    **********************************************************************************/
    std::string getOutputAlias() const
    {
        return (mOutputAlias);
    }

    /******************************************************************************//**
     * @brief Return name used by the user to identify the function.
     * @return user-defined function name
    **********************************************************************************/
    std::string getOperationName() const
    {
        return (mOperationName);
    }

    /******************************************************************************//**
     * @brief Return function's identifier, i.e. Mean Plus StdDev.
     * @return function's identifier, the user shall use "Mean Plus StdDev" as the
     *   function identifier. the function identifier is used by the PLATO application
     *   to create the function.
    **********************************************************************************/
    std::string getFunctionIdentifier() const
    {
        return (mFunctionIdentifier);
    }

    /******************************************************************************//**
     * @brief Input and output data layout.
     * return data layout, valid options are element field, node field and scalar value
    **********************************************************************************/
    Plato::data::layout_t getDataLayout() const
    {
        return (mDataLayout);
    }

    /******************************************************************************//**
     * @brief Return list of standard deviation multipliers in ascending order
     * @return standard deviation multipliers
    **********************************************************************************/
    std::vector<double> getStandardDeviationMultipliers() const
    {
        std::vector<double> tOuput;
        for(auto tIterator = mOutArgumentToSigma.begin(); tIterator != mOutArgumentToSigma.end(); ++ tIterator)
        {
            tOuput.push_back(tIterator->second);
        }
        std::sort(tOuput.begin(), tOuput.end());
        return (tOuput);
    }

    /******************************************************************************//**
     * @brief Return probability associated with input argument name
     * @param [in] aInput input argument name
     * @return probability
    **********************************************************************************/
    double getProbability(const std::string& aInput) const
    {
        auto tIterator = mInArgumentToProbability.find(aInput);
        if(tIterator != mInArgumentToProbability.end())
        {
            return(tIterator->second);
        }
        else
        {
            const std::string tError = std::string("INPUT ARGUMENT NAME = ") + aInput + " IS NOT DEFINED.\n";
            THROWERR(tError);
        }
    }

    /******************************************************************************//**
     * @brief Return output argument name associated with statistic measure
     * @param [in] aInput upper case statistic's measure name, valid options are: MEAN, STDDEV
     *   and MEAN_PLUS_#_STDDEV, where # denotes the standard deviation multiplier
     * @return output argument name
    **********************************************************************************/
    std::string getOutputArgument(const std::string& aInput) const
    {
        auto tIterator = mStatisticsToOutArgument.find(aInput);
        if(tIterator != mStatisticsToOutArgument.end())
        {
            return(tIterator->second);
        }
        else
        {
            const std::string tError = std::string("INPUT STATISTIC'S MEASURE NAME = ") + aInput
                    + " IS NOT DEFINED.\n";
            THROWERR(tError);
        }
    }

private:
    /******************************************************************************//**
     * @brief Initialize mean plus variance local operation
     * @param [in] aOperationNode input data associated with statistics XML node
    **********************************************************************************/
    void initialize(const Plato::InputData& aOperationNode)
    {
        this->parseName(aOperationNode);
        this->parseAlias(aOperationNode);
        this->parseFunction(aOperationNode);
        this->parseDataLayout(aOperationNode);
        this->parseInputs(aOperationNode);
        this->parseOutputs(aOperationNode);
    }

    /******************************************************************************//**
     * @brief Parse user defined function name
     * @param [in] aOperationNode input data associated with the statistics XML node
    **********************************************************************************/
    void parseName(const Plato::InputData& aOperationNode)
    {
        mOperationName = Plato::Get::String(aOperationNode, "Name");
        if(mOperationName.empty() == true)
        {
            const std::string tOperationName = Plato::Get::String(aOperationNode, "Name");
            THROWERR("USER DEFINED FUNCTION NAME IS NOT.\n");
        }
    }

    /******************************************************************************//**
     * @brief Parse function identifier
     * @param [in] aOperationNode input data associated with the statistics XML node
    **********************************************************************************/
    void parseFunction(const Plato::InputData& aOperationNode)
    {
        mFunctionIdentifier = Plato::Get::String(aOperationNode, "Function");
        if(mFunctionIdentifier.empty() == true)
        {
            const std::string tError = std::string("FUNCTION IS NOT DEFINED IN OPERATION = ") + mOperationName + ".\n";
            THROWERR(tError);
        }
    }

    /******************************************************************************//**
     * @brief Parse output argument alias from input file
     * @param [in] aOperationNode input data associated with the statistics XML node
    **********************************************************************************/
    void parseAlias(const Plato::InputData& aOperationNode)
    {
        mOutputAlias = Plato::Get::String(aOperationNode, "Alias");
        if(mOutputAlias.empty() == true)
        {
            const std::string tOperationName = Plato::Get::String(aOperationNode, "Name");
            const std::string tError = std::string("ALIAS IS NOT DEFINED IN OPERATION = ") + tOperationName + ".\n";
            THROWERR(tError);
        }
    }

    /******************************************************************************//**
     * @brief Parse data layout from input file
     * @param [in] aOperationNode input data associated with the statistics XML node
    **********************************************************************************/
    void parseDataLayout(const Plato::InputData& aOperationNode)
    {
        const std::string tLayout = Plato::Get::String(aOperationNode, "Layout");
        if(tLayout.empty() == true)
        {
            const std::string tOperationName = Plato::Get::String(aOperationNode, "Name");
            const std::string tError = std::string("DATA LAYOUT IS NOT DEFINED IN OPERATION = ") + tOperationName + ".\n";
            THROWERR(tError);
        }
        mDataLayout = Plato::getLayout(tLayout);
    }

    /******************************************************************************//**
     * @brief Parse input arguments from input files
     * @param [in] aOperationNode input data associated with the statistics XML node
    **********************************************************************************/
    void parseInputs(const Plato::InputData& aOperationNode)
    {
        for(auto tInputNode : aOperationNode.getByName<Plato::InputData>("Input"))
        {
            std::string tInputArgumentName = Plato::Get::String(tInputNode, "ArgumentName");
            this->setMyLocalArgument(tInputArgumentName);
            double tProbability = this->getMyProbability(tInputNode);
            mInArgumentToProbability[tInputArgumentName] = tProbability;
        }
    }

    /******************************************************************************//**
     * @brief Parse output arguments from input files
     * @param [in] aOperationNode input data associated with the statistics XML node
    **********************************************************************************/
    void parseOutputs(const Plato::InputData& aOperationNode)
    {
        for(auto tOutputNode : aOperationNode.getByName<Plato::InputData>("Output"))
        {
            std::string tStatisticMeasure = Plato::Get::String(tOutputNode, "ArgumentName", true);
            this->setOutputArgumentName(tStatisticMeasure);
            this->setMySigmaValue(tStatisticMeasure);
            const std::string& tOutputArgumentName = mStatisticsToOutArgument.find(tStatisticMeasure)->second;
            this->setMyLocalArgument(tOutputArgumentName);
        }
    }

    /******************************************************************************//**
     * @brief Set output argument to standard deviation (i.e. sigma) multiplier map
     * @param [in] aStatisticMeasure statistic measure name
    **********************************************************************************/
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
            const double tMySigmaValue = this->getMySigmaValue(tStringList[2]);
            const std::string& tOutputArgumentName = mStatisticsToOutArgument.find(aStatisticMeasure)->second;
            mOutArgumentToSigma[tOutputArgumentName] = tMySigmaValue;
        }
    }

    /******************************************************************************//**
     * @brief Convert string standard deviation multiplier to double
     * @param [in] aInput string standard deviation multiplier
     * @return double standard deviation multiplier
    **********************************************************************************/
    double getMySigmaValue(const std::string& aInput)
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

    /******************************************************************************//**
     * @brief Set statistic to output argument map
     * @param [in] aStatisticMeasure statistic measure name
    **********************************************************************************/
    void setOutputArgumentName(const std::string & aStatisticMeasure)
    {
        if(aStatisticMeasure.empty() == true)
        {
            THROWERR("OUTPUT ARGUMENT NAME IS EMPTY, I.E. ARGUMENT'S NAME IS NOT DEFINED.\n");
        }
        std::string tOutputArgumentName = mOutputAlias + "_" + aStatisticMeasure;
        mStatisticsToOutArgument[aStatisticMeasure] = tOutputArgumentName;
    }

    /******************************************************************************//**
     * @brief Return probability measure
     * @param [in] aInputNode input data XML node
     * @return return probability measure
    **********************************************************************************/
    double getMyProbability(const Plato::InputData& aInputNode)
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

    /******************************************************************************//**
     * @brief Add argument to local argument list
     * @param [in] aArgumentName argument name
    **********************************************************************************/
    void setMyLocalArgument(const std::string & aArgumentName)
    {
        if(aArgumentName.empty() == true)
        {
            THROWERR("INPUT ARGUMENT NAME IS EMPTY, I.E. ARGUMENT'S NAME IS NOT DEFINED.\n");
        }
        mLocalArguments.push_back(Plato::LocalArg { mDataLayout, aArgumentName });
    }

    /******************************************************************************//**
     * @brief Compute mean measure
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Compute mean measure for a scalar value
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Compute mean measure for a node field
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Compute mean measure for an element field
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Compute standard deviation measure
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Compute standard deviation measure for a scalar value
    **********************************************************************************/
    void computeStandardDeviationScalarValue()
    {
        const std::string& tOutputArgumentMean = mStatisticsToOutArgument["MEAN"];
        std::vector<double>* tOutputMeanData = mPlatoApp->getValue(tOutputArgumentMean);

        double tLocalValue = 0;
        for(auto tIterator = mInArgumentToProbability.begin(); tIterator != mInArgumentToProbability.end(); ++tIterator)
        {
            const std::string& tInputArgumentName = tIterator->first;
            std::vector<double>* tInputData = mPlatoApp->getValue(tInputArgumentName);
            double tSampleMinusMean = (*tInputData)[0] - (*tOutputMeanData)[0];
            tLocalValue += tIterator->second * std::pow(tSampleMinusMean, 2.0); // tIterator->second = Probability
        }

        double tGlobalValue = 0.;
        mPlatoApp->reduceScalarValue(tLocalValue, tGlobalValue);
        const std::string& tOutputArgumentStdDev = mStatisticsToOutArgument["STDDEV"];
        std::vector<double>* tOutputSigmaData = mPlatoApp->getValue(tOutputArgumentStdDev);
        (*tOutputSigmaData)[0] = tGlobalValue;
    }

    /******************************************************************************//**
     * @brief Compute standard deviation measure for a node field
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Compute standard deviation measure for an element field
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Compute mean plus standard deviation measure
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Compute mean plus standard deviation measure for a scalar value
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Compute mean plus standard deviation measure for a node field
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Compute mean plus standard deviation measure for an element field
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Zero all entries
     * @param [in] aLength container's length
     * @param [in/out] aData container's data
    **********************************************************************************/
    void zero(const size_t& aLength, double* aData)
    {
        for(size_t tIndex = 0; tIndex < aLength; tIndex++)
        {
            aData[tIndex] = 0.0;
        }
    }

    /******************************************************************************//**
     * @brief Split collection of strings separated by the '_' delimiter.
     * @param [in] aInput input string
     * @param [in/out] aOutput list of strings
    **********************************************************************************/
    void split(const std::string & aInput, std::vector<std::string> & aOutput)
    {
        std::string tSegment;
        std::stringstream tArgument(aInput);
        while(std::getline(tArgument, tSegment, '_'))
        {
           aOutput.push_back(tSegment);
        }
    }

private:
    std::string mOutputAlias; /*!< alias used for output QoI */
    std::string mOperationName; /*!< user defined function name */
    std::string mFunctionIdentifier; /*!< function identifier */
    Plato::data::layout_t mDataLayout; /*!< output/input data layout */
    std::vector<Plato::LocalArg> mLocalArguments; /*!< input/output shared data set */

    std::map<std::string, double> mInArgumentToProbability; /*!< sample to probability map */
    std::map<std::string, double> mOutArgumentToSigma; /*!< output argument name to standard deviation multiplier map */
    std::map<std::string, std::string> mStatisticsToOutArgument; /*!< statistics to output argument name map */

};
// class MeanPlusVarianceMeasure

}
// namespace Plato

namespace MeanPlusVarianceMeasureTest
{

TEST(PlatoTest, MeanPlusVarianceMeasure)
{
    Plato::InputData tOperations("Operation");
    tOperations.add<std::string>("Function", "Mean Plus StdDev");
    tOperations.add<std::string>("Name", "Objective Statistics");
    tOperations.add<std::string>("Layout", "Scalar");
    tOperations.add<std::string>("Alias", "Objective");

    Plato::InputData tInput1("Input");
    tInput1.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective");
    tInput1.add<std::string>("Probability", "0.5");
    tOperations.add<Plato::InputData>("Input", tInput1);
    Plato::InputData tInput2("Input");
    tInput2.add<std::string>("ArgumentName", "sierra_sd1_lc2_objective");
    tInput2.add<std::string>("Probability", "0.25");
    tOperations.add<Plato::InputData>("Input", tInput2);
    Plato::InputData tInput3("Input");
    tInput3.add<std::string>("ArgumentName", "sierra_sd1_lc3_objective");
    tInput3.add<std::string>("Probability", "0.25");
    tOperations.add<Plato::InputData>("Input", tInput3);

    Plato::InputData tOutput1("Output");
    tOutput1.add<std::string>("ArgumentName", "Mean");
    tOperations.add<Plato::InputData>("Output", tOutput1);
    Plato::InputData tOutput2("Output");
    tOutput2.add<std::string>("ArgumentName", "StdDev");
    tOperations.add<Plato::InputData>("Output", tOutput2);
    Plato::InputData tOutput3("Output");
    tOutput3.add<std::string>("ArgumentName", "Mean_Plus_1_StdDev");
    tOperations.add<Plato::InputData>("Output", tOutput3);
    Plato::InputData tOutput4("Output");
    tOutput4.add<std::string>("ArgumentName", "Mean_Plus_5_StdDev");
    tOperations.add<Plato::InputData>("Output", tOutput4);

    // TEST THAT INPUT DATA IS PARSED
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    Plato::MeanPlusVarianceMeasure tOperation(&tPlatoApp, tOperations);

    // TEST DATA
    ASSERT_EQ(Plato::data::SCALAR, tOperation.getDataLayout());
    ASSERT_STREQ("Objective", tOperation.getOutputAlias().c_str());
    ASSERT_STREQ("Objective Statistics", tOperation.getOperationName().c_str());
    ASSERT_STREQ("Mean Plus StdDev", tOperation.getFunctionIdentifier().c_str());

    std::vector<Plato::LocalArg> tLocalArguments;
    tOperation.getArguments(tLocalArguments);
    ASSERT_EQ(7u, tLocalArguments.size());
    std::vector<std::string> tArgumentNames =
        { "sierra_sd1_lc1_objective", "sierra_sd1_lc2_objective", "sierra_sd1_lc3_objective", "Objective_MEAN",
                "Objective_STDDEV", "Objective_MEAN_PLUS_1_STDDEV", "Objective_MEAN_PLUS_5_STDDEV" };
    for(size_t tIndex = 0; tIndex < tArgumentNames.size(); tIndex++)
    {
        bool tFoundGoldValue =
                std::find(tArgumentNames.begin(), tArgumentNames.end(), tLocalArguments[tIndex].mName) != tArgumentNames.end();
        ASSERT_TRUE(tFoundGoldValue);
    }

    double tTolerance = 1e-6;
    ASSERT_NEAR(0.5, tOperation.getProbability("sierra_sd1_lc1_objective"), tTolerance);
    ASSERT_NEAR(0.25, tOperation.getProbability("sierra_sd1_lc2_objective"), tTolerance);
    ASSERT_NEAR(0.25, tOperation.getProbability("sierra_sd1_lc3_objective"), tTolerance);
    ASSERT_THROW(tOperation.getProbability("sierra_sd1_lc4_objective"), std::runtime_error);

    std::vector<double> tMultipliers = tOperation.getStandardDeviationMultipliers();
    ASSERT_EQ(2u, tMultipliers.size());
    ASSERT_NEAR(1.0, tMultipliers[0], tTolerance);
    ASSERT_NEAR(5.0, tMultipliers[1], tTolerance);

    ASSERT_STREQ("Objective_MEAN", tOperation.getOutputArgument("MEAN").c_str());
    ASSERT_STREQ("Objective_STDDEV", tOperation.getOutputArgument("STDDEV").c_str());
    ASSERT_STREQ("Objective_MEAN_PLUS_1_STDDEV", tOperation.getOutputArgument("MEAN_PLUS_1_STDDEV").c_str());
    ASSERT_STREQ("Objective_MEAN_PLUS_5_STDDEV", tOperation.getOutputArgument("MEAN_PLUS_5_STDDEV").c_str());
    ASSERT_THROW(tOperation.getOutputArgument("MEAN_PLUS_2_STDDEV"), std::runtime_error);
}

}
// namespace MeanPlusVarianceMeasureTest
