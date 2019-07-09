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
#include "Plato_InputData.hpp"
#include "Plato_MeanPlusVarianceMeasure.hpp"

#include "Plato_Macros.hpp"
#include "Plato_OperationsUtilities.hpp"
#include "Plato_StatisticsOperationsUtilities.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Compute mean plus standard deviation measure's gradient
**********************************************************************************/
class MeanPlusVarianceGradient : public Plato::LocalOp
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aPlatoApp PLATO application interface
     * @param [in] aOperationNode input data
    **********************************************************************************/
    MeanPlusVarianceGradient(PlatoApp* aPlatoApp, const Plato::InputData& aOperationNode) :
            Plato::LocalOp(aPlatoApp),
            mStdDevMultiplier(-1),
            mCriteriaValueDataLayout(Plato::data::UNDEFINED),
            mCriteriaGradientDataLayout(Plato::data::UNDEFINED)
    {
        this->initialize(aOperationNode);
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~MeanPlusVarianceGradient()
    {
    }

    /******************************************************************************//**
     * @brief Perform local operation - compute mean plus standard deviation measure's gradient
    **********************************************************************************/
    void operator()()
    {
        bool tStandardDeviationMultiplierNotDefined = mStdDevMultiplier <= 0.0;
        if(tStandardDeviationMultiplierNotDefined == true)
        {
            this->computeMeanCriterionGradient();
        }
        else
        {
            Plato::compute_scalar_value_mean(mCriterionSamplesArgNameToProbability,
                                             mCriterionValueStatisticsToOutputName,
                                             mPlatoApp);
            Plato::compute_scalar_value_standard_deviation(mCriterionSamplesArgNameToProbability,
                                                           mCriterionValueStatisticsToOutputName,
                                                           mPlatoApp);
            this->computeMeanPlusStandardDeviationCriterionGradient();
        }
    }

    /******************************************************************************//**
     * @brief Get the input and output arguments associated with the local operation
     * @param [in/out] aLocalArgs local input and output arguments
    **********************************************************************************/
    void getArguments(std::vector<LocalArg>& aLocalArgs)
    {
        aLocalArgs = mLocalArguments;
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
     * @brief Return function's identifier, i.e. MeanPlusStdDev.
     * @return function's identifier, the user shall use "MeanPlusStdDev" as the
     *   function identifier. the function identifier is used by the PLATO application
     *   to create the function.
    **********************************************************************************/
    std::string getFunctionIdentifier() const
    {
        return (mFunctionIdentifier);
    }

    /******************************************************************************//**
     * @brief Return criteria's data layout.
     * return data layout, the only valid option is scalar value
    **********************************************************************************/
    Plato::data::layout_t getCriteriaValueDataLayout() const
    {
        return (mCriteriaValueDataLayout);
    }

    /******************************************************************************//**
     * @brief Return criteria gradient's data layout.
     * return data layout, valid options are element and node field
    **********************************************************************************/
    Plato::data::layout_t getCriteriaGradientDataLayout() const
    {
        return (mCriteriaGradientDataLayout);
    }

    /******************************************************************************//**
     * @brief Return list of standard deviation multipliers in ascending order
     * @return standard deviation multipliers
    **********************************************************************************/
    double getStandardDeviationMultipliers() const
    {
        return (mStdDevMultiplier);
    }

    /******************************************************************************//**
     * @brief Return the probability associated with a given criteria value sample
     * @param [in] aInput criteria value sample's argument name
     * @return probability
    **********************************************************************************/
    double getCriterionValueProbability(const std::string& aInput) const
    {
        auto tIterator = mCriterionSamplesArgNameToProbability.find(aInput);
        if(tIterator != mCriterionSamplesArgNameToProbability.end())
        {
            return (tIterator->second);
        }
        else
        {
            const std::string tError = std::string("INPUT ARGUMENT NAME = ") + aInput + " IS NOT DEFINED.\n";
            THROWERR(tError)
        }
    }

    /******************************************************************************//**
     * @brief Return the probability associated with a given criteria gradient sample
     * @param [in] aInput criteria gradient sample's argument name
     * @return probability
    **********************************************************************************/
    double getCriterionGradientProbability(const std::string& aInput) const
    {
        auto tIterator = mCriterionGradSamplesArgNameToProbability.find(aInput);
        if(tIterator != mCriterionGradSamplesArgNameToProbability.end())
        {
            return (tIterator->second);
        }
        else
        {
            const std::string tError = std::string("INPUT ARGUMENT NAME = ") + aInput + " IS NOT DEFINED.\n";
            THROWERR(tError)
        }
    }

    /******************************************************************************//**
     * @brief Return output argument associated with a given criterion value sample
     * @param [in] aInput output argument name
     * @return output argument name
    **********************************************************************************/
    std::string getCriterionValueOutputArgument(const std::string& aInput) const
    {
        auto tIterator = mCriterionValueStatisticsToOutputName.find(aInput);
        if(tIterator != mCriterionValueStatisticsToOutputName.end())
        {
            return (tIterator->second);
        }
        else
        {
            std::string tError = std::string("INPUT STATISTIC'S MEASURE NAME = ") + aInput + " IS NOT DEFINED.\n";
            THROWERR(tError)
        }
    }

    /******************************************************************************//**
     * @brief Return output argument associated with a given criterion gradient sample
     * @param [in] aInput output argument name
     * @return output argument name
    **********************************************************************************/
    std::string getCriterionGradientOutputArgument(const std::string& aInput) const
    {
        auto tIterator = mCriterionGradientStatisticsToOutputName.find(aInput);
        if(tIterator != mCriterionGradientStatisticsToOutputName.end())
        {
            return (tIterator->second);
        }
        else
        {
            std::string tError = std::string("INPUT STATISTIC'S MEASURE NAME = ") + aInput + " IS NOT DEFINED.\n";
            THROWERR(tError)
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
        this->parseFunction(aOperationNode);
        this->parseCriteriaGradientArguments(aOperationNode);
        bool tStandardDeviationMultiplierIsDefined = mStdDevMultiplier > 0.0;
        if(tStandardDeviationMultiplierIsDefined == true)
        {
            this->parseCriteriaValueArguments(aOperationNode);
            this->checkInputProbabilityValues();
        }
    }

    void checkInputProbabilityValues()
    {
        if(mCriterionSamplesArgNameToProbability.size() != mCriterionGradSamplesArgNameToProbability.size())
        {
            const std::string tError = std::string("THE NUMBER OF CRITERION VALUE AND GRADIENT SAMPLES DON'T MATCH. ") +
                    "THE NUMBER OF CRITERION VALUE SAMPLES IS " + std::to_string(mCriterionSamplesArgNameToProbability.size()) +
                    " AND THE NUMBER OF CRITERION GRADIENT SAMPLES IS " + std::to_string(mCriterionGradSamplesArgNameToProbability.size()) + ".\n";
            THROWERR(tError)
        }

        auto tGradIterator = mCriterionGradSamplesArgNameToProbability.begin();
        for(auto tValIterator = mCriterionSamplesArgNameToProbability.begin(); tValIterator != mCriterionSamplesArgNameToProbability.end(); ++tValIterator)
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

    void parseName(const Plato::InputData& aOperationNode)
    {
        mOperationName = Plato::Get::String(aOperationNode, "Name");
        if(mOperationName.empty() == true)
        {
            mOperationName = std::string("Stochastic Criterion Gradient");
        }
    }

    void parseFunction(const Plato::InputData& aOperationNode)
    {
        mFunctionIdentifier = Plato::Get::String(aOperationNode, "Function");
        if(mFunctionIdentifier.empty() == true)
        {
            const std::string tError = std::string("FUNCTION KEYWORD IS NOT DEFINED IN OPERATION = ") + mOperationName +
                    ". PLATO EXPECTS THE FOLLOWING FUNCTION NAME, MeanPlusStdDevGradient, AS INPUT.\n";
            THROWERR(tError)
        }
    }

    Plato::data::layout_t parseDataLayout(const Plato::InputData& aOperationNode)
    {
        const std::string tLayout = Plato::Get::String(aOperationNode, "Layout");
        if(tLayout.empty() == true)
        {
            const std::string tError = std::string("DATA LAYOUT IS NOT DEFINED IN OPERATION = ") +
                    mOperationName + ".\n";
            THROWERR(tError)
        }
        return(Plato::getLayout(tLayout));
    }

    void parseInputs(const Plato::data::layout_t& aDataLayout,
                     const Plato::InputData& aInput,
                     std::map<std::string, double> & aOutput)
    {
        for(auto tInputNode : aInput.getByName<Plato::InputData>("Input"))
        {
            std::string tInputArgumentName = Plato::Get::String(tInputNode, "ArgumentName");
            this->setMyLocalArgument(aDataLayout, tInputArgumentName);
            double tProbability = this->getMyProbability(tInputNode);
            aOutput[tInputArgumentName] = tProbability;
        }
    }

    void parseCriteriaValueArguments(const Plato::InputData& aOperationNode)
    {
        for(auto tCriteriaNode : aOperationNode.getByName<Plato::InputData>("CriteriaValue"))
        {
            mCriteriaValueDataLayout = this->parseDataLayout(tCriteriaNode);
            this->parseInputs(mCriteriaValueDataLayout, tCriteriaNode, mCriterionSamplesArgNameToProbability);
            this->parseCriteriaValueOutputs(tCriteriaNode);
        }
    }

    void parseCriteriaGradientArguments(const Plato::InputData& aOperationNode)
    {
        for(auto tCriteriaNode : aOperationNode.getByName<Plato::InputData>("CriteriaGradient"))
        {
            mCriteriaGradientDataLayout = this->parseDataLayout(tCriteriaNode);
            this->parseInputs(mCriteriaGradientDataLayout, tCriteriaNode, mCriterionGradSamplesArgNameToProbability);
            this->parseCriteriaGradientOutputs(tCriteriaNode);
        }
    }

    std::string getStatisticMeasure(const Plato::InputData& aOutputNode) const
    {
        std::string tStatisticMeasure = Plato::Get::String(aOutputNode, "Statistic", true);
        if(tStatisticMeasure.empty() == true)
        {
            THROWERR("STATISTIC KEYWORD IS EMPTY, I.E. STATISTIC MEASURE IS NOT DEFINED.\n");
        }
        return (tStatisticMeasure);
    }

    std::string getOutputArgument(const Plato::InputData& aOutputNode) const
    {
        std::string tOutputArgumentName = Plato::Get::String(aOutputNode, "ArgumentName");
        if(tOutputArgumentName.empty() == true)
        {
            THROWERR("OUTPUT ARGUMENT NAME KEYWORD IS EMPTY, I.E. OUTPUT ARGUMENT NAME IS NOT DEFINED.\n");
        }
        return (tOutputArgumentName);
    }

    void parseCriteriaValueOutputs(const Plato::InputData& aCriteriaNode)
    {
        for(auto tOutputNode : aCriteriaNode.getByName<Plato::InputData>("Output"))
        {
            const std::string tOutputArgumentName = this->getOutputArgument(tOutputNode);
            const std::string tStatisticMeasure = this->getStatisticMeasure(tOutputNode);
            mCriterionValueStatisticsToOutputName[tStatisticMeasure] = tOutputArgumentName;
            this->setMyLocalArgument(mCriteriaValueDataLayout, tOutputArgumentName);
        }
    }

    void parseCriteriaGradientOutputs(const Plato::InputData& aCriteriaNode)
    {
        for(auto tOutputNode : aCriteriaNode.getByName<Plato::InputData>("Output"))
        {
            const std::string tOutputArgumentName = this->getOutputArgument(tOutputNode);
            const std::string tStatisticMeasure = this->getStatisticMeasure(tOutputNode);
            mCriterionGradientStatisticsToOutputName[tStatisticMeasure] = tOutputArgumentName;
            this->setMyStandardDeviationMultiplier(tStatisticMeasure);
            this->setMyLocalArgument(mCriteriaGradientDataLayout, tOutputArgumentName);
        }
    }

    void setMyStandardDeviationMultiplier(const std::string& aStatisticMeasure)
    {
        std::vector<std::string> tStringList;
        Plato::split(aStatisticMeasure, tStringList);
        if(tStringList.size() > 2u)
        {
            mStdDevMultiplier = this->getMyStandardDeviationMultiplier(tStringList[2]);
            mOutputGradientArgumentName = mCriterionGradientStatisticsToOutputName.find(aStatisticMeasure)->second;
        }
    }

    double getMyStandardDeviationMultiplier(const std::string& aInput)
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

    double getMyProbability(const Plato::InputData& aInputNode)
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

    void setMyLocalArgument(const Plato::data::layout_t& aDataLayout, const std::string & aArgumentName)
    {
        if(aArgumentName.empty() == true)
        {
            THROWERR("INPUT ARGUMENT NAME IS EMPTY, I.E. ARGUMENT'S NAME IS NOT DEFINED.\n");
        }
        mLocalArguments.push_back(Plato::LocalArg{ aDataLayout, aArgumentName });
    }

    void computeMeanCriterionGradient()
    {
        if(mCriteriaValueDataLayout == Plato::data::ELEMENT_FIELD)
        {
            Plato::compute_element_field_mean(mCriterionGradSamplesArgNameToProbability,
                                              mCriterionGradientStatisticsToOutputName,
                                              mPlatoApp);
        }
        else if(mCriteriaValueDataLayout == Plato::data::SCALAR_FIELD)
        {
            Plato::compute_node_field_mean(mCriterionGradSamplesArgNameToProbability,
                                           mCriterionGradientStatisticsToOutputName,
                                           mPlatoApp);
        }
        else
        {
            const std::string tParsedLayout = Plato::getLayout(mCriteriaValueDataLayout);
            const std::string tError = std::string("MEAN CRITERIA GRADIENT CAN ONLY BE COMPUTED FOR NODAL AND ELEMENT FIELD QoIs.") +
                    " INVALID INPUT DATA LAYOUT = " + tParsedLayout + ".\n";
            THROWERR(tError)
        }
    }

    void computeMeanPlusStandardDeviationCriterionGradient()
    {
        if(mCriteriaValueDataLayout == Plato::data::ELEMENT_FIELD)
        {
            this->computeMeanPlusStdDevCriterionGradientElementField();
        }
        else if(mCriteriaValueDataLayout == Plato::data::SCALAR_FIELD)
        {
            this->computeMeanPlusStdDevCriterionGradientNodeField();
        }
        else
        {
            const std::string tParsedLayout = Plato::getLayout(mCriteriaValueDataLayout);
            const std::string tError = std::string("MEAN PLUS STANDARD DEVIATION CRITERION GRADIENT CAN ONLY BE COMPUTED FOR ")
                    + "NODAL AND ELEMENT FIELD QoIs. " + "INVALID INPUT DATA LAYOUT = " + tParsedLayout + ".\n";
            THROWERR(tError)
        }
    }

    void computeMeanPlusStdDevCriterionGradientNodeField()
    {
        const std::string& tCriterionValueArgumentMean = mCriterionValueStatisticsToOutputName["MEAN"];
        std::vector<double>* tCriterionMean = mPlatoApp->getValue(tCriterionValueArgumentMean);
        const std::string& tCriterionValueArgumentStdDev = mCriterionValueStatisticsToOutputName["STD_DEV"];
        std::vector<double>* tCriterionStdDev = mPlatoApp->getValue(tCriterionValueArgumentStdDev);

        const size_t tLength = mPlatoApp->getNodeFieldLength(mOutputGradientArgumentName);
        double* tOutputGradient = mPlatoApp->getNodeFieldData(mOutputGradientArgumentName);
        Plato::zero(tLength, tOutputGradient);

        // tIterator->first = Argument name & tIterator->second = Probability
        auto tCriterionValIter = mCriterionSamplesArgNameToProbability.begin();
        for(auto tCriterionGradIter = mCriterionGradSamplesArgNameToProbability.begin();
                tCriterionGradIter != mCriterionGradSamplesArgNameToProbability.end(); ++tCriterionGradIter)
        {
            double* tMyCriterionGradientSample = mPlatoApp->getNodeFieldData(tCriterionGradIter->first);
            std::vector<double>* tMyCriterionValueSample = mPlatoApp->getValue(tCriterionValIter->first);

            const double tMultiplierOverStdDev = mStdDevMultiplier / (*tCriterionStdDev)[0];
            const double tMultiplier = tCriterionGradIter->second + ( tMultiplierOverStdDev *
                    tCriterionGradIter->second * ( (*tMyCriterionValueSample)[0] - (*tCriterionMean)[0] ) );

            for(size_t tIndex = 0; tIndex < tLength; tIndex++)
            {
                tOutputGradient[tIndex] += tMultiplier * tMyCriterionGradientSample[tIndex];
            }

            ++tCriterionValIter;
        }

        mPlatoApp->compressAndUpdateNodeField(mOutputGradientArgumentName);
    }

    void computeMeanPlusStdDevCriterionGradientElementField()
    {
        const std::string& tCriterionValueArgumentMean = mCriterionValueStatisticsToOutputName["MEAN"];
        std::vector<double>* tCriterionMean = mPlatoApp->getValue(tCriterionValueArgumentMean);
        const std::string& tCriterionValueArgumentStdDev = mCriterionValueStatisticsToOutputName["STD_DEV"];
        std::vector<double>* tCriterionStdDev = mPlatoApp->getValue(tCriterionValueArgumentStdDev);

        const size_t tLength = mPlatoApp->getLocalNumElements();
        double* tOutputGradient = mPlatoApp->getElementFieldData(mOutputGradientArgumentName);
        Plato::zero(tLength, tOutputGradient);

        // tIterator->first = Argument name & tIterator->second = Probability
        auto tCriterionValIter = mCriterionSamplesArgNameToProbability.begin();
        for(auto tCriterionGradIter = mCriterionGradSamplesArgNameToProbability.begin();
                tCriterionGradIter != mCriterionGradSamplesArgNameToProbability.end(); ++tCriterionGradIter)
        {
            double* tMyCriterionGradientSample = mPlatoApp->getElementFieldData(tCriterionGradIter->first);
            std::vector<double>* tMyCriterionValueSample = mPlatoApp->getValue(tCriterionValIter->first);

            const double tMultiplierOverStdDev = mStdDevMultiplier / (*tCriterionStdDev)[0];
            const double tMultiplier = tCriterionGradIter->second + ( tMultiplierOverStdDev *
                    tCriterionGradIter->second * ( (*tMyCriterionValueSample)[0] - (*tCriterionMean)[0] ) );

            for(size_t tIndex = 0; tIndex < tLength; tIndex++)
            {
                tOutputGradient[tIndex] += tMultiplier * tMyCriterionGradientSample[tIndex];
            }

            ++tCriterionValIter;
        }
    }

private:
    double mStdDevMultiplier; /*!< standard deviation multiplier */
    std::string mOperationName; /*!< user defined function name */
    std::string mFunctionIdentifier; /*!< function identifier */
    std::string mOutputGradientArgumentName; /*!< output criterion gradient argument name */
    Plato::data::layout_t mCriteriaValueDataLayout; /*!< criteria value data layout */
    Plato::data::layout_t mCriteriaGradientDataLayout; /*!< criteria gradient data layout */
    std::vector<Plato::LocalArg> mLocalArguments; /*!< input/output shared data set */

    std::map<std::string, double> mCriterionSamplesArgNameToProbability; /*!< criterion value samples to probability map */
    std::map<std::string, double> mCriterionGradSamplesArgNameToProbability; /*!< criterion gradient samples to probability map */
    std::map<std::string, std::string> mCriterionValueStatisticsToOutputName; /*!< criterion value statistics to output argument name map */
    std::map<std::string, std::string> mCriterionGradientStatisticsToOutputName; /*!< criterion gradient statistics to output argument name map */
};
// class MeanPlusVarianceGradient

}
// namespace Plato

namespace MeanPlusVarianceMeasureTest
{

TEST(PlatoTest, MeanPlusVarianceMeasure)
{
    Plato::InputData tOperations("Operation");
    tOperations.add<std::string>("Function", "MeanPlusStdDev");
    tOperations.add<std::string>("Name", "Stochastic Objective Value");
    tOperations.add<std::string>("Layout", "Scalar");

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
    tOutput1.add<std::string>("Statistic", "mean");
    tOutput1.add<std::string>("ArgumentName", "objective_Mean");
    tOperations.add<Plato::InputData>("Output", tOutput1);
    Plato::InputData tOutput2("Output");
    tOutput2.add<std::string>("Statistic", "std_dev");
    tOutput2.add<std::string>("ArgumentName", "objective_Std_Dev");
    tOperations.add<Plato::InputData>("Output", tOutput2);
    Plato::InputData tOutput3("Output");
    tOutput3.add<std::string>("Statistic", "mean_plus_1_std_dev");
    tOutput3.add<std::string>("ArgumentName", "objective_Mean_Plus_1_StdDev");
    tOperations.add<Plato::InputData>("Output", tOutput3);
    Plato::InputData tOutput4("Output");
    tOutput4.add<std::string>("Statistic", "mean_plus_5_std_dev");
    tOutput4.add<std::string>("ArgumentName", "objective_Mean_Plus_5_StdDev");
    tOperations.add<Plato::InputData>("Output", tOutput4);

    // TEST THAT INPUT DATA IS PARSED
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    Plato::MeanPlusVarianceMeasure tOperation(&tPlatoApp, tOperations);

    // TEST DATA
    ASSERT_EQ(Plato::data::SCALAR, tOperation.getDataLayout());
    ASSERT_STREQ("Stochastic Objective Value", tOperation.getOperationName().c_str());
    ASSERT_STREQ("MeanPlusStdDev", tOperation.getFunctionIdentifier().c_str());

    std::vector<Plato::LocalArg> tLocalArguments;
    tOperation.getArguments(tLocalArguments);
    ASSERT_EQ(7u, tLocalArguments.size());
    std::vector<std::string> tArgumentNames =
        { "sierra_sd1_lc1_objective", "sierra_sd1_lc2_objective", "sierra_sd1_lc3_objective", "objective_Mean",
                "objective_Std_Dev", "objective_Mean_Plus_1_StdDev", "objective_Mean_Plus_5_StdDev" };
    for(size_t tIndex = 0; tIndex < tArgumentNames.size(); tIndex++)
    {
        bool tFoundGoldValue = std::find(tArgumentNames.begin(), tArgumentNames.end(), tLocalArguments[tIndex].mName)
                != tArgumentNames.end();
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

    ASSERT_STREQ("objective_Mean", tOperation.getOutputArgument("MEAN").c_str());
    ASSERT_STREQ("objective_Std_Dev", tOperation.getOutputArgument("STD_DEV").c_str());
    ASSERT_STREQ("objective_Mean_Plus_1_StdDev", tOperation.getOutputArgument("MEAN_PLUS_1_STD_DEV").c_str());
    ASSERT_STREQ("objective_Mean_Plus_5_StdDev", tOperation.getOutputArgument("MEAN_PLUS_5_STD_DEV").c_str());
    ASSERT_THROW(tOperation.getOutputArgument("MEAN_PLUS_2_STDDEV"), std::runtime_error);
}

TEST(PlatoTest, MeanPlusVarianceGradient)
{
    Plato::InputData tOperations("Operation");
    tOperations.add<std::string>("Function", "MeanPlusStdDevGradient");

    Plato::InputData tCriteriaValue("CriteriaValue");
    tCriteriaValue.add<std::string>("Layout", "Scalar");
    Plato::InputData tInput1("Input");
    tInput1.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective");
    tInput1.add<std::string>("Probability", "0.5");
    tCriteriaValue.add<Plato::InputData>("Input", tInput1);
    Plato::InputData tInput2("Input");
    tInput2.add<std::string>("ArgumentName", "sierra_sd1_lc2_objective");
    tInput2.add<std::string>("Probability", "0.25");
    tCriteriaValue.add<Plato::InputData>("Input", tInput2);
    Plato::InputData tInput3("Input");
    tInput3.add<std::string>("ArgumentName", "sierra_sd1_lc3_objective");
    tInput3.add<std::string>("Probability", "0.25");
    tCriteriaValue.add<Plato::InputData>("Input", tInput3);

    Plato::InputData tOutput1("Output");
    tOutput1.add<std::string>("Statistic", "mean");
    tOutput1.add<std::string>("ArgumentName", "objective_mean");
    tCriteriaValue.add<Plato::InputData>("Output", tOutput1);
    Plato::InputData tOutput2("Output");
    tOutput2.add<std::string>("Statistic", "std_dev");
    tOutput2.add<std::string>("ArgumentName", "objective_std_dev");
    tCriteriaValue.add<Plato::InputData>("Output", tOutput2);
    tOperations.add<Plato::InputData>("CriteriaValue", tCriteriaValue);

    Plato::InputData tCriteriaGradient("CriteriaGradient");
    tCriteriaGradient.add<std::string>("Layout", "Nodal Field");
    Plato::InputData tInput11("Input");
    tInput11.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective_gradient");
    tInput11.add<std::string>("Probability", "0.5");
    tCriteriaGradient.add<Plato::InputData>("Input", tInput11);
    Plato::InputData tInput12("Input");
    tInput12.add<std::string>("ArgumentName", "sierra_sd1_lc2_objective_gradient");
    tInput12.add<std::string>("Probability", "0.25");
    tCriteriaGradient.add<Plato::InputData>("Input", tInput12);
    Plato::InputData tInput13("Input");
    tInput13.add<std::string>("ArgumentName", "sierra_sd1_lc3_objective_gradient");
    tInput13.add<std::string>("Probability", "0.25");
    tCriteriaGradient.add<Plato::InputData>("Input", tInput13);

    Plato::InputData tOutput11("Output");
    tOutput11.add<std::string>("Statistic", "mean_plus_1_std_dev");
    tOutput11.add<std::string>("ArgumentName", "objective_mean_plus_1_std_dev_gradient");
    tCriteriaGradient.add<Plato::InputData>("Output", tOutput11);
    tOperations.add<Plato::InputData>("CriteriaGradient", tCriteriaGradient);

    // TEST THAT INPUT DATA IS PARSED
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    Plato::MeanPlusVarianceGradient tOperation(&tPlatoApp, tOperations);

    // TEST DATA
    ASSERT_EQ(Plato::data::SCALAR, tOperation.getCriteriaValueDataLayout());
    ASSERT_STREQ("Stochastic Criterion Gradient", tOperation.getOperationName().c_str()); // using default function name
    ASSERT_STREQ("MeanPlusStdDevGradient", tOperation.getFunctionIdentifier().c_str());

    std::vector<Plato::LocalArg> tLocalArguments;
    tOperation.getArguments(tLocalArguments);
    ASSERT_EQ(9u, tLocalArguments.size());
    std::vector<std::string> tArgumentNames =
        { "sierra_sd1_lc1_objective", "sierra_sd1_lc2_objective", "sierra_sd1_lc3_objective", "objective_mean",
                "objective_std_dev", "sierra_sd1_lc1_objective_gradient", "sierra_sd1_lc2_objective_gradient",
                "sierra_sd1_lc3_objective_gradient", "objective_mean_plus_1_std_dev_gradient" };
    for(size_t tIndex = 0; tIndex < tArgumentNames.size(); tIndex++)
    {
        bool tFoundGoldValue = std::find(tArgumentNames.begin(), tArgumentNames.end(), tLocalArguments[tIndex].mName)
                != tArgumentNames.end();
        ASSERT_TRUE(tFoundGoldValue);
    }

    double tTolerance = 1e-6;
    ASSERT_NEAR(0.5, tOperation.getCriterionValueProbability("sierra_sd1_lc1_objective"), tTolerance);
    ASSERT_NEAR(0.25, tOperation.getCriterionValueProbability("sierra_sd1_lc2_objective"), tTolerance);
    ASSERT_NEAR(0.25, tOperation.getCriterionValueProbability("sierra_sd1_lc3_objective"), tTolerance);
    ASSERT_THROW(tOperation.getCriterionValueProbability("sierra_sd1_lc4_objective"), std::runtime_error);
    ASSERT_THROW(tOperation.getCriterionValueProbability("sierra_sd1_lc1_objective_gradient"), std::runtime_error);
    ASSERT_NEAR(0.5, tOperation.getCriterionGradientProbability("sierra_sd1_lc1_objective_gradient"), tTolerance);
    ASSERT_NEAR(0.25, tOperation.getCriterionGradientProbability("sierra_sd1_lc2_objective_gradient"), tTolerance);
    ASSERT_NEAR(0.25, tOperation.getCriterionGradientProbability("sierra_sd1_lc3_objective_gradient"), tTolerance);
    ASSERT_THROW(tOperation.getCriterionGradientProbability("sierra_sd1_lc4_objective_gradient"), std::runtime_error);
    ASSERT_THROW(tOperation.getCriterionGradientProbability("sierra_sd1_lc1_objective"), std::runtime_error);

    double tMultiplier = tOperation.getStandardDeviationMultipliers();
    ASSERT_NEAR(1.0, tMultiplier, tTolerance);

    ASSERT_STREQ("objective_mean", tOperation.getCriterionValueOutputArgument("MEAN").c_str());
    ASSERT_STREQ("objective_std_dev", tOperation.getCriterionValueOutputArgument("STD_DEV").c_str());
    ASSERT_THROW(tOperation.getCriterionValueOutputArgument("MEAN_PLUS_2_STDDEV"), std::runtime_error);
    ASSERT_STREQ("objective_mean_plus_1_std_dev_gradient", tOperation.getCriterionGradientOutputArgument("MEAN_PLUS_1_STD_DEV").c_str());
    ASSERT_THROW(tOperation.getCriterionGradientOutputArgument("MEAN_PLUS_2_STDDEV"), std::runtime_error);
}

TEST(PlatoTest, MeanPlusVarianceGradient_checkInputProbabilityValues_Error1)
{
    Plato::InputData tOperations("Operation");
    tOperations.add<std::string>("Function", "MeanPlusStdDevGradient");
    tOperations.add<std::string>("Name", "Stochastic Objective Gradient");

    Plato::InputData tCriteriaValue("CriteriaValue");
    tCriteriaValue.add<std::string>("Layout", "Scalar");
    Plato::InputData tInput1("Input");
    tInput1.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective");
    tInput1.add<std::string>("Probability", "0.5");
    tCriteriaValue.add<Plato::InputData>("Input", tInput1);

    Plato::InputData tOutput1("Output");
    tOutput1.add<std::string>("Statistic", "mean");
    tOutput1.add<std::string>("ArgumentName", "objective_mean");
    tCriteriaValue.add<Plato::InputData>("Output", tOutput1);
    Plato::InputData tOutput2("Output");
    tOutput2.add<std::string>("Statistic", "std_dev");
    tOutput2.add<std::string>("ArgumentName", "objective_std_dev");
    tCriteriaValue.add<Plato::InputData>("Output", tOutput2);
    tOperations.add<Plato::InputData>("CriteriaValue", tCriteriaValue);

    Plato::InputData tCriteriaGradient("CriteriaGradient");
    tCriteriaGradient.add<std::string>("Layout", "Nodal Field");
    Plato::InputData tInput11("Input");
    tInput11.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective_gradient");
    tInput11.add<std::string>("Probability", "0.45");
    tCriteriaGradient.add<Plato::InputData>("Input", tInput11);

    Plato::InputData tOutput11("Output");
    tOutput11.add<std::string>("Statistic", "mean_plus_1_std_dev");
    tOutput11.add<std::string>("ArgumentName", "objective_mean_plus_1_std_dev_gradient");
    tCriteriaGradient.add<Plato::InputData>("Output", tOutput11);
    tOperations.add<Plato::InputData>("CriteriaGradient", tCriteriaGradient);

    // TEST ERROR - SAMPLE PROBABILITIES DO NOT MATCH. CRITERIA VALUE AND GRADIENT PROBABILITIES ARE EXPECTED TO BE THE SAME
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    ASSERT_THROW(Plato::MeanPlusVarianceGradient tOperation(&tPlatoApp, tOperations), std::runtime_error);
}

TEST(PlatoTest, MeanPlusVarianceGradient_checkInputProbabilityValues_Error2)
{
    Plato::InputData tOperations("Operation");
    tOperations.add<std::string>("Function", "MeanPlusStdDevGradient");
    tOperations.add<std::string>("Name", "Stochastic Objective Gradient");

    Plato::InputData tCriteriaValue("CriteriaValue");
    tCriteriaValue.add<std::string>("Layout", "Scalar");
    Plato::InputData tInput1("Input");
    tInput1.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective");
    tInput1.add<std::string>("Probability", "0.5");
    tCriteriaValue.add<Plato::InputData>("Input", tInput1);

    Plato::InputData tOutput1("Output");
    tOutput1.add<std::string>("Statistic", "mean");
    tOutput1.add<std::string>("ArgumentName", "objective_mean");
    tCriteriaValue.add<Plato::InputData>("Output", tOutput1);
    Plato::InputData tOutput2("Output");
    tOutput2.add<std::string>("Statistic", "std_dev");
    tOutput2.add<std::string>("ArgumentName", "objective_std_dev");
    tCriteriaValue.add<Plato::InputData>("Output", tOutput2);
    tOperations.add<Plato::InputData>("CriteriaValue", tCriteriaValue);

    Plato::InputData tCriteriaGradient("CriteriaGradient");
    tCriteriaGradient.add<std::string>("Layout", "Nodal Field");
    Plato::InputData tInput11("Input");
    tInput11.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective_gradient");
    tInput11.add<std::string>("Probability", "0.5");
    Plato::InputData tInput12("Input");
    tInput12.add<std::string>("ArgumentName", "sierra_sd1_lc2_objective_gradient");
    tInput12.add<std::string>("Probability", "0.25");
    tCriteriaGradient.add<Plato::InputData>("Input", tInput12);
    tCriteriaGradient.add<Plato::InputData>("Input", tInput11);

    Plato::InputData tOutput11("Output");
    tOutput11.add<std::string>("Statistic", "mean_plus_1_std_dev");
    tOutput11.add<std::string>("ArgumentName", "objective_mean_plus_1_std_dev_gradient");
    tCriteriaGradient.add<Plato::InputData>("Output", tOutput11);
    tOperations.add<Plato::InputData>("CriteriaGradient", tCriteriaGradient);

    // TEST ERROR - MISMATCH IN NUMBER OF CRITERIA VALUE AND CRITERIA GRADIENT SAMPLES. THE NUMBER OF SAMPLES SHOULD BE THE SAME
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    ASSERT_THROW(Plato::MeanPlusVarianceGradient tOperation(&tPlatoApp, tOperations), std::runtime_error);
}

TEST(PlatoTest, MeanPlusVarianceGradient_parseFunction_Error)
{
    Plato::InputData tOperations("Operation");
    tOperations.add<std::string>("Function", "");
    tOperations.add<std::string>("Name", "Stochastic Objective Gradient");

    // TEST ERROR - FUNCTION NAME IS NOT DEFINED
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    ASSERT_THROW(Plato::MeanPlusVarianceGradient tOperation(&tPlatoApp, tOperations), std::runtime_error);
}

TEST(PlatoTest, MeanPlusVarianceGradient_parseCriteriaValueDataLayout_Error)
{
    Plato::InputData tOperations("Operation");
    tOperations.add<std::string>("Function", "MeanPlusStdDevGradient");

    Plato::InputData tCriteriaValue("CriteriaValue");
    Plato::InputData tInput1("Input");
    tInput1.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective");
    tInput1.add<std::string>("Probability", "0.5");
    tCriteriaValue.add<Plato::InputData>("Input", tInput1);
    Plato::InputData tInput2("Input");
    tInput2.add<std::string>("ArgumentName", "sierra_sd1_lc2_objective");
    tInput2.add<std::string>("Probability", "0.25");
    tCriteriaValue.add<Plato::InputData>("Input", tInput2);
    Plato::InputData tInput3("Input");
    tInput3.add<std::string>("ArgumentName", "sierra_sd1_lc3_objective");
    tInput3.add<std::string>("Probability", "0.25");
    tCriteriaValue.add<Plato::InputData>("Input", tInput3);

    Plato::InputData tOutput1("Output");
    tOutput1.add<std::string>("Statistic", "mean");
    tOutput1.add<std::string>("ArgumentName", "objective_mean");
    tCriteriaValue.add<Plato::InputData>("Output", tOutput1);
    Plato::InputData tOutput2("Output");
    tOutput2.add<std::string>("Statistic", "std_dev");
    tOutput2.add<std::string>("ArgumentName", "objective_std_dev");
    tCriteriaValue.add<Plato::InputData>("Output", tOutput2);
    tOperations.add<Plato::InputData>("CriteriaValue", tCriteriaValue);

    Plato::InputData tCriteriaGradient("CriteriaGradient");
    tCriteriaGradient.add<std::string>("Layout", "Nodal Field");
    Plato::InputData tInput11("Input");
    tInput11.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective_gradient");
    tInput11.add<std::string>("Probability", "0.5");
    tCriteriaGradient.add<Plato::InputData>("Input", tInput11);
    Plato::InputData tInput12("Input");
    tInput12.add<std::string>("ArgumentName", "sierra_sd1_lc2_objective_gradient");
    tInput12.add<std::string>("Probability", "0.25");
    tCriteriaGradient.add<Plato::InputData>("Input", tInput12);
    Plato::InputData tInput13("Input");
    tInput13.add<std::string>("ArgumentName", "sierra_sd1_lc3_objective_gradient");
    tInput13.add<std::string>("Probability", "0.25");
    tCriteriaGradient.add<Plato::InputData>("Input", tInput13);

    Plato::InputData tOutput11("Output");
    tOutput11.add<std::string>("Statistic", "mean_plus_1_std_dev");
    tOutput11.add<std::string>("ArgumentName", "objective_mean_plus_1_std_dev_gradient");
    tCriteriaGradient.add<Plato::InputData>("Output", tOutput11);
    tOperations.add<Plato::InputData>("CriteriaGradient", tCriteriaGradient);

    // TEST ERROR - CRITERION VALUE DATA LAYOUT IS NOT DEFINED
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    ASSERT_THROW(Plato::MeanPlusVarianceGradient tOperation(&tPlatoApp, tOperations), std::runtime_error);
}

TEST(PlatoTest, MeanPlusVarianceGradient_getMyProbability_Error)
{
    Plato::InputData tOperations("Operation");
    tOperations.add<std::string>("Function", "MeanPlusStdDevGradient");
    tOperations.add<std::string>("Name", "Stochastic Objective Gradient");

    Plato::InputData tCriteriaValue("CriteriaValue");
    tCriteriaValue.add<std::string>("Layout", "Scalar");
    Plato::InputData tInput1("Input");
    tInput1.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective");
    tInput1.add<std::string>("Probability", "-0.5");
    tCriteriaValue.add<Plato::InputData>("Input", tInput1);

    Plato::InputData tOutput1("Output");
    tOutput1.add<std::string>("Statistic", "mean");
    tOutput1.add<std::string>("ArgumentName", "objective_mean");
    tCriteriaValue.add<Plato::InputData>("Output", tOutput1);
    Plato::InputData tOutput2("Output");
    tOutput2.add<std::string>("Statistic", "std_dev");
    tOutput2.add<std::string>("ArgumentName", "objective_std_dev");
    tCriteriaValue.add<Plato::InputData>("Output", tOutput2);
    tOperations.add<Plato::InputData>("CriteriaValue", tCriteriaValue);

    Plato::InputData tCriteriaGradient("CriteriaGradient");
    tCriteriaGradient.add<std::string>("Layout", "Nodal Field");
    Plato::InputData tInput11("Input");
    tInput11.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective_gradient");
    tInput11.add<std::string>("Probability", "0.5");
    tCriteriaGradient.add<Plato::InputData>("Input", tInput11);

    Plato::InputData tOutput11("Output");
    tOutput11.add<std::string>("Statistic", "mean_plus_1_std_dev");
    tOutput11.add<std::string>("ArgumentName", "objective_mean_plus_1_std_dev_gradient");
    tCriteriaGradient.add<Plato::InputData>("Output", tOutput11);
    tOperations.add<Plato::InputData>("CriteriaGradient", tCriteriaGradient);

    // TEST ERROR - NEGATIVE PROBABILITY
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    ASSERT_THROW(Plato::MeanPlusVarianceGradient tOperation(&tPlatoApp, tOperations), std::runtime_error);
}

}
// namespace MeanPlusVarianceMeasureTest
