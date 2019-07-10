/*
 * Plato_MeanPlusVarianceGradient.hpp
 *
 *  Created on: Jul 10, 2019
 */

#pragma once

#include "Plato_LocalOperation.hpp"

class PlatoApp;

namespace Plato
{

class InputData;

/******************************************************************************//**
 * @brief Compute mean plus standard deviation measure's gradient
**********************************************************************************/
class MeanPlusVarianceGradient : public Plato::LocalOp
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aPlatoApp PLATO application interface
     * @param [in] aOperationNode local operation input data
    **********************************************************************************/
    MeanPlusVarianceGradient(PlatoApp* aPlatoApp, const Plato::InputData& aOperationNode);

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~MeanPlusVarianceGradient();

    /******************************************************************************//**
     * @brief Perform local operation - compute mean plus standard deviation measure's gradient
    **********************************************************************************/
    void operator()();

    /******************************************************************************//**
     * @brief Get the input and output arguments associated with the local operation
     * @param [in/out] aLocalArgs local input and output arguments
    **********************************************************************************/
    void getArguments(std::vector<LocalArg>& aLocalArgs);

    /******************************************************************************//**
     * @brief Return name used by the user to identify the function.
     * @return user-defined function name
    **********************************************************************************/
    std::string getOperationName() const;

    /******************************************************************************//**
     * @brief Return function's identifier, i.e. MeanPlusStdDev.
     * @return function's identifier, the user shall use "MeanPlusStdDev" as the
     *   function identifier. the function identifier is used by the PLATO application
     *   to create the function.
    **********************************************************************************/
    std::string getFunctionIdentifier() const;

    /******************************************************************************//**
     * @brief Return criteria's data layout.
     * return data layout, the only valid option is scalar value
    **********************************************************************************/
    Plato::data::layout_t getCriterionValueDataLayout() const;

    /******************************************************************************//**
     * @brief Return criteria gradient's data layout.
     * return data layout, valid options are element and node field
    **********************************************************************************/
    Plato::data::layout_t getCriterionGradientDataLayout() const;

    /******************************************************************************//**
     * @brief Return list of standard deviation multipliers in ascending order
     * @return standard deviation multipliers
    **********************************************************************************/
    double getStandardDeviationMultipliers() const;

    /******************************************************************************//**
     * @brief Return the probability associated with a given criteria value sample
     * @param [in] aInput criteria value sample's argument name
     * @return probability
    **********************************************************************************/
    double getCriterionValueProbability(const std::string& aInput) const;

    /******************************************************************************//**
     * @brief Return the probability associated with a given criteria gradient sample
     * @param [in] aInput criteria gradient sample's argument name
     * @return probability
    **********************************************************************************/
    double getCriterionGradientProbability(const std::string& aInput) const;

    /******************************************************************************//**
     * @brief Return output argument associated with a given criterion value sample
     * @param [in] aInput output argument name
     * @return output argument name
    **********************************************************************************/
    std::string getCriterionValueOutputArgument(const std::string& aInput) const;

    /******************************************************************************//**
     * @brief Return output argument associated with a given criterion gradient sample
     * @param [in] aInput output argument name
     * @return output argument name
    **********************************************************************************/
    std::string getCriterionGradientOutputArgument(const std::string& aInput) const;

private:
    /******************************************************************************//**
     * @brief Initialize mean plus variance local operation
     * @param [in] aOperationNode operation input data
    **********************************************************************************/
    void initialize(const Plato::InputData& aOperationNode);

    /******************************************************************************//**
     * @brief Check that the criterion value and criterion gradient sample probabilities match.
    **********************************************************************************/
    void checkInputProbabilityValues();

    /******************************************************************************//**
     * @brief Parse user defined function name,. Use the default value, Stochastic Criterion
     *   Gradient, if the user does not provide a name.
     * @param [in] aOperationNode operation input data
    **********************************************************************************/
    void parseName(const Plato::InputData& aOperationNode);

    /******************************************************************************//**
     * @brief Parse function identifier
     * @param [in] aOperationNode operation input data
    **********************************************************************************/
    void parseFunction(const Plato::InputData& aOperationNode);

    /******************************************************************************//**
     * @brief Parse data layout from input file
     * @param [in] aOperationNode operation input data
     * @param [in] aCriteriaName criteria name
     **********************************************************************************/
    Plato::data::layout_t parseDataLayout(const Plato::InputData& aOperationNode, const std::string& aCriteriaName);

    /******************************************************************************//**
     * @brief Parse input arguments from XML files
     * @param [in] aDataLayout data layout for input sample set
     * @param [in] aInput input arguments
     * @param [in] aOutput input argument name to probability map
    **********************************************************************************/
    void parseInputs(const Plato::data::layout_t& aDataLayout,
                     const Plato::InputData& aInput,
                     std::map<std::string, double> & aOutput);

    /******************************************************************************//**
     * @brief Parse criteria value input arguments
     * @param [in] aOperationNode operation input data
    **********************************************************************************/
    void parseCriterionValueArguments(const Plato::InputData& aOperationNode);

    /******************************************************************************//**
     * @brief Parse criteria gradient input arguments
     * @param [in] aOperationNode operation input data
    **********************************************************************************/
    void parseCriterionGradientArguments(const Plato::InputData& aOperationNode);

    /******************************************************************************//**
     * @brief Return statistic measure, e.g. 'MEAN', 'STD_DEV' or 'MEAN_PLUS_1_STD_DEV'
     * @param [in] aOutputNode XML input data associated with the output arguments
     * @param [in] aOutputArgumentName output argument name
     * @return statistic measure string
    **********************************************************************************/
    std::string getStatisticMeasure(const Plato::InputData& aOutputNode, const std::string& aOutputArgumentName) const;

    /******************************************************************************//**
     * @brief Return output argument
     * @param [in] aOutputNode XML input data associated with the output arguments
     * @param [in] aCriterionName criterion block name, e.g. CriterionValue
     * @return output argument
    **********************************************************************************/
    std::string getOutputArgument(const Plato::InputData& aOutputNode, const std::string& aCriterionName) const;

    /******************************************************************************//**
     * @brief Parse criteria value output arguments
     * @param [in] aCriteriaNode XML input data associated with a given criteria node
    **********************************************************************************/
    void parseCriterionValueOutputs(const Plato::InputData& aCriteriaNode);

    /******************************************************************************//**
     * @brief Parse criteria gradient output arguments
     * @param [in] aCriteriaNode XML input data associated with a given criteria node
    **********************************************************************************/
    void parseCriterionGradientOutputs(const Plato::InputData& aCriteriaNode);

    /******************************************************************************//**
     * @brief Set standard deviation multiplier
     * @param [in] aStatisticMeasure string with statistic measure, e.g. 'MEAN_PLUS_1_STD_DEV'
    **********************************************************************************/
    void setMyStandardDeviationMultiplier(const std::string& aStatisticMeasure);

    /******************************************************************************//**
     * @brief Return numeric value for the probability
     * @param [in] aInput string with probability value
     * @return numeric value for the probability
    **********************************************************************************/
    double getMyStandardDeviationMultiplier(const std::string& aInput);

    /******************************************************************************//**
     * @brief Return sample probability
     * @param [in] aInputNode XML inputs associated with a given input argument
     * @return sample probability
    **********************************************************************************/
    double getMyProbability(const Plato::InputData& aInputNode);

    /******************************************************************************//**
     * @brief Add a new local argument to the set of local arguments
     * @param [in] aDataLayout data layout, e.g. SCALAR, NODE FIELD, ELEMENT FIELD
     * @param [in] aArgumentName argument name
    **********************************************************************************/
    void addLocalArgument(const Plato::data::layout_t& aDataLayout, const std::string & aArgumentName);

    /******************************************************************************//**
     * @brief Compute the gradient of the mean criterion
    **********************************************************************************/
    void computeMeanCriterionGradient();

    /******************************************************************************//**
     * @brief Compute the gradient of the mean plus standard deviation criterion
    **********************************************************************************/
    void computeMeanPlusStandardDeviationCriterionGradient();

    /******************************************************************************//**
     * @brief Compute the gradient of the mean plus standard deviation criterion -
     *     node field gradient type
    **********************************************************************************/
    void computeMeanPlusStdDevCriterionGradientNodeField();

    /******************************************************************************//**
     * @brief Compute the gradient of the mean plus standard deviation criterion -
     *     element field gradient type
    **********************************************************************************/
    void computeMeanPlusStdDevCriterionGradientElementField();

private:
    double mStdDevMultiplier; /*!< standard deviation multiplier */
    std::string mOperationName; /*!< user defined function name */
    std::string mFunctionIdentifier; /*!< function identifier */
    std::string mOutputGradientArgumentName; /*!< output criterion gradient argument name */
    Plato::data::layout_t mCriterionValueDataLayout; /*!< criteria value data layout */
    Plato::data::layout_t mCriterionGradientDataLayout; /*!< criteria gradient data layout */
    std::vector<Plato::LocalArg> mLocalArguments; /*!< input/output shared data set */

    std::map<std::string, double> mCriterionSamplesArgNameToProbability; /*!< criterion value samples to probability map */
    std::map<std::string, double> mCriterionGradSamplesArgNameToProbability; /*!< criterion gradient samples to probability map */
    std::map<std::string, std::string> mCriterionValueStatisticsToOutputName; /*!< criterion value statistics to output argument name map */
    std::map<std::string, std::string> mCriterionGradientStatisticsToOutputName; /*!< criterion gradient statistics to output argument name map */
};
// class MeanPlusVarianceGradient

}
// namespace Plato
