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
 * Plato_MeanPlusVarianceMeasure.hpp
 *
 *  Created on: Jul 5, 2019
 */

#pragma once

#include "Plato_LocalOperation.hpp"

class PlatoApp;

namespace Plato
{

class InputData;

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
    MeanPlusVarianceMeasure(PlatoApp* aPlatoApp, const Plato::InputData& aOperationNode);

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~MeanPlusVarianceMeasure();

    /******************************************************************************//**
     * @brief Compute statistics
    **********************************************************************************/
    void operator()();

    /******************************************************************************//**
     * @brief Get input and output arguments associated with the local operation
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
     * @brief Input and output data layout.
     * return data layout, valid options are element field, node field and scalar value
    **********************************************************************************/
    Plato::data::layout_t getDataLayout() const;

    /******************************************************************************//**
     * @brief Return list of standard deviation multipliers in ascending order
     * @return standard deviation multipliers
    **********************************************************************************/
    std::vector<double> getStandardDeviationMultipliers() const;

    /******************************************************************************//**
     * @brief Return probability associated with input argument name
     * @param [in] aInput input argument name
     * @return probability
    **********************************************************************************/
    double getProbability(const std::string& aInput) const;

    /******************************************************************************//**
     * @brief Return output argument name associated with statistic measure
     * @param [in] aInput upper case statistic's measure name, valid options are: MEAN, STDDEV
     *   and MEAN_PLUS_#_STDDEV, where # denotes the standard deviation multiplier
     * @return output argument name
    **********************************************************************************/
    std::string getOutputArgument(const std::string& aInput) const;

private:
    /******************************************************************************//**
     * @brief Initialize mean plus variance local operation
     * @param [in] aOperationNode input data associated with statistics XML node
    **********************************************************************************/
    void initialize(const Plato::InputData& aOperationNode);

    /******************************************************************************//**
     * @brief Parse user defined function name
     * @param [in] aOperationNode input data associated with the statistics XML node
    **********************************************************************************/
    void parseName(const Plato::InputData& aOperationNode);

    /******************************************************************************//**
     * @brief Parse function identifier
     * @param [in] aOperationNode input data associated with the statistics XML node
    **********************************************************************************/
    void parseFunction(const Plato::InputData& aOperationNode);

    /******************************************************************************//**
     * @brief Parse data layout from input file
     * @param [in] aOperationNode input data associated with the statistics XML node
    **********************************************************************************/
    void parseDataLayout(const Plato::InputData& aOperationNode);

    /******************************************************************************//**
     * @brief Parse input arguments from input files
     * @param [in] aOperationNode input data associated with the statistics XML node
    **********************************************************************************/
    void parseInputs(const Plato::InputData& aOperationNode);

    /******************************************************************************//**
     * @brief Parse output arguments from input files
     * @param [in] aOperationNode input data associated with the statistics XML node
    **********************************************************************************/
    void parseOutputs(const Plato::InputData& aOperationNode);

    /******************************************************************************//**
     * @brief Set output argument to standard deviation (i.e. sigma) multiplier map
     * @param [in] aStatisticMeasure statistic measure name
    **********************************************************************************/
    void setMySigmaValue(const std::string & aStatisticMeasure);

    /******************************************************************************//**
     * @brief Convert string standard deviation multiplier to double
     * @param [in] aInput string standard deviation multiplier
     * @return double standard deviation multiplier
    **********************************************************************************/
    double getMySigmaValue(const std::string& aInput);

    /******************************************************************************//**
     * @brief Set statistic to output argument map
     * @param [in] aStatisticMeasure statistic measure name
     * @param [in] aStatisticMeasure output argument name
     **********************************************************************************/
    void setStatisticsToOutputArgumentNameMap(const std::string & aStatisticMeasure,
                                              const std::string & aOutputArgumentName);

    /******************************************************************************//**
     * @brief Return probability measure
     * @param [in] aInputNode input data XML node
     * @return return probability measure
    **********************************************************************************/
    double getMyProbability(const Plato::InputData& aInputNode);

    /******************************************************************************//**
     * @brief Add argument to local argument list
     * @param [in] aArgumentName argument name
    **********************************************************************************/
    void setMyLocalArgument(const std::string & aArgumentName);

    /******************************************************************************//**
     * @brief Compute mean measure
    **********************************************************************************/
    void computeMean();

    /******************************************************************************//**
     * @brief Compute standard deviation measure
    **********************************************************************************/
    void computeStandardDeviation();

    /******************************************************************************//**
     * @brief Compute mean plus standard deviation measure
    **********************************************************************************/
    void computeMeanPlusStdDev();

    /******************************************************************************//**
     * @brief Compute mean plus standard deviation measure for a scalar value
    **********************************************************************************/
    void computeMeanPlusStdDevScalarValue();

    /******************************************************************************//**
     * @brief Compute mean plus standard deviation measure for a node field
    **********************************************************************************/
    void computeMeanPlusStdDevNodeField();

    /******************************************************************************//**
     * @brief Compute mean plus standard deviation measure for an element field
    **********************************************************************************/
    void computeMeanPlusStdDevElementField();

private:
    std::string mOperationName; /*!< user defined function name */
    std::string mFunctionIdentifier; /*!< function identifier */
    Plato::data::layout_t mDataLayout; /*!< output/input data layout */
    std::vector<Plato::LocalArg> mLocalArguments; /*!< input/output shared data set */

    std::map<std::string, double> mInArgumentToProbability; /*!< sample to probability map */
    std::map<std::string, double> mOutArgumentToStdDevMultiplier; /*!< output argument name to standard deviation multiplier map */
    std::map<std::string, std::string> mStatisticsToOutArgument; /*!< statistics to output argument name map */
};
// class MeanPlusVarianceMeasure

}
// namespace Plato
