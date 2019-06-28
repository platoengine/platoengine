/*
 * Plato_Filter.hpp
 *
 *  Created on: Jun 28, 2019
 */

#pragma once

#include "Plato_LocalOperation.hpp"

class PlatoApp;

namespace Plato
{

class InputData;
class AbstractFilter;

/******************************************************************************//**
 * @brief Manages application of filter to a quantity of interest
 **********************************************************************************/
class Filter : public Plato::LocalOp
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aPlatoApp PLATO application
     * @param [in] aNode input XML data
    **********************************************************************************/
    Filter(PlatoApp* aPlatoApp, Plato::InputData& aNode);

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~Filter();

    /******************************************************************************//**
     * @brief perform local operation - apply filter
    **********************************************************************************/
    void operator()();

    /******************************************************************************//**
     * @brief Return local operation's argument list
     * @param [out] aLocalArgs argument list
    **********************************************************************************/
    void getArguments(std::vector<Plato::LocalArg>& aLocalArgs);

private:
    Plato::AbstractFilter* mFilter; /*!< Kernel filter interface */
    std::string mInputToFilterName; /*!< input argument name */
    std::string mInputBaseFieldName; /*!< input base field argument name */
    std::string mOutputFromFilterName; /*!< output argument name */
    bool mIsGradient; /*!< is the gradient the input argument to the filter */
};
// class Filter;

}
