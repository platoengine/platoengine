/*
 * Plato_InitializeValues.hpp
 *
 *  Created on: Jun 30, 2019
 */

#pragma once

#include "Plato_LocalOperation.hpp"

class PlatoApp;

namespace Plato
{

class InputData;

/******************************************************************************//**
 * @brief Set all design variables to the same initial value
 **********************************************************************************/
class InitializeValues : public Plato::LocalOp
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aPlatoApp PLATO application
     * @param [in] aNode input XML data
    **********************************************************************************/
    InitializeValues(PlatoApp* aPlatoApp, Plato::InputData & aNode);

    /******************************************************************************//**
     * @brief perform local operation - set all design variables to the same initial value
    **********************************************************************************/
    void operator()();

    /******************************************************************************//**
     * @brief Return local operation's argument list
     * @param [out] aLocalArgs argument list
    **********************************************************************************/
    void getArguments(std::vector<Plato::LocalArg> & aLocalArgs);

private:
    std::string mValuesName; /*!< initial value argument name */
    double mValue; /*!< input value */
};
// class InitializeValues;

}
// namespace Plato
