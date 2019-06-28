/*
 * Plato_UpdateProblem.hpp
 *
 *  Created on: Jun 28, 2019
 */

#pragma once

#include "Plato_LocalOperation.hpp"

class PlatoApp;

namespace Plato
{

class InputData;

/******************************************************************************//**
 * @brief Manages continuation operations on application-specific data.
 **********************************************************************************/
class UpdateProblem : public Plato::LocalOp
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aPlatoApp PLATO application
     * @param [in] aNode input XML data
    **********************************************************************************/
    UpdateProblem(PlatoApp* aPlatoApp, Plato::InputData& aNode);

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~UpdateProblem();

    /******************************************************************************//**
     * @brief perform local operation - update problem
    **********************************************************************************/
    void operator()();

    /******************************************************************************//**
     * @brief Return local operation's argument list
     * @param [out] aLocalArgs argument list
    **********************************************************************************/
    void getArguments(std::vector<Plato::LocalArg>& aLocalArgs);
};
// class Filter

}
// namespace Plato
