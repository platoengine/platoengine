/*
 * Plato_EnforceBounds.hpp
 *
 *  Created on: Jun 29, 2019
 */

#pragma once

#include "Plato_LocalOperation.hpp"

class PlatoApp;

namespace Plato
{

class InputData;

/******************************************************************************//**
 * @brief Enforce design variables' upper and lower bounds
**********************************************************************************/
class EnforceBounds : public Plato::LocalOp
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aPlatoApp PLATO application
     * @param [in] aNode input XML data
    **********************************************************************************/
    EnforceBounds(PlatoApp* aPlatoApp, Plato::InputData& aNode);

    /******************************************************************************//**
     * @brief perform local operation - enforce lower and upper bounds
    **********************************************************************************/
    void operator()();

    /******************************************************************************//**
     * @brief Return local operation's argument list
     * @param [out] aLocalArgs argument list
    **********************************************************************************/
    void getArguments(std::vector<Plato::LocalArg>& aLocalArgs);
private:
    std::string mLowerBoundVectorFieldName; /*!< lower bound argument name */
    std::string mUpperBoundVectorFieldName; /*!< upper bound argument name */
    std::string mTopologyFieldName; /*!< topology argument name */
};
// class EnforceBounds;

}
// namespace Plato
