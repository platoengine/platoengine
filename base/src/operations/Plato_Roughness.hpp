/*
 * Plato_Roughness.hpp
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
 * @brief Compute surface roughness
 **********************************************************************************/
class Roughness : public Plato::LocalOp
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aPlatoApp PLATO application
     * @param [in] aNode input XML data
    **********************************************************************************/
    Roughness(PlatoApp* aPlatoApp, Plato::InputData & aNode);

    /******************************************************************************//**
     * @brief perform local operation - compute surface roughness
    **********************************************************************************/
    void operator()();

    /******************************************************************************//**
     * @brief Return local operation's argument list
     * @param [out] aLocalArgs argument list
    **********************************************************************************/
    void getArguments(std::vector<Plato::LocalArg> & aLocalArgs);

private:
    std::string mTopologyName; /*!< topology field argument name */
    std::string mRoughnessName; /*!< roughness value argument name */
    std::string mGradientName; /*!< roughness gradient argument name */
};
// class Roughness;

}
// namespace Plato
