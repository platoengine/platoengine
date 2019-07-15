/*
 * Plato_ComputeVolume.hpp
 *
 *  Created on: Jun 28, 2019
 */

#pragma once

#include "Plato_LocalOperation.hpp"

class PlatoApp;

namespace Plato
{

class InputData;
class PenaltyModel;

/******************************************************************************//**
 * @brief Compute current structural volume
**********************************************************************************/
class ComputeVolume : public Plato::LocalOp
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aPlatoApp PLATO application
     * @param [in] aNode input XML data
    **********************************************************************************/
    ComputeVolume(PlatoApp* aPlatoApp, Plato::InputData& aNode);

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~ComputeVolume();

    /******************************************************************************//**
     * @brief perform local operation - compute structural volume
    **********************************************************************************/
    void operator()();

    /******************************************************************************//**
     * @brief Return local operation's argument list
     * @param [out] aLocalArgs argument list
    **********************************************************************************/
    void getArguments(std::vector<Plato::LocalArg>& aLocalArgs);

private:
    std::string mTopologyName; /*!< topology field argument name */
    std::string mVolumeName;  /*!< volume argument name */
    std::string mGradientName; /*!< volume gradient argument name */
    Plato::PenaltyModel* mPenaltyModel; /*!< material penalty model, e.g. SIMP */
};
// class ComputeVolume

}
// namespace Plato
