/*
 * Plato_SetLowerBounds.hpp
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
 * @brief Compute design variables' lower bound
**********************************************************************************/
class SetLowerBounds : public Plato::LocalOp
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aPlatoApp PLATO application
     * @param [in] aNode input XML data
    **********************************************************************************/
    SetLowerBounds(PlatoApp* aPlatoApp, Plato::InputData& aNode);

    /******************************************************************************//**
     * @brief perform local operation - compute upper bounds
    **********************************************************************************/
    void operator()();

    /******************************************************************************//**
     * @brief Return local operation's argument list
     * @param [out] aLocalArgs argument list
    **********************************************************************************/
    void getArguments(std::vector<Plato::LocalArg>& aLocalArgs);

private:
    int mOutputSize; /*!< output field length */

    std::string mInputName; /*!< input argument name */
    std::string mOutputName; /*!< output argument name */
    std::string mDiscretization; /*!< topology/design representation, levelset or density */
    Plato::data::layout_t mOutputLayout; /*!< output field data layout */

    std::vector<int> mFixedBlocks; /*!< fixed blocks' identifiers */
    std::vector<int> mFixedSidesets; /*!< fixed blocks' sideset identifiers */
    std::vector<int> mFixedNodesets; /*!< fixed blocks' nodeset identifiers */
};
// class SetLowerBounds;

}
// namespace Plato
