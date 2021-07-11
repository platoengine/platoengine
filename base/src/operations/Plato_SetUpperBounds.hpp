/*
 //\HEADER
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
 // Questions? Contact the Plato team (plato3D-help\sandia.gov)
 //
 // *************************************************************************
 //\HEADER
 */

/*
 * Plato_SetUpperBounds.hpp
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
 * \brief Compute design variables' upper bound
**********************************************************************************/
class SetUpperBounds : public Plato::LocalOp
{
public:
    /******************************************************************************//**
     * \brief Constructor
     * \param [in] aPlatoApp PLATO application
     * \param [in] aNode input XML data
    **********************************************************************************/
    SetUpperBounds(PlatoApp* aPlatoApp, Plato::InputData& aNode);

    /******************************************************************************//**
     * \brief perform local operation - compute upper bounds
    **********************************************************************************/
    void operator()();

    /******************************************************************************//**
     * \brief Return local operation's argument list
     * \param [out] aLocalArgs argument list
    **********************************************************************************/
    void getArguments(std::vector<Plato::LocalArg> & aLocalArgs);

private:
    /******************************************************************************//**
     * \brief Parse node and side sets metadata.
     * \param [in] aNode XML metadata for this operation
    **********************************************************************************/
    void parseEntitySets(Plato::InputData& aNode);

    /******************************************************************************//**
     * \brief Parse fixed block metadata.
     * \param [in] aNode XML metadata for this operation
    **********************************************************************************/
    void parseFixedBlocks(Plato::InputData& aNode);

    /******************************************************************************//**
     * \brief Parse operation input and output arguments.
     * \param [in] aNode XML metadata for this operation
    **********************************************************************************/
    void parseOperationArguments(Plato::InputData& aNode);

private:
    int mOutputSize; /*!< output field length */

    std::string mInputArgumentName; /*!< input argument name */
    std::string mOutputArgumentName; /*!< output argument name */
    std::string mDiscretization; /*!< topology/design representation, levelset or density */
    Plato::data::layout_t mOutputLayout; /*!< output field data layout */

    std::vector<int> mFixedBlockIDs; /*!< fixed block identification number */
    std::vector<int> mFixedSidesetIDs; /*!< fixed sideset identification number */
    std::vector<int> mFixedNodesetIDs; /*!< fixed nodeset identification number */
    std::vector<double> mDomainValues; /*!< fixed block domain values */
    std::vector<double> mBoundaryValues; /*!< fixed block boundary values */
    std::vector<std::string> mMaterialStates; /*!< fixed block material states */
};
// class SetUpperBounds;

}
// namespace Plato
