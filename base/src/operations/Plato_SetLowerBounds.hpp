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
 * Plato_SetLowerBounds.hpp
 *
 *  Created on: Jun 28, 2019
 */

#pragma once

#include "Plato_LocalOperation.hpp"
#include "Plato_OperationMetadata.hpp"

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
    /******************************************************************************//**
     * \brief Initialize lower bound vector to optimization region value.
     * \param [in,out] aToData lower bound vector
    **********************************************************************************/
    void initializeLowerBoundVector(double* aToData);

    /******************************************************************************//**
     * \brief Update lower bound vector based on fixed entitiies for Density-Based 
     *        Topology Optimization Problems (DBTOP).
     * \param [in,out] aToData lower bound vector
    **********************************************************************************/
    void updateLowerBoundsBasedOnFixedEntitiesForDBTOP(double* aToData);

    /******************************************************************************//**
     * \brief Parse member data
     * \param [in] aNode XML metadata for this operation
    **********************************************************************************/
    void parseMemberData(Plato::InputData& aNode);

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

    /******************************************************************************//**
     * \brief Set fixed block identification number.
     * \param [in] aNode XML metadata for this operation
    **********************************************************************************/
    void setBlockIndex(Plato::InputData& aNode);

    /******************************************************************************//**
     * \brief Set nodal value for all the design variables inside the fixed block domain.
     * \param [in] aNode XML metadata for this operation
    **********************************************************************************/
    void setDomainValue(Plato::InputData& aNode);

    /******************************************************************************//**
     * \brief Set nodal value for all the design variables on the fixed block boundary.
     * \param [in] aNode XML metadata for this operation
    **********************************************************************************/
    void setBoundaryValue(Plato::InputData& aNode);

    /******************************************************************************//**
     * \brief Set fixed block material state.
     * \param [in] aNode XML metadata for this operation
    **********************************************************************************/
    void setMaterialState(Plato::InputData& aNode);

    /******************************************************************************//**
     * \brief Set lower bound vector for density-based topology optimization problems.
     * \param [in] aMetadata fixed blocks metadata
     * \param [in,out] aToData lower bound vector
    **********************************************************************************/
    void updateLowerBoundsForDensityProblems
    (const Plato::FixedBlock::Metadata& aMetadata, double* aToData);

private:
    int mOutputSize = 0; /*!< output field length */
    int mLowerBoundVectorLength = 0; /*!< lower bound vector length */

    std::string mInputArgumentName; /*!< input argument name */
    std::string mOutputArgumentName; /*!< output argument name */
    std::string mDiscretization; /*!< topology/design representation, levelset or density, default = 'density' */
    std::string mMaterialUseCase; /*!< main material state use case for the problem, default = 'solid' */
    Plato::data::layout_t mOutputLayout; /*!< output field data layout */

    Plato::FixedBlock::Metadata mFixedBlockMetadata; /*!< data describing fixed blocks */
};
// class SetLowerBounds;

}
// namespace Plato
