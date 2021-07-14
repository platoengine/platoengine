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
 * Plato_OperationMetadata.hpp
 *
 *  Created on: Jul 11, 2021
 */

#pragma once

#include <vector>
#include <string>

#include "Plato_Macros.hpp"
#include "Plato_FreeFunctions.hpp"

namespace Plato
{

namespace FixedBlock
{

enum node_type
{
    UNDEFINED = 0,      // not defined
    OPTIMIZABLE = 1,    // inside optimization region
    FIXED_BOUNDARY = 2, // between optization and fixed region
    FIXED_DOMAIN = 3    // inside fixed region
};
      
/******************************************************************************//**
 * \struct Metadata describing fixed blocks.
**********************************************************************************/
struct Metadata
{
    double mOptimizationBlockValue; /*!< optimization region values. design variables in the optimization region are set to this value. */

    std::vector<int> mBlockIDs; /*!< fixed block identification number */
    std::vector<int> mSidesetIDs; /*!< fixed sideset identification number */
    std::vector<int> mNodesetIDs; /*!< fixed nodeset identification number */
    std::vector<double> mDomainValues; /*!< fixed block domain values */
    std::vector<double> mBoundaryValues; /*!< fixed block boundary values */
    std::vector<std::string> mMaterialStates; /*!< fixed block material states */
};
// struct Metadata

/******************************************************************************//**
 * \fn get_fixed_fluid_blocks_metadata
 * \brief Return only the fixed blocks metadata belonging to fluid material states.
 * \param [in] aMetadata all fixed block metadata
 * \return fluids fixed block metadta
**********************************************************************************/
inline Plato::FixedBlock::Metadata 
get_fixed_fluid_blocks_metadata
(Plato::FixedBlock::Metadata& aMetadata)
{
    Plato::FixedBlock::Metadata tFluidFixedBlocks;
    for(auto& tMaterialState : aMetadata.mMaterialStates)
    {
        auto tLowerMaterialState = Plato::tolower(tMaterialState);
        if(tLowerMaterialState != "fluid") { continue; }
        auto tIndex = &tMaterialState - &aMetadata.mMaterialStates[0];
        tFluidFixedBlocks.mMaterialStates.push_back(tMaterialState);
        tFluidFixedBlocks.mBlockIDs.push_back(aMetadata.mBlockIDs[tIndex]);
        tFluidFixedBlocks.mDomainValues.push_back(aMetadata.mDomainValues[tIndex]);
        tFluidFixedBlocks.mBoundaryValues.push_back(aMetadata.mBoundaryValues[tIndex]);
    }
    tFluidFixedBlocks.mNodesetIDs = aMetadata.mNodesetIDs;
    tFluidFixedBlocks.mSidesetIDs = aMetadata.mSidesetIDs;
    tFluidFixedBlocks.mOptimizationBlockValue = aMetadata.mOptimizationBlockValue;
    return tFluidFixedBlocks;
}

/******************************************************************************//**
 * \fn get_fixed_solid_blocks_metadata
 * \brief Return only the fixed blocks metadata belonging to fluid material states.
 * \param [in] aMetadata all fixed block metadata
 * \return fluids fixed block metadta
**********************************************************************************/
inline Plato::FixedBlock::Metadata 
get_fixed_solid_blocks_metadata
(Plato::FixedBlock::Metadata& aMetadata)
{
    Plato::FixedBlock::Metadata tSolidFixedBlocks;
    for(auto& tMaterialState : aMetadata.mMaterialStates)
    {
        auto tLowerMaterialState = Plato::tolower(tMaterialState);
        if(tLowerMaterialState != "solid") { continue; }
        auto tIndex = &tMaterialState - &aMetadata.mMaterialStates[0];
        tSolidFixedBlocks.mMaterialStates.push_back(tMaterialState);
        tSolidFixedBlocks.mBlockIDs.push_back(aMetadata.mBlockIDs[tIndex]);
        tSolidFixedBlocks.mDomainValues.push_back(aMetadata.mDomainValues[tIndex]);
        tSolidFixedBlocks.mBoundaryValues.push_back(aMetadata.mBoundaryValues[tIndex]);
    }
    tSolidFixedBlocks.mNodesetIDs = aMetadata.mNodesetIDs;
    tSolidFixedBlocks.mSidesetIDs = aMetadata.mSidesetIDs;
    tSolidFixedBlocks.mOptimizationBlockValue = aMetadata.mOptimizationBlockValue;
    return tSolidFixedBlocks;
}

}
// namespace FixedBlock

}
// namepsace Plato