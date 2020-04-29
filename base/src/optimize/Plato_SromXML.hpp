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
 * Plato_SromXML.hpp
 *
 *  Created on: June 18, 2019
 */

#pragma once

#include "Plato_SromLoadUtils.hpp"
#include "Plato_SromMaterialUtils.hpp"

namespace Plato
{

namespace srom
{

/******************************************************************************//**
 * \struct OutputMetaData
 * \brief Output metadata for Stochastic Reduced Order Model (SROM) problem with random loads.
**********************************************************************************/
struct OutputMetaData
{
    std::vector<Plato::srom::RandomLoadCase> mLoadCases; /*!< set of random load cases */
    std::vector<Plato::srom::RandomMaterialCase> mMaterialCases; /*!< set of random material cases */
};
// struct OutputMetaData

/******************************************************************************//**
 * \struct InputMetaData
 * \brief Input metadata for Stochastic Reduced Order Model (SROM) problem with random loads.
**********************************************************************************/
struct InputMetaData
{
private:
    /*!< defines non-deterministic use case: 1) load, 2) material, and 3) mix (load and material) */
    std::string mUseCase;

    std::vector<Plato::srom::Load> mLoads; /*!< set of loads */
    std::vector<Plato::srom::Material> mMaterials; /*!< set of materials */

public:
    /******************************************************************************//**
     * \fn usecase
     * \brief Return use case tag.
     * \return use case tag
    **********************************************************************************/
    std::string usecase() const
    {
        return mUseCase;
    }

    /******************************************************************************//**
     * \fn loads
     * \brief Return set of loads.
     * \return set of loads
    **********************************************************************************/
    std::vector<Plato::srom::Load> loads() const
    {
        return mLoads;
    }

    /******************************************************************************//**
     * \fn loads
     * \brief Return set of materials.
     * \return set of materials
    **********************************************************************************/
    std::vector<Plato::srom::Material> materials() const
    {
        return mMaterials;
    }

    /******************************************************************************//**
     * \fn usecase
     * \brief Set use case tag. Options are:\n
     *   -# load: use case only has non-deterministic loads,
     *   -# material: use case only has non-deterministic materials, and
     *   -# mix: use case has non-deterministic loads and materials.
     * \param[in] use case tag
    **********************************************************************************/
    void usecase(const std::string& aUseCase)
    {
        mUseCase = Plato::srom::tolower(aUseCase);
    }

    /******************************************************************************//**
     * \fn loads
     * \brief Initialize set of loads.
     * \param [in] aLoads set of loads
    **********************************************************************************/
    void loads(const std::vector<Plato::srom::Load>& aLoads)
    {
        mLoads = aLoads;
    }

    /******************************************************************************//**
     * \fn loads
     * \brief Initialize set of materials.
     * \param [in] aMaterials set of materials
    **********************************************************************************/
    void materials(const std::vector<Plato::srom::Material>& aMaterials)
    {
        mMaterials = aMaterials;
    }
};
// struct InputMetaData

/******************************************************************************//**
 * \fn build_sroms
 * \brief Build Stochastic Reduced Order Models (SROMs)
 * \param [in] aInput  input metadata
 * \param [in] aOutput output metadata
**********************************************************************************/
inline void build_sroms
(const Plato::srom::InputMetaData & aInput,
 Plato::srom::OutputMetaData & aOutput)
{
    if(aInput.usecase() == "load")
    {
        Plato::srom::build_load_sroms(aInput.loads(), aOutput.mLoadCases);
    }
    else if(aInput.usecase() == "material")
    {
        Plato::srom::build_material_sroms(aInput.materials(), aOutput.mMaterialCases);
    }
    else
    {
        std::ostringstream tMsg;
        tMsg << "Build SROMs: Input use case '" << aInput.usecase() << "' is not supported. "
            << "Supported options are: 'load' and 'material'.";
        THROWERR(tMsg.str().c_str())
    }
}
// function build_sroms

}
// namespace srom

}
// namespace Plato
