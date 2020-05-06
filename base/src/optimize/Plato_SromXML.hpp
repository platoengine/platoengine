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
private:
    Plato::srom::usecase mUseCase;
    std::vector<Plato::srom::RandomLoadCase> mLoadCases; /*!< set of random load cases */
    std::vector<Plato::srom::RandomMaterialCase> mMaterialCases; /*!< set of random material cases */

public:
    /******************************************************************************//**
     * \fn loadCases
     * \brief Set container of load cases.
     * \param [in] aCases load cases
    **********************************************************************************/
    void loadCases(const std::vector<Plato::srom::RandomLoadCase>& aCases)
    {
        mLoadCases = aCases;
    }

    /******************************************************************************//**
     * \fn materialCases
     * \brief Set container of material cases.
     * \param [in] aCases material cases
    **********************************************************************************/
    void materialCases(const std::vector<Plato::srom::RandomMaterialCase>& aCases)
    {
        mMaterialCases = aCases;
    }

    /******************************************************************************//**
     * \fn loadCases
     * \brief Return container of load cases.
     * \return load cases
    **********************************************************************************/
    std::vector<Plato::srom::RandomLoadCase> loadCases() const
    {
        return mLoadCases;
    }

    /******************************************************************************//**
     * \fn materialCases
     * \brief Return container of material cases.
     * \return material cases
    **********************************************************************************/
    std::vector<Plato::srom::RandomMaterialCase> materialCases() const
    {
        return mMaterialCases;
    }

    /******************************************************************************//**
     * \fn usecase
     * \brief Return use case tag.
     * \return use case tag
    **********************************************************************************/
    Plato::srom::usecase usecase() const
    {
        return mUseCase;
    }

    /******************************************************************************//**
     * \fn usecase
     * \brief Set use case tag. Options are:\n
     *   -# load: use case only has non-deterministic loads,
     *   -# material: use case only has non-deterministic materials, and
     *   -# mix: use case has non-deterministic loads and materials.
     * \param[in] use case tag
    **********************************************************************************/
    void usecase(const Plato::srom::usecase& aUseCase)
    {
        mUseCase = aUseCase;
    }
};
// struct OutputMetaData

/******************************************************************************//**
 * \struct InputMetaData
 * \brief Input metadata for Stochastic Reduced Order Model (SROM) problem with random loads.
**********************************************************************************/
struct InputMetaData
{
private:
    /*!< defines non-deterministic use case: 1) load, 2) material, and 3) load plus material */
    Plato::srom::usecase mUseCase = Plato::srom::usecase::LOAD;

    std::vector<Plato::srom::Load> mLoads; /*!< set of loads */
    std::vector<Plato::srom::Material> mMaterials; /*!< set of materials */

public:
    /******************************************************************************//**
     * \fn usecase
     * \brief Return use case tag.
     * \return use case tag
    **********************************************************************************/
    Plato::srom::usecase usecase() const
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
     * \fn materials
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
    void usecase(const Plato::srom::usecase& aUseCase)
    {
        mUseCase = aUseCase;
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
     * \fn materials
     * \brief Initialize set of materials.
     * \param [in] aMaterials set of materials
    **********************************************************************************/
    void materials(const std::vector<Plato::srom::Material>& aMaterials)
    {
        mMaterials = aMaterials;
    }

    /******************************************************************************//**
     * \fn append
     * \brief Append load.
     * \param [in] aLoad load metadata
    **********************************************************************************/
    void append(const Plato::srom::Load& aLoad)
    {
        mLoads.push_back(aLoad);
    }

    /******************************************************************************//**
     * \fn append
     * \brief Append material.
     * \param [in] aMaterial material metadata
    **********************************************************************************/
    void append(const Plato::srom::Material& aMaterial)
    {
        mMaterials.push_back(aMaterial);
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
    switch(aInput.usecase())
    {
        case Plato::srom::usecase::LOAD:
        {
            std::vector<Plato::srom::RandomLoadCase> tLoadCases;
            Plato::srom::build_load_sroms(aInput.loads(), tLoadCases);
            aOutput.loadCases(tLoadCases);
            break;
        }
        case Plato::srom::usecase::MATERIAL:
        {
            std::vector<Plato::srom::RandomMaterialCase> tMaterialCases;
            Plato::srom::build_material_sroms(aInput.materials(), tMaterialCases);
            aOutput.materialCases(tMaterialCases);
            break;
        }
        default:
        case Plato::srom::usecase::UNDEFINED:
        case Plato::srom::usecase::MATERIAL_PLUS_LOAD:
        {
            std::ostringstream tMsg;
            tMsg << "Build SROMs: Input use case '" << Plato::srom::to_string(Plato::srom::usecase::UNDEFINED)
                << "' is not supported. " << "Supported options are: 'load' and 'material'.";
            THROWERR(tMsg.str().c_str())
        }
    }
    aOutput.usecase(aInput.usecase());
}
// function build_sroms

}
// namespace srom

}
// namespace Plato
