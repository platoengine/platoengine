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
 * Plato_RandomMaterialMetadata.hpp
 *
 *  Created on: Apr 28, 2020
 */

#pragma once

#include <map>
#include <cmath>
#include <vector>
#include <unordered_map>

#include "Plato_Macros.hpp"
#include "Plato_SromHelpers.hpp"
#include "Plato_SromMetadata.hpp"

namespace Plato
{

namespace srom
{

/******************************************************************************//**
 * \struct Material
 * \brief Struct used to define a material. Each material is defined by:\n
 *     -# block identification number,
 *     -# material identification number,
 *     -# material category, e.g. isotropic, orthotropic, etc.,
 *     -# list of random material properties, and
 *     -# list of deterministic material properties.
 * \note There are three material categories:\n
 *     -# random: all material properties are random,
 *     -# mix: a mix of random and deterministic material properties, and
 *     -# deterministic: all material properties are deterministic.
**********************************************************************************/
struct Material
{
private:
    std::string mBlockID;     /*!< material block identification number */
    std::string mCategory;    /*!< isotropic, orthotropic, anisotropic, etc. */
    std::string mName;        /*!< material name */
    std::string mMaterialID;  /*!< material identification number */

    std::vector<Plato::srom::RandomVariable> mRandomVars; /*!< non-deterministic material property */
    std::vector<Plato::srom::DeterministicVariable> mDeterministicVars; /*!< deterministic material property */

public:
    /******************************************************************************//**
     * \brief Checks if material is random.
     * \return flag - true if material is random
    **********************************************************************************/
    bool isRandom() const
    {
        return (!mRandomVars.empty());
    }

    /******************************************************************************//**
     * \brief Checks if material is deterministic.
     * \return flag - true if material is deterministic
    **********************************************************************************/
    bool isDeterministic() const
    {
        return (mRandomVars.empty());
    }

    /******************************************************************************//**
     * \brief Checks if material is properly defined.
    **********************************************************************************/
    void check() const
    {
        if(mMaterialID.empty())
        {
            THROWERR("SROM::Material: Material identification number is not defined.")
        }

        auto tNumMaterialProperties = mRandomVars.size() + mDeterministicVars.size();
        if(tNumMaterialProperties == static_cast<size_t>(0))
        {
            std::ostringstream tMsg;
            tMsg << "SROM Material: Material properties for material with identification number '"
                << mMaterialID.c_str() << "' are not defined.";
            THROWERR(tMsg.str().c_str())
        }
    }

    /******************************************************************************//**
     * \fn blockID
     * \brief Set material block identification number.
     * \param [in] aID identification number
    **********************************************************************************/
    void blockID(const std::string& aID)
    {
        mBlockID = aID;
    }

    /******************************************************************************//**
     * \fn blockID
     * \brief Return material block identification number.
     * \return material block identification number
    **********************************************************************************/
    std::string blockID() const
    {
        return (mBlockID);
    }

    /******************************************************************************//**
     * \brief Set material identification number.
     * \param [in] aID identification number
    **********************************************************************************/
    void materialID(const std::string& aID)
    {
        mMaterialID = aID;
    }

    /******************************************************************************//**
     * \brief Return material identification number.
     * \return material identification number
    **********************************************************************************/
    std::string materialID() const
    {
        return (mMaterialID);
    }

    /******************************************************************************//**
     * \brief Set material category, e.g. isotropic, orthotropic, etc.
     * \param [in] aCategory material category
    **********************************************************************************/
    void category(const std::string& aCategory)
    {
        mCategory = aCategory;
    }

    /******************************************************************************//**
     * \brief Return material category.
     * \return material category
    **********************************************************************************/
    std::string category() const
    {
        return mCategory;
    }

    /******************************************************************************//**
     * \brief Set material name
     * \param [in] aName material name
    **********************************************************************************/
    void name(const std::string& aName)
    {
        mName = aName;
    }

    /******************************************************************************//**
     * \brief Return material name.
     * \return material name
    **********************************************************************************/
    std::string name() const
    {
        return mName;
    }

    /******************************************************************************//**
     * \brief Return tags of material properties that define the material.
     * \return list with material property tags
    **********************************************************************************/
    std::vector<std::string> tags() const
    {
        std::vector<std::string> tTags;

        // append random variables tags
        for(auto& tRandomVar : mRandomVars)
        {
            tTags.push_back(tRandomVar.tag());
        }

        // append deterministic variables tags
        for(auto& tDeterministicVar : mDeterministicVars)
        {
            tTags.push_back(tDeterministicVar.tag());
        }

        return (tTags);
    }

    /******************************************************************************//**
     * \brief Return list of random material properties that define the material.
     * \return list of random material properties
    **********************************************************************************/
    std::vector<Plato::srom::RandomVariable> randomVars() const
    {
        return (mRandomVars);
    }

    /******************************************************************************//**
     * \brief Return list of deterministic material properties that define the material.
     * \return list of deterministic material properties
    **********************************************************************************/
    std::vector<Plato::srom::DeterministicVariable> deterministicVars() const
    {
        return (mDeterministicVars);
    }

    /******************************************************************************//**
     * \brief Append random material property.
     * \param [in] aTag       material property tag/label
     * \param [in] aAttribute material property attribute
     * \param [in] aStats     material property statistics metadata
    **********************************************************************************/
    void append(const std::string &aTag,
                const std::string &aAttribute,
                const Plato::srom::Statistics &aStats)
    {
        Plato::srom::RandomVariable tVariable;
        tVariable.define(aTag, aAttribute, aStats);
        mRandomVars.push_back(tVariable);
    }

    /******************************************************************************//**
     * \brief Append deterministic material property.
     * \param [in] aTag       material property tag/label
     * \param [in] aAttribute material property attribute
     * \param [in] aValue     material property value
    **********************************************************************************/
    void append(const std::string &aTag,
                const std::string &aAttribute,
                const std::string &aValue)
    {
        Plato::srom::DeterministicVariable tVariable;
        tVariable.tag(aTag);
        tVariable.value(aValue);
        tVariable.attribute(aAttribute);
        mDeterministicVars.push_back(tVariable);
    }
};
// struct Material

/******************************************************************************//**
 * \struct RandomMaterial
 * \brief Struct used to define a random material. Each random material is defined by:\n
 *     -# random material probability,
 *     -# block identification number,
 *     -# material identification number,
 *     -# material category, e.g. isotropic, orthotropic, etc., and
 *     -# list of material properties tags/labels
**********************************************************************************/
using MaterialProps = std::map<std::string,std::pair<std::string, std::string>>;
struct RandomMaterial
{
private:
    double mProbability;       /*!< probability for this material instance */
    std::string mBlockID;      /*!< material block identification number */
    std::string mCategory;     /*!< isotropic, orthotropic, anisotropic, etc. */
    std::string mName;         /*!< material name */
    std::string mMaterialID;   /*!< material identification number */

    MaterialProps mTags; /*!< map between material property tag/label and its attribute and value, i.e. tags(tag,(attribute,value)) */

public:
    /******************************************************************************//**
     * \brief Checks if random material is properly defined.
    **********************************************************************************/
    void check() const
    {
        if(mMaterialID.empty())
        {
            THROWERR("SROM::RandomMaterial: Material identification number is not defined.")
        }

        if(mTags.empty())
        {
            std::ostringstream tMsg;
            tMsg << "SROM::RandomMaterial: Tags for random material with material identification number '"
                << mMaterialID << ", and category '"<< mCategory << "' are not defined.";
            THROWERR(tMsg.str().c_str())
        }

        if(std::isfinite(mProbability) == false)
        {
            std::ostringstream tMsg;
            tMsg << "SROM::RandomMaterial: Probability for random material with material identification number '"
                << mMaterialID << ", and category '"<< mCategory  << "' is not a finite number.";
            THROWERR(tMsg.str().c_str())
        }

        if(mProbability <= static_cast<double>(0.0))
        {
            std::ostringstream tMsg;
            tMsg << "SROM::RandomMaterial: Probability for random material with material identification number '"
                << mMaterialID << ", and category '"<< mCategory << "' is not a positive number or it's set to zero."
                << "The probability value is set to '" << mProbability << "'.";
            THROWERR(tMsg.str().c_str())
        }
    }

    /******************************************************************************//**
     * \fn blockID
     * \brief Set material block identification number.
     * \param [in] aID identification number
    **********************************************************************************/
    void blockID(const std::string& aID)
    {
        mBlockID = aID;
    }

    /******************************************************************************//**
     * \fn blockID
     * \brief Return material block identification number.
     * \return material block identification number
    **********************************************************************************/
    std::string blockID() const
    {
        return (mBlockID);
    }

    /******************************************************************************//**
     * \brief Set random material identification number.
     * \param [in] aID identification number
    **********************************************************************************/
    void materialID(const std::string& aID)
    {
        mMaterialID = aID;
    }

    /******************************************************************************//**
     * \brief Return random material identification number.
     * \return random material identification number
    **********************************************************************************/
    std::string materialID() const
    {
        return (mMaterialID);
    }

    /******************************************************************************//**
     * \brief Set random material category, e.g. isotropic, orthotropic, etc.
     * \param [in] aCategory random material category
    **********************************************************************************/
    void category(const std::string& aCategory)
    {
        mCategory = aCategory;
    }

    /******************************************************************************//**
     * \brief Return random material category, e.g. isotropic, orthotropic, etc.
     * \return random material category
    **********************************************************************************/
    std::string category() const
    {
        return mCategory;
    }

    /******************************************************************************//**
     * \brief Set material name
     * \param [in] aName material name
    **********************************************************************************/
    void name(const std::string& aName)
    {
        mName = aName;
    }

    /******************************************************************************//**
     * \brief Return material name.
     * \return material name
    **********************************************************************************/
    std::string name() const
    {
        return mName;
    }

    /******************************************************************************//**
     * \brief Set random material probability.
     * \param [in] aProbability random material probability
    **********************************************************************************/
    void probability(const double& aProbability)
    {
        mProbability = aProbability;
    }

    /******************************************************************************//**
     * \brief Return random material probability.
     * \return random material probability
    **********************************************************************************/
    double probability() const
    {
        return mProbability;
    }

    /******************************************************************************//**
     * \brief Return random material property value.
     * \param [in] aTag random material property label
    **********************************************************************************/
    std::string value(const std::string& aTag) const
    {
        auto tItr = mTags.find(aTag);
        if(tItr == mTags.end())
        {
            std::ostringstream tMsg;
            tMsg << "Random Material: Material property with tag '" << aTag.c_str() << "' is not defined.";
            THROWERR(tMsg.str().c_str())
        }
        return (tItr->second.second);
    }

    /******************************************************************************//**
     * \brief Return random material property attribute.
     * \param [in] aTag random material property label
    **********************************************************************************/
    std::string attribute(const std::string& aTag) const
    {
        auto tItr = mTags.find(aTag);
        if(tItr == mTags.end())
        {
            std::ostringstream tMsg;
            tMsg << "Random Material: Material property with tag '" << aTag.c_str() << "' is not defined.";
            THROWERR(tMsg.str().c_str())
        }
        return (tItr->second.first);
    }

    /******************************************************************************//**
     * \brief Return tags of random material properties that define the material.
     * \return list with random material property tags
    **********************************************************************************/
    std::vector<std::string> tags() const
    {
        std::vector<std::string> tTags;
        for(auto& tItr : mTags)
        {
            tTags.push_back(tItr.first);
        }
        return tTags;
    }

    /******************************************************************************//**
     * \brief Append random material property.
     * \param [in] aTag       random material property tag/label
     * \param [in] aAttribute random material property attribute
     * \param [in] aValue     random material property value
    **********************************************************************************/
    void append(const std::string& aTag, const std::string& aAttribute, const std::string& aValue)
    {
        mTags.insert(std::pair<std::string, std::pair<std::string, std::string>>
            (aTag, std::pair<std::string, std::string>(aAttribute, aValue)));
    }
};
// struct RandomMaterial

/******************************************************************************//**
 * \struct RandomMaterialCase
 * \brief Struct used to define a random material case. Each random material case \n
 * is defined by:
 *     -# random material case probability,
 *     -# case identification number, and
 *     -# list of random materials
 *     -# map between material identification number and block identification number
**********************************************************************************/
struct RandomMaterialCase
{
private:
    double mProbability; /*!< probability associated with this random load case */
    std::string mCaseID; /*!< random material case identification number */
    std::unordered_map<std::string, std::string> mBlockIDMap; /*!< map(material_id, block_id) */
    std::map<std::string, Plato::srom::RandomMaterial> mMaterialMap; /*!< map(material_id, random material) */

public:
    /******************************************************************************//**
     * \brief Return number of materials that define the random material case.
     * \return number of materials
    **********************************************************************************/
    size_t numMaterials() const
    {
        return (mMaterialMap.size());
    }

    /******************************************************************************//**
     * \brief Set random material case identification number.
     * \param [in] aID identification number
    **********************************************************************************/
    void caseID(const std::string& aCaseID)
    {
        mCaseID = aCaseID;
    }

    /******************************************************************************//**
     * \brief Return random material case identification number.
     * \return random material case identification number
    **********************************************************************************/
    std::string caseID() const
    {
        return (mCaseID);
    }

    /******************************************************************************//**
     * \brief Set random material case probability.
     * \param [in] aProbability probability
    **********************************************************************************/
    void probability(const double& aProbability)
    {
        mProbability = aProbability;
    }

    /******************************************************************************//**
     * \brief Return random material case probability.
     * \return probability
    **********************************************************************************/
    double probability() const
    {
        return (mProbability);
    }

    /******************************************************************************//**
     * \brief Return string with random material case probability.
     * \return random material case probability
    **********************************************************************************/
    std::string probabilityToString() const
    {
        auto tOutput = Plato::srom::to_string(mProbability);
        return (tOutput);
    }

    /******************************************************************************//**
     * \brief Return category for material with input identification number.
     * \param [in] aMaterialID material identification number
     * \return material category
    **********************************************************************************/
    std::string category(const std::string& aMaterialID) const
    {
        auto tItr = mMaterialMap.find(aMaterialID);
        if(tItr == mMaterialMap.end())
        {
            std::ostringstream tMsg;
            tMsg << "Random Material Case: Material with identification number '" << aMaterialID.c_str() << "' is not defined.";
            THROWERR(tMsg.str().c_str())
        }
        return (tItr->second.category());
    }

    /******************************************************************************//**
     * \brief Return material property value for material with input identification number and tag.
     * \param [in] aMaterialID material identification number
     * \param [in] aTag        material property tag
     * \return material property value
    **********************************************************************************/
    std::string value(const std::string& aMaterialID, const std::string& aTag) const
    {
        auto tItr = mMaterialMap.find(aMaterialID);
        if(tItr == mMaterialMap.end())
        {
            std::ostringstream tMsg;
            tMsg << "Random Material Case: Material with identification number '" << aMaterialID.c_str() << "' is not defined.";
            THROWERR(tMsg.str().c_str())
        }
        return (tItr->second.value(aTag));
    }

    /******************************************************************************//**
     * \brief Return material property attribute for material with input identification number and tag.
     * \param [in] aMaterialID material identification number
     * \param [in] aTag        material property tag
     * \return material property attribute
    **********************************************************************************/
    std::string attribute(const std::string& aMaterialID, const std::string& aTag) const
    {
        auto tItr = mMaterialMap.find(aMaterialID);
        if(tItr == mMaterialMap.end())
        {
            std::ostringstream tMsg;
            tMsg << "Random Material Case: Material with identification number '" << aMaterialID.c_str() << "' is not defined.";
            THROWERR(tMsg.str().c_str())
        }
        return (tItr->second.attribute(aTag));
    }

    /******************************************************************************//**
     * \brief Return material property tags for material with input identification number.
     * \param [in] aMaterialID material identification number
     * \return list of material property tags
    **********************************************************************************/
    std::vector<std::string> tags(const std::string& aMaterialID) const
    {
        auto tItr = mMaterialMap.find(aMaterialID);
        if(tItr == mMaterialMap.end())
        {
            std::ostringstream tMsg;
            tMsg << "Random Material Case: Material with identification number '" << aMaterialID.c_str() << "' is not defined.";
            THROWERR(tMsg.str().c_str())
        }
        return (tItr->second.tags());
    }

    /******************************************************************************//**
     * \brief Return material block identification number.
     * \param [in] aMaterialID material identification number
     * \return material block identification number
    **********************************************************************************/
    std::string blockID(const std::string& aMaterialID) const
    {
        auto tIterator = mBlockIDMap.find(aMaterialID);
        if(tIterator == mBlockIDMap.end())
        {
            THROWERR(std::string("Random Material Case: Material with identification number '"
                + aMaterialID + "' is not defined in the material set."))
        }
        return (tIterator->second);
    }

    /******************************************************************************//**
     * \brief Return material block identification numbers for this random material case.
     * \return list of material block identification numbers
    **********************************************************************************/
    std::vector<std::string> blockIDs() const
    {
        std::vector<std::string> tIDs;
        for(auto& tItr : mMaterialMap)
        {
            tIDs.push_back(tItr.second.blockID());
        }
        return tIDs;
    }

    /******************************************************************************//**
     * \brief Return material identification numbers for this random material case.
     * \return list of material identification numbers
    **********************************************************************************/
    std::vector<std::string> materialIDs() const
    {
        std::vector<std::string> tIDs;
        for(auto& tItr : mMaterialMap)
        {
            tIDs.push_back(tItr.first);
        }
        return tIDs;
    }

    /******************************************************************************//**
     * \brief Return list of random materials that define this random material case.
     * \return list of random materials
    **********************************************************************************/
    std::vector<Plato::srom::RandomMaterial> materials() const
    {
        std::vector<Plato::srom::RandomMaterial> tMaterials;
        for(auto& tItr : mMaterialMap)
        {
            tMaterials.push_back(tItr.second);
        }
        return tMaterials;
    }

    /******************************************************************************//**
     * \brief Append random material.
     * \param [in] aMaterialID random material identification number
     * \param [in] aMaterial   random material metadata
    **********************************************************************************/
    void append(const std::string& aMaterialID, const Plato::srom::RandomMaterial& aMaterial)
    {
        mMaterialMap.insert(std::pair<std::string, Plato::srom::RandomMaterial>(aMaterialID, aMaterial));
        mBlockIDMap.insert({aMaterialID, aMaterial.blockID()});
    }
};
// struct RandomMaterialCase

}
// namespace srom

}
// namespace Plato
