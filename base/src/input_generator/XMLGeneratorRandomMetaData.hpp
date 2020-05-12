/*
 * XMLGeneratorRandomMetaData.hpp
 *
 *  Created on: May 12, 2020
 */

#pragma once

#include <unordered_map>

#include "XMLGeneratorDataStruct.hpp"
#include "Plato_SromHelpers.hpp"

namespace XMLGen
{

/*!< map between block identification number and material metadata */
using MaterialSet = std::unordered_map<std::string, XMLGen::Material>;
/******************************************************************************//**
 * \struct RandomUseCase
 * \brief Metadata for random use cases.
**********************************************************************************/
struct RandomUseCase
{
private:
    std::string mProbability; /*!< random use case probability */
    XMLGen::LoadCase mLoadCase; /*!< load case metadata */
    XMLGen::MaterialSet mMaterials; /*!< materials metadata */
    Plato::srom::usecase mUseCase = Plato::srom::usecase::UNDEFINED; /*!< use case type */

private:
    /******************************************************************************//**
     * \fn check
     * \brief Check input material metadata.
     * \param [in] aMaterial material metadata
    **********************************************************************************/
    void check(const XMLGen::Material& aMaterial)
    {
        if(aMaterial.id().empty())
        {
            THROWERR("Random Use Case: Material identification number is empty, i.e. material id is not defined.")
        }
        if(aMaterial.category().empty())
        {
            THROWERR("Random Use Case: Material category is empty, i.e. category is not defined.")
        }
        if(aMaterial.attribute().empty())
        {
            THROWERR("Random Use Case: Material attribute is empty, i.e. attribute is not defined.")
        }
        if(aMaterial.tags().empty())
        {
            THROWERR("Random Use Case: Material properties are empty, i.e. properties are not defined.")
        }
    }

    /******************************************************************************//**
     * \fn check
     * \brief Check load case metadata.
     * \param [in] aLoadCase load case metadata
    **********************************************************************************/
    void check(const XMLGen::LoadCase& aLoadCase)
    {
        if(aLoadCase.id.empty())
        {
            THROWERR("Random Use Case: Load case identification number is empty, i.e. load case id is not defined.")
        }
        if(aLoadCase.loads.empty())
        {
            THROWERR("Random Use Case: Load case loads container is empty, i.e. set of loads is not defined.")
        }
    }

public:
    /******************************************************************************//**
     * \fn usecase
     * \brief Set random use case.
     * \param [in] aUseCase random use case type
    **********************************************************************************/
    void usecase(const Plato::srom::usecase& aUseCase)
    {
        mUseCase = aUseCase;
    }

    /******************************************************************************//**
     * \fn usecase
     * \brief Return random use case.
     * \return random use case type
    **********************************************************************************/
    Plato::srom::usecase usecase() const
    {
        return mUseCase;
    }

    /******************************************************************************//**
     * \fn probability
     * \brief Set random use case probability.
     * \param [in] aProbability random use case probability
    **********************************************************************************/
    void probability(const std::string& aProbability)
    {
        if(aProbability.empty())
        {
            THROWERR("Random Use Case: Probability is empty, i.e. probability is not defined.")
        }
        mProbability = aProbability;
    }

    /******************************************************************************//**
     * \fn probability
     * \brief Return random use case probability.
     * \return random use case probability
    **********************************************************************************/
    std::string probability() const
    {
        return mProbability;
    }

    /******************************************************************************//**
     * \fn loads
     * \brief Return load case metadata.
     * \return load case metadata
    **********************************************************************************/
    XMLGen::LoadCase loads() const
    {
        return mLoadCase;
    }

    /******************************************************************************//**
     * \fn material
     * \brief Return material metadata.
     * \param [in] aBlockID block identification number
     * \return material metadata
    **********************************************************************************/
    XMLGen::Material material(const std::string& aBlockID) const
    {
        auto tIterator = mMaterials.find(aBlockID);
        if(tIterator == mMaterials.end())
        {
            THROWERR(std::string("Random Use Case: Did not find a material block with identification number '")
                + aBlockID + "' in material set.")
        }
        return (tIterator->second);
    }

    /******************************************************************************//**
     * \fn set
     * \brief Set load case metadata.
     * \param [in] aLoadCase load case metadata
    **********************************************************************************/
    void set(const XMLGen::LoadCase& aLoadCase)
    {
        this->check(aLoadCase);
        mLoadCase = aLoadCase;
    }

    /******************************************************************************//**
     * \fn append
     * \brief Append material metadata.
     * \param [in] aBlockID block identification number
     * \param [in] aMaterial material metadata
    **********************************************************************************/
    void append(const std::string& aBlockID, const XMLGen::Material& aMaterial)
    {
        if(aBlockID.empty())
        {
            THROWERR("Random Use Case: Material block identification number empty, i.e. block id is not defined.")
        }
        this->check(aMaterial);
        mMaterials.insert({aBlockID, aMaterial});
    }
};
// struct RandomUseCase

}
