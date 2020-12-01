/*
 * XMLGeneratorCriterionMetadata.hpp
 *
 *  Created on: May 14, 2020
 */

#pragma once

#include <unordered_map>
#include <vector>
#include <string>

#include "Plato_FreeFunctions.hpp"
#include "XMLG_Macros.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \struct Criterion
 * \brief Criterion metadata for Plato problems.
**********************************************************************************/
struct Criterion
{
private:
    std::string mID; /*!< criterion identification number */
    std::string mType;  /*!< criterion type */
    std::unordered_map<std::string, std::string> mMetaData; /*!< Scenario metadata, map< tag, value > */

public:
    /******************************************************************************//**
     * \fn id
     * \brief Return criterion identification number.
     * \return identification number
    **********************************************************************************/
    std::string id() const
    {
        return mID;
    }

    /******************************************************************************//**
     * \fn id
     * \brief Set criterion identification number.
     * \param [in] aID identification number
    **********************************************************************************/
    void id(const std::string& aID)
    {
        mID = aID;
    }

    /******************************************************************************//**
     * \fn type
     * \brief Return criterion type.
     * \return type
    **********************************************************************************/
    std::string type() const
    {
        return mType;
    }

    /******************************************************************************//**
     * \fn type
     * \brief Set criterion type.
     * \param [in] aType type
    **********************************************************************************/
    void type(const std::string& aType)
    {
        mType = aType;
    }

    /******************************************************************************//**
     * \fn value
     * \brief If criterion metadata is defined, return its value; else, return an empty string.
     * \param [in]  aTag    criterion metadata
     * \return criterion metadata value
    **********************************************************************************/
    std::string value(const std::string& aTag) const
    {
        auto tTag = Plato::tolower(aTag);
        auto tItr = mMetaData.find(tTag);
        auto tOutput = tItr == mMetaData.end() ? "" : tItr->second;
        return tOutput;
    }

    /******************************************************************************//**
     * \fn append
     * \brief Append parameter to metadata.
     * \param [in] aTag   parameter tag
     * \param [in] aValue parameter value
    **********************************************************************************/
    void append(const std::string& aTag, const std::string& aValue)
    {
        if (aTag.empty())
        {
            THROWERR(std::string("XML Generator Scenario Metadata: Parameter with tag '") + aTag + "' is empty.")
        }
        auto tTag = Plato::tolower(aTag);
        mMetaData[aTag] = aValue;
    }

    /******************************************************************************//**
     * \fn materialPenaltyExponent
     * \brief Set string value for keyword 'material_penalty_exponent'.
     * \param [in] aInput string value
     **********************************************************************************/
    void materialPenaltyExponent(const std::string& aInput)
    {
        this->append("material_penalty_exponent", aInput);
    }

    /******************************************************************************//**
     * \fn materialPenaltyExponent
     * \brief Return string value for keyword 'material_penalty_exponent'.
     * \return value
     **********************************************************************************/
    std::string materialPenaltyExponent() const
    {
        return (this->value("material_penalty_exponent"));
    }

    /******************************************************************************//**
     * \fn minErsatzMaterialConstant
     * \brief Set string value for keyword 'minimum_ersatz_material_value'.
     * \param [in] aInput string value
     **********************************************************************************/
    void minErsatzMaterialConstant(const std::string& aInput)
    {
        this->append("minimum_ersatz_material_value", aInput);
    }

    /******************************************************************************//**
     * \fn minErsatzMaterialConstant
     * \brief Return string value for keyword 'minimum_ersatz_material_value'.
     * \return value
     **********************************************************************************/
    std::string minErsatzMaterialConstant() const
    {
        return (this->value("minimum_ersatz_material_value"));
    }

    /******************************************************************************//**
     * \fn pnormExponent
     * \brief Set string value for keyword 'pnorm_exponent'.
     * \param [in] aInput string value
     **********************************************************************************/
    void pnormExponent(const std::string& aInput)
    {
        this->append("stress_p_norm_exponent", aInput);
    }

    /******************************************************************************//**
     * \fn pnormExponent
     * \brief Return string value for keyword 'pnorm_exponent'.
     * \return value
     **********************************************************************************/
    std::string pnormExponent() const
    {
        return (this->value("stress_p_norm_exponent"));
    }


};
// struct Criterion

}
// namespace XMLGen
