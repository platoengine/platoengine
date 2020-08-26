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
    std::unordered_map<std::string, std::string> mParameters; /*!< list of criterion parameters, map< tag, value> */

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
     * \fn parameter
     * \brief If criterion parameter is defined, return its value; else, throw an error.
     * \param [in] aTag criterion parameter
     * \return criterion parameter value
    **********************************************************************************/
    std::string parameter(const std::string& aTag) const
    {
        auto tTag = Plato::tolower(aTag);
        auto tItr = mParameters.find(tTag);
        if(tItr == mParameters.end())
        {
            THROWERR(std::string("XML Generator Criterion: Criterion parameter '") + aTag + "' is not defined.")
        }
        return (tItr->second);
    }

    /******************************************************************************//**
     * \fn parameter
     * \brief Set criterion parameter value.
     * \param [in] aTag       criterion parameter
     * \param [in] aValue     criterion parameter value
     * \param [in] aAttribute criterion attribute 
    **********************************************************************************/
    void parameter(const std::string& aTag, const std::string& aValue)
    {
        if(aTag.empty()) { THROWERR("XML Generator Criterion: Criterion parameter tag is empty.") }
        if(aValue.empty()) { THROWERR("XML Generator Criterion: Criterion parameter value is empty.") }
        auto tTag = Plato::tolower(aTag);
        mParameters[aTag] = aValue;
    }

    /******************************************************************************//**
     * \fn parameters
     * \brief Return list of criterion parameters
     * \return criterion parameters
    **********************************************************************************/
    std::vector<std::string> parameters() const
    {
        std::vector<std::string> tTags;
        for(auto& tParameter : mParameters)
        {
            tTags.push_back(tParameter.first);
        }
        return tTags;
    }
};
// struct Criterion

}
// namespace XMLGen
