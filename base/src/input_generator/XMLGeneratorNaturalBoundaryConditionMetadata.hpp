/*
 * XMLGeneratorNaturalBoundaryConditionMetadata.hpp
 *
 *  Created on: Jan 5, 2021
 */

#pragma once

#include <map>
#include <vector>
#include <string>

#include "Plato_FreeFunctions.hpp"
#include "XMLG_Macros.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \struct NaturalBoundaryCondition
 * \brief NaturalBoundaryCondition metadata for Plato problems.
**********************************************************************************/
struct NaturalBoundaryCondition
{
private:
    std::map<std::string, std::pair<std::string, std::string>> mProperties; /*!< list of natural boundary condition properties, map< tag, pair<attribute,value> > */
    std::vector<std::string> mValues;

public:
    /******************************************************************************//**
     * \fn id
     * \brief Return natural boundary condition identification number.
     * \return identification number
    **********************************************************************************/
    std::string id() const
    {
        return value("id");
    }

    /******************************************************************************//**
     * \fn id
     * \brief Set natural boundary condition identification number.
     * \param [in] aID identification number
    **********************************************************************************/
    void id(const std::string& aID)
    {
        if(!aID.empty())
        {
            property("id", aID);
        }
    }

    /******************************************************************************//**
     * \fn type
     * \brief Return natural boundary condition type
     * \return type
    **********************************************************************************/
    std::string type() const
    {
        return value("type");
    }

    /******************************************************************************//**
     * \fn type
     * \brief Set natural boundary condition type
     * \param [in] aType type
    **********************************************************************************/
    void type(const std::string& aType)
    {
        if(!aType.empty())
        {
            property("type", aType);
        }
    }

    /******************************************************************************//**
     * \fn locationType
     * \brief Return natural boundary condition location type
     * \return location type
    **********************************************************************************/
    std::string location_type() const
    {
        return value("location_type");
    }

    /******************************************************************************//**
     * \fn locationType
     * \brief Set natural boundary condition location type
     * \param [in] aLocationType location type
    **********************************************************************************/
    void location_type(const std::string& aLocationType)
    {
        if(!aLocationType.empty())
        {
            property("location_type", aLocationType);
        }
    }

    /******************************************************************************//**
     * \fn location_name
     * \brief Return natural boundary condition location name
     * \return location name
    **********************************************************************************/
    std::string location_name() const
    {
        return value("location_name");
    }

    /******************************************************************************//**
     * \fn location_name
     * \brief Set natural boundary condition location name
     * \param [in] aLocationType location name
    **********************************************************************************/
    void location_name(const std::string& aLocationName)
    {
        if(!aLocationName.empty())
        {
            property("location_name", aLocationName);
        }
    }

    /******************************************************************************//**
     * \fn location_id
     * \brief Return natural boundary condition location id
     * \return location id
    **********************************************************************************/
    std::string location_id() const
    {
        return value("location_id");
    }

    /******************************************************************************//**
     * \fn location_id
     * \brief Set natural boundary condition location id
     * \param [in] aLocationType location id
    **********************************************************************************/
    void location_id(const std::string& aLocationID)
    {
        if(!aLocationID.empty())
        {
            property("location_id", aLocationID);
        }
    }

    /******************************************************************************//**
     * \fn is_random
     * \brief Return whether natural boundary condition is randome
     * \return location id
    **********************************************************************************/
    bool is_random() const
    {
        return (Plato::tolower(value("is_random")) == "true");
    }

    /******************************************************************************//**
     * \fn is_random
     * \brief Set whether natural boundary condition is random
     * \param [in] aIsRandom input string specifying value of is_random variable
    **********************************************************************************/
    void is_random(const std::string& aIsRandom)
    {
        if(!aIsRandom.empty())
        {
            property("is_random", aIsRandom);
        }
    }

    /******************************************************************************//**
     * \fn attribute
     * \brief Return nbc attribute.
     * \return attribute
    **********************************************************************************/
    std::string attribute(const std::string& aTag) const
    {
        auto tTag = Plato::tolower(aTag);
        auto tItr = mProperties.find(tTag);
        if(tItr == mProperties.end())
        {
            THROWERR(std::string("XML Generator NaturalBoundaryCondition: NaturalBoundaryCondition property '") + aTag + "' is not supported.")
        }
        return (tItr->second.first);
    }

    /******************************************************************************//**
     * \fn value
     * \brief If nbc property is defined, return its value; else, return an empty string.
     * \param [in]  aTag    nbc property tag
     * \return nbc property value
    **********************************************************************************/
    std::string value(const std::string& aTag) const
    {
        auto tTag = Plato::tolower(aTag);
        auto tItr = mProperties.find(tTag);
        auto tOutput = tItr == mProperties.end() ? "" : tItr->second.second;
        return tOutput;
    }

    /******************************************************************************//**
     * \fn load_values
     * \brief Return the values for this load
     * \return load values
    **********************************************************************************/
    const std::vector<std::string>& load_values() const
    {
        return mValues;
    }

    /******************************************************************************//**
     * \fn load_values
     * \brief Return the values for this load
     * \return load values
    **********************************************************************************/
    void load_values(const std::vector<std::string>& aValues) 
    {
        mValues = aValues;
    }

    /******************************************************************************//**
     * \fn property
     * \brief If nbc property is defined, return its value; else, throw an error.
     * \param [in] aTag nbc property tag
     * \return nbc property value
    **********************************************************************************/
    std::string property(const std::string& aTag) const
    {
        auto tTag = Plato::tolower(aTag);
        auto tItr = mProperties.find(tTag);
        if(tItr == mProperties.end())
        {
            THROWERR(std::string("XML Generator NaturalBoundaryCondition: NaturalBoundaryCondition property '") + aTag + "' is not defined.")
        }
        return (tItr->second.second);
    }

    /******************************************************************************//**
     * \fn property
     * \brief Set nbc property value.
     * \param [in] aTag       nbc property tag
     * \param [in] aValue     nbc property value
     * \param [in] aAttribute nbc attribute
    **********************************************************************************/
    void property(const std::string& aTag, const std::string& aValue, std::string aAttribute = "homogeneous")
    {
        if(aTag.empty()) 
        { 
            THROWERR("XML Generator NaturalBoundaryCondition: NaturalBoundaryCondition property tag is empty.")        }
        if(aValue.empty()) 
        { 
            THROWERR("XML Generator NaturalBoundaryCondition: NaturalBoundaryCondition property value is empty.")      
        }
        if(aAttribute.empty()) 
        { 
            THROWERR("XML Generator NaturalBoundaryCondition: NaturalBoundaryCondition property attribute is empty.")  }
        auto tTag = Plato::tolower(aTag);
        mProperties[aTag] = std::make_pair(aAttribute, aValue);
    }

    /******************************************************************************//**
     * \fn tags
     * \brief Return list of nbc property tags.
     * \return nbc property tags
    **********************************************************************************/
    std::vector<std::string> tags() const
    {
        std::vector<std::string> tTags;
        for(auto& tProperty : mProperties)
        {
            tTags.push_back(tProperty.first);
        }
        return tTags;
    }

    /******************************************************************************//**
     * \fn empty
     * \brief Throw error if nbc is empty, i.e. nbc properties are not defined.
    **********************************************************************************/
    void empty() const
    {
        if(mProperties.empty())
        {
            THROWERR(std::string("XML Generator NaturalBoundaryCondition: NaturalBoundaryCondition with identification (id) '")
                + id() + "' is empty, i.e. " + "nbc properties are not defined.")
        }
    }
};
// struct NaturalBoundaryCondition

}
// namespace XMLGen
