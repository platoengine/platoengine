/*
 * XMLGeneratorEssentialBoundaryConditionMetadata.hpp
 *
 *  Created on: Dec 8, 2020
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
 * \struct EssentialBoundaryCondition
 * \brief EssentialBoundaryCondition metadata for Plato problems.
**********************************************************************************/
struct EssentialBoundaryCondition
{
private:
    std::map<std::string, std::pair<std::string, std::string>> mProperties; /*!< list of essential boundary condition properties, map< tag, pair<attribute,value> > */

public:
    /******************************************************************************//**
     * \fn id
     * \brief Return essential boundary condition identification number.
     * \return identification number
    **********************************************************************************/
    std::string id() const
    {
        return value("id");
    }

    /******************************************************************************//**
     * \fn id
     * \brief Set essential boundary condition identification number.
     * \param [in] aID identification number
    **********************************************************************************/
    void id(const std::string& aID)
    {
        property("id", aID);
    }

    /******************************************************************************//**
     * \fn attribute
     * \brief Return ebc attribute.
     * \return attribute
    **********************************************************************************/
    std::string attribute(const std::string& aTag) const
    {
        auto tTag = Plato::tolower(aTag);
        auto tItr = mProperties.find(tTag);
        if(tItr == mProperties.end())
        {
            THROWERR(std::string("XML Generator EssentialBoundaryCondition: EssentialBoundaryCondition property '") + aTag + "' is not supported.")
        }
        return (tItr->second.first);
    }

    /******************************************************************************//**
     * \fn value
     * \brief If ebc property is defined, return its value; else, return an empty string.
     * \param [in]  aTag    ebc property tag
     * \return ebc property value
    **********************************************************************************/
    std::string value(const std::string& aTag) const
    {
        auto tTag = Plato::tolower(aTag);
        auto tItr = mProperties.find(tTag);
        auto tOutput = tItr == mProperties.end() ? "" : tItr->second.second;
        return tOutput;
    }

    /******************************************************************************//**
     * \fn location_type
     * \brief Type of application (nodeset or sideset)
     * \return ebc location_type
    **********************************************************************************/
    std::string location_type() const
    {
        auto tItr = mProperties.find("location_type");
        auto tOutput = tItr == mProperties.end() ? "" : tItr->second.second;
        return tOutput;
    }

    /******************************************************************************//**
     * \fn location_name
     * \brief Name of application 
     * \return ebc location_name
    **********************************************************************************/
    std::string location_name() const
    {
        auto tItr = mProperties.find("location_name");
        auto tOutput = tItr == mProperties.end() ? "" : tItr->second.second;
        return tOutput;
    }

    /******************************************************************************//**
     * \fn location_id
     * \brief ID of application 
     * \return ebc location_id
    **********************************************************************************/
    std::string location_id() const
    {
        auto tItr = mProperties.find("location_id");
        auto tOutput = tItr == mProperties.end() ? "" : tItr->second.second;
        return tOutput;
    }

    /******************************************************************************//**
     * \fn degree_of_freedom
     * \brief degree of freedom of EBC 
     * \return ebc degree of freedom
    **********************************************************************************/
    std::string degree_of_freedom() const
    {
        auto tItr = mProperties.find("degree_of_freedom");
        auto tOutput = tItr == mProperties.end() ? "" : tItr->second.second;
        return tOutput;
    }

    /******************************************************************************//**
     * \fn dof_value
     * \brief degree of freedom value of EBC 
     * \return ebc degree of freedom value
    **********************************************************************************/
    std::string dof_value() const
    {
        auto tItr = mProperties.find("value");
        auto tOutput = tItr == mProperties.end() ? "" : tItr->second.second;
        return tOutput;
    }

    /******************************************************************************//**
     * \fn property
     * \brief If ebc property is defined, return its value; else, throw an error.
     * \param [in] aTag ebc property tag
     * \return ebc property value
    **********************************************************************************/
    std::string property(const std::string& aTag) const
    {
        auto tTag = Plato::tolower(aTag);
        auto tItr = mProperties.find(tTag);
        if(tItr == mProperties.end())
        {
            THROWERR(std::string("XML Generator EssentialBoundaryCondition: EssentialBoundaryCondition property '") + aTag + "' is not defined.")
        }
        return (tItr->second.second);
    }

    /******************************************************************************//**
     * \fn property
     * \brief Set ebc property value.
     * \param [in] aTag       ebc property tag
     * \param [in] aValue     ebc property value
     * \param [in] aAttribute ebc attribute
    **********************************************************************************/
    void property(const std::string& aTag, const std::string& aValue, std::string aAttribute = "homogeneous")
    {
        if(aTag.empty()) { THROWERR("XML Generator EssentialBoundaryCondition: EssentialBoundaryCondition property tag is empty.") }
        if(aValue.empty()) { THROWERR("XML Generator EssentialBoundaryCondition: EssentialBoundaryCondition property value is empty.") }
        if(aAttribute.empty()) { THROWERR("XML Generator EssentialBoundaryCondition: EssentialBoundaryCondition property attribute is empty.") }
        auto tTag = Plato::tolower(aTag);
        mProperties[aTag] = std::make_pair(aAttribute, aValue);
    }

    /******************************************************************************//**
     * \fn tags
     * \brief Return list of ebc property tags.
     * \return ebc property tags
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
     * \brief Throw error if ebc is empty, i.e. ebc properties are not defined.
    **********************************************************************************/
    void empty() const
    {
        if(mProperties.empty())
        {
            THROWERR(std::string("XML Generator EssentialBoundaryCondition: EssentialBoundaryCondition with identification (id) '")
                + id() + "' is empty, i.e. " + "ebc properties are not defined.")
        }
    }
};
// struct EssentialBoundaryCondition

}
// namespace XMLGen
