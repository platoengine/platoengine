/*
 * XMLGeneratorMaterialMetadata.hpp
 *
 *  Created on: May 14, 2020
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
 * \struct Material
 * \brief Material metadata for Plato problems.
**********************************************************************************/
struct Material
{
private:
    std::string mID; /*!< material identification number */
    std::string mCode = "plato_analyze";  /*!< performer/owner of material model, default: plato_analyze */
    std::string mCategory = "isotropic linear elastic";  /*!< material category, default: isotropic linear elastic */
    std::string mName;  /*!< material name, default: material_mID */
    std::map<std::string, std::pair<std::string, std::string>> mProperties; /*!< list of material properties, map< tag, pair<attribute,value> > */

public:
    /******************************************************************************//**
     * \fn id
     * \brief Return material identification number.
     * \return identification number
    **********************************************************************************/
    std::string id() const
    {
        return mID;
    }

    /******************************************************************************//**
     * \fn id
     * \brief Set material identification number.
     * \param [in] aID identification number
    **********************************************************************************/
    void id(const std::string& aID)
    {
        mID = aID;
    }
    /******************************************************************************//**
     * \fn code
     * \brief Return code responsible for material definition.
     * \return simulation code name
    **********************************************************************************/
    std::string code() const
    {
        return mCode;
    }

    /******************************************************************************//**
     * \fn code
     * \brief Set code responsible for material definition.
     * \param [in] aCode simulation code name
    **********************************************************************************/
    void code(const std::string& aCode)
    {
        mCode = aCode;
    }

    /******************************************************************************//**
     * \fn attribute
     * \brief Return material attribute.
     * \return attribute
    **********************************************************************************/
    std::string attribute(const std::string& aTag) const
    {
        auto tTag = Plato::tolower(aTag);
        auto tItr = mProperties.find(tTag);
        if(tItr == mProperties.end())
        {
            THROWERR(std::string("XML Generator Material: Material property '") + aTag + "' is not supported.")
        }
        return (tItr->second.first);
    }

    /******************************************************************************//**
     * \fn materialModel
     * \brief Return material category.
     * \return category
    **********************************************************************************/
    std::string materialModel() const
    {
        return mCategory;
    }

    /******************************************************************************//**
     * \fn materialModel
     * \brief Set material category.
     * \param [in] aCategory category
    **********************************************************************************/
    void materialModel(const std::string& aCategory)
    {
        mCategory = aCategory;
    }

    /******************************************************************************//**
     * \fn name
     * \brief Return material name.
     * \return name
    **********************************************************************************/
    std::string name() const
    {
        return mName;
    }

    /******************************************************************************//**
     * \fn name
     * \brief Set material name.
     * \param [in] aName name
    **********************************************************************************/
    void name(const std::string& aName)
    {
        mName = aName;
    }

    /******************************************************************************//**
     * \fn value
     * \brief If material property is defined, return its value; else, return an empty string.
     * \param [in]  aTag    material property tag
     * \return material property value
    **********************************************************************************/
    std::string value(const std::string& aTag) const
    {
        auto tTag = Plato::tolower(aTag);
        auto tItr = mProperties.find(tTag);
        auto tOutput = tItr == mProperties.end() ? "" : tItr->second.second;
        return tOutput;
    }

    /******************************************************************************//**
     * \fn property
     * \brief If material property is defined, return its value; else, throw an error.
     * \param [in] aTag material property tag
     * \return material property value
    **********************************************************************************/
    std::string property(const std::string& aTag) const
    {
        auto tTag = Plato::tolower(aTag);
        auto tItr = mProperties.find(tTag);
        if(tItr == mProperties.end())
        {
            THROWERR(std::string("XML Generator Material: Material property '") + aTag + "' is not defined.")
        }
        return (tItr->second.second);
    }

    /******************************************************************************//**
     * \fn property
     * \brief Set material property value.
     * \param [in] aTag       material property tag
     * \param [in] aValue     material property value
     * \param [in] aAttribute material attribute
    **********************************************************************************/
    void property(const std::string& aTag, const std::string& aValue, std::string aAttribute = "homogeneous")
    {
        if(aTag.empty()) { THROWERR("XML Generator Material: Material property tag is empty.") }
        if(aValue.empty()) { THROWERR("XML Generator Material: Material property value is empty.") }
        if(aAttribute.empty()) { THROWERR("XML Generator Material: Material property attribute is empty.") }
        auto tTag = Plato::tolower(aTag);
        mProperties[aTag] = std::make_pair(aAttribute, aValue);
    }

    /******************************************************************************//**
     * \fn tags
     * \brief Return list of material property tags.
     * \return material property tags
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
     * \brief Throw error if material is empty, i.e. material properties are not defined.
    **********************************************************************************/
    void empty() const
    {
        if(mProperties.empty())
        {
            THROWERR(std::string("XML Generator Material: Material with identification (id) '")
                + mID + "' is empty, i.e. " + "material properties are not defined.")
        }
    }
};
// struct Material

}
// namespace XMLGen
