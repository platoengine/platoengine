/*
 * XMLGeneratorConstraintMetadata.hpp
 *
 *  Created on: Jul 22, 2020
 */

#pragma once

#include <vector>
#include <string>
#include <unordered_map>

namespace XMLGen
{

/******************************************************************************//**
 * \struct Constraint
 * \brief Constraint metadata for Plato problems.
**********************************************************************************/
struct Constraint
{
// private member data
private:
    std::unordered_map<std::string, std::string> mMetaData; /*!< scenario metadata, map< tag, value > */

// private member functions
private:
    /******************************************************************************//**
     * \fn getValue
     * \brief Return string value for property with input tag; else, throw an error if \n
     * property is not defined in metadata.
     * \param [in] aTag property tag
     * \return property string value
    **********************************************************************************/
    std::string getValue(const std::string& aTag) const;

// public member functions
public:
    /******************************************************************************//**
     * \fn value
     * \brief Return value for property with input tag; else, throw an error if \n
     * property is not defined in the metadata.
     * \param [in] aTag property tag
     * \return property string value
    **********************************************************************************/
    std::string value(const std::string& aTag) const;

    /******************************************************************************//**
     * \fn tags
     * \brief Return list of parameter tags.
     * \return parameter tags
    **********************************************************************************/
    std::vector<std::string> tags() const;

    /******************************************************************************//**
     * \fn append
     * \brief Append parameter to metadata.
     * \param [in] aTag   parameter tag
     * \param [in] aValue parameter value
    **********************************************************************************/
    void append(const std::string& aTag, const std::string& aValue);

    /******************************************************************************//**
     * \fn id
     * \brief Set string value for keyword 'id'.
     * \param [in] aInput string value
    **********************************************************************************/
    void id(const std::string& aInput);

    /******************************************************************************//**
     * \fn name
     * \brief Return string value for keyword 'id'.
     * \return output string
    **********************************************************************************/
    std::string id() const;

    /******************************************************************************//**
     * \fn name
     * \brief Set string value for keyword 'name'.
     * \param [in] aInput string value
    **********************************************************************************/
    void name(const std::string& aInput);

    /******************************************************************************//**
     * \fn name
     * \brief Return string value for keyword 'name'.
     * \return output string
    **********************************************************************************/
    std::string name() const;

    /******************************************************************************//**
     * \fn code
     * \brief Set string value for keyword 'code'.
     * \param [in] aInput string value
    **********************************************************************************/
    void code(const std::string& aInput);

    /******************************************************************************//**
     * \fn code
     * \brief Return string value for keyword 'code'.
     * \return output string
    **********************************************************************************/
    std::string code() const;

    /******************************************************************************//**
     * \fn weight
     * \brief Set string value for keyword 'weight'.
     * \param [in] aInput string value
    **********************************************************************************/
    void weight(const std::string& aInput);

    /******************************************************************************//**
     * \fn weight
     * \brief Return string value for keyword 'weight'.
     * \return output string
    **********************************************************************************/
    std::string weight() const;

    /******************************************************************************//**
     * \fn category
     * \brief Set string value for keyword 'category'.
     * \param [in] aInput string value
    **********************************************************************************/
    void category(const std::string& aInput);

    /******************************************************************************//**
     * \fn category
     * \brief Return string value for keyword 'category'.
     * \return output string
    **********************************************************************************/
    std::string category() const;

    /******************************************************************************//**
     * \fn performer
     * \brief Set string value for keyword 'performer'.
     * \param [in] aInput string value
    **********************************************************************************/
    void performer(const std::string& aInput);

    /******************************************************************************//**
     * \fn performer
     * \brief Return string value for keyword 'performer'.
     * \return output string
    **********************************************************************************/
    std::string performer() const;

    /******************************************************************************//**
     * \fn pnormExponent
     * \brief Set string value for keyword 'pnorm exponent'.
     * \param [in] aInput string value
    **********************************************************************************/
    void pnormExponent(const std::string& aInput);

    /******************************************************************************//**
     * \fn pnormExponent
     * \brief Return string value for keyword 'pnorm exponent'.
     * \return output string
    **********************************************************************************/
    std::string pnormExponent() const;

    /******************************************************************************//**
     * \fn normalizedTarget
     * \brief Set string value for keyword 'target normalized'.
     * \param [in] aInput string value
    **********************************************************************************/
    void normalizedTarget(const std::string& aInput);

    /******************************************************************************//**
     * \fn normalizedTarget
     * \brief Return string value for keyword 'target normalized'.
     * \return output string
    **********************************************************************************/
    std::string normalizedTarget() const;

    /******************************************************************************//**
     * \fn absoluteTarget
     * \brief Set string value for keyword 'target absolute'.
     * \param [in] aInput string value
    **********************************************************************************/
    void absoluteTarget(const std::string& aInput);

    /******************************************************************************//**
     * \fn absoluteTarget
     * \brief Return string value for keyword 'target absolute'.
     * \return output string
    **********************************************************************************/
    std::string absoluteTarget() const;

    /******************************************************************************//**
     * \fn materialPenaltyExponent
     * \brief Set string value for keyword 'penalty power'.
     * \param [in] aInput string value
    **********************************************************************************/
    void materialPenaltyExponent(const std::string& aInput);

    /******************************************************************************//**
     * \fn materialPenaltyExponent
     * \brief Return string value for keyword 'penalty power'.
     * \return output string
    **********************************************************************************/
    std::string materialPenaltyExponent() const;

    /******************************************************************************//**
     * \fn minErsatzMaterialConstant
     * \brief Set string value for keyword 'minimum ersatz material value'.
     * \param [in] aInput string value
    **********************************************************************************/
    void minErsatzMaterialConstant(const std::string& aInput);

    /******************************************************************************//**
     * \fn minErsatzMaterialConstant
     * \brief Return string value for keyword 'minimum ersatz material value'.
     * \return output string
    **********************************************************************************/
    std::string minErsatzMaterialConstant() const;
};
// struct Constraint

}
// namespace XMLGen
