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
    std::unordered_map<std::string, std::string> mMetaData; /*!< service metadata, map< tag, value > */

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
     * \fn id
     * \brief Return string value for keyword 'id'.
     * \return output string
    **********************************************************************************/
    std::string id() const;

    /******************************************************************************//**
     * \fn criterion
     * \brief Set string value for keyword 'criterion'.
     * \param [in] aInput string value
    **********************************************************************************/
    void criterion(const std::string& aInput);

    /******************************************************************************//**
     * \fn criterion
     * \brief Return string value for keyword 'criterion'.
     * \return output string
    **********************************************************************************/
    std::string criterion() const;

    /******************************************************************************//**
     * \fn service
     * \brief Set string value for keyword 'service'.
     * \param [in] aInput string value
    **********************************************************************************/
    void service(const std::string& aInput);

    /******************************************************************************//**
     * \fn service
     * \brief Return string value for keyword 'service'.
     * \return output string
    **********************************************************************************/
    std::string service() const;

    /******************************************************************************//**
     * \fn scenario
     * \brief Set string value for keyword 'scenario'.
     * \param [in] aInput string value
    **********************************************************************************/
    void scenario(const std::string& aInput);

    /******************************************************************************//**
     * \fn scenario
     * \brief Return string value for keyword 'scenario'.
     * \return output string
    **********************************************************************************/
    std::string scenario() const;
    /******************************************************************************//**
     * \fn relativeTarget
     * \brief Set string value for keyword 'relative_target'.
     * \param [in] aInput string value
    **********************************************************************************/
    void relativeTarget(const std::string& aInput);

    /******************************************************************************//**
     * \fn relativeTarget
     * \brief Return string value for keyword 'relative_target'.
     * \return output string
    **********************************************************************************/
    std::string relativeTarget() const;

    /******************************************************************************//**
     * \fn absoluteTarget
     * \brief Set string value for keyword 'absolute_target'.
     * \param [in] aInput string value
    **********************************************************************************/
    void absoluteTarget(const std::string& aInput);

    /******************************************************************************//**
     * \fn absoluteTarget
     * \brief Return string value for keyword 'absolute_target'.
     * \return output string
    **********************************************************************************/
    std::string absoluteTarget() const;

    /******************************************************************************//**
     * \fn weight
     * \brief Set string value for keyword 'weight'.
     * \param [in] aInput string value
     **********************************************************************************/
    void weight(const std::string& aInput);

    /******************************************************************************//**
     * \fn weight
     * \brief Return string value for keyword 'weight'.
     * \return value
     **********************************************************************************/
    std::string weight() const;
};
// struct Constraint

}
// namespace XMLGen
