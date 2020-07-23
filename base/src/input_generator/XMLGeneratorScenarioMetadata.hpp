/*
 * XMLGeneratorScenarioMetadata.hpp
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
 * \struct Scenario
 * \brief Scenario metadata for Plato problems.
**********************************************************************************/
struct Scenario
{
// private member data
private:
    std::unordered_map<std::string, std::string> mMetaData; /*!< scenario metadata, map< tag, value > */

// private member functions
private:
    /******************************************************************************//**
     * \fn getValue
     * \brief Return string value for property with input tag; else, throw an error if \n
     * property is not defined in the metadata.
     * \param [in] aTag property tag
     * \return property string value
    **********************************************************************************/
    std::string getValue(const std::string& aTag) const;

    /******************************************************************************//**
     * \fn getBool
     * \brief Return bool value for property with input tag; else, throw an error if \n
     * property is not defined in the metadata.
     * \param [in] aTag property tag
     * \return property bool value
    **********************************************************************************/
    bool getBool(const std::string& aTag) const;

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
     * \fn physics
     * \brief Set string value for keyword 'physics'.
     * \param [in] aInput string value
    **********************************************************************************/
    void physics(const std::string& aInput);

    /******************************************************************************//**
     * \fn physics
     * \brief Return string value for keyword 'physics'.
     * \return output string
    **********************************************************************************/
    std::string physics() const;

    /******************************************************************************//**
     * \fn dimensions
     * \brief Set string value for keyword 'dimensions'.
     * \param [in] aInput string value
    **********************************************************************************/
    void dimensions(const std::string& aInput);

    /******************************************************************************//**
     * \fn dimensions
     * \brief Return string value for keyword 'dimensions'.
     * \return output string
    **********************************************************************************/
    std::string dimensions() const;

    /******************************************************************************//**
     * \fn materialPenaltyExponent
     * \brief Set string value for keyword 'material_penalty_exponent'.
     * \param [in] aInput string value
    **********************************************************************************/
    void materialPenaltyExponent(const std::string& aInput);

    /******************************************************************************//**
     * \fn materialPenaltyExponent
     * \brief Return string value for keyword 'material_penalty_exponent'.
     * \return output string
    **********************************************************************************/
    std::string materialPenaltyExponent() const;

    /******************************************************************************//**
     * \fn minErsatzMaterialConstant
     * \brief Set string value for keyword 'minimum_ersatz_material_value'.
     * \param [in] aInput string value
    **********************************************************************************/
    void minErsatzMaterialConstant(const std::string& aInput);

    /******************************************************************************//**
     * \fn minErsatzMaterialConstant
     * \brief Return string value for keyword 'minimum_ersatz_material_value'.
     * \return output string
    **********************************************************************************/
    std::string minErsatzMaterialConstant() const;

    /******************************************************************************//**
     * \fn additiveContinuation
     * \brief Set string value for keyword 'additive_continuation'.
     * \param [in] aInput string value
    **********************************************************************************/
    void additiveContinuation(const std::string& aInput);

    /******************************************************************************//**
     * \fn additiveContinuation
     * \brief Return string value for keyword 'additive_continuation'.
     * \return output string
    **********************************************************************************/
    std::string additiveContinuation() const;

    /******************************************************************************//**
     * \fn cacheState
     * \brief Set string value for keyword 'enable_cache_state'.
     * \param [in] aInput string value
    **********************************************************************************/
    void cacheState(const std::string& aInput);

    /******************************************************************************//**
     * \fn cacheState
     * \brief Return bool value for keyword 'enable_cache_state'.
     * \return output bool
    **********************************************************************************/
    bool cacheState() const;

    /******************************************************************************//**
     * \fn updateProblem
     * \brief Set string value for keyword 'enable_update_problem'.
     * \param [in] aInput string value
    **********************************************************************************/
    void updateProblem(const std::string& aInput);

    /******************************************************************************//**
     * \fn updateProblem
     * \brief Return bool value for keyword 'enable_update_problem'.
     * \return output bool
    **********************************************************************************/
    bool updateProblem() const;

    /******************************************************************************//**
     * \fn useNewAnalyzeUQWorkflow
     * \brief Set string value for keyword 'use_new_analyze_uq_workflow'.
     * \param [in] aInput string value
    **********************************************************************************/
    void useNewAnalyzeUQWorkflow(const std::string& aInput);

    /******************************************************************************//**
     * \fn useNewAnalyzeUQWorkflow
     * \brief Return bool value for keyword 'use_new_analyze_uq_workflow'.
     * \return output bool
    **********************************************************************************/
    bool useNewAnalyzeUQWorkflow() const;
};
// struct Scenario

}
