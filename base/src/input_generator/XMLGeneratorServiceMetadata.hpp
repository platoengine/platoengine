/*
 * XMLGeneratorServiceMetadata.hpp
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
 * \struct Service
 * \brief Service metadata for Plato problems.
**********************************************************************************/
struct Service
{
// private member data
private:
    std::unordered_map<std::string, std::string> mMetaData; /*!< Service metadata, map< tag, value > */

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
     * \return value
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
     * \return value
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
     * \return value
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
     * \return value
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
     * \return value
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
     * \return value
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
     * \return value
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
     * \return value
    **********************************************************************************/
    std::string additiveContinuation() const;

    /******************************************************************************//**
     * \fn timeStep
     * \brief Set string value for keyword 'time_step'.
     * \param [in] aInput string value
    **********************************************************************************/
    void timeStep(const std::string& aInput);

    /******************************************************************************//**
     * \fn timeStep
     * \brief Return string value for keyword 'time_step'.
     * \return value
    **********************************************************************************/
    std::string timeStep() const;

    /******************************************************************************//**
     * \fn numTimeSteps
     * \brief Set string value for keyword 'number_time_steps'.
     * \param [in] aInput string value
    **********************************************************************************/
    void numTimeSteps(const std::string& aInput);

    /******************************************************************************//**
     * \fn numTimeSteps
     * \brief Return string value for keyword 'number_time_steps'.
     * \return value
    **********************************************************************************/
    std::string numTimeSteps() const;

    /******************************************************************************//**
     * \fn maxNumTimeSteps
     * \brief Set string value for keyword 'max_number_time_steps'.
     * \param [in] aInput string value
    **********************************************************************************/
    void maxNumTimeSteps(const std::string& aInput);

    /******************************************************************************//**
     * \fn maxNumTimeSteps
     * \brief Return string value for keyword 'max_number_time_steps'.
     * \return value
    **********************************************************************************/
    std::string maxNumTimeSteps() const;

    /******************************************************************************//**
     * \fn timeStepExpansion
     * \brief Set string value for keyword 'time_step_expansion_multiplier'.
     * \param [in] aInput string value
    **********************************************************************************/
    void timeStepExpansion(const std::string& aInput);

    /******************************************************************************//**
     * \fn timeStepExpansion
     * \brief Return string value for keyword 'time_step_expansion_multiplier'.
     * \return value
    **********************************************************************************/
    std::string timeStepExpansion() const;

    /******************************************************************************//**
     * \fn newmarkBeta
     * \brief Set string value for keyword 'newmark_beta'.
     * \param [in] aInput string value
    **********************************************************************************/
    void newmarkBeta(const std::string& aInput);

    /******************************************************************************//**
     * \fn newmarkBeta
     * \brief Return string value for keyword 'newmark_beta'.
     * \return value
    **********************************************************************************/
    std::string newmarkBeta() const;

    /******************************************************************************//**
     * \fn newmarkGamma
     * \brief Set string value for keyword 'newmark_gamma'.
     * \param [in] aInput string value
    **********************************************************************************/
    void newmarkGamma(const std::string& aInput);

    /******************************************************************************//**
     * \fn newmarkGamma
     * \brief Return string value for keyword 'newmark_gamma'.
     * \return value
    **********************************************************************************/
    std::string newmarkGamma() const;

    /******************************************************************************//**
     * \fn solverTolerance
     * \brief Set string value for keyword 'tolerance'.
     * \param [in] aInput string value
    **********************************************************************************/
    void solverTolerance(const std::string& aInput);

    /******************************************************************************//**
     * \fn solverTolerance
     * \brief Return string value for keyword 'tolerance'.
     * \return value
    **********************************************************************************/
    std::string solverTolerance() const;

    /******************************************************************************//**
     * \fn solverConvergenceCriterion
     * \brief Set string value for keyword 'convergence_criterion'.
     * \param [in] aInput string value
    **********************************************************************************/
    void solverConvergenceCriterion(const std::string& aInput);

    /******************************************************************************//**
     * \fn solverConvergenceCriterion
     * \brief Return string value for keyword 'convergence_criterion'.
     * \return value
    **********************************************************************************/
    std::string solverConvergenceCriterion() const;

    /******************************************************************************//**
     * \fn solverMaxNumIterations
     * \brief Set string value for keyword 'max_number_iterations'.
     * \param [in] aInput string value
    **********************************************************************************/
    void solverMaxNumIterations(const std::string& aInput);

    /******************************************************************************//**
     * \fn solverMaxNumIterations
     * \brief Return string value for keyword 'max_number_iterations'.
     * \return value
    **********************************************************************************/
    std::string solverMaxNumIterations() const;

    /******************************************************************************//**
     * \fn cacheState
     * \brief Set string value for keyword 'cache_state'.
     * \param [in] aInput string value
    **********************************************************************************/
    void cacheState(const std::string& aInput);

    /******************************************************************************//**
     * \fn cacheState
     * \brief Return bool value for keyword 'cache_state'.
     * \return output bool
    **********************************************************************************/
    bool cacheState() const;

    /******************************************************************************//**
     * \fn updateProblem
     * \brief Set string value for keyword 'update_problem'.
     * \param [in] aInput string value
    **********************************************************************************/
    void updateProblem(const std::string& aInput);

    /******************************************************************************//**
     * \fn updateProblem
     * \brief Return bool value for keyword 'update_problem'.
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
// struct Service

}
