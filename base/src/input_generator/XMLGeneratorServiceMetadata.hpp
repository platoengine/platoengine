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
public:

// private member data
private:
    std::unordered_map<std::string, std::string> mMetaData; /*!< Service metadata, map< tag, value > */
    std::vector<std::string> mDeviceIDs;

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
     * \fn numberRanks
     * \brief Set string value for keyword 'number_ranks'.
     * \param [in] aInput string value
    **********************************************************************************/
    void numberRanks(const std::string& aInput);

    /******************************************************************************//**
     * \fn numberRanks
     * \brief Return string value for keyword 'number_ranks'.
     * \return value
    **********************************************************************************/
    std::string numberRanks() const;

    /******************************************************************************//**
     * \fn deviceIDs
     * \brief Set the vector of device ids
     * \param [in] aInput string value
    **********************************************************************************/
    void deviceIDs(const std::vector<std::string>& aInput);

    /******************************************************************************//**
     * \fn deviceIDs
     * \brief Return list of device ids
     * \return value
    **********************************************************************************/
    std::vector<std::string> deviceIDs() const;

    /******************************************************************************//**
     * \fn numberProcessors
     * \brief Set string value for keyword 'number_processors'.
     * \param [in] aInput string value
    **********************************************************************************/
    void numberProcessors(const std::string& aInput);

    /******************************************************************************//**
     * \fn numberProcessors
     * \brief Return string value for keyword 'number_processors'.
     * \return value
    **********************************************************************************/
    std::string numberProcessors() const;

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
     * \fn newtonSolverTolerance
     * \brief Set string value for keyword 'newton_solver_tolerance'.
     * \param [in] aInput string value
    **********************************************************************************/
    void newtonSolverTolerance(const std::string& aInput);

    /******************************************************************************//**
     * \fn newtonSolverTolerance
     * \brief Return string value for keyword 'newton_solver_tolerance'.
     * \return value
    **********************************************************************************/
    std::string newtonSolverTolerance() const;

    /******************************************************************************//**
     * \fn linearSolverTolerance
     * \brief Set string value for keyword 'linear_solver_tolerance'.
     * \param [in] aInput string value
    **********************************************************************************/
    void linearSolverTolerance(const std::string& aInput);

    /******************************************************************************//**
     * \fn linearSolverTolerance
     * \brief Return string value for keyword 'linear_solver_tolerance'.
     * \return value
    **********************************************************************************/
    std::string linearSolverTolerance() const;

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
     * \fn performer
     * \brief Return derived string value for performer.
     * \return value
    **********************************************************************************/
    std::string performer() const;

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
};
// struct Service

}
