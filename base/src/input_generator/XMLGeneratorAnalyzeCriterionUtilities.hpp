/*
 * XMLGeneratorAnalyzeCriterionUtilities.hpp
 *
 *  Created on: Jun 15, 2020
 */

#pragma once

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorValidInputKeys.hpp"

namespace XMLGen
{

namespace Private
{

/******************************************************************************//**
 * \fn is_criterion_supported_in_plato_analyze
 * \tparam Criterion criterion metadata
 * \brief Return Plato Analyze's criterion keyword if criterion is supported in Plato Analyze.
 * \param [in] aCriterion criterion metadata
 **********************************************************************************/
template<typename Criterion>
std::string is_criterion_supported_in_plato_analyze
(const Criterion& aCriterion)
{
    XMLGen::ValidAnalyzeCriteriaKeys tValidKeys;
    auto tLowerCriterion = Plato::tolower(aCriterion.type());
    auto tItr = tValidKeys.mKeys.find(tLowerCriterion);
    if (tItr == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Is Criterion Supported in Plato Analyze: Criterion '")
            + tLowerCriterion + "' is not supported.")
    }
    return tItr->second.first;
}

/******************************************************************************//**
 * \fn is_criterion_linear
 * \tparam Criterion criterion metadata
 * \brief Return Plato Analyze's criterion linearity type 
 * \param [in] aCriterion criterion metadata
 **********************************************************************************/
template<typename Criterion>
std::string is_criterion_linear
(const Criterion& aCriterion)
{
    XMLGen::ValidAnalyzeCriteriaIsLinearKeys tValidKeys;
    auto tLowerCriterion = Plato::tolower(aCriterion.type());
    auto tItr = tValidKeys.mKeys.find(tLowerCriterion);
    if (tItr == tValidKeys.mKeys.end())
    {
        return "false";
    }
    return tItr->second;
}

/******************************************************************************//**
 * \fn append_simp_penalty_function
 * \tparam MetaData criterion metadata
 * \brief Append SIMP penalty model parameters to criterion parameter list.
 * \param [in] aCriterion criterion metadata
 * \param [in/out] aParentNode  pugi::xml_node
 **********************************************************************************/
template<typename MetaData>
void append_simp_penalty_function
(const MetaData& aMetadata,
 pugi::xml_node& aParentNode)
{
    auto tPenaltyFunction = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Penalty Function"}, tPenaltyFunction);

    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Type", "string", "SIMP"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tPenaltyFunction);
    tValues = {"Exponent", "double", aMetadata.materialPenaltyExponent()};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tPenaltyFunction);
    auto tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.minErsatzMaterialConstant());
    tValues = {"Minimum Value", "double", tPropertyValue};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tPenaltyFunction);
}

/******************************************************************************//**
 * \fn append_scalar_function_criterion
 * \tparam Criterion criterion metadata
 * \brief Append scalar function parameters to criterion parameter list.
 * \param [in] aCriterion criterion metadata
 * \param [in/out] aParentNode  pugi::xml_node
 **********************************************************************************/
template<typename Criterion>
pugi::xml_node append_scalar_function_criterion
(const Criterion& aCriterion,
 pugi::xml_node& aParentNode)
{
    auto tDesignCriterionName = XMLGen::Private::is_criterion_supported_in_plato_analyze(aCriterion);
    auto tCriterionLinearFlag = XMLGen::Private::is_criterion_linear(aCriterion);

    auto tName = std::string("my ") + Plato::tolower(aCriterion.type());
    //auto tName = std::string("my ") + Plato::tolower(aCriterion.category());
    auto tObjective = aParentNode.append_child("ParameterList");
    std::vector<std::string> tKeys = {"name"};
    std::vector<std::string> tValues = {tName};
    XMLGen::append_attributes(tKeys, tValues, tObjective);

    tKeys = {"name", "type", "value"}; tValues = {"Type", "string", "Scalar Function"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tObjective);
    if(tCriterionLinearFlag == "true")
    {
        tKeys = {"name", "type", "value"}; tValues = {"Linear", "bool", "true"};
        XMLGen::append_parameter_plus_attributes(tKeys, tValues, tObjective);
    }
    tValues = {"Scalar Function Type", "string", tDesignCriterionName};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tObjective);
    XMLGen::Private::append_simp_penalty_function(aCriterion, tObjective);
    return tObjective;
}

/******************************************************************************//**
 * \fn append_pnorm_criterion
 * \tparam Criterion criterion metadata
 * \brief Append p-norm function parameters to criterion parameter list.
 * \param [in] aCriterion criterion metadata
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
template<typename Criterion>
pugi::xml_node append_pnorm_criterion
(const Criterion& aCriterion,
 pugi::xml_node& aParentNode)
{
    auto tCriterion = XMLGen::Private::append_scalar_function_criterion(aCriterion, aParentNode);
 //   auto tCriterion = aParentNode.child("ParameterList");
    if(tCriterion.empty())
    {
        THROWERR("Append P-Norm Criterion: Criterion parameter list is empty. Most likely, "
            + "there was an error appending the scalar function criterion.")
    }
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Exponent", "double", aCriterion.pnormExponent()};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tCriterion);
    return tCriterion;
}

/******************************************************************************//**
 * \fn append_stress_constrained_mass_minimization_criterion_parameters
 * \brief Append stress constrained mass_minimization criterion input parameters \n
 * to criterion parameter list.
 * \param [in] aCriterion criterion metadata
 * \param [in/out] aParentNode  pugi::xml_node
 **********************************************************************************/
inline void append_stress_constrained_mass_minimization_criterion_parameters
(const XMLGen::Criterion& aCriterion,
 pugi::xml_node& aParentNode)
{
    // append stress constrained mass minimization scalar function parameters
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Stress Limit", "double", aCriterion.stressLimit()};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);

    std::string tWeight = (aCriterion.scmmMassWeight() == "" ? std::string("1.0") : aCriterion.scmmMassWeight());
    tValues = {"Mass Criterion Weight", "double", tWeight};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);

    tWeight = (aCriterion.scmmStressWeight() == "" ? std::string("1.0") : aCriterion.scmmStressWeight());
    tValues = {"Stress Criterion Weight", "double", tWeight};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);

    tValues = {"Initial Penalty", "double", aCriterion.scmmInitialPenalty()};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);

    tValues = {"Penalty Upper Bound", "double", aCriterion.scmmPenaltyUpperBound()};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);

    tValues = {"Penalty Expansion Multiplier", "double", aCriterion.scmmPenaltyExpansionMultiplier()};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}

/******************************************************************************//**
 * \fn append_stress_constrained_mass_minimization_criterion
 * \brief Append stress constrained mass_minimization criterion to criterion parameter list.
 * \param [in] aCriterion criterion metadata
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
inline pugi::xml_node append_stress_constrained_mass_minimization_criterion
(const XMLGen::Criterion& aCriterion,
 pugi::xml_node& aParentNode)
{
    auto tDesignCriterionName = XMLGen::Private::is_criterion_supported_in_plato_analyze(aCriterion);
    auto tName = std::string("my ") + Plato::tolower(aCriterion.type());
    auto tObjective = aParentNode.append_child("ParameterList");
    std::vector<std::string> tKeys = {"name"};
    std::vector<std::string> tValues = {tName};
    XMLGen::append_attributes(tKeys, tValues, tObjective);

    // append stress constrained mass minimization scalar function
    tKeys = {"name", "type", "value"};
    tValues = {"Type", "string", "Scalar Function"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tObjective);
    tValues = {"Scalar Function Type", "string", tDesignCriterionName};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tObjective);
    XMLGen::Private::append_stress_constrained_mass_minimization_criterion_parameters(aCriterion, tObjective);
    return tObjective;
}

}
// namespace Private

}
// namespace XMLGen
