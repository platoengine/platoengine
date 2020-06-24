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
    auto tLowerCriterion = Plato::tolower(aCriterion.category());
    auto tItr = tValidKeys.mKeys.find(tLowerCriterion);
    if (tItr == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Is Criterion Supported in Plato Analyze: Criterion '")
            + tLowerCriterion + "' is not supported.")
    }
    return tItr->second.first;
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
    tValues = {"Minimum Value", "double", aMetadata.minErsatzMaterialConstant()};
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
void append_scalar_function_criterion
(const Criterion& aCriterion,
 pugi::xml_node& aParentNode)
{
    auto tDesignCriterionName = XMLGen::Private::is_criterion_supported_in_plato_analyze(aCriterion);

    auto tName = std::string("my ") + Plato::tolower(aCriterion.category());
    auto tObjective = aParentNode.append_child("ParameterList");
    std::vector<std::string> tKeys = {"name"};
    std::vector<std::string> tValues = {tName};
    XMLGen::append_attributes(tKeys, tValues, tObjective);

    tKeys = {"name", "type", "value"}; tValues = {"Type", "string", "Scalar Function"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tObjective);
    tValues = {"Scalar Function Type", "string", tDesignCriterionName};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tObjective);
    XMLGen::Private::append_simp_penalty_function(aCriterion, tObjective);
}

/******************************************************************************//**
 * \fn append_pnorm_criterion
 * \tparam Criterion criterion metadata
 * \brief Append p-norm function parameters to criterion parameter list.
 * \param [in] aCriterion criterion metadata
 * \param [in/out] aParentNode  pugi::xml_node
 **********************************************************************************/
template<typename Criterion>
void append_pnorm_criterion
(const Criterion& aCriterion,
 pugi::xml_node& aParentNode)
{
    XMLGen::Private::append_scalar_function_criterion(aCriterion, aParentNode);
    auto tCriterion = aParentNode.child("ParameterList");
    if(tCriterion.empty())
    {
        THROWERR("Append P-Norm Criterion: Criterion parameter list is empty. Most likely, "
            + "there was an error appending the scalar function criterion.")
    }
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Exponent", "double", aCriterion.pnormExponent()};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tCriterion);
}

}
// namespace Private

}
// namespace XMLGen
