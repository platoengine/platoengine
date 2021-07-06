/*
 * XMLGeneratorScenarioMetadata.cpp
 *
 *  Created on: Jul 22, 2020
 */

#include "XMLG_Macros.hpp"
#include "Plato_FreeFunctions.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorScenarioMetadata.hpp"

namespace XMLGen
{

std::string Scenario::getValue(const std::string& aTag) const
{
    auto tItr = mMetaData.find(aTag);
    if(tItr == mMetaData.end())
    {
        return "";
    }
    return tItr->second;
}

bool Scenario::getBool(const std::string& aTag) const
{
    auto tItr = mMetaData.find(aTag);
    if(tItr == mMetaData.end())
    {
        THROWERR(std::string("XML Generator Scenario Metadata: '") + aTag + "' keyword is not defined.")
    }
    return (XMLGen::transform_boolean_key(tItr->second));
}

std::string Scenario::value(const std::string& aTag) const
{
    auto tTag = Plato::tolower(aTag);
    auto tItr = mMetaData.find(tTag);
    if(tItr == mMetaData.end())
    {
        THROWERR(std::string("XML Generator Scenario Metadata: Parameter with tag '") + aTag + "' is not defined in metadata.")
    }
    return (tItr->second);
}

std::vector<std::string> Scenario::tags() const
{
    std::vector<std::string> tTags;
    for(auto& tProperty : mMetaData)
    {
        tTags.push_back(tProperty.first);
    }
    return tTags;
}

void Scenario::append(const std::string& aTag, const std::string& aValue)
{
    if (aTag.empty())
    {
        THROWERR(std::string("XML Generator Scenario Metadata: Parameter with tag '") + aTag + "' is empty.")
    }
    auto tTag = Plato::tolower(aTag);
    mMetaData[aTag] = aValue;
}

void Scenario::id(const std::string& aInput)
{
    mMetaData["id"] = aInput;
}

std::string Scenario::id() const
{
    return (this->getValue("id"));
}

void Scenario::physics(const std::string& aInput)
{
    mMetaData["physics"] = aInput;
}

std::string Scenario::physics() const
{
    return (this->getValue("physics"));
}

void Scenario::dimensions(const std::string& aInput)
{
    mMetaData["dimensions"] = aInput;
}

std::string Scenario::dimensions() const
{
    return (this->getValue("dimensions"));
}

void Scenario::materialPenaltyExponent(const std::string& aInput)
{
    mMetaData["material_penalty_exponent"] = aInput;
}

std::string Scenario::materialPenaltyExponent() const
{
    return (this->getValue("material_penalty_exponent"));
}

void Scenario::minErsatzMaterialConstant(const std::string& aInput)
{
    mMetaData["minimum_ersatz_material_value"] = aInput;
}

std::string Scenario::minErsatzMaterialConstant() const
{
    return (this->getValue("minimum_ersatz_material_value"));
}

std::string Scenario::material() const
{
    return (this->getValue("material"));
}

void Scenario::timeStep(const std::string& aInput)
{
    mMetaData["time_step"] = aInput;
}

std::string Scenario::timeStep() const
{
    return (this->getValue("time_step"));
}

void Scenario::numTimeSteps(const std::string& aInput)
{
    mMetaData["number_time_steps"] = aInput;
}

std::string Scenario::numTimeSteps() const
{
    return (this->getValue("number_time_steps"));
}

void Scenario::maxNumTimeSteps(const std::string& aInput)
{
    mMetaData["max_number_time_steps"] = aInput;
}

std::string Scenario::maxNumTimeSteps() const
{
    return (this->getValue("max_number_time_steps"));
}

void Scenario::timeStepExpansion(const std::string& aInput)
{
    mMetaData["time_step_expansion_multiplier"] = aInput;
}

std::string Scenario::timeStepExpansion() const
{
    return (this->getValue("time_step_expansion_multiplier"));
}

void Scenario::newmarkBeta(const std::string& aInput)
{
    mMetaData["newmark_beta"] = aInput;
}

std::string Scenario::newmarkBeta() const
{
    return (this->getValue("newmark_beta"));
}

void Scenario::newmarkGamma(const std::string& aInput)
{
    mMetaData["newmark_gamma"] = aInput;
}

std::string Scenario::newmarkGamma() const
{
    return (this->getValue("newmark_gamma"));
}

void Scenario::solverTolerance(const std::string& aInput)
{
    mMetaData["tolerance"] = aInput;
}

std::string Scenario::solverTolerance() const
{
    return (this->getValue("tolerance"));
}

void Scenario::newtonSolverTolerance(const std::string& aInput)
{
    mMetaData["newton_solver_tolerance"] = aInput;
}

std::string Scenario::newtonSolverTolerance() const
{
    return (this->getValue("newton_solver_tolerance"));
}

void Scenario::solverConvergenceCriterion(const std::string& aInput)
{
    mMetaData["convergence_criterion"] = aInput;
}

std::string Scenario::solverConvergenceCriterion() const
{
    return (this->getValue("convergence_criterion"));
}

void Scenario::solverMaxNumIterations(const std::string& aInput)
{
    mMetaData["max_number_iterations"] = aInput;
}

std::string Scenario::solverMaxNumIterations() const
{
    return (this->getValue("max_number_iterations"));
}

void Scenario::additiveContinuation(const std::string& aInput)
{
    mMetaData["additive_continuation"] = aInput;
}

std::string Scenario::additiveContinuation() const
{
    return (this->getValue("additive_continuation"));
}

void Scenario::weightMassScaleFactor(const std::string& aInput)
{
    mMetaData["weight_mass_scale_factor"] = aInput;
}

std::string Scenario::weightMassScaleFactor() const
{
    return (this->getValue("weight_mass_scale_factor"));
}

void Scenario::shapeSideset(const std::string& aInput)
{
    mMetaData["shape_sideset"] = aInput;
}

std::string Scenario::shapeSideset() const
{
    return (this->getValue("shape_sideset"));
}

void Scenario::pressureScaling(const std::string& aInput)
{
    mMetaData["pressure_scaling"] = aInput;
}

std::string Scenario::pressureScaling() const
{
    return (this->getValue("pressure_scaling"));
}

void Scenario::temperatureScaling(const std::string& aInput)
{
    mMetaData["temperature_scaling"] = aInput;
}

std::string Scenario::temperatureScaling() const
{
    return (this->getValue("temperature_scaling"));
}


}
// namespace XMLGen
