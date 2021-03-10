/*
 * XMLGeneratorServiceMetadata.cpp
 *
 *  Created on: Jul 22, 2020
 */

#include "XMLG_Macros.hpp"
#include "Plato_FreeFunctions.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorServiceMetadata.hpp"

namespace XMLGen
{

std::string Service::getValue(const std::string& aTag) const
{
    auto tItr = mMetaData.find(aTag);
    if(tItr == mMetaData.end())
    {
        return "";
    }
    return tItr->second;
}

bool Service::getBool(const std::string& aTag) const
{
    auto tItr = mMetaData.find(aTag);
    if(tItr == mMetaData.end())
    {
        THROWERR(std::string("XML Generator Service Metadata: '") + aTag + "' keyword is not defined.")
    }
    return (XMLGen::transform_boolean_key(tItr->second));
}

std::string Service::value(const std::string& aTag) const
{
    auto tTag = Plato::tolower(aTag);
    auto tItr = mMetaData.find(tTag);
    if(tItr == mMetaData.end())
    {
        THROWERR(std::string("XML Generator Service Metadata: Parameter with tag '") + aTag + "' is not defined in metadata.")
    }
    return (tItr->second);
}

std::vector<std::string> Service::tags() const
{
    std::vector<std::string> tTags;
    for(auto& tProperty : mMetaData)
    {
        tTags.push_back(tProperty.first);
    }
    return tTags;
}

void Service::append(const std::string& aTag, const std::string& aValue)
{
    if (aTag.empty())
    {
        THROWERR(std::string("XML Generator Service Metadata: Input tag '") + aTag + "' is empty.")
    }
    auto tTag = Plato::tolower(aTag);
    mMetaData[aTag] = aValue;
}

void Service::id(const std::string& aInput)
{
    mMetaData["id"] = aInput;
}

std::string Service::id() const
{
    return (this->getValue("id"));
}

void Service::code(const std::string& aInput)
{
    mMetaData["code"] = aInput;
}

std::string Service::code() const
{
    return (this->getValue("code"));
}

void Service::deviceIDs(const std::vector<std::string>& aInput)
{
    mDeviceIDs = aInput;
}

std::vector<std::string> Service::deviceIDs() const
{
    return mDeviceIDs;
}

void Service::numberRanks(const std::string& aInput)
{
    mMetaData["number_ranks"] = aInput;
}

std::string Service::numberRanks() const
{
    return (this->getValue("number_ranks"));
}

void Service::numberProcessors(const std::string& aInput)
{
    mMetaData["number_processors"] = aInput;
}

std::string Service::numberProcessors() const
{
    return (this->getValue("number_processors"));
}

void Service::dimensions(const std::string& aInput)
{
    mMetaData["dimensions"] = aInput;
}

std::string Service::dimensions() const
{
    return (this->getValue("dimensions"));
}

void Service::materialPenaltyExponent(const std::string& aInput)
{
    mMetaData["material_penalty_exponent"] = aInput;
}

std::string Service::materialPenaltyExponent() const
{
    return (this->getValue("material_penalty_exponent"));
}

void Service::minErsatzMaterialConstant(const std::string& aInput)
{
    mMetaData["minimum_ersatz_material_value"] = aInput;
}

std::string Service::minErsatzMaterialConstant() const
{
    return (this->getValue("minimum_ersatz_material_value"));
}

void Service::additiveContinuation(const std::string& aInput)
{
    mMetaData["additive_continuation"] = aInput;
}

std::string Service::additiveContinuation() const
{
    return (this->getValue("additive_continuation"));
}

void Service::maxNumTimeSteps(const std::string& aInput)
{
    mMetaData["max_number_time_steps"] = aInput;
}

std::string Service::maxNumTimeSteps() const
{
    return (this->getValue("max_number_time_steps"));
}

void Service::timeStepExpansion(const std::string& aInput)
{
    mMetaData["time_step_expansion_multiplier"] = aInput;
}

std::string Service::timeStepExpansion() const
{
    return (this->getValue("time_step_expansion_multiplier"));
}

void Service::newtonSolverTolerance(const std::string& aInput)
{
    mMetaData["newton_solver_tolerance"] = aInput;
}

std::string Service::newtonSolverTolerance() const
{
    return (this->getValue("newton_solver_tolerance"));
}

void Service::linearSolverTolerance(const std::string& aInput)
{
    mMetaData["linear_solver_tolerance"] = aInput;
}

std::string Service::linearSolverTolerance() const
{
    return (this->getValue("linear_solver_tolerance"));
}

void Service::solverConvergenceCriterion(const std::string& aInput)
{
    mMetaData["convergence_criterion"] = aInput;
}

std::string Service::solverConvergenceCriterion() const
{
    return (this->getValue("convergence_criterion"));
}

void Service::solverMaxNumIterations(const std::string& aInput)
{
    mMetaData["max_number_iterations"] = aInput;
}

std::string Service::solverMaxNumIterations() const
{
    return (this->getValue("max_number_iterations"));
}

std::string Service::performer() const
{
    return (code() + "_" + id());
}

void Service::cacheState(const std::string& aInput)
{
    mMetaData["cache_state"] = aInput;
}

bool Service::cacheState() const
{
    return (this->getBool("cache_state"));
}

void Service::updateProblem(const std::string& aInput)
{
    mMetaData["update_problem"] = aInput;
}

bool Service::updateProblem() const
{
    return (this->getBool("update_problem"));
}

}
// namespace XMLGen
