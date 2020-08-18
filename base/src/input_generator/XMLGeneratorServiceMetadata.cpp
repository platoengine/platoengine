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
        THROWERR(std::string("XML Generator Service Metadata: Parameter with tag '") + aTag + "' is empty.")
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

void Service::additiveContinuation(const std::string& aInput)
{
    mMetaData["additive_continuation"] = aInput;
}

std::string Service::additiveContinuation() const
{
    return (this->getValue("additive_continuation"));
}

void Service::timeStep(const std::string& aInput)
{
    mMetaData["time_step"] = aInput;
}

std::string Service::timeStep() const
{
    return (this->getValue("time_step"));
}

void Service::numTimeSteps(const std::string& aInput)
{
    mMetaData["number_time_steps"] = aInput;
}

std::string Service::numTimeSteps() const
{
    return (this->getValue("number_time_steps"));
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

void Service::newmarkBeta(const std::string& aInput)
{
    mMetaData["newmark_beta"] = aInput;
}

std::string Service::newmarkBeta() const
{
    return (this->getValue("newmark_beta"));
}

void Service::newmarkGamma(const std::string& aInput)
{
    mMetaData["newmark_gamma"] = aInput;
}

std::string Service::newmarkGamma() const
{
    return (this->getValue("newmark_gamma"));
}

void Service::solverTolerance(const std::string& aInput)
{
    mMetaData["tolerance"] = aInput;
}

std::string Service::solverTolerance() const
{
    return (this->getValue("tolerance"));
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

void Service::performer(const std::string& aInput)
{
    mMetaData["performer"] = aInput;
}

std::string Service::performer() const
{
    return (this->getValue("performer"));
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
