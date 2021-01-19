/*
 * XMLGeneratorOptimizationParametersMetadata.cpp
 *
 *  Created on: Jan 11, 2021
 */

#include "XMLG_Macros.hpp"
#include "Plato_FreeFunctions.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorOptimizationParametersMetadata.hpp"

namespace XMLGen
{

std::string OptimizationParameters::getValue(const std::string& aTag) const
{
    auto tItr = mMetaData.find(aTag);
    if(tItr == mMetaData.end())
    {
        return "";
    }
    return tItr->second;
}

bool OptimizationParameters::getBool(const std::string& aTag) const
{
    auto tItr = mMetaData.find(aTag);
    if(tItr == mMetaData.end())
    {
        THROWERR(std::string("XML Generator OptimizationParameters Metadata: '") + aTag + "' keyword is not defined.")
    }
    return (XMLGen::transform_boolean_key(tItr->second));
}

std::string OptimizationParameters::value(const std::string& aTag) const
{
    auto tTag = Plato::tolower(aTag);
    auto tItr = mMetaData.find(tTag);
    if(tItr == mMetaData.end())
    {
        return "";
    }
    return (tItr->second);
}

std::vector<std::string> OptimizationParameters::tags() const
{
    std::vector<std::string> tTags;
    for(auto& tProperty : mMetaData)
    {
        tTags.push_back(tProperty.first);
    }
    return tTags;
}

void OptimizationParameters::append(const std::string& aTag, const std::string& aValue)
{
    if (aTag.empty())
    {
        THROWERR(std::string("XML Generator OptimizationParameters Metadata: Parameter with tag '") + aTag + "' is empty.")
    }
    auto tTag = Plato::tolower(aTag);
    mMetaData[aTag] = aValue;
}

bool OptimizationParameters::needsMeshMap() const
{
    if(mSymmetryPlaneOrigin.size() == 3 && mSymmetryPlaneNormal.size() == 3)
    {
        return true;
    }
    return false;
}

std::string OptimizationParameters::filter_before_symmetry_enforcement() const
{
    return (this->getValue("filter_before_symmetry_enforcement"));
}

std::string OptimizationParameters::mesh_map_filter_radius() const
{
    return (this->getValue("mesh_map_filter_radius"));
}




}
// namespace XMLGen
