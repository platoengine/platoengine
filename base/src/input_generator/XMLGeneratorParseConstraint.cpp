/*
 * XMLGeneratorParseConstraint.cpp
 *
 *  Created on: Jun 19, 2020
 */

#include <iostream>

#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorParseConstraint.hpp"

namespace XMLGen
{

void ParseConstraint::allocate()
{
    mTags.clear();

    mTags.insert({ "type", { {"type"}, "" } });
    mTags.insert({ "code", { {"code"}, "" } });
    mTags.insert({ "name", { {"name"}, "" } });
    mTags.insert({ "target absolute", { {"target", "absolute"}, "" } });
    mTags.insert({ "target normalized", { {"target", "normalized"}, "" } });
    mTags.insert({ "weight", { {"weight"}, "" } });
    mTags.insert({ "performer", { {"performer"}, "" } });
    mTags.insert({ "penalty power", { {"penalty", "power"}, "" } });
    mTags.insert({ "pnorm exponent", { {"pnorm", "exponent"}, "" } });
    mTags.insert({ "surface_area_sideset_id", { {"surface_area_sideset_id"}, "" } });
    mTags.insert({ "minimum ersatz material value", { {"minimum", "ersatz", "material", "value"}, "" } });

    // TO BE DEPRECATED SOON!
    mTags.insert({ "surface_area", { {"surface_area"}, "" } });
    mTags.insert({ "volume absolute", { {"volume", "absolute"}, "" } });
    mTags.insert({ "volume fraction", { {"volume", "fraction"}, "" } });
}

void ParseConstraint::finalize()
{
    this->setIdentification();
    this->checkTargeValue();
}

void ParseConstraint::checkTargeValue()
{
    for(auto& tConstraint : mData)
    {
        if ((tConstraint.normalizedTarget().empty()) && (tConstraint.absoluteTarget().empty()))
        {
            THROWERR(std::string("Parse Constraint: 'target normalized' and 'target absolute' keywords are empty in constraint with category '")
              + tConstraint.category() + "', tag '" + tConstraint.name() + "', and computed by '" + tConstraint.code()
              + "'.\nOne of the two keywords: 'target normalized' or 'target absolute' " + "must be defined in a constrained optimization problem.")
        }
    }
}

void ParseConstraint::setIdentification()
{
    for (auto &tOuterCriterion : mData)
    {
        // For each code name we will make sure there are names set
        auto tMyCodeName = tOuterCriterion.code();
        size_t tObjectiveIdentificationNumber = 0;
        for (auto &tInnerCriterion : mData)
        {
            if (!tInnerCriterion.code().compare(tMyCodeName))
            {
                tObjectiveIdentificationNumber++;
                if (tInnerCriterion.name().empty())
                {
                    tInnerCriterion.name(std::to_string(tObjectiveIdentificationNumber));
                }
            }
        }
    }
}

void ParseConstraint::setCode(XMLGen::Constraint &aMetadata)
{
    auto tItr = mTags.find("code");
    if (tItr != mTags.end() && !tItr->second.second.empty())
    {
        auto tValidTag = XMLGen::check_code_keyword(tItr->second.second);
        aMetadata.code(tValidTag);
    }
    else
    {
        auto tValidTag = XMLGen::check_code_keyword("plato_analyze");
        aMetadata.code(tValidTag);
    }
}

void ParseConstraint::setName(XMLGen::Constraint &aMetadata)
{
    auto tItr = mTags.find("name");
    if (tItr != mTags.end() && !tItr->second.second.empty())
    {
        aMetadata.name(tItr->second.second);
    }
}

void ParseConstraint::setCategory(XMLGen::Constraint &aMetadata)
{
    auto tItr = mTags.find("type");
    if (tItr != mTags.end() && !tItr->second.second.empty())
    {
        auto tCategory = XMLGen::check_criterion_category_keyword(tItr->second.second);
        aMetadata.category(tCategory);
    }
    else
    {
        THROWERR(std::string("Parse Constraint: constraint 'type' keyword is empty. ")
            + "At least one constraint must be defined for a constrained optimization problem.")
    }
}

void ParseConstraint::setWeight(XMLGen::Constraint &aMetadata)
{
    auto tItr = mTags.find("weight");
    if (tItr != mTags.end() && !tItr->second.second.empty())
    {
        aMetadata.weight(tItr->second.second);
    }
    else
    {
        aMetadata.weight("1.0");
    }
}

void ParseConstraint::setPerformer(XMLGen::Constraint &aMetadata)
{
    auto tItr = mTags.find("performer");
    if (tItr != mTags.end() && !tItr->second.second.empty())
    {
        aMetadata.performer(tItr->second.second);
    }
    else
    {
        auto tPerformer = aMetadata.code();
        if(tPerformer.empty())
        {
            THROWERR("Parse Constraint: 'code' keyword is empty. Default performer is set to the 'code' keyword.")
        }
        aMetadata.performer(tPerformer);
    }
}

void ParseConstraint::setPnormExponent(XMLGen::Constraint &aMetadata)
{
    auto tItr = mTags.find("pnorm exponent");
    if (tItr != mTags.end() && !tItr->second.second.empty())
    {
        aMetadata.pnormExponent(tItr->second.second);
    }
    else
    {
        aMetadata.pnormExponent("6.0");
    }
}

void ParseConstraint::setNormalizedTarget(XMLGen::Constraint &aMetadata)
{
    XMLGen::ValidConstraintTargetNormalizedKeys tValidKeys;
    for(auto tKey : tValidKeys.mKeys)
    {
        auto tItr = mTags.find(tKey);
        if (tItr != mTags.end() && !tItr->second.second.empty())
        {
            aMetadata.normalizedTarget(tItr->second.second);
            break;
        }
    }
}

void ParseConstraint::setAbsoluteTarget(XMLGen::Constraint &aMetadata)
{
    XMLGen::ValidConstraintTargetAbsoluteKeys tValidKeys;
    for(auto tKey : tValidKeys.mKeys)
    {
        auto tItr = mTags.find(tKey);
        if (tItr != mTags.end() && !tItr->second.second.empty())
        {
            aMetadata.absoluteTarget(tItr->second.second);
            break;
        }
    }
}

void ParseConstraint::setSurfaceArea(XMLGen::Constraint &aMetadata)
{
    // TO BE DEPRECATED SOON!
    auto tItr = mTags.find("surface_area");
    if (tItr != mTags.end() && !tItr->second.second.empty())
    {
        aMetadata.surface_area = tItr->second.second;
    }

    auto tItro = mTags.find("surface_area_sideset_id");
    if (tItro != mTags.end() && !tItro->second.second.empty())
    {
        aMetadata.surface_area_ssid = tItro->second.second;
    }
}

void ParseConstraint::setMinimumErsatzValue(XMLGen::Constraint &aMetadata)
{
    auto tItr = mTags.find("minimum ersatz material value");
    if (tItr != mTags.end() && !tItr->second.second.empty())
    {
        aMetadata.minErsatzMaterialConstant(tItr->second.second);
    }
    else
    {
        aMetadata.minErsatzMaterialConstant("1e-9");
    }
}

void ParseConstraint::setPenaltyPower(XMLGen::Constraint &aMetadata)
{
    auto tItr = mTags.find("penalty power");
    if (tItr != mTags.end() && !tItr->second.second.empty())
    {
        aMetadata.materialPenaltyExponent(tItr->second.second);
    }
    else
    {
        aMetadata.materialPenaltyExponent("3.0");
    }
}

void ParseConstraint::setMetaData(XMLGen::Constraint& aMetadata)
{
    this->setCode(aMetadata);
    this->setName(aMetadata);
    this->setWeight(aMetadata);
    this->setCategory(aMetadata);
    this->setPerformer(aMetadata);
    this->setSurfaceArea(aMetadata);
    this->setPenaltyPower(aMetadata);
    this->setPnormExponent(aMetadata);
    this->setAbsoluteTarget(aMetadata);
    this->setNormalizedTarget(aMetadata);
    this->setMinimumErsatzValue(aMetadata);
}

std::vector<XMLGen::Constraint> ParseConstraint::data() const
{
    return mData;
}

void ParseConstraint::parse(std::istream &aInputFile)
{
    mData.clear();
    this->allocate();

    constexpr int MAX_CHARS_PER_LINE = 10000;
    std::vector<char> tBuffer(MAX_CHARS_PER_LINE);
    while (!aInputFile.eof())
    {
        // read an entire line into memory
        std::vector<std::string> tTokens;
        aInputFile.getline(tBuffer.data(), MAX_CHARS_PER_LINE);
        XMLGen::parse_tokens(tBuffer.data(), tTokens);
        XMLGen::to_lower(tTokens);

        std::string tTag;
        if (XMLGen::parse_single_value(tTokens, { "begin", "constraint" }, tTag))
        {
            XMLGen::Constraint tMetadata;
            XMLGen::erase_tags(mTags);
            XMLGen::parse_input_metadata( { "end", "constraint" }, aInputFile, mTags);
            this->setMetaData(tMetadata);
            mData.push_back(tMetadata);
        }
    }

    this->finalize();
}

}
// namespace XMLGen
