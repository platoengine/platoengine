/*
 * XMLGeneratorParseUncertainty.cpp
 *
 *  Created on: Jun 17, 2020
 */

#include <map>
#include <string>
#include <vector>
#include <numeric>

#include "XMLGeneratorParseUncertainty.hpp"

namespace XMLGen
{

void ParseUncertainty::allocate()
{
    mTags.clear();
    mTags.insert({ "tag", { {"tag"}, "" } });
    mTags.insert({ "mean", { {"mean"}, "" } });
    mTags.insert({ "load id", { {"load","id"}, "" } });
    mTags.insert({ "material id", { {"material","id"}, "" } });
    mTags.insert({ "category", { {"category"}, "" } });
    mTags.insert({ "attribute", { {"attribute"}, "" } });
    mTags.insert({ "distribution", { {"distribution"}, "" } });
    mTags.insert({ "num samples", { {"num", "samples"}, "" } });
    mTags.insert({ "lower bound", { {"lower", "bound"}, "" } });
    mTags.insert({ "upper bound", { {"upper", "bound"}, "" } });
    mTags.insert({ "standard deviation", { {"standard", "deviation"}, "" } });
}

void ParseUncertainty::erase()
{
    for(auto& tTag : mTags)
    {
        tTag.second.second.clear();
    }
}

void ParseUncertainty::setCategory(XMLGen::Uncertainty& aMetadata)
{
    auto tItr = mTags.find("category");
    if(tItr != mTags.end() && !tItr->second.second.empty())
    {
        aMetadata.variable_type = tItr->second.second;
    }
}

void ParseUncertainty::setIdentificationNumber(XMLGen::Uncertainty& aMetadata)
{
    aMetadata.id = aMetadata.variable_type == "load"
        ? mTags.find("load id")->second.second : mTags.find("material id")->second.second;
    if(aMetadata.id.empty())
    {
        THROWERR("Parse Uncertainty: Failed to parse identification number.")
    }

}

void ParseUncertainty::setMetaData(XMLGen::Uncertainty& aMetadata)
{
    this->setCategory(aMetadata);
    this->setIdentificationNumber(aMetadata);
    aMetadata.type = mTags.find("tag")->second.second;
    aMetadata.mean = mTags.find("mean")->second.second;
    aMetadata.axis = mTags.find("attribute")->second.second;
    aMetadata.lower = mTags.find("lower bound")->second.second;
    aMetadata.upper = mTags.find("upper bound")->second.second;
    aMetadata.num_samples = mTags.find("num samples")->second.second;
    aMetadata.distribution = mTags.find("distribution")->second.second;
    aMetadata.standard_deviation = mTags.find("standard deviation")->second.second;
}

void ParseUncertainty::checkCategory(const XMLGen::Uncertainty& aMetadata)
{
    if(aMetadata.variable_type.empty())
    {
        THROWERR("Parse Uncertainty: 'category' keyword is empty.")
    }

    std::vector<std::string> tValidTags = {"load", "material"};
    if (std::find(tValidTags.begin(), tValidTags.end(), aMetadata.variable_type) == tValidTags.end())
    {
        std::ostringstream tMsg;
        tMsg << "Parse Uncertainty: 'category' keyword '" << aMetadata.variable_type << "' is not supported. ";
        THROWERR(tMsg.str().c_str())
    }
}

void ParseUncertainty::checkTag(const XMLGen::Uncertainty& aMetadata)
{
    if(aMetadata.type.empty())
    {
        THROWERR("Parse Uncertainty: 'tag' keyword is empty.")
    }

    std::vector<std::string> tValidTags = {"angle variation", "poissons ratio", "elastic modulus",
        "yield stress", "thermal conductivity", "density"};
    if (std::find(tValidTags.begin(), tValidTags.end(), aMetadata.type) == tValidTags.end())
    {
        std::ostringstream tMsg;
        tMsg << "Parse Uncertainty: 'tag' keyword '" << aMetadata.type << "' is not supported. ";
        THROWERR(tMsg.str().c_str())
    }
}

void ParseUncertainty::checkAttribute(const XMLGen::Uncertainty& aMetadata)
{
    if(aMetadata.axis.empty())
    {
        THROWERR("Parse Uncertainty: 'attribute' keyword is empty.")
    }

    std::vector<std::string> tValidTags = {"x", "y", "z", "homogeneous"};
    if (std::find(tValidTags.begin(), tValidTags.end(), aMetadata.axis) == tValidTags.end())
    {
        std::ostringstream tMsg;
        tMsg << "Parse Uncertainty: 'attribute' keyword '" << aMetadata.axis << "' is not supported. ";
        THROWERR(tMsg.str().c_str())
    }
}

void ParseUncertainty::checkDistribution(const XMLGen::Uncertainty& aMetadata)
{
    if(aMetadata.axis.empty())
    {
        THROWERR("Parse Uncertainty: 'distribution' keyword is empty.")
    }

    std::vector<std::string> tValidTags = {"normal", "beta", "uniform"};
    if (std::find(tValidTags.begin(), tValidTags.end(), aMetadata.distribution) == tValidTags.end())
    {
        std::ostringstream tMsg;
        tMsg << "Parse Uncertainty: 'distribution' keyword '" << aMetadata.distribution << "' is not supported. ";
        THROWERR(tMsg.str().c_str())
    }
}

void ParseUncertainty::checkMean(const XMLGen::Uncertainty& aMetadata)
{
    if(aMetadata.mean.empty())
    {
        THROWERR("Parse Uncertainty: 'mean' keyword is empty.")
    }
}

void ParseUncertainty::checkID(const XMLGen::Uncertainty& aMetadata)
{
    if(aMetadata.id.empty())
    {
        THROWERR("Parse Uncertainty: 'id' keyword is empty.")
    }
}

void ParseUncertainty::checkNumSamples(const XMLGen::Uncertainty& aMetadata)
{
    if(aMetadata.num_samples.empty())
    {
        THROWERR("Parse Uncertainty: 'num samples' keyword is empty.")
    }
}

void ParseUncertainty::checkLowerBound(const XMLGen::Uncertainty& aMetadata)
{
    if(aMetadata.lower.empty())
    {
        THROWERR("Parse Uncertainty: 'lower bound' keyword is empty.")
    }
}

void ParseUncertainty::checkUpperBound(const XMLGen::Uncertainty& aMetadata)
{
    if(aMetadata.upper.empty())
    {
        THROWERR("Parse Uncertainty: 'upper bound' keyword is empty.")
    }
}

void ParseUncertainty::checkStandardDeviation(const XMLGen::Uncertainty& aMetadata)
{
    if(aMetadata.standard_deviation.empty())
    {
        THROWERR("Parse Uncertainty: 'standard deviation' keyword is empty.")
    }
}

void ParseUncertainty::checkBeta(const XMLGen::Uncertainty& aMetadata)
{
    this->checkMean(aMetadata);
    this->checkLowerBound(aMetadata);
    this->checkUpperBound(aMetadata);
    this->checkNumSamples(aMetadata);
    this->checkStandardDeviation(aMetadata);
}

void ParseUncertainty::checkNormal(const XMLGen::Uncertainty& aMetadata)
{
    this->checkMean(aMetadata);
    this->checkNumSamples(aMetadata);
    this->checkStandardDeviation(aMetadata);
}

void ParseUncertainty::checkUniform(const XMLGen::Uncertainty& aMetadata)
{
    this->checkNumSamples(aMetadata);
    this->checkLowerBound(aMetadata);
    this->checkUpperBound(aMetadata);
}

void ParseUncertainty::checkStatistics(const XMLGen::Uncertainty& aMetadata)
{
    this->checkDistribution(aMetadata);
    if(aMetadata.distribution == "beta")
    {
        this->checkBeta(aMetadata);
    }
    else if(aMetadata.distribution == "normal")
    {
        this->checkNormal(aMetadata);
    }
    else if(aMetadata.distribution == "uniform")
    {
        this->checkUniform(aMetadata);
    }
}

void ParseUncertainty::checkMetaData(const XMLGen::Uncertainty& aMetadata)
{
    this->checkID(aMetadata);
    this->checkTag(aMetadata);
    this->checkCategory(aMetadata);
    this->checkAttribute(aMetadata);
    this->checkStatistics(aMetadata);
}

std::vector<XMLGen::Uncertainty> ParseUncertainty::data() const
{
    return mData;
}

void ParseUncertainty::parse(std::istream& aInputFile)
{
    mData.clear();
    this->allocate();
    constexpr int MAX_CHARS_PER_LINE = 512;
    std::vector<char> tBuffer(MAX_CHARS_PER_LINE);
    while (!aInputFile.eof())
    {
        // read an entire line into memory
        std::vector<std::string> tTokens;
        aInputFile.getline(tBuffer.data(), MAX_CHARS_PER_LINE);
        XMLGen::parse_tokens(tBuffer.data(), tTokens);
        XMLGen::to_lower(tTokens);

        std::string tTag;
        if(XMLGen::parse_single_value(tTokens, {"begin","uncertainty"}, tTag))
        {
            XMLGen::Uncertainty tMetadata;
            this->erase();
            XMLGen::parse_input_metadata({"end","uncertainty"}, aInputFile, mTags);
            this->setMetaData(tMetadata);
            this->checkMetaData(tMetadata);
            mData.push_back(tMetadata);
        }
    }
}

}
// namespace XMLGen
