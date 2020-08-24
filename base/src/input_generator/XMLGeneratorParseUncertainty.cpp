/*
 * XMLGeneratorParseUncertainty.cpp
 *
 *  Created on: Jun 17, 2020
 */

#include <map>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>

#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorParseUncertainty.hpp"

namespace XMLGen
{

namespace Private
{

inline void is_mean_plus_std_dev_greater_than_upper_bound
(const XMLGen::Uncertainty& aMetadata)
{
    auto tMean = std::stod(aMetadata.mean());
    auto tUpper = std::stod(aMetadata.upper());
    auto tStdDev = std::stod(aMetadata.std());

    auto tMeanPlusStd = tMean + tStdDev;
    auto tIsGreaterThanUpperBound = tMeanPlusStd >= tUpper;
    if(tIsGreaterThanUpperBound)
    {
        THROWERR(std::string("Parse Uncertainty: Check input random variable with tag '") + aMetadata.tag() + "' and id '"
            + aMetadata.id() + "'. The 'mean plus standard deviation' operation must yields a value below the "
            + "'upper bound' to avoid 'NaN' errors during the creation of the stochastic reduced order model. The "
            + "operation 'mean + std_dev' yields '" + std::to_string(tMeanPlusStd) + "' and the 'upper bound' is set to '"
            + std::to_string(tUpper) + "'.")
    }
}

inline void is_mean_minus_std_dev_lesser_than_lower_bound
(const XMLGen::Uncertainty& aMetadata)
{
    auto tMean = std::stod(aMetadata.mean());
    auto tLower = std::stod(aMetadata.lower());
    auto tStdDev = std::stod(aMetadata.std());

    auto tMeanMinusStd = tMean - tStdDev;
    auto tIsLesserThanLowerBound = tMeanMinusStd <= tLower;
    if(tIsLesserThanLowerBound)
    {
        THROWERR(std::string("Parse Uncertainty: Check input random variable with tag '") + aMetadata.tag() + "' and id '"
            + aMetadata.id() + "'. The 'mean minus standard deviation' operation must yields a value above the "
            + "'lower bound' to avoid 'NaN' errors during the creation of the stochastic reduced order model. The "
            + "operation 'mean - std_dev' yields '" + std::to_string(tMeanMinusStd) + "' and the 'lower bound' is set to '"
            + std::to_string(tLower) + "'.")
    }
}

}

void ParseUncertainty::allocate()
{
    mTags.clear();
    mTags.insert({ "tag", { { {"tag"}, ""}, "" } });
    mTags.insert({ "mean", { { {"mean"}, ""}, "" } });
    mTags.insert({ "load_id", { { {"load_id"}, ""}, "" } });
    mTags.insert({ "filename", { { {"filename"}, ""}, "" } });
    mTags.insert({ "category", { { {"category"}, ""}, "" } });
    mTags.insert({ "attribute", { { {"attribute"}, ""}, "" } });
    mTags.insert({ "lower_bound", { { {"lower_bound"}, ""}, "" } });
    mTags.insert({ "upper_bound", { { {"upper_bound"}, ""}, "" } });
    mTags.insert({ "material_id", { { {"material_id"}, ""}, "" } });
    mTags.insert({ "distribution", { { {"distribution"}, ""}, "" } });
    mTags.insert({ "number_samples", { { {"number_samples"}, ""}, "" } });
    mTags.insert({ "initial_guess", { { {"initial_guess"}, ""}, "random" } });
    mTags.insert({ "standard_deviation", { { {"standard_deviation"}, ""}, "" } });
}

void ParseUncertainty::setMetaData(XMLGen::Uncertainty& aMetadata)
{
    for(auto& tTag : mTags)
    {
        if(tTag.second.first.second.empty())
        {
            auto tDefaultValue = tTag.second.second;
            aMetadata.append(tTag.first, tDefaultValue);
        }
        else
        {
            auto tInputValue = tTag.second.first.second;
            aMetadata.append(tTag.first, tInputValue);
        }
    }
    this->setID(aMetadata);
    this->setAttribute(aMetadata);
}

void ParseUncertainty::setID(XMLGen::Uncertainty& aMetadata)
{
    if(aMetadata.category().empty())
    {
        THROWERR("Parse Uncertainty: 'category' keyword is empty, i.e. is not defined.")
    }

    XMLGen::ValidRandomIdentificationKeys tValidKeys;
    auto tItr = tValidKeys.mKeys.find(aMetadata.category());
    if(tItr == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Parse Uncertainty: 'category' keyword '") + aMetadata.category() + "' is not supported.")
    }

    auto tValidIdItr = mTags.find(tItr->second) ;
    if(tValidIdItr == mTags.end())
    {
        THROWERR(std::string("Parse Uncertainty: Did not find keyword '") + tItr->second + "' in uncertainty block keywords map.")
    }

    aMetadata.append("id", tValidIdItr->second.first.second);
    if(aMetadata.id().empty())
    {
        THROWERR(std::string("Parse Uncertainty: Failed to parse uncertain parameter identification number. ")
            + "User must define the 'load id' or 'material id' keyword based on the uncertain parameter category. "
            + "For instance, if the uncertain parameter 'category' is 'load', then the 'load id' keyword must be defined.")
    }
}

void ParseUncertainty::setAttribute(XMLGen::Uncertainty& aMetadata)
{
    if(aMetadata.category().compare("material") == 0 && aMetadata.attribute().empty())
    {
        aMetadata.attribute("homogeneous");
    }
    else if(aMetadata.category().compare("load") == 0 && aMetadata.attribute().empty())
    {
        THROWERR(std::string("Parse Uncertainty: 'attribute' keyword of random variable with tag '") + aMetadata.tag()
                 + "', category '" + aMetadata.category() + "', and id '" + aMetadata.id() + "' is empty.")
    }
}

void ParseUncertainty::checkCategory(const XMLGen::Uncertainty& aMetadata)
{
    XMLGen::ValidRandomCategoryKeys tValidKeys;
    auto tLowerKey = XMLGen::to_lower(aMetadata.category());
    auto tItr = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), tLowerKey);
    if (tItr == tValidKeys.mKeys.end())
    {
        std::ostringstream tMsg;
        tMsg << "Parse Uncertainty: 'category' keyword '" << tLowerKey << "' is not supported. ";
        THROWERR(tMsg.str().c_str())
    }
}

void ParseUncertainty::checkTag(const XMLGen::Uncertainty& aMetadata)
{
    if(aMetadata.tag().empty())
    {
        THROWERR("Parse Uncertainty: 'tag' keyword is empty.")
    }

    XMLGen::ValidRandomPropertyKeys tValidKeys;
    auto tLowerKey = XMLGen::to_lower(aMetadata.tag());
    auto tItr = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), tLowerKey);
    if (tItr == tValidKeys.mKeys.end())
    {
        std::ostringstream tMsg;
        tMsg << "Parse Uncertainty: 'tag' keyword '" << tLowerKey << "' is not supported. ";
        THROWERR(tMsg.str().c_str())
    }
}

void ParseUncertainty::checkAttribute(const XMLGen::Uncertainty& aMetadata)
{
    XMLGen::ValidRandomAttributeKeys tValidKeys;
    auto tLowerKey = XMLGen::to_lower(aMetadata.attribute());
    auto tItr = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), tLowerKey);
    if (tItr == tValidKeys.mKeys.end())
    {
        std::ostringstream tMsg;
        tMsg << "Parse Uncertainty: 'attribute' keyword '" << tLowerKey << "' is not supported. ";
        THROWERR(tMsg.str().c_str())
    }
}

void ParseUncertainty::checkDistribution(const XMLGen::Uncertainty& aMetadata)
{
    if(aMetadata.attribute().empty())
    {
        THROWERR("Parse Uncertainty: 'distribution' keyword is empty.")
    }

    XMLGen::ValidStatisticalDistributionKeys tValidKeys;
    auto tLowerKey = XMLGen::to_lower(aMetadata.distribution());
    auto tItr = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), tLowerKey);
    if (tItr == tValidKeys.mKeys.end())
    {
        std::ostringstream tMsg;
        tMsg << "Parse Uncertainty: 'distribution' keyword '" << tLowerKey << "' is not supported. ";
        THROWERR(tMsg.str().c_str())
    }
}

void ParseUncertainty::checkMean(const XMLGen::Uncertainty& aMetadata)
{
    if(aMetadata.mean().empty())
    {
        THROWERR("Parse Uncertainty: 'mean' keyword is empty.")
    }

    if(!aMetadata.lower().empty() && !aMetadata.upper().empty())
    {
        auto tMean = std::stod(aMetadata.mean());
        auto tLower = std::stod(aMetadata.lower());
        auto tUpper = std::stod(aMetadata.upper());
        auto tIsGreaterThanUpper = tMean > tUpper;
        auto tIsLesserThanLower = tMean < tLower;
        if(tIsGreaterThanUpper || tIsLesserThanLower)
        {
            THROWERR(std::string("Parse Uncertainty: Mean is not within the lower and upper bounds, ")
                + "mean must be within the bounds. The condition lower bound = '" + aMetadata.lower()
                + "' < mean = '" + aMetadata.mean() + "' < upper bound = '" + aMetadata.upper() + "' is not met.")
        }
    }
}

void ParseUncertainty::checkID(const XMLGen::Uncertainty& aMetadata)
{
    if(aMetadata.id().empty())
    {
        THROWERR("Parse Uncertainty: 'id' keyword is empty.")
    }
}

void ParseUncertainty::checkNumSamples(const XMLGen::Uncertainty& aMetadata)
{
    if(aMetadata.samples().empty())
    {
        THROWERR("Parse Uncertainty: 'num samples' keyword is empty.")
    }
}

void ParseUncertainty::checkLowerBound(const XMLGen::Uncertainty& aMetadata)
{
    if(aMetadata.lower().empty())
    {
        THROWERR("Parse Uncertainty: 'lower bound' keyword is empty.")
    }

    auto tLower = std::stod(aMetadata.lower());
    auto tUpper = std::stod(aMetadata.upper());
    auto tLowerIsGreaterThanUpper = tLower >= tUpper;
    if(tLowerIsGreaterThanUpper)
    {
        THROWERR(std::string("Parse Uncertainty: Lower bound is greater or equal than the upper bound. ")
            + "Lower bound must be less than the upper bound. The condition lower bound = '"
            + aMetadata.lower() + "' < upper bound = '" + aMetadata.upper() + "' is not met.")
    }
}

void ParseUncertainty::checkUpperBound(const XMLGen::Uncertainty& aMetadata)
{
    if(aMetadata.upper().empty())
    {
        THROWERR("Parse Uncertainty: 'upper bound' keyword is empty.")
    }

    auto tLower = std::stod(aMetadata.lower());
    auto tUpper = std::stod(aMetadata.upper());
    auto tIsUpperLesserThanLower = tUpper <= tLower;
    if(tIsUpperLesserThanLower)
    {
        THROWERR(std::string("Parse Uncertainty: Upper bound is lesser or equal than the lower bound. ")
            + "Upper bound must be greater than the lower bound. The condition upper bound = '"
            + aMetadata.upper() + "' > lower bound = '" + aMetadata.lower() + "' is not met.")
    }
}

void ParseUncertainty::checkStandardDeviation(const XMLGen::Uncertainty& aMetadata)
{
    if(aMetadata.std().empty())
    {
        THROWERR("Parse Uncertainty: 'standard deviation' keyword is empty.")
    }

    XMLGen::Private::is_mean_plus_std_dev_greater_than_upper_bound(aMetadata);
    XMLGen::Private::is_mean_minus_std_dev_lesser_than_lower_bound(aMetadata);
}

void ParseUncertainty::checkBounds(const XMLGen::Uncertainty& aMetadata)
{
    this->checkLowerBound(aMetadata);
    this->checkUpperBound(aMetadata);
}

void ParseUncertainty::checkBeta(const XMLGen::Uncertainty& aMetadata)
{
    this->checkMean(aMetadata);
    this->checkBounds(aMetadata);
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
    this->checkBounds(aMetadata);
    this->checkNumSamples(aMetadata);
}

void ParseUncertainty::checkStatistics(const XMLGen::Uncertainty& aMetadata)
{
    this->checkDistribution(aMetadata);
    if(aMetadata.distribution() == "beta")
    {
        this->checkBeta(aMetadata);
    }
    else if(aMetadata.distribution() == "normal")
    {
        this->checkNormal(aMetadata);
    }
    else if(aMetadata.distribution() == "uniform")
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
            XMLGen::erase_tag_values(mTags);
            XMLGen::parse_input_metadata({"end","uncertainty"}, aInputFile, mTags);
            this->setMetaData(tMetadata);
            this->checkMetaData(tMetadata);
            mData.push_back(tMetadata);
        }
    }
}

}
// namespace XMLGen
