/*
 * XMLGeneratorParser.hpp
 *
 *  Created on: May 7, 2020
 */

#pragma once

#include <map>
#include <string>
#include <vector>
#include <numeric>

#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \fn ParseUncertainty
 * \brief Class responsible for parsing uncertainty blocks.
**********************************************************************************/
class ParseUncertainty
{
private:
    XMLGen::UseCaseTags mTags; /*!< map from valid tags to valid tokens-value pairs, i.e. map<tag, pair<tokens,value> > */
    std::vector<XMLGen::Uncertainty> mData; /*!< list of uncertainty metadata */

private:
    /******************************************************************************//**
     * \fn allocate
     * \brief Allocate map from valid tags to valid tokens-value pair
    **********************************************************************************/
    void allocate()
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

    /******************************************************************************//**
     * \fn erase
     * \brief Erases value key content in map from valid tags to valid tokens-value pair.
    **********************************************************************************/
    void erase()
    {
        for(auto& tTag : mTags)
        {
            tTag.second.second.clear();
        }
    }

    /******************************************************************************//**
     * \fn parseMetadata
     * \brief Parse uncertainty blocks.
     * \param [in] aInputFile input file metadata
    **********************************************************************************/
    void parseMetadata(std::istream& aInputFile)
    {
        constexpr int tMAX_CHARS_PER_LINE = 512;
        std::vector<char> tBuffer(tMAX_CHARS_PER_LINE);
        // found an uncertainty. parse it.
        while (!aInputFile.eof())
        {
            std::vector<std::string> tTokens;
            aInputFile.getline(tBuffer.data(), tMAX_CHARS_PER_LINE);
            XMLGen::parse_tokens(tBuffer.data(), tTokens);
            XMLGen::to_lower(tTokens);

            std::string tTag;
            if (XMLGen::parse_single_value(tTokens, std::vector<std::string> { "end", "uncertainty" }, tTag))
            {
                break;
            }
            XMLGen::parse_tag_values(tTokens, mTags);
        }
    }

    /******************************************************************************//**
     * \fn setCategory
     * \brief Set non-deterministic variable's category.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void setCategory(XMLGen::Uncertainty& aMetadata)
    {
        auto tItr = mTags.find("category");
        if(tItr != mTags.end() && !tItr->second.second.empty())
        {
            aMetadata.variable_type = tItr->second.second;
        }
    }

    /******************************************************************************//**
     * \fn setIdentificationNumber
     * \brief Set non-deterministic variable's identification number.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void setIdentificationNumber(XMLGen::Uncertainty& aMetadata)
    {
        aMetadata.id = aMetadata.variable_type == "load"
            ? mTags.find("load id")->second.second : mTags.find("material id")->second.second;
        if(aMetadata.id.empty())
        {
            THROWERR("Parse Uncertainty: Failed to parse identification number.")
        }

    }

    /******************************************************************************//**
     * \fn setMetadata
     * \brief Set uncertainty metadata.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void setMetadata(XMLGen::Uncertainty& aMetadata)
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

    /******************************************************************************//**
     * \fn checkCategory
     * \brief Check if category keyword is empty.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkCategory(const XMLGen::Uncertainty& aMetadata)
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

    /******************************************************************************//**
     * \fn checkTag
     * \brief Check if tag keyword is empty.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkTag(const XMLGen::Uncertainty& aMetadata)
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

    /******************************************************************************//**
     * \fn checkAttribute
     * \brief Check if attribute keyword is empty.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkAttribute(const XMLGen::Uncertainty& aMetadata)
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

    /******************************************************************************//**
     * \fn checkDistribution
     * \brief Check if distribution keyword is empty.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkDistribution(const XMLGen::Uncertainty& aMetadata)
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

    /******************************************************************************//**
     * \fn checkMean
     * \brief Check if mean keyword is empty.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkMean(const XMLGen::Uncertainty& aMetadata)
    {
        if(aMetadata.mean.empty())
        {
            THROWERR("Parse Uncertainty: 'mean' keyword is empty.")
        }
    }

    /******************************************************************************//**
     * \fn checkID
     * \brief Check if identification number (id) keyword is empty.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkID(const XMLGen::Uncertainty& aMetadata)
    {
        if(aMetadata.id.empty())
        {
            THROWERR("Parse Uncertainty: 'id' keyword is empty.")
        }
    }

    /******************************************************************************//**
     * \fn checkNumSamples
     * \brief Check if number of samples keyword is empty.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkNumSamples(const XMLGen::Uncertainty& aMetadata)
    {
        if(aMetadata.num_samples.empty())
        {
            THROWERR("Parse Uncertainty: 'num samples' keyword is empty.")
        }
    }

    /******************************************************************************//**
     * \fn checkLowerBound
     * \brief Check if lower bounds keyword is empty.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkLowerBound(const XMLGen::Uncertainty& aMetadata)
    {
        if(aMetadata.lower.empty())
        {
            THROWERR("Parse Uncertainty: 'lower bound' keyword is empty.")
        }
    }

    /******************************************************************************//**
     * \fn checkUpperBound
     * \brief Check if upper bounds keyword is empty.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkUpperBound(const XMLGen::Uncertainty& aMetadata)
    {
        if(aMetadata.upper.empty())
        {
            THROWERR("Parse Uncertainty: 'upper bound' keyword is empty.")
        }
    }

    /******************************************************************************//**
     * \fn checkStandardDeviation
     * \brief Check if standard deviation keyword is empty.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkStandardDeviation(const XMLGen::Uncertainty& aMetadata)
    {
        if(aMetadata.standard_deviation.empty())
        {
            THROWERR("Parse Uncertainty: 'standard deviation' keyword is empty.")
        }
    }

    /******************************************************************************//**
     * \fn checkBeta
     * \brief Check if beta distribution is defined accordingly.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkBeta(const XMLGen::Uncertainty& aMetadata)
    {
        this->checkMean(aMetadata);
        this->checkLowerBound(aMetadata);
        this->checkUpperBound(aMetadata);
        this->checkNumSamples(aMetadata);
        this->checkStandardDeviation(aMetadata);
    }

    /******************************************************************************//**
     * \fn checkNormal
     * \brief Check if normal distribution is defined accordingly.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkNormal(const XMLGen::Uncertainty& aMetadata)
    {
        this->checkMean(aMetadata);
        this->checkNumSamples(aMetadata);
        this->checkStandardDeviation(aMetadata);
    }

    /******************************************************************************//**
     * \fn checkUniform
     * \brief Check if uniform distribution is defined accordingly.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkUniform(const XMLGen::Uncertainty& aMetadata)
    {
        this->checkNumSamples(aMetadata);
        this->checkLowerBound(aMetadata);
        this->checkUpperBound(aMetadata);
    }

    /******************************************************************************//**
     * \fn checkStatistics
     * \brief Check if statistics are defined accordingly.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkStatistics(const XMLGen::Uncertainty& aMetadata)
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

    /******************************************************************************//**
     * \fn checkMetadata
     * \brief Check if uncertainty metadata is defined accordingly.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkMetadata(const XMLGen::Uncertainty& aMetadata)
    {
        this->checkID(aMetadata);
        this->checkTag(aMetadata);
        this->checkCategory(aMetadata);
        this->checkAttribute(aMetadata);
        this->checkStatistics(aMetadata);
    }

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return list of uncertainty metadata.
     * \return list of uncertainty metadata
    **********************************************************************************/
    std::vector<XMLGen::Uncertainty> data() const
    {
        return mData;
    }

    /******************************************************************************//**
     * \fn parse
     * \brief Parse uncertainty metadata.
     * \param [in] aInputFile parsed input file metadata
    **********************************************************************************/
    void parse(std::istream& aInputFile)
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
            std::vector<std::string> tMatchTokens;
            if(XMLGen::parse_single_value(tTokens, tMatchTokens = {"begin","uncertainty"}, tTag))
            {
                XMLGen::Uncertainty tMetadata;
                this->erase();
                this->parseMetadata(aInputFile);
                this->setMetadata(tMetadata);
                this->checkMetadata(tMetadata);
                mData.push_back(tMetadata);
            }
        }
    }
};
// class ParseUncertainty

}
// namespace XMLGen
