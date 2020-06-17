/*
 * XMLGeneratorParseUncertainty.hpp
 *
 *  Created on: May 7, 2020
 */

#pragma once

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
    void allocate();

    /******************************************************************************//**
     * \fn erase
     * \brief Erases value key content in map from valid tags to valid tokens-value pair.
    **********************************************************************************/
    void erase();

    /******************************************************************************//**
     * \fn parseMetadata
     * \brief Parse uncertainty blocks.
     * \param [in] aInputFile input file metadata
    **********************************************************************************/
    void parseMetadata(std::istream& aInputFile);

    /******************************************************************************//**
     * \fn setCategory
     * \brief Set non-deterministic variable's category.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void setCategory(XMLGen::Uncertainty& aMetadata);

    /******************************************************************************//**
     * \fn setIdentificationNumber
     * \brief Set non-deterministic variable's identification number.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void setIdentificationNumber(XMLGen::Uncertainty& aMetadata);

    /******************************************************************************//**
     * \fn setMetadata
     * \brief Set uncertainty metadata.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void setMetadata(XMLGen::Uncertainty& aMetadata);

    /******************************************************************************//**
     * \fn checkCategory
     * \brief Check if category keyword is empty.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkCategory(const XMLGen::Uncertainty& aMetadata);

    /******************************************************************************//**
     * \fn checkTag
     * \brief Check if tag keyword is empty.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkTag(const XMLGen::Uncertainty& aMetadata);

    /******************************************************************************//**
     * \fn checkAttribute
     * \brief Check if attribute keyword is empty.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkAttribute(const XMLGen::Uncertainty& aMetadata);

    /******************************************************************************//**
     * \fn checkDistribution
     * \brief Check if distribution keyword is empty.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkDistribution(const XMLGen::Uncertainty& aMetadata);

    /******************************************************************************//**
     * \fn checkMean
     * \brief Check if mean keyword is empty.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkMean(const XMLGen::Uncertainty& aMetadata);

    /******************************************************************************//**
     * \fn checkID
     * \brief Check if identification number (id) keyword is empty.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkID(const XMLGen::Uncertainty& aMetadata);

    /******************************************************************************//**
     * \fn checkNumSamples
     * \brief Check if number of samples keyword is empty.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkNumSamples(const XMLGen::Uncertainty& aMetadata);

    /******************************************************************************//**
     * \fn checkLowerBound
     * \brief Check if lower bounds keyword is empty.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkLowerBound(const XMLGen::Uncertainty& aMetadata);

    /******************************************************************************//**
     * \fn checkUpperBound
     * \brief Check if upper bounds keyword is empty.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkUpperBound(const XMLGen::Uncertainty& aMetadata);

    /******************************************************************************//**
     * \fn checkStandardDeviation
     * \brief Check if standard deviation keyword is empty.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkStandardDeviation(const XMLGen::Uncertainty& aMetadata);

    /******************************************************************************//**
     * \fn checkBeta
     * \brief Check if beta distribution is defined accordingly.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkBeta(const XMLGen::Uncertainty& aMetadata);

    /******************************************************************************//**
     * \fn checkNormal
     * \brief Check if normal distribution is defined accordingly.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkNormal(const XMLGen::Uncertainty& aMetadata);

    /******************************************************************************//**
     * \fn checkUniform
     * \brief Check if uniform distribution is defined accordingly.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkUniform(const XMLGen::Uncertainty& aMetadata);

    /******************************************************************************//**
     * \fn checkStatistics
     * \brief Check if statistics are defined accordingly.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkStatistics(const XMLGen::Uncertainty& aMetadata);

    /******************************************************************************//**
     * \fn checkMetadata
     * \brief Check if uncertainty metadata is defined accordingly.
     * \param [in] aMetadata uncertainty metadata
    **********************************************************************************/
    void checkMetadata(const XMLGen::Uncertainty& aMetadata);

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return list of uncertainty metadata.
     * \return list of uncertainty metadata
    **********************************************************************************/
    std::vector<XMLGen::Uncertainty> data() const;

    /******************************************************************************//**
     * \fn parse
     * \brief Parse uncertainty metadata.
     * \param [in] aInputFile parsed input file metadata
    **********************************************************************************/
    void parse(std::istream& aInputFile);
};
// class ParseUncertainty

}
// namespace XMLGen
