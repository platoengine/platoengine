/*
 * XMLGeneratorParseCriteria.hpp
 *
 *  Created on: Jun 23, 2020
 */

#pragma once

#include "XMLGeneratorParseMetadata.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorCriterionMetadata.hpp"

namespace XMLGen
{

class ParseCriteria : public XMLGen::ParseMetadata<std::vector<XMLGen::Criterion>>
{
private:
    XMLGen::MetaDataTags mTags; /*!< map from plato input file tags to valid tokens-value pairs, i.e. map<tag, pair<tokens,value> > */
    std::vector<XMLGen::Criterion> mData; /*!< criteria metadata */

private:
    /******************************************************************************//**
     * \fn allocate
     * \brief Allocate map from valid tags to valid tokens-value pair
    **********************************************************************************/
    void allocate();

    /******************************************************************************//**
     * \fn setCriterionType
     * \brief Set 'criterion type' keyword, throw error if it is not defined.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setCriterionType(XMLGen::Criterion& aMetadata);

    /******************************************************************************//**
     * \fn setCriterionParameters
     * \brief Set criterion parameter, throw error if not defined.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setCriterionParameters(XMLGen::Criterion& aMetadata);

    /******************************************************************************//**
     * \fn setMetaData
     * \brief Set XMLGen::Criterion metadata.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setMetadata(XMLGen::Criterion& aMetadata);

    /******************************************************************************//**
     * \fn checkUniqueIDs
     * \brief Throw error if criterion block identification numbers are not unique.
    **********************************************************************************/
    void checkUniqueIDs();

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return criterion metadata.
     * \return metadata
    **********************************************************************************/
    std::vector<XMLGen::Criterion> data() const override;

    /******************************************************************************//**
     * \fn parse
     * \brief Parse output metadata.
     * \param [in] aInputFile input file metadata.
    **********************************************************************************/
    void parse(std::istream &aInputFile) override;
};
// class ParseCriteria

}
// namespace XMLGen