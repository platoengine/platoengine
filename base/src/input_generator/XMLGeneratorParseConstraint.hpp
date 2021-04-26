/*
 * XMLGeneratorParseConstraint.hpp
 *
 *  Created on: Jun 19, 2020
 */

#pragma once

#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorParseMetadata.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \class ParseConstraint
 * \brief Parse inputs in constraint function blocks and store their values in XMLGen::Constraint.
**********************************************************************************/
class ParseConstraint : public XMLGen::ParseMetadata<std::vector<XMLGen::Constraint>>
{
private:
    XMLGen::MetaDataTags mTags; /*!< map from plato input file tags to valid tokens-value pairs, i.e. map<tag, pair<tokens,value> > */
    std::vector<XMLGen::Constraint> mData; /*!< constraint functions metadata */

private:
    /******************************************************************************//**
     * \fn allocate
     * \brief Allocate map from valid tags to valid tokens-value pair
    **********************************************************************************/
    void allocate();

    /******************************************************************************//**
     * \fn set
     * \brief Set XMLGen::Service metadata.
     * \param [in] aMetaData XMLGen::Constraint metadata
    **********************************************************************************/
    void set(XMLGen::Constraint& aMetaData);

    /******************************************************************************//**
     * \fn check
     * \brief Check XMLGen::Service metadata.
     * \param [in] aMetaData XMLGen::Constraint metadata
    **********************************************************************************/
    void check(XMLGen::Constraint& aMetaData);

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return list of constraints and corresponding metadata.
     * \return list of uncertainty metadata
    **********************************************************************************/
    std::vector<XMLGen::Constraint> data() const override;

    /******************************************************************************//**
     * \fn parse
     * \brief Parse constraint functions metadata.
     * \param [in] aInputFile input metadata
    **********************************************************************************/
    void parse(std::istream &aInputFile) override;
};

}
