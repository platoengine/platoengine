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
     * \fn checkIDs
     * \brief Set constraint functions identification number/name. If there are constraint \n
     * functions without an identification number/name, assign default values.
    **********************************************************************************/
    void checkIDs();

    /******************************************************************************//**
     * \fn set
     * \brief Set XMLGen::Scenario metadata.
     * \param [in] aMetaData XMLGen::Constraint metadata
    **********************************************************************************/
    void set(XMLGen::Constraint& aMetaData);

    /******************************************************************************//**
     * \fn check
     * \brief Check XMLGen::Scenario metadata.
     * \param [in] aMetaData XMLGen::Constraint metadata
    **********************************************************************************/
    void check(XMLGen::Constraint& aMetaData);

    /******************************************************************************//**
     * \fn checkTargeValue
     * \brief Verify that target constraint value keyword is set.
     * \param [in] aMetaData XMLGen::Constraint metadata
    **********************************************************************************/
    void checkTargeValue(XMLGen::Constraint& aMetaData);

    /******************************************************************************//**
     * \fn checkCode
     * \brief If 'code' keyword value is not supported, throw error.
     * \param [in] aMetaData XMLGen::Constraint metadata
    **********************************************************************************/
    void checkCode(XMLGen::Constraint &aMetadata);

    /******************************************************************************//**
     * \fn checkPerformer
     * \brief If 'performer' keyword value is not supported, throw error.
     * \param [in] aMetaData XMLGen::Constraint metadata
    **********************************************************************************/
    void checkPerformer(XMLGen::Constraint &aMetadata);

    /******************************************************************************//**
     * \fn checkCode
     * \brief If 'type' keyword value is not supported, throw error.
     * \param [in] aMetaData XMLGen::Constraint metadata
    **********************************************************************************/
    void checkCategory(XMLGen::Constraint &aMetadata);

    /******************************************************************************//**
     * \fn returnConstraintTargetAbsoluteKeywordSet
     * \brief Return constraint target absolute keyword defined in input file.
     * \param [in] aMetaData XMLGen::Constraint metadata
     * \return keyword
    **********************************************************************************/
    std::string returnConstraintTargetAbsoluteKeywordSet(XMLGen::Constraint& aMetaData) const;

    /******************************************************************************//**
     * \fn returnConstraintTargetNormalizedKeywordSet
     * \brief Return constraint target normalized keyword defined in input file.
     * \param [in] aMetaData XMLGen::Constraint metadata
     * \return keyword
    **********************************************************************************/
    std::string returnConstraintTargetNormalizedKeywordSet(XMLGen::Constraint& aMetaData) const;

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
