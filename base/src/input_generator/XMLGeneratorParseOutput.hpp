/*
 * XMLGeneratorParseOutput.hpp
 *
 *  Created on: Jun 18, 2020
 */

#pragma once

#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorParseMetadata.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \class ParseOutput
 * \brief Parse inputs in output block and store values in XMLGen::Output.
**********************************************************************************/
class ParseOutput : public XMLGen::ParseMetadata<XMLGen::Output>
{
private:
    XMLGen::Output mData; /*!< output metadata */
    XMLGen::MetaDataTags mTags; /*!< map from plato input file tags to valid tokens-value pairs, i.e. map<tag, pair<tokens,value> > */

private:
    /******************************************************************************//**
     * \fn setParameters
     * \brief Set output parameter values.
    **********************************************************************************/
    void setParameters();

    /******************************************************************************//**
     * \fn checkScenario
     * \brief Check if 'scenario' keyword is defined.
    **********************************************************************************/
    void checkScenario();

    /******************************************************************************//**
     * \fn checkOutputData
     * \brief Check if 'statistics' and 'data' keywords are requested at the same time.
    **********************************************************************************/
    void checkOutputData();

    /******************************************************************************//**
     * \fn checkMetaData
     * \brief Check if output metadata is set correctly.
    **********************************************************************************/
    void checkMetaData();

    /******************************************************************************//**
     * \fn allocate
     * \brief Allocate map from valid tags to valid tokens-value pair
    **********************************************************************************/
    void allocate();

    /******************************************************************************//**
     * \fn setRandomQoI
     * \brief Set random quantity of interests (QoI) metadata.
    **********************************************************************************/
    void setRandomQoI();

    /******************************************************************************//**
     * \fn setDeterministicQoI
     * \brief Set deterministic quantity of interests (QoI) metadata.
    **********************************************************************************/
    void setDeterministicQoI();

    /******************************************************************************//**
     * \fn setMetaData
     * \brief Set output metadata.
    **********************************************************************************/
    void setMetaData();

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return output metadata.
     * \return metadata
    **********************************************************************************/
    XMLGen::Output data() const override;

    /******************************************************************************//**
     * \fn parse
     * \brief Parse output metadata.
     * \param [in] aInputFile input file metadata.
    **********************************************************************************/
    void parse(std::istream &aInputFile) override;
};
// class ParseOutput

}
// namespace XMLGen
