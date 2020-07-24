/*
 * XMLGeneratorParseScenario.hpp
 *
 *  Created on: Jun 18, 2020
 */

#pragma once

#include "XMLGeneratorParseMetadata.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorScenarioMetadata.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \class ParseScenario
 * \brief Parse inputs in scenario block and store values in XMLGen::Scenario.
**********************************************************************************/
class ParseScenario : public XMLGen::ParseMetadata<std::vector<XMLGen::Scenario>>
{
private:
    std::vector<XMLGen::Scenario> mData; /*!< scenarios metadata */
    /*!< map from main scenario tags to pair< valid tokens, pair<value,default> >, \n
     * i.e. map< tag, pair<valid tokens, pair<value,default>> > */
    XMLGen::MetaDataTags mTags;

private:
    /******************************************************************************//**
     * \fn allocate
     * \brief Allocate map from valid tags to valid tokens-value pair
    **********************************************************************************/
    void allocate();

    /******************************************************************************//**
     * \fn setTags
     * \brief Set scenario metadata.
     * \param [in] aScenario scenario metadata
    **********************************************************************************/
    void setTags(XMLGen::Scenario& aScenario);

    /******************************************************************************//**
     * \fn checkTags
     * \brief Check scenario metadata.
     * \param [in] aScenario scenario metadata
    **********************************************************************************/
    void checkTags(XMLGen::Scenario& aScenario);

    /******************************************************************************//**
     * \fn checkCode
     * \brief If 'code' keyword value is not supported, throw error.
     * \param [in] aScenario scenario metadata
    **********************************************************************************/
    void checkCode(XMLGen::Scenario& aScenario);

    /******************************************************************************//**
     * \fn checkPhysics
     * \brief Set 'physics' keyword, throw error if input keyword is empty.
     * \param [in] aScenario scenario metadata
    **********************************************************************************/
    void checkPhysics(XMLGen::Scenario& aScenario);

    /******************************************************************************//**
     * \fn checkPerformer
     * \brief If 'performer' keyword is empty, set 'performer' keyword to default \n
     * value: 'code_keyword' + '0', where 'code_keyword' denotes the value set for \n
     * 'code' keyword.
     * \param [in] aScenario scenario metadata
    **********************************************************************************/
    void checkPerformer(XMLGen::Scenario& aScenario);

    /******************************************************************************//**
     * \fn checkSpatialDimensions
     * \brief If 'dimensions' keyword value is not supported, throw error.
     * \param [in] aScenario scenario metadata
    **********************************************************************************/
    void checkSpatialDimensions(XMLGen::Scenario& aScenario);

    /******************************************************************************//**
     * \fn checkScenarioID
     * \brief If scenario 'id' keyword is empty, set 'id' to default = 'code_keyword' + \n
     * 'physics_keyword' + '0', where \n 'code_keyword' denotes the value set for \n
     * 'code' keyword and 'physics_keyword' denotes the value set for 'physics' keyword.
     * \param [in] aScenario scenario metadata
    **********************************************************************************/
    void checkScenarioID(XMLGen::Scenario& aScenario);

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return scenarios metadata.
     * \return metadata
    **********************************************************************************/
    std::vector<XMLGen::Scenario> data() const override;

    /******************************************************************************//**
     * \fn parse
     * \brief Parse scenarios metadata.
     * \param [in] aInputFile input file metadata.
    **********************************************************************************/
    void parse(std::istream &aInputFile) override;
};

}
// namespace XMLGen
