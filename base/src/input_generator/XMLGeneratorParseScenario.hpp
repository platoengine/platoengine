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
 * \brief Parse inputs in Scenario block and store values in XMLGen::Scenario.
**********************************************************************************/
class ParseScenario : public XMLGen::ParseMetadata<std::vector<XMLGen::Scenario>>
{
private:
    std::vector<XMLGen::Scenario> mData; /*!< Scenarios metadata */
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
     * \fn setLoadIDs
     * \brief Set Scenario load ids.
     * \param [in] aScenario Scenario metadata
    **********************************************************************************/
    void setLoadIDs(XMLGen::Scenario& aScenario);

    /******************************************************************************//**
     * \fn setFRFMatchNodesetIDs
     * \brief Set nodeset ids to be used when matching frfs.
     * \param [in] aScenario Scenario metadata
    **********************************************************************************/
    void setFRFMatchNodesetIDs(XMLGen::Scenario &aMetadata);

    /******************************************************************************//**
     * \fn setBCIDs
     * \brief Set Scenario boundary condition ids.
     * \param [in] aScenario Scenario metadata
    **********************************************************************************/
    void setBCIDs(XMLGen::Scenario& aScenario);

    /******************************************************************************//**
     * \fn checkTags
     * \brief Check Scenario metadata.
     * \param [in] aScenario Scenario metadata
    **********************************************************************************/
    void checkTags(XMLGen::Scenario& aScenario);

    /******************************************************************************//**
     * \fn checkPhysics
     * \brief Set 'physics' keyword, throw error if input keyword is empty.
     * \param [in] aScenario Scenario metadata
    **********************************************************************************/
    void checkPhysics(XMLGen::Scenario& aScenario);

    /******************************************************************************//**
     * \fn checkSpatialDimensions
     * \brief If 'dimensions' keyword value is not supported, throw error.
     * \param [in] aScenario Scenario metadata
    **********************************************************************************/
    void checkSpatialDimensions(XMLGen::Scenario& aScenario);

    /******************************************************************************//**
     * \fn checkIDs
     * \brief If load or boundary condition IDs are not defined, throw error.
     * \param [in] aScenario Scenario metadata
    **********************************************************************************/
    void checkIDs(XMLGen::Scenario& aScenario);

    /******************************************************************************//**
     * \fn checkScenarioID
     * \brief If scenario 'id' keyword is empty, set 'id' to default = 'physics_keyword' 
     * + '0', where 'physics_keyword' denotes the value set for 'physics' keyword.
    **********************************************************************************/
    void checkScenarioID();

    /******************************************************************************//**
     * \fn finalize
     * \brief If required keywords are not defined, set values to default.
    **********************************************************************************/
    void finalize();

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return scenarios metadata.
     * \return metadata
    **********************************************************************************/
    std::vector<XMLGen::Scenario> data() const override;

    /******************************************************************************//**
     * \fn parse
     * \brief Parse Scenarios metadata.
     * \param [in] aInputFile input file metadata.
    **********************************************************************************/
    void parse(std::istream &aInputFile) override;
};

}
// namespace XMLGen
