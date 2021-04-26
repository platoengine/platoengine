/*
 * XMLGeneratorParseOptimizationParameters.hpp
 *
 *  Created on: Jan 11, 2021
 */

#pragma once

#include "XMLGeneratorParseMetadata.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorOptimizationParametersMetadata.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \class ParseOptimizationParameters
 * \brief Parse inputs in OptimizationParameters block and store values in XMLGen::OptimizationParameters.
**********************************************************************************/
class ParseOptimizationParameters : public XMLGen::ParseMetadata<std::vector<XMLGen::OptimizationParameters>>
{
private:
    std::vector<XMLGen::OptimizationParameters> mData; /*!< OptimizationParameterss metadata */
    /*!< map from main optimization parameters tags to pair< valid tokens, pair<value,default> >, \n
     * i.e. map< tag, pair<valid tokens, pair<value,default>> > */
    XMLGen::MetaDataTags mTags;

private:
    /******************************************************************************//**
     * \fn setMeshMapData
     * \brief Set Scenario mesh map data
     * \param [in] aScenario Scenario metadata
    **********************************************************************************/
    void setMeshMapData(XMLGen::OptimizationParameters &aMetadata);

    /******************************************************************************//**
     * \fn allocate
     * \brief Allocate map from valid tags to valid tokens-value pair
    **********************************************************************************/
    void allocate();

    /******************************************************************************//**
     * \fn setTags
     * \brief Set optimization parameters metadata.
     * \param [in] aOptimizationParameters optimization parameters metadata
    **********************************************************************************/
    void setTags(XMLGen::OptimizationParameters& aOptimizationParameters);

    /******************************************************************************//**
     * \fn setLevelsetNodesetIDs
     * \brief Set the levelset nodeset metadata
     * \param [in] aMetadata optimization parameters metadata
    **********************************************************************************/
    void setLevelsetNodesetIDs(XMLGen::OptimizationParameters &aMetadata);

    /******************************************************************************//**
     * \fn setFixedBlockIDs
     * \brief Set the fixed block metadata
     * \param [in] aMetadata optimization parameters metadata
    **********************************************************************************/
    void setFixedBlockIDs(XMLGen::OptimizationParameters &aMetadata);

    /******************************************************************************//**
     * \fn setFixedNodesetIDs
     * \brief Set the fixed nodeset metadata
     * \param [in] aMetadata optimization parameters metadata
    **********************************************************************************/
    void setFixedNodesetIDs(XMLGen::OptimizationParameters &aMetadata);

    /******************************************************************************//**
     * \fn setFixedSidesetIDs
     * \brief Set the fixed sideset metadata
     * \param [in] aMetadata optimization parameters metadata
    **********************************************************************************/
    void setFixedSidesetIDs(XMLGen::OptimizationParameters &aMetadata);

    /******************************************************************************//**
     * \fn setMaterialBoxExtents
     * \brief Set the material box extent metadata
     * \param [in] aMetadata optimization parameters metadata
    **********************************************************************************/
    void setMaterialBoxExtents(XMLGen::OptimizationParameters &aMetadata);

    /******************************************************************************//**
     * \fn checkHeavisideFilterParams
     * \brief Check for valid heaviside filter params
     * \param [in] aMetadata optimization parameters metadata
    **********************************************************************************/
    void checkHeavisideFilterParams(XMLGen::OptimizationParameters &aMetadata);

    /******************************************************************************//**
     * \fn setMetaData
     * \brief Set the metadata from the tag values
     * \param [in] aMetadata optimization parameters metadata
    **********************************************************************************/
    void setMetaData(XMLGen::OptimizationParameters &aMetadata);

    /******************************************************************************//**
     * \fn setCSMParameters
     * \brief Set the csm parameters 
     * \param [in] aMetadata optimization parameters metadata
    **********************************************************************************/
    void setCSMParameters(XMLGen::OptimizationParameters &aMetadata);

    /******************************************************************************//**
     * \fn autoFillRestartParameters
     * \brief Auto populate restart parameters based on user input
     * \param [in] aMetadata optimization parameters metadata
    **********************************************************************************/
    void autoFillRestartParameters(XMLGen::OptimizationParameters &aMetadata);
public:

    /******************************************************************************//**
     * \fn data
     * \brief Return optimization parameters metadata.
     * \return metadata
    **********************************************************************************/
    std::vector<XMLGen::OptimizationParameters> data() const override;

    /******************************************************************************//**
     * \fn parse
     * \brief Parse OptimizationParameters metadata.
     * \param [in] aInputFile input file metadata.
    **********************************************************************************/
    void parse(std::istream &aInputFile) override;
};

}
// namespace XMLGen
