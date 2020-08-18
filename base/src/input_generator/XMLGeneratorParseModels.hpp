/*
 * XMLGeneratorParseModels.hpp
 *
 *  Created on: Jun 18, 2020
 */

#pragma once

#include "XMLGeneratorParseMetadata.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorModelMetadata.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \class ParseModel
 * \brief Parse inputs in model block and store values in XMLGen::Model.
**********************************************************************************/
class ParseModel : public XMLGen::ParseMetadata<std::vector<XMLGen::Model>>
{
private:
    std::vector<XMLGen::Model> mData; /*!< Models metadata */
    /*!< map from main model tags to pair< valid tokens, pair<value,default> >, \n
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
     * \brief Set model metadata.
     * \param [in] aModel model metadata
    **********************************************************************************/
    void setTags(XMLGen::Model& aModel);

    /******************************************************************************//**
     * \fn checkTags
     * \brief Check Model metadata.
     * \param [in] aModel Model metadata
    **********************************************************************************/
    void checkTags(XMLGen::Model& aModel);

    /******************************************************************************//**
     * \fn checkCode
     * \brief If 'code' keyword value is not supported, throw error.
     * \param [in] aModel Model metadata
    **********************************************************************************/
    void checkCode(XMLGen::Model& aModel);

    /******************************************************************************//**
     * \fn checkModelID
     * \brief If model 'id' keyword is empty, set 'id' to default = 'code_keyword'+ '0',
     * where 'code_keyword' denotes the value set for 'code' keyword.
    **********************************************************************************/
    void checkModelID();

    /******************************************************************************//**
     * \fn finalize
     * \brief If required keywords are not defined, set values to default.
    **********************************************************************************/
    void finalize();

    /******************************************************************************//**
     * \fn setBlockIDs
     * \brief Set model block ids.
     * \param [in] aModel model metadata
    **********************************************************************************/
    void setBlockIDs(XMLGen::Model& aModel);

    /******************************************************************************//**
     * \fn setMaterialIDs
     * \brief Set model material ids.
     * \param [in] aModel model metadata
    **********************************************************************************/
    void setMaterialIDs(XMLGen::Model& aModel);

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return models metadata.
     * \return metadata
    **********************************************************************************/
    std::vector<XMLGen::Model> data() const override;

    /******************************************************************************//**
     * \fn parse
     * \brief Parse Models metadata.
     * \param [in] aInputFile input file metadata.
    **********************************************************************************/
    void parse(std::istream &aInputFile) override;
};

}
// namespace XMLGen
