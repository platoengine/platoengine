/*
 * XMLGeneratorParseService.hpp
 *
 *  Created on: Jun 18, 2020
 */

#pragma once

#include "XMLGeneratorParseMetadata.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorServiceMetadata.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \class ParseService
 * \brief Parse inputs in Service block and store values in XMLGen::Service.
**********************************************************************************/
class ParseService : public XMLGen::ParseMetadata<std::vector<XMLGen::Service>>
{
private:
    std::vector<XMLGen::Service> mData; /*!< Services metadata */
    /*!< map from main service tags to pair< valid tokens, pair<value,default> >, \n
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
     * \brief Set service metadata.
     * \param [in] aService service metadata
    **********************************************************************************/
    void setTags(XMLGen::Service& aService);

    /******************************************************************************//**
     * \fn checkTags
     * \brief Check service metadata.
     * \param [in] aService service metadata
    **********************************************************************************/
    void checkTags(XMLGen::Service& aService);

    /******************************************************************************//**
     * \fn checkCode
     * \brief If 'code' keyword value is not supported, throw error.
     * \param [in] aService service metadata
    **********************************************************************************/
    void checkCode(XMLGen::Service& aService);

    /******************************************************************************//**
     * \fn checkPhysics
     * \brief Set 'physics' keyword, throw error if input keyword is empty.
     * \param [in] aService service metadata
    **********************************************************************************/
    void checkPhysics(XMLGen::Service& aService);

    /******************************************************************************//**
     * \fn checkSpatialDimensions
     * \brief If 'dimensions' keyword value is not supported, throw error.
     * \param [in] aService service metadata
    **********************************************************************************/
    void checkSpatialDimensions(XMLGen::Service& aService);

    /******************************************************************************//**
     * \fn checkPerformer
     * \brief If 'performer' keyword is empty, set 'performer' keyword to default \n
     * value: 'code_keyword' + '0', where 'code_keyword' denotes the value set for \n
     * 'code' keyword.
    **********************************************************************************/
    void checkPerformer();

    /******************************************************************************//**
     * \fn checkServiceID
     * \brief If service 'id' keyword is empty, set 'id' to default = 'code_keyword' + \n
     * 'physics_keyword' + '0', where \n 'code_keyword' denotes the value set for \n
     * 'code' keyword and 'physics_keyword' denotes the value set for 'physics' keyword.
    **********************************************************************************/
    void checkServiceID();

    /******************************************************************************//**
     * \fn finalize
     * \brief If required keywords are not defined, set values to default.
    **********************************************************************************/
    void finalize();

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return services metadata.
     * \return metadata
    **********************************************************************************/
    std::vector<XMLGen::Service> data() const override;

    /******************************************************************************//**
     * \fn parse
     * \brief Parse services metadata.
     * \param [in] aInputFile input file metadata.
    **********************************************************************************/
    void parse(std::istream &aInputFile) override;
};

}
// namespace XMLGen
