/*
 * XMLGeneratorParseServices.hpp
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
 * \brief Parse inputs in service block and store values in XMLGen::Service.
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
     * \brief Check Service metadata.
     * \param [in] aService Service metadata
    **********************************************************************************/
    void checkTags(XMLGen::Service& aService);

    /******************************************************************************//**
     * \fn checkCode
     * \brief If 'code' keyword value is not supported, throw error.
     * \param [in] aService Service metadata
    **********************************************************************************/
    void checkCode(XMLGen::Service& aService);

    /******************************************************************************//**
     * \fn checkCacheState
     * \brief Set cache state variable appropriately if needed
     * \param [in] aService Service metadata
    **********************************************************************************/
    void checkCacheState(XMLGen::Service& aService);

    /******************************************************************************//**
     * \fn checkServiceID
     * \brief If service 'id' keyword is empty, set 'id' to default = 'code_keyword'+ '0',
     * where 'code_keyword' denotes the value set for 'code' keyword.
    **********************************************************************************/
    void checkServiceID();

    /******************************************************************************//**
     * \fn finalize
     * \brief If required keywords are not defined, set values to default.
    **********************************************************************************/
    void finalize();

    /******************************************************************************//**
     * \fn setDeviceIDs
     * \brief Set the device ids from the parsed data
     * \param [in] aService Service metadata
    **********************************************************************************/
    void setDeviceIDs(XMLGen::Service &aMetadata);

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return services metadata.
     * \return metadata
    **********************************************************************************/
    std::vector<XMLGen::Service> data() const override;

    /******************************************************************************//**
     * \fn parse
     * \brief Parse Services metadata.
     * \param [in] aInputFile input file metadata.
    **********************************************************************************/
    void parse(std::istream &aInputFile) override;
};

}
// namespace XMLGen
