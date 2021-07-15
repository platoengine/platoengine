/*
 * XMLGeneratorParseLoads.hpp
 *
 *  Created on: Jan 5, 2021
 */

#pragma once

#include "XMLGeneratorParseMetadata.hpp"
#include "XMLGeneratorLoadMetadata.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

class ParseLoad : public XMLGen::ParseMetadata<std::vector<XMLGen::Load>>
{
private:
    XMLGen::MetaDataTags mTags; /*!< map from plato input file tags to valid tokens-value pairs, i.e. map<tag, pair<tokens,value> > */
    std::vector<XMLGen::Load> mData; /*!< load metadata */

private:
    /******************************************************************************//**
     * \fn allocate
     * \brief Allocate map from valid tags to valid tokens-value pair
    **********************************************************************************/
    void allocate();

    /******************************************************************************//**
     * \fn insertCoreProperties
     * \brief Insert core load properties, e.g. identifiers, to map from plato \n
     * input file tags to valid tokens-value pairs
    **********************************************************************************/
    void insertCoreProperties();

    /******************************************************************************//**
     * \fn setMetaData
     * \brief Set XMLGen::Load metadata.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setMetadata(XMLGen::Load& aMetadata);

    /******************************************************************************//**
     * \fn checkUniqueIDs
     * \brief Throw error if Load block identification numbers are not unique.
    **********************************************************************************/
    void checkUniqueIDs();

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return Load metadata.
     * \return metadata
    **********************************************************************************/
    std::vector<XMLGen::Load> data() const override;

    /******************************************************************************//**
     * \fn parse
     * \brief Parse Load metadata.
     * \param [in] aInputFile input file metadata.
    **********************************************************************************/
    void parse(std::istream &aInputFile) override;

    /******************************************************************************//**
     * \fn expandDofs
     * \brief Expand multi-dof/value specifications into multiple BCs
    **********************************************************************************/
    void expandDofs();

    /******************************************************************************//**
     * \fn setLoadIdentification
     * \brief Set the Load id.
     * \param [in] aMetadata load metadata.
    **********************************************************************************/
    void setLoadIdentification(XMLGen::Load& aMetadata);

    /******************************************************************************//**
     * \fn setType
     * \brief Set the Load type.
     * \param [in] aMetadata load metadata.
    **********************************************************************************/
    void setType(XMLGen::Load& aMetadata);

    /******************************************************************************//**
     * \fn setLocationType
     * \brief Set the Load location type.
     * \param [in] aMetadata load metadata.
    **********************************************************************************/
    void setLocationType(XMLGen::Load& aMetadata);

    /******************************************************************************//**
     * \fn setLocationID
     * \brief Set the Load location id.
     * \param [in] aMetadata load metadata.
    **********************************************************************************/
    void setLocationID(XMLGen::Load& aMetadata);

    /******************************************************************************//**
     * \fn setLocationName
     * \brief Set the Load location name.
     * \param [in] aMetadata load metadata.
    **********************************************************************************/
    void setLocationName(XMLGen::Load& aMetadata);

    /******************************************************************************//**
     * \fn setValueMetadata
     * \brief Set the Load value.
     * \param [in] aMetadata load metadata.
    **********************************************************************************/
    void setValueMetadata(XMLGen::Load& aMetadata);

};
// class ParseLoad

}
// namespace XMLGen
