/*
 * XMLGeneratorParseEssentialBoundaryCondition.hpp
 *
 *  Created on: Dec 8, 2020
 */

#pragma once

#include "XMLGeneratorParseMetadata.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorEssentialBoundaryConditionMetadata.hpp"

namespace XMLGen
{

class ParseEssentialBoundaryCondition : public XMLGen::ParseMetadata<std::vector<XMLGen::EssentialBoundaryCondition>>
{
private:
    XMLGen::MetaDataTags mTags; /*!< map from plato input file tags to valid tokens-value pairs, i.e. map<tag, pair<tokens,value> > */
    std::vector<XMLGen::EssentialBoundaryCondition> mData; /*!< ebc metadata */

private:
    /******************************************************************************//**
     * \fn allocate
     * \brief Allocate map from valid tags to valid tokens-value pair
    **********************************************************************************/
    void allocate();

    /******************************************************************************//**
     * \fn insertCoreProperties
     * \brief Insert core ebc properties, e.g. identifiers, to map from plato \n
     * input file tags to valid tokens-value pairs
    **********************************************************************************/
    void insertCoreProperties();

    /******************************************************************************//**
     * \fn setMetaData
     * \brief Set XMLGen::EssentialBoundaryCondition metadata.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setMetadata(XMLGen::EssentialBoundaryCondition& aMetadata);

    /******************************************************************************//**
     * \fn checkUniqueIDs
     * \brief Throw error if EssentialBoundaryCondition block identification numbers are not unique.
    **********************************************************************************/
    void checkUniqueIDs();

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return EssentialBoundaryCondition metadata.
     * \return metadata
    **********************************************************************************/
    std::vector<XMLGen::EssentialBoundaryCondition> data() const override;

    /******************************************************************************//**
     * \fn parse
     * \brief Parse EssentialBoundaryCondition metadata.
     * \param [in] aInputFile input file metadata.
    **********************************************************************************/
    void parse(std::istream &aInputFile) override;

    /******************************************************************************//**
     * \fn setEssentialBoundaryConditionIdentification
     * \brief Set the EssentialBoundaryCondition id.
     * \param [in] aMetadata essential boundary condition metadata.
    **********************************************************************************/
    void setEssentialBoundaryConditionIdentification(XMLGen::EssentialBoundaryCondition& aMetadata);

    /******************************************************************************//**
     * \fn setType
     * \brief Set the EssentialBoundaryCondition type.
     * \param [in] aMetadata essential boundary condition metadata.
    **********************************************************************************/
    void setType(XMLGen::EssentialBoundaryCondition& aMetadata);

    /******************************************************************************//**
     * \fn setLocationType
     * \brief Set the EssentialBoundaryCondition location type.
     * \param [in] aMetadata essential boundary condition metadata.
    **********************************************************************************/
    void setLocationType(XMLGen::EssentialBoundaryCondition& aMetadata);

    /******************************************************************************//**
     * \fn setLocationName
     * \brief Set the EssentialBoundaryCondition location name.
     * \param [in] aMetadata essential boundary condition metadata.
    **********************************************************************************/
    void setLocationName(XMLGen::EssentialBoundaryCondition& aMetadata);

    /******************************************************************************//**
     * \fn setLocationID
     * \brief Set the EssentialBoundaryCondition location id.
     * \param [in] aMetadata essential boundary condition metadata.
    **********************************************************************************/
    void setLocationID(XMLGen::EssentialBoundaryCondition& aMetadata);

    /******************************************************************************//**
     * \fn setDegreeOfFreedom
     * \brief Set the EssentialBoundaryCondition degree of freedom.
     * \param [in] aMetadata essential boundary condition metadata.
    **********************************************************************************/
    void setDegreeOfFreedom(XMLGen::EssentialBoundaryCondition& aMetadata);

    /******************************************************************************//**
     * \fn setValue
     * \brief Set the EssentialBoundaryCondition value.
     * \param [in] aMetadata essential boundary condition metadata.
    **********************************************************************************/
    void setValue(XMLGen::EssentialBoundaryCondition& aMetadata);

};
// class ParseEssentialBoundaryCondition

}
// namespace XMLGen
