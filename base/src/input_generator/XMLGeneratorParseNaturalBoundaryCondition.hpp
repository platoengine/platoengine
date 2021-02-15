/*
 * XMLGeneratorParseNaturalBoundaryCondition.hpp
 *
 *  Created on: Jan 5, 2021
 */

#pragma once

#include "XMLGeneratorParseMetadata.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorNaturalBoundaryConditionMetadata.hpp"

namespace XMLGen
{

class ParseNaturalBoundaryCondition : public XMLGen::ParseMetadata<std::vector<XMLGen::NaturalBoundaryCondition>>
{
private:
    XMLGen::MetaDataTags mTags; /*!< map from plato input file tags to valid tokens-value pairs, i.e. map<tag, pair<tokens,value> > */
    std::vector<XMLGen::NaturalBoundaryCondition> mData; /*!< nbc metadata */

private:
    /******************************************************************************//**
     * \fn allocate
     * \brief Allocate map from valid tags to valid tokens-value pair
    **********************************************************************************/
    void allocate();

    /******************************************************************************//**
     * \fn insertCoreProperties
     * \brief Insert core nbc properties, e.g. identifiers, to map from plato \n
     * input file tags to valid tokens-value pairs
    **********************************************************************************/
    void insertCoreProperties();

    /******************************************************************************//**
     * \fn setMetaData
     * \brief Set XMLGen::NaturalBoundaryCondition metadata.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setMetadata(XMLGen::NaturalBoundaryCondition& aMetadata);

    /******************************************************************************//**
     * \fn checkUniqueIDs
     * \brief Throw error if NaturalBoundaryCondition block identification numbers are not unique.
    **********************************************************************************/
    void checkUniqueIDs();

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return NaturalBoundaryCondition metadata.
     * \return metadata
    **********************************************************************************/
    std::vector<XMLGen::NaturalBoundaryCondition> data() const override;

    /******************************************************************************//**
     * \fn parse
     * \brief Parse NaturalBoundaryCondition metadata.
     * \param [in] aInputFile input file metadata.
    **********************************************************************************/
    void parse(std::istream &aInputFile) override;

    /******************************************************************************//**
     * \fn expandDofs
     * \brief Expand multi-dof/value specifications into multiple BCs
    **********************************************************************************/
    void expandDofs();

    /******************************************************************************//**
     * \fn setNaturalBoundaryConditionIdentification
     * \brief Set the NaturalBoundaryCondition id.
     * \param [in] aMetadata natural boundary condition metadata.
    **********************************************************************************/
    void setNaturalBoundaryConditionIdentification(XMLGen::NaturalBoundaryCondition& aMetadata);

    /******************************************************************************//**
     * \fn setType
     * \brief Set the NaturalBoundaryCondition type.
     * \param [in] aMetadata natural boundary condition metadata.
    **********************************************************************************/
    void setType(XMLGen::NaturalBoundaryCondition& aMetadata);

    /******************************************************************************//**
     * \fn setLocationType
     * \brief Set the NaturalBoundaryCondition location type.
     * \param [in] aMetadata natural boundary condition metadata.
    **********************************************************************************/
    void setLocationType(XMLGen::NaturalBoundaryCondition& aMetadata);

    /******************************************************************************//**
     * \fn setLocationID
     * \brief Set the NaturalBoundaryCondition location id.
     * \param [in] aMetadata natural boundary condition metadata.
    **********************************************************************************/
    void setLocationID(XMLGen::NaturalBoundaryCondition& aMetadata);

    /******************************************************************************//**
     * \fn setLocationName
     * \brief Set the NaturalBoundaryCondition location name.
     * \param [in] aMetadata natural boundary condition metadata.
    **********************************************************************************/
    void setLocationName(XMLGen::NaturalBoundaryCondition& aMetadata);

    /******************************************************************************//**
     * \fn setValueMetadata
     * \brief Set the NaturalBoundaryCondition value.
     * \param [in] aMetadata natural boundary condition metadata.
    **********************************************************************************/
    void setValueMetadata(XMLGen::NaturalBoundaryCondition& aMetadata);

};
// class ParseNaturalBoundaryCondition

}
// namespace XMLGen
