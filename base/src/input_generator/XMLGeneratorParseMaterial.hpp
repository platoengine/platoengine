/*
 * XMLGeneratorParseMaterial.hpp
 *
 *  Created on: Jun 23, 2020
 */

#pragma once

#include "XMLGeneratorParseMetadata.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorMaterialMetadata.hpp"

namespace XMLGen
{

class ParseMaterial : public XMLGen::ParseMetadata<std::vector<XMLGen::Material>>
{
private:
    XMLGen::UseCaseTags mTags; /*!< map from plato input file tags to valid tokens-value pairs, i.e. map<tag, pair<tokens,value> > */
    std::vector<XMLGen::Material> mData; /*!< materials metadata */

private:
    /******************************************************************************//**
     * \fn allocate
     * \brief Allocate map from valid tags to valid tokens-value pair
    **********************************************************************************/
    void allocate();

    /******************************************************************************//**
     * \fn setCode
     * \brief Set 'code' keyword, default = 'plato_analyze'.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setCode(XMLGen::Material& aMetadata);

    /******************************************************************************//**
     * \fn setMaterialModel
     * \brief Set 'material model' keyword, throw error if it is not defined.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setMaterialModel(XMLGen::Material& aMetadata);

    /******************************************************************************//**
     * \fn setMaterialIdentification
     * \brief Set 'id' keyword, throw error if it is not defined.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setMaterialIdentification(XMLGen::Material& aMetadata);

    /******************************************************************************//**
     * \fn setMaterialProperties
     * \brief Set material properties, throw error if not defined.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setMaterialProperties(XMLGen::Material& aMetadata);

    /******************************************************************************//**
     * \fn setMetaData
     * \brief Set XMLGen::Material metadata.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setMetadata(XMLGen::Material& aMetadata);

    /******************************************************************************//**
     * \fn setPenaltyExponent
     * \brief Set penalty model exponent value.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setPenaltyExponent(XMLGen::Material& aMetadata);

    /******************************************************************************//**
     * \fn checkMaterialProperties
     * \brief Throw error if material properties are not defined.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void checkMaterialProperties(XMLGen::Material& aMetadata);

    /******************************************************************************//**
     * \fn checkUniqueIDs
     * \brief Throw error if material block identification numbers are not unique.
    **********************************************************************************/
    void checkUniqueIDs();

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return material metadata.
     * \return metadata
    **********************************************************************************/
    std::vector<XMLGen::Material> data() const override;

    /******************************************************************************//**
     * \fn parse
     * \brief Parse output metadata.
     * \param [in] aInputFile input file metadata.
    **********************************************************************************/
    void parse(std::istream &aInputFile) override;
};
// class ParseMaterial

}
// namespace XMLGen