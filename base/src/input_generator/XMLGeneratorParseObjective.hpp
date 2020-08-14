/*
 * XMLGeneratorParseObjective.hpp
 *
 *  Created on: Jun 17, 2020
 */

#pragma once

#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorParseMetadata.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \class ParseObjective
 * \brief Parse inputs in objective function blocks and store values in XMLGen::Objective.
**********************************************************************************/
class ParseObjective : public XMLGen::ParseMetadata<XMLGen::Objective>
{
private:
    XMLGen::MetaDataTags mTags; /*!< map from plato input file tags to valid tokens-value pairs, i.e. map<tag, pair<tokens,value> > */
    XMLGen::Objective mObjective; /*!< objective functions metadata */

private:
    /******************************************************************************//**
     * \fn allocate
     * \brief Allocate map from valid tags to valid tokens-value pair
    **********************************************************************************/
    void allocate();

    /******************************************************************************//**
     * \fn setIdentification
     * \brief Set objective functions identification number/name. If there are objective \n
     * functions without an identification number/name, assign default values.
    **********************************************************************************/
    void setIdentification();

    /******************************************************************************//**
     * \fn checkPerfomerNames
     * \brief If performer keywords are empty, set to default value 'code_name' + 'id', \n
     * where 'code_name' denotes the input code name provided by the user and 'id' \n
     * denotes the identification name/number for an objective function.
    **********************************************************************************/
    void checkPerfomerNames();

    /******************************************************************************//**
     * \fn finalize
     * \brief Perform final checks and member data definitions for objective functions.
    **********************************************************************************/
    void finalize();

    /******************************************************************************//**
     * \fn setType
     * \brief Set objective type.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setType(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setWeight
     * \brief Set objective function weight.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setWeights(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setCriteriaIDs
     * \brief Set IDs of criteria that compose the objective.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setCriteriaIDs(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setServiceIDs
     * \brief Set IDs of services that will provide the criteria evaluations.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setServiceIDs(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setMetaData
     * \brief Set objective metadata.
     * \param [in] aMetadata objective metadata
    **********************************************************************************/
    void setMetaData(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn checkType
     * \brief Throw error if input objective function category is not supported.
     * \param [in] aMetadata objective metadata
    **********************************************************************************/
    void checkType(const XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn checkMetaData
     * \brief Check if objective metadata is properly defined.
     * \param [in] aMetadata objective metadata
    **********************************************************************************/
    void checkMetaData(XMLGen::Objective& aMetadata);

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return objective functions metadata.
     * \return container of objective functions and corresponding metadata
    **********************************************************************************/
    XMLGen::Objective data() const override;

    /******************************************************************************//**
     * \fn parse
     * \brief Parse objective functions metadata.
     * \param [in] aInputFile input metadata
    **********************************************************************************/
    void parse(std::istream& aInputFile) override;
};
// class ParseObjective

}
// namespace XMLGen
