/*
 * XMLGeneratorParseScenario.hpp
 *
 *  Created on: Jun 18, 2020
 */

#pragma once

#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorParseMetadata.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \class ParseScenario
 * \brief Parse inputs in scenario block and store values in XMLGen::Scenario.
**********************************************************************************/
class ParseScenario : public XMLGen::ParseMetadata<XMLGen::Scenario>
{
private:
    XMLGen::Scenario mData; /*!< scenario metadata */
    XMLGen::UseCaseTags mTags; /*!< map from plato input file tags to valid tokens-value pairs, i.e. map<tag, pair<tokens,value> > */

private:
    /******************************************************************************//**
     * \fn allocate
     * \brief Allocate map from valid tags to valid tokens-value pair
    **********************************************************************************/
    void allocate();

    /******************************************************************************//**
     * \fn setCode
     * \brief Set 'code' keyword, default = 'plato_analyze'.
    **********************************************************************************/
    void setCode();

    /******************************************************************************//**
     * \fn setPhysics
     * \brief Set 'physics' keyword, throw error if input keyword is empty.
    **********************************************************************************/
    void setPhysics();

    /******************************************************************************//**
     * \fn setPhysics
     * \brief Set 'performer' keyword, default = 'code_keyword' + '0', where \n
     * 'code_keyword' denotes the value set for 'code' keyword.
    **********************************************************************************/
    void setPerformer();

    /******************************************************************************//**
     * \fn setScenarioID
     * \brief Set 'scenario_id' keyword, default = 'code_keyword' + \n
     * 'physics_keyword' + '0', where \n 'code_keyword' denotes the value set for \n
     * 'code' keyword and 'physics_keyword' denotes the value set for 'physics' keyword.
    **********************************************************************************/
    void setScenarioID();

    /******************************************************************************//**
     * \fn setScenarioID
     * \brief Set 'dimensions' keyword, throw error if input spatial dimensions \n
     * are not supported.
    **********************************************************************************/
    void setDimensions();

    /******************************************************************************//**
     * \fn setMateriaPenaltyExponent
     * \brief Set 'material_penalty_exponent' keyword, default = '3.0'.
    **********************************************************************************/
    void setMateriaPenaltyExponent();

    /******************************************************************************//**
     * \fn setMinimumErsatzMaterialValue
     * \brief Set 'minimum_ersatz_material_value' keyword, default = '1e-9'.
    **********************************************************************************/
    void setMinimumErsatzMaterialValue();

    /******************************************************************************//**
     * \fn setCacheState
     * \brief Set cache state flag, which is used to enable or disable the cache state \n
     * operation, default = false.
    **********************************************************************************/
    void setCacheState();

    /******************************************************************************//**
     * \fn setUpdateProblem
     * \brief Set update problem flag, which is used to enable or disable the update \n
     * problem operation, default = false.
    **********************************************************************************/
    void setUpdateProblem();

    /******************************************************************************//**
     * \fn setUseAnalyzeNewUQWorkflow
     * \brief Set use analyze new uncertainty quantification workflow flag, default = false.
    **********************************************************************************/
    void setUseAnalyzeNewUQWorkflow();

    /******************************************************************************//**
     * \fn setMetaData
     * \brief Set XMLGen::Scenario metadata.
    **********************************************************************************/
    void setMetaData();

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return scenario metadata.
     * \return metadata
    **********************************************************************************/
    XMLGen::Scenario data() const override;

    /******************************************************************************//**
     * \fn parse
     * \brief Parse output metadata.
     * \param [in] aInputFile input file metadata.
    **********************************************************************************/
    void parse(std::istream &aInputFile) override;
};

}
// namespace XMLGen
