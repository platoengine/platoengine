/*
 * XMLGeneratorAnalyzeLoadTagFunctionInterface.hpp
 *
 *  Created on: Jun 15, 2020
 */

#pragma once

#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorAnalyzeFunctionMapTypes.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \struct Interface used to define the load tags supported in Plato Analyze. \n
 * This interface reduces cyclomatic complexity due to support of multiple load 
 * types in Plato Analyze.
**********************************************************************************/
struct LoadTag
{
private:
    /*!< map from load category to function used to define its tag */
    XMLGen::Analyze::LoadTagFuncMap mMap;

    /******************************************************************************//**
     * \fn insert
     * \brief Insert functions to load functions map.
     **********************************************************************************/
    void insert();

public:
    /******************************************************************************//**
     * \fn LoadTag
     * \brief Default constructor
    **********************************************************************************/
    LoadTag() { this->insert(); }

    /******************************************************************************//**
     * \fn call
     * \brief Return load name for plato_analyze_input_deck.xml.
     * \param [in] aLoad load metadata
     * \return load name
    **********************************************************************************/
    std::string call(const XMLGen::Load& aLoad) const;
};
// struct LoadTag

}
// namespace XMLGen
