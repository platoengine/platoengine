/*
 * XMLGeneratorAnalyzeLoadFunctionInterface.hpp
 *
 *  Created on: Jun 15, 2020
 */

#pragma once

#include "pugixml.hpp"

#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorAnalyzeFunctionMapTypes.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \struct Interface to functions used to define the load tags supported in Plato \n
 * Analyze. The interface reduces cyclomatic complexity due to the support of \n 
 * multiple load categories in Plato Analyze.
**********************************************************************************/
struct AppendLoad
{
private:
    /*!< map from load category to function used to define its properties */
    XMLGen::Analyze::LoadFuncMap mMap;

    /******************************************************************************//**
     * \fn insert
     * \brief Insert load functions to map.
     **********************************************************************************/
    void insert();

public:
    /******************************************************************************//**
     * \fn AppendLoad
     * \brief Default constructor
    **********************************************************************************/
    AppendLoad() { this->insert(); }

    /******************************************************************************//**
     * \fn call
     * \brief Append load parameters to plato_analyze_input_deck.xml.
     * \param [in]     aName        load identification name
     * \param [in]     aLoad        load metadata
     * \param [in/out] aParentNode  pugi::xml_node
    **********************************************************************************/
    void call(const std::string& aName, const XMLGen::Load& aLoad, pugi::xml_node& aParentNode) const;
};
// struct AppendLoad

}
