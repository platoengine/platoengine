/*
 * XMLGeneratorAnalyzeNaturalBCFunctionInterface.hpp
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
 * \struct The goal of this C++ struct is to provide an interface for the \n
 * functions used to define natural boundary conditions' names for \n
 * plato_analyze_input_deck.xml. This interface reduces cyclomatic complexity \n
 * due to multiple natural boundary conditions categories in Plato Analyze.
**********************************************************************************/
struct AppendNaturalBoundaryCondition
{
private:
    /*!< map from natural boundary condition category to function used to define its properties */
    XMLGen::Analyze::NaturalBCFuncMap mMap;

    /******************************************************************************//**
     * \fn insert
     * \brief Insert natural boundary condition functions to map.
     **********************************************************************************/
    void insert();

public:
    /******************************************************************************//**
     * \fn AppendNaturalBoundaryCondition
     * \brief Default constructor
    **********************************************************************************/
    AppendNaturalBoundaryCondition() { this->insert(); }

    /******************************************************************************//**
     * \fn call
     * \brief Append natural boundary condition parameters to plato_analyze_input_deck.xml.
     * \param [in]     aName        natural boundary condition identification name
     * \param [in]     aLoad        natural boundary condition metadata
     * \param [in/out] aParentNode  pugi::xml_node
    **********************************************************************************/
    void call(const std::string& aName, const XMLGen::NaturalBoundaryCondition& aLoad, pugi::xml_node& aParentNode) const;
};
// struct AppendNaturalBoundaryCondition

}
