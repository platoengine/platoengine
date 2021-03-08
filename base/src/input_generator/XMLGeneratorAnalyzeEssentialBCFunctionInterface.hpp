/*
 * XMLGeneratorAnalyzeEssentialBCFunctionInterface.hpp
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
 * functions used to define essential boundary conditions' names for \n
 * plato_analyze_input_deck.xml. This interface reduces cyclomatic complexity \n
 * due to multiple essential boundary conditions categories in Plato Analyze.
**********************************************************************************/
struct AppendEssentialBoundaryCondition
{
private:
    /*!< map from essential boundary condition category to function used to append its properties */
    XMLGen::Analyze::EssentialBCFuncMap mMap;

    /******************************************************************************//**
     * \fn insert
     * \brief Insert essential boundary condition functions to map.
     **********************************************************************************/
    void insert();

public:
    /******************************************************************************//**
     * \fn AppendEssentialBoundaryCondition
     * \brief Default constructor
    **********************************************************************************/
    AppendEssentialBoundaryCondition() { this->insert(); }

    /******************************************************************************//**
     * \fn call
     * \brief Append essential boundary condition parameters to plato_analyze_input_deck.xml.
     * \param [in]     aName        essential boundary condition name
     * \param [in]     aPhysics     essential boundary condition physics
     * \param [in]     aBC          essential boundary condition metadata
     * \param [in/out] aParentNode  pugi::xml_node
    **********************************************************************************/
    void call(const std::string& aName, const std::string& aPhysics, const XMLGen::EssentialBoundaryCondition& aBC, pugi::xml_node& aParentNode) const;
};
// struct AppendEssentialBoundaryCondition

}
// namespace XMLGen
