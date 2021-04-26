/*
 * XMLGeneratorAnalyzeEssentialBCFunctionInterface.hpp
 *
 *  Created on: Jun 15, 2020
 */

#pragma once

#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorAnalyzeFunctionMapTypes.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \struct The goal of this C++ struct is to provide an interface for the \n
 * functions used to define essential boundary conditions' tags for \n
 * plato_analyze_input_deck.xml. This interface reduces cyclomatic complexity \n
 * due to multiple essential boundary conditions categories in Plato Analyze.
**********************************************************************************/
struct EssentialBoundaryConditionTag
{
private:
    /*!< map from Neumman load category to function used to define its tag */
    XMLGen::Analyze::EssentialBCTagFuncMap mMap;

    /******************************************************************************//**
     * \fn insert
     * \brief Insert functions to essential boundary condition functions map.
     **********************************************************************************/
    void insert();

public:
    /******************************************************************************//**
     * \fn EssentialBoundaryConditionTag
     * \brief Default constructor
    **********************************************************************************/
    EssentialBoundaryConditionTag() { this->insert(); }

    /******************************************************************************//**
     * \fn call
     * \brief Return essential boundary condition tag for plato_analyze_input_deck.xml.
     * \param [in] aBC essential boundary condition metadata
     * \return essential boundary condition tag
    **********************************************************************************/
    std::string call(const XMLGen::EssentialBoundaryCondition& aBC) const;
};
// struct EssentialBoundaryConditionTag

}
// namespace XMLGen
