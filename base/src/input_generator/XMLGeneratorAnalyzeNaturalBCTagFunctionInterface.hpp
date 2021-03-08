/*
 * XMLGeneratorAnalyzeNaturalBCTagFunctionInterface.hpp
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
 * functions used to define natural boundary conditions' tags for \n
 * plato_analyze_input_deck.xml. This interface reduces cyclomatic complexity \n
 * due to multiple natural boundary conditions categories in Plato Analyze.
**********************************************************************************/
struct NaturalBoundaryConditionTag
{
private:
    /*!< map from natural boundary condition category to function used to define its tag */
    XMLGen::Analyze::NaturalBCTagFuncMap mMap;

    /******************************************************************************//**
     * \fn insert
     * \brief Insert functions to natural boundary condition functions map.
     **********************************************************************************/
    void insert();

public:
    /******************************************************************************//**
     * \fn NaturalBoundaryConditionTag
     * \brief Default constructor
    **********************************************************************************/
    NaturalBoundaryConditionTag() { this->insert(); }

    /******************************************************************************//**
     * \fn call
     * \brief Return natural boundary condition name for plato_analyze_input_deck.xml.
     * \param [in] aLoad natural boundary condition metadata
     * \return natural boundary condition name
    **********************************************************************************/
    std::string call(const XMLGen::NaturalBoundaryCondition& aLoad) const;
};
// struct NaturalBoundaryConditionTag

}
// namespace XMLGen
