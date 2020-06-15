/*
 * XMLGeneratorAnalyzeMaterialModelFunctionInterface.hpp
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
 * functions used to append material models to plato_analyze_input_deck.xml. \n
 * This interface reduces cyclomatic complexity due to having multiple material \n
 * models in Plato Analyze.
**********************************************************************************/
struct AppendMaterialModelParameters
{
private:
    /*!< map from material model category to function used to append material properties and respective properties */
    XMLGen::Analyze::MaterialModelFuncMap mMap;

    /******************************************************************************//**
     * \fn insert
     * \brief Insert material functions to material model function map.
     **********************************************************************************/
    void insert();

public:
    /******************************************************************************//**
     * \fn AppendMaterialModelParameters
     * \brief Default constructor
    **********************************************************************************/
    AppendMaterialModelParameters();

    /******************************************************************************//**
     * \fn call
     * \brief Append material model and corresponding parameters to plato_analyze_input_deck.xml file.
     * \param [in]     aMaterial    material model metadata
     * \param [in/out] aParentNode  pugi::xml_node
    **********************************************************************************/
    void call(const XMLGen::Material& aMaterial, pugi::xml_node &aParentNode) const;
};
// struct AppendMaterialModelParameters

}
// namespace XMLGen
