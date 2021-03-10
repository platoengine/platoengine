/*
 * XMLGeneratorMaterialFunctionInterface.hpp
 *
 *  Created on: Jun 5, 2020
 */

#pragma once

#include <vector>

#include "pugixml.hpp"
#include "XMLGeneratorAnalyzeFunctionMapTypes.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \struct The goal of this C++ struct is to provide an interface for the functions \n
 * used to append material properties, i.e. material models, to plato_analyze_operation.xml \n
 * file. This interface reduces cyclomatic complexity due to having multiple \n
 * material models implemented in Plato Analyze.
**********************************************************************************/
struct MaterialFunctionInterface
{
private:
    /*!< map from material category to function used to append material properties */
    XMLGen::Analyze::MaterialFunctionMap mMap;

    /******************************************************************************//**
     * \fn insert
     * \brief Insert each function used to append material properties to material function map.
    **********************************************************************************/
    void insert();

public:
    /******************************************************************************//**
     * \fn MaterialFunctionInterface
     * \brief Default constructor
    **********************************************************************************/
    MaterialFunctionInterface();

    /******************************************************************************//**
     * \fn call
     * \brief Append material properties to plato_analyze_operation.xml file.
     * \param [in]     aMaterialName material name
     * \param [in]     aCategory     material category, e.g. 'isotropic linear elastic'
     * \param [in]     aMaterialTags material tags, i.e. vector<pair<material_property_argument_name_tag, material_property_tag>>
     * \param [in/out] aParentNode   pugi::xml_node
    **********************************************************************************/
    void call
    (const std::string &aMaterialName,
     const std::string &aCategory,
     const std::vector<std::pair<std::string,std::string>>& aMaterialTags,
     pugi::xml_node &aParentNode);
};
// struct MaterialFunctionInterface

}
// namespace XMLGen
