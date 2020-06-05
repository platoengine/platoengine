/*
 * XMLGeneratorMaterialFunctionInterface.hpp
 *
 *  Created on: Jun 5, 2020
 */

#pragma once

#include <string>
#include <typeindex>
#include <unordered_map>

#include "pugixml.hpp"

namespace XMLGen
{

namespace Analyze
{
/*!< function pointer type */
typedef void (*MaterialFunction)(void);

/*!< map from material category to material function used to append material parameter tags, \n
 * i.e. map<material_category, function> */
typedef std::unordered_map<std::string, MaterialFunction> MaterialFunctionMap;
}

/******************************************************************************//**
 * \struct The objective of this C++ struct is to serve as an interface to the \n
 * functions used to append material properties to the plato_analyze_operation.xml \n
 * file. This interface reduces the cyclomatic complexity associated with having
 * multiple material models in Plato Analyze.
**********************************************************************************/
struct MaterialFunctionInterface
{
private:
    /*!< map from material category to function used to append material properties */
    std::unordered_map<std::string, std::pair<XMLGen::Analyze::MaterialFunction, std::type_index>> mMap;

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
     * \brief Call function used to append material properties to the \n
     *   plato_analyze_operation.xml file.
     * \param [in]     aMaterialCategory material category, e.g. 'isotropic linear elastic'
     * \param [in]     aMaterialTags     material tags, \n
     *   i.e. vector<pair<material_property_argument_name_tag, material_property_tag>>
     * \param [in/out] aDocument         pugi::xml_document
    **********************************************************************************/
    void call
    (const std::string &aMaterialCategory,
     const std::vector<std::pair<std::string,std::string>>& aMaterialTags,
     pugi::xml_document &aDocument);
};
// struct MaterialFunctionInterface

}
// namespace XMLGen
