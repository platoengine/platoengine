/*
 * XMLGeneratorMaterialFunctionInterface.cpp
 *
 *  Created on: Jun 5, 2020
 */

#include "XMLGeneratorMaterialFunctionInterface.hpp"
#include "XMLGeneratorPlatoAnalyzeOperationsFileUtilities.hpp"

namespace XMLGen
{

MaterialFunctionInterface::MaterialFunctionInterface()
{
    this->insert();
}

void MaterialFunctionInterface::insert()
{
    auto tFuncIndex = std::type_index(typeid(append_isotropic_linear_elastic_material_properties_to_plato_analyze_operation));
    mMap.insert(std::make_pair("isotropic linear elastic",
      std::make_pair((XMLGen::Analyze::MaterialFunction)append_isotropic_linear_elastic_material_properties_to_plato_analyze_operation, tFuncIndex)));

    tFuncIndex = std::type_index(typeid(append_isotropic_linear_thermoelastic_material_properties_to_plato_analyze_operation));
    mMap.insert(std::make_pair("isotropic linear thermoelastic",
      std::make_pair((XMLGen::Analyze::MaterialFunction)append_isotropic_linear_thermoelastic_material_properties_to_plato_analyze_operation, tFuncIndex)));
}

void MaterialFunctionInterface::call
(const std::string &aCategory,
 const std::vector<std::pair<std::string,std::string>>& aMaterialTags,
 pugi::xml_node &aParentNode)
{
    auto tLowerFuncLabel = Plato::tolower(aCategory);
    auto tMapItr = mMap.find(tLowerFuncLabel);
    if(tMapItr == mMap.end())
    {
        THROWERR(std::string("Material Function Interface: Did not find material function with tag '") + aCategory + "' in list.")
    }
    auto tTypeCastedFunc =
        reinterpret_cast<void(*)(const std::vector<std::pair<std::string,std::string>>&, pugi::xml_node&)>(tMapItr->second.first);
    if(tMapItr->second.second == std::type_index(typeid(tTypeCastedFunc)))
    {
        THROWERR(std::string("Material Function Interface: Reinterpret cast for material function with tag '") + aCategory + "' failed.")
    }
    tTypeCastedFunc(aMaterialTags, aParentNode);
}

}
// namespace XMLGen
