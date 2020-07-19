/*
 * XMLGeneratorAnalyzeMaterialModelFunctionInterface.cpp
 *
 *  Created on: Jun 15, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeInputFileUtilities.hpp"
#include "XMLGeneratorAnalyzeMaterialModelFunctionInterface.hpp"

namespace XMLGen
{

namespace Private
{

void is_material_property_tags_container_empty
(const XMLGen::Material& aMaterial)
{
    auto tTags = aMaterial.tags();
    if(tTags.empty())
    {
        THROWERR("Check Material Property Tags Container: Material tags container is empty, "
            + "i.e. material properties are not defined.")
    }
}
// function check_material_property_tags_container

void append_material_property
(const std::string& aMaterialPropertyTag,
 const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    auto tMaterialModelTag = aMaterial.category();
    XMLGen::ValidAnalyzeMaterialPropertyKeys tValidKeys;
    auto tValueType = tValidKeys.type(tMaterialModelTag, aMaterialPropertyTag);
    auto tAnalyzeMatPropertyTag = tValidKeys.tag(tMaterialModelTag, aMaterialPropertyTag);
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {tAnalyzeMatPropertyTag, tValueType, aMaterial.property(aMaterialPropertyTag)};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}
// function append_material_property

void append_material_properties_to_plato_analyze_material_model
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    XMLGen::Private::is_material_property_tags_container_empty(aMaterial);

    auto tTags = aMaterial.tags();
    auto tMaterialModelTag = aMaterial.category();
    XMLGen::ValidAnalyzeMaterialPropertyKeys tValidKeys;
    std::vector<std::string> tKeys = {"name", "type", "value"};
    for(auto& tMaterialPropTag : tTags)
    {
        auto tValueType = tValidKeys.type(tMaterialModelTag, tMaterialPropTag);
        auto tValidAnalyzeMaterialPropertyTag = tValidKeys.tag(tMaterialModelTag, tMaterialPropTag);
        std::vector<std::string> tValues = {tValidAnalyzeMaterialPropertyTag, tValueType, aMaterial.property(tMaterialPropTag)};
        XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
    }
}
// function append_material_properties_to_plato_analyze_material_model

void append_isotropic_linear_elastic_material_to_plato_problem
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    auto tMaterialModel = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Material Model"}, tMaterialModel);
    auto tIsotropicLinearElasticMaterial = tMaterialModel.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Isotropic Linear Elastic"}, tIsotropicLinearElasticMaterial);
    XMLGen::Private::append_material_properties_to_plato_analyze_material_model(aMaterial, tIsotropicLinearElasticMaterial);
}
// function append_isotropic_linear_elastic_material_to_plato_problem

void append_isotropic_linear_thermal_material_to_plato_problem
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    auto tMaterialModel = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Material Model"}, tMaterialModel);

    // append thermal conduction property
    auto tThermalConduction = tMaterialModel.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Thermal Conduction"}, tThermalConduction);
    XMLGen::Private::append_material_property("thermal_conductivity", aMaterial, tThermalConduction);

    // append thermal mass properties
    auto tThermalMass = tMaterialModel.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Thermal Mass"}, tThermalMass);
    XMLGen::Private::append_material_property("mass_density", aMaterial, tThermalMass);
    XMLGen::Private::append_material_property("specific_heat", aMaterial, tThermalMass);
}
// function append_isotropic_linear_thermal_material_to_plato_problem

void append_isotropic_linear_thermoelastic_material_to_plato_problem
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    auto tMaterialModel = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Material Model"}, tMaterialModel);

    // append thermal properties
    auto tThermalProperties = tMaterialModel.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Thermoelastic"}, tThermalProperties);
    XMLGen::Private::append_material_property("thermal_expansivity", aMaterial, tThermalProperties);
    XMLGen::Private::append_material_property("thermal_conductivity", aMaterial, tThermalProperties);
    XMLGen::Private::append_material_property("reference_temperature", aMaterial, tThermalProperties);

    // append elastic properties
    auto tElasticProperties = tThermalProperties.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Elastic Stiffness"}, tElasticProperties);
    XMLGen::Private::append_material_property("youngs_modulus", aMaterial, tElasticProperties);
    XMLGen::Private::append_material_property("poissons_ratio", aMaterial, tElasticProperties);
}
// function append_isotropic_linear_thermoelastic_material_to_plato_problem

void append_isotropic_linear_electroelastic_material_to_plato_problem
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    auto tMaterialModel = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Material Model"}, tMaterialModel);
    auto tIsotropicLinearElasticMaterial = tMaterialModel.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Isotropic Linear Electroelastic"}, tIsotropicLinearElasticMaterial);
    XMLGen::Private::append_material_properties_to_plato_analyze_material_model(aMaterial, tIsotropicLinearElasticMaterial);
}
// function append_isotropic_linear_electroelastic_material_to_plato_problem

void append_orthotropic_linear_elastic_material_to_plato_problem
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    auto tMaterialModel = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Material Model"}, tMaterialModel);
    auto tIsotropicLinearElasticMaterial = tMaterialModel.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Orthotropic Linear Elastic"}, tIsotropicLinearElasticMaterial);
    XMLGen::Private::append_material_properties_to_plato_analyze_material_model(aMaterial, tIsotropicLinearElasticMaterial);
}
// function append_orthotropic_linear_elastic_material_to_plato_problem

}
// namespace Private

}
// namespace XMLGen

namespace XMLGen
{

AppendMaterialModelParameters::AppendMaterialModelParameters()
{
    this->insert();
}

void AppendMaterialModelParameters::insert()
{
    // orthotropic linear elastic material
    auto tFuncIndex = std::type_index(typeid(XMLGen::Private::append_orthotropic_linear_elastic_material_to_plato_problem));
    mMap.insert(std::make_pair("orthotropic linear elastic",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_orthotropic_linear_elastic_material_to_plato_problem, tFuncIndex)));

    // isotropic linear electroelastic material
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_isotropic_linear_electroelastic_material_to_plato_problem));
    mMap.insert(std::make_pair("isotropic linear electroelastic",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_isotropic_linear_electroelastic_material_to_plato_problem, tFuncIndex)));

    // isotropic linear thermoelastic material
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_isotropic_linear_thermoelastic_material_to_plato_problem));
    mMap.insert(std::make_pair("isotropic linear thermoelastic",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_isotropic_linear_thermoelastic_material_to_plato_problem, tFuncIndex)));

    // isotropic linear thermal material
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_isotropic_linear_thermal_material_to_plato_problem));
    mMap.insert(std::make_pair("isotropic linear thermal",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_isotropic_linear_thermal_material_to_plato_problem, tFuncIndex)));

    // isotropic linear elastic material
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_isotropic_linear_elastic_material_to_plato_problem));
    mMap.insert(std::make_pair("isotropic linear elastic",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_isotropic_linear_elastic_material_to_plato_problem, tFuncIndex)));
}

void AppendMaterialModelParameters::call(const XMLGen::Material& aMaterial, pugi::xml_node &aParentNode) const
{
    auto tLowerPerformer = Plato::tolower(aMaterial.code());
    if(Plato::tolower(tLowerPerformer).compare("plato_analyze") != 0)
    {
        return;
    }

    auto tCategory = Plato::tolower(aMaterial.category());
    auto tMapItr = mMap.find(tCategory);
    if(tMapItr == mMap.end())
    {
        THROWERR(std::string("Material Model Function Interface: Did not find material model function with tag '") + tCategory
            + "' in list. " + "Material model is not supported in Plato Analyze.")
    }
    auto tTypeCastedFunc = reinterpret_cast<void(*)(const XMLGen::Material&, pugi::xml_node&)>(tMapItr->second.first);
    if(tMapItr->second.second == std::type_index(typeid(tTypeCastedFunc)))
    {
        THROWERR(std::string("Material Model Function Interface: Reinterpret cast for material function with tag '") + tCategory + "' failed.")
    }
    tTypeCastedFunc(aMaterial, aParentNode);
}

}
// namespace XMLGen
