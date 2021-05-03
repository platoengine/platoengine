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
    auto tMaterialModelTag = aMaterial.materialModel();
    XMLGen::ValidMaterialPropertyKeys tValidKeys;
    auto tValueType = tValidKeys.type(tMaterialModelTag, aMaterialPropertyTag);
    auto tAnalyzeMatPropertyTag = tValidKeys.tag(tMaterialModelTag, aMaterialPropertyTag);
    std::vector<std::string> tKeys = {"name", "type", "value"};
    auto tMaterialPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMaterial.value(aMaterialPropertyTag));
    std::vector<std::string> tValues = {tAnalyzeMatPropertyTag, tValueType, tMaterialPropertyValue};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}
// function append_material_property

void append_material_properties_to_plato_analyze_material_model
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    XMLGen::Private::is_material_property_tags_container_empty(aMaterial);

    auto tTags = aMaterial.tags();
    auto tMaterialModelTag = aMaterial.materialModel();
    XMLGen::ValidMaterialPropertyKeys tValidKeys;
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
    auto tElasticModel = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {aMaterial.name()}, tElasticModel);
    auto tIsotropicLinearElasticModel = tElasticModel.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Isotropic Linear Elastic"}, tIsotropicLinearElasticModel);
    XMLGen::Private::append_material_property("poissons_ratio", aMaterial, tIsotropicLinearElasticModel);
    XMLGen::Private::append_material_property("youngs_modulus", aMaterial, tIsotropicLinearElasticModel);
}
// function append_isotropic_linear_elastic_material_to_plato_problem

void append_isotropic_linear_thermoelastic_material_to_thermoplasticity_in_plato_problem
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    auto tElasticModel = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {aMaterial.name()}, tElasticModel);
    auto tIsotropicLinearThermoelasticModel = tElasticModel.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Isotropic Linear Thermoelastic"}, tIsotropicLinearThermoelasticModel);
    XMLGen::Private::append_material_property("poissons_ratio", aMaterial, tIsotropicLinearThermoelasticModel);
    XMLGen::Private::append_material_property("youngs_modulus", aMaterial, tIsotropicLinearThermoelasticModel);
    XMLGen::Private::append_material_property("thermal_expansivity", aMaterial, tIsotropicLinearThermoelasticModel);
    XMLGen::Private::append_material_property("thermal_conductivity", aMaterial, tIsotropicLinearThermoelasticModel);
    XMLGen::Private::append_material_property("reference_temperature", aMaterial, tIsotropicLinearThermoelasticModel);
}
// function append_isotropic_linear_elastic_material_to_plato_problem

void append_isotropic_linear_thermal_material_to_plato_problem
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    auto tMaterialModel = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {aMaterial.name()}, tMaterialModel);

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
    XMLGen::append_attributes({"name"}, {aMaterial.name()}, tMaterialModel);

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
    XMLGen::append_attributes({"name"}, {aMaterial.name()}, tMaterialModel);
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
    XMLGen::append_attributes({"name"}, {aMaterial.name()}, tMaterialModel);
    auto tIsotropicLinearElasticMaterial = tMaterialModel.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Orthotropic Linear Elastic"}, tIsotropicLinearElasticMaterial);
    XMLGen::Private::append_material_properties_to_plato_analyze_material_model(aMaterial, tIsotropicLinearElasticMaterial);
}
// function append_orthotropic_linear_elastic_material_to_plato_problem

void append_j2_plasticity_material_properties
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    auto tMaterialModel = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Plasticity Model"}, tMaterialModel);
    auto tJ2PlasticityModel = tMaterialModel.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"J2 Plasticity"}, tJ2PlasticityModel);

    XMLGen::Private::append_material_property("initial_yield_stress", aMaterial, tJ2PlasticityModel);
    XMLGen::Private::append_material_property("hardening_modulus_isotropic", aMaterial, tJ2PlasticityModel);
    XMLGen::Private::append_material_property("hardening_modulus_kinematic", aMaterial, tJ2PlasticityModel);
    XMLGen::Private::append_material_property("elastic_properties_minimum_ersatz", aMaterial, tJ2PlasticityModel);
    XMLGen::Private::append_material_property("elastic_properties_penalty_exponent", aMaterial, tJ2PlasticityModel);
    XMLGen::Private::append_material_property("plastic_properties_minimum_ersatz", aMaterial, tJ2PlasticityModel);
    XMLGen::Private::append_material_property("plastic_properties_penalty_exponent", aMaterial, tJ2PlasticityModel);
}
// function append_j2_plasticity_material_properties

void append_j2_plasticity_material_to_plato_problem
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    // elastic properties
    XMLGen::Private::append_isotropic_linear_elastic_material_to_plato_problem(aMaterial, aParentNode);
    auto tMaterialModel = aParentNode.child("ParameterList");
    // plastic properties
    XMLGen::Private::append_j2_plasticity_material_properties(aMaterial, tMaterialModel);
}
// function append_j2_plasticity_material_to_plato_problem

void append_thermoplasticity_material_to_plato_problem
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    // thermoelastic properties
    XMLGen::Private::append_isotropic_linear_thermoelastic_material_to_thermoplasticity_in_plato_problem(aMaterial, aParentNode);
    auto tMaterialModel = aParentNode.child("ParameterList");
    // plastic properties
    XMLGen::Private::append_j2_plasticity_material_properties(aMaterial, tMaterialModel);
}
// function append_thermoplasticity_material_to_plato_problem

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
    // orthotropic linear elastic material model
    auto tFuncIndex = std::type_index(typeid(XMLGen::Private::append_orthotropic_linear_elastic_material_to_plato_problem));
    mMap.insert(std::make_pair("orthotropic_linear_elastic",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_orthotropic_linear_elastic_material_to_plato_problem, tFuncIndex)));

    // isotropic linear electroelastic material model
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_isotropic_linear_electroelastic_material_to_plato_problem));
    mMap.insert(std::make_pair("isotropic_linear_electroelastic",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_isotropic_linear_electroelastic_material_to_plato_problem, tFuncIndex)));

    // isotropic linear thermoelastic material model
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_isotropic_linear_thermoelastic_material_to_plato_problem));
    mMap.insert(std::make_pair("isotropic_linear_thermoelastic",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_isotropic_linear_thermoelastic_material_to_plato_problem, tFuncIndex)));

    // isotropic linear thermal material model
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_isotropic_linear_thermal_material_to_plato_problem));
    mMap.insert(std::make_pair("isotropic_linear_thermal",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_isotropic_linear_thermal_material_to_plato_problem, tFuncIndex)));

    // isotropic linear elastic material model
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_isotropic_linear_elastic_material_to_plato_problem));
    mMap.insert(std::make_pair("isotropic_linear_elastic",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_isotropic_linear_elastic_material_to_plato_problem, tFuncIndex)));

    // j2 plasticity material model
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_j2_plasticity_material_to_plato_problem));
    mMap.insert(std::make_pair("j2_plasticity",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_j2_plasticity_material_to_plato_problem, tFuncIndex)));
    
    // thermoplasticity material model
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_thermoplasticity_material_to_plato_problem));
    mMap.insert(std::make_pair("thermoplasticity",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_thermoplasticity_material_to_plato_problem, tFuncIndex)));
}

void AppendMaterialModelParameters::call(const XMLGen::Material& aMaterial, pugi::xml_node &aParentNode) const
{
    auto tLowerPerformer = Plato::tolower(aMaterial.code());
    if(Plato::tolower(tLowerPerformer).compare("plato_analyze") != 0)
    {
        return;
    }

    aMaterial.empty();

    auto tCategory = Plato::tolower(aMaterial.materialModel());
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
