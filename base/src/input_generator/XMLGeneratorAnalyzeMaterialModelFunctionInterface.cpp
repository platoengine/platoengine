/*
 * XMLGeneratorAnalyzeMaterialModelFunctionInterface.cpp
 *
 *  Created on: Jun 15, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorPlatoAnalyzeInputFileUtilities.hpp"
#include "XMLGeneratorAnalyzeMaterialModelFunctionInterface.hpp"

namespace XMLGen
{

namespace Private
{

void check_material_property_tags_container
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

void append_material_properties_to_plato_analyze_material_model
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    XMLGen::Private::check_material_property_tags_container(aMaterial);
    XMLGen::ValidAnalyzeMaterialPropertyKeys tValidKeys;
    auto tLowerMaterialModel = Plato::tolower(aMaterial.category());
    auto tMaterialModelItr = tValidKeys.mKeys.find(tLowerMaterialModel);
    if(tMaterialModelItr == tValidKeys.mKeys.end())
    {
        THROWERR("Append Material Properties To Plato Analyze Material Model: Material model '"
            + tLowerMaterialModel + "' is not supported in Plato Analyze.")
    }

    auto tTags = aMaterial.tags();
    std::vector<std::string> tKeys = {"name", "type", "value"};
    for(auto& tTag : tTags)
    {
        auto tLowerTag = Plato::tolower(tTag);
        auto tItr = tMaterialModelItr->second.find(tLowerTag);
        if (tItr == tMaterialModelItr->second.end())
        {
            THROWERR(std::string("Append Material Properties To Plato Analyze Material Model: Material property with tag '")
                + tTag + "' is not supported in Plato Analyze by '" + tLowerMaterialModel + "' material model.")
        }
        auto tMaterialTag = tItr->second.first;
        auto tValueType = tItr->second.second;
        std::vector<std::string> tValues = {tMaterialTag, tValueType, aMaterial.property(tTag)};
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
    auto tIsotropicLinearElasticMaterial = tMaterialModel.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Isotropic Linear Thermal"}, tIsotropicLinearElasticMaterial);
    XMLGen::Private::append_material_properties_to_plato_analyze_material_model(aMaterial, tIsotropicLinearElasticMaterial);
}
// function append_isotropic_linear_thermal_material_to_plato_problem

void append_isotropic_linear_thermoelastic_material_to_plato_problem
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    auto tMaterialModel = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Material Model"}, tMaterialModel);
    auto tIsotropicLinearElasticMaterial = tMaterialModel.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Isotropic Linear Thermoelastic"}, tIsotropicLinearElasticMaterial);
    XMLGen::Private::append_material_properties_to_plato_analyze_material_model(aMaterial, tIsotropicLinearElasticMaterial);
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
