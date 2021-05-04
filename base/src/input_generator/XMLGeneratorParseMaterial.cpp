/*
 * XMLGeneratorParseMaterial.cpp
 *
 *  Created on: Jun 23, 2020
 */

#include <algorithm>

#include "XMLGeneratorParseMaterial.hpp"
#include "XMLGeneratorValidInputKeys.hpp"

namespace XMLGen
{

std::string check_material_model_key
(const std::string& aKeyword)
{
    XMLGen::ValidMaterialModelKeys tValidKeys;
    auto tValue = tValidKeys.value(aKeyword);
    if(tValue.empty())
    {
        THROWERR(std::string("Check Material Model Key: Material model keyword '") + aKeyword + "' is not supported.")
    }
    return tValue;
}

void ParseMaterial::insertCoreProperties()
{
    mTags.insert({ "id", { { {"id"}, ""}, "" } });
    mTags.insert({ "code", { { {"code"}, ""}, "" } });
    mTags.insert({ "name", { { {"name"}, ""}, "" } });
    mTags.insert({ "attribute", { { {"attribute"}, ""}, "" } });
    mTags.insert({ "material_model", { { {"material_model"}, ""}, "" } });
    mTags.insert({ "penalty_exponent", { { {"penalty_exponent"}, ""}, "" } });
}

void ParseMaterial::insertElasticProperties()
{
    mTags.insert({ "mass_density", { { {"mass_density"}, ""}, "" } });
    mTags.insert({ "youngs_modulus", { { {"youngs_modulus"}, ""}, "" } });
    mTags.insert({ "poissons_ratio", { { {"poissons_ratio"}, ""}, "" } });
}

void ParseMaterial::insertElasticOrthoProperties()
{
    mTags.insert({ "youngs_modulus_x", { { {"youngs_modulus_x"}, ""}, "" } });
    mTags.insert({ "youngs_modulus_y", { { {"youngs_modulus_y"}, ""}, "" } });
    mTags.insert({ "youngs_modulus_z", { { {"youngs_modulus_z"}, ""}, "" } });
    mTags.insert({ "shear_modulus_xy", { { {"shear_modulus_xy"}, ""}, "" } });
    mTags.insert({ "shear_modulus_xz", { { {"shear_modulus_xz"}, ""}, "" } });
    mTags.insert({ "shear_modulus_yz", { { {"shear_modulus_yz"}, ""}, "" } });
    mTags.insert({ "poissons_ratio_xy", { { {"poissons_ratio_xy"}, ""}, "" } });
    mTags.insert({ "poissons_ratio_xz", { { {"poissons_ratio_xz"}, ""}, "" } });
    mTags.insert({ "poissons_ratio_yz", { { {"poissons_ratio_yz"}, ""}, "" } });
}

void ParseMaterial::insertElectricProperties()
{
    mTags.insert({ "piezoelectric_coupling_15", { { {"piezoelectric_coupling_15"}, ""}, "" } });
    mTags.insert({ "piezoelectric_coupling_33", { { {"piezoelectric_coupling_33"}, ""}, "" } });
    mTags.insert({ "piezoelectric_coupling_31", { { {"piezoelectric_coupling_31"}, ""}, "" } });
    mTags.insert({ "dielectric_permittivity_11", { { {"dielectric_permittivity_11"}, ""}, "" } });
    mTags.insert({ "dielectric_permittivity_33", { { {"dielectric_permittivity_33"}, ""}, "" } });
}

void ParseMaterial::insertThermalProperties()
{
    mTags.insert({ "specific_heat", { { {"specific_heat"}, ""}, "" } });
    mTags.insert({ "thermal_expansivity", { { {"thermal_expansivity"}, ""}, "" } });
    mTags.insert({ "thermal_conductivity", { { {"thermal_conductivity"}, ""}, "" } });
    mTags.insert({ "reference_temperature", { { {"reference_temperature"}, ""}, "" } });
}

void ParseMaterial::insertPlasticProperties()
{
    mTags.insert({ "initial_yield_stress", { { {"initial_yield_stress"}, ""}, "" } });
    mTags.insert({ "hardening_modulus_isotropic", { { {"hardening_modulus_isotropic"}, ""}, "" } });
    mTags.insert({ "hardening_modulus_kinematic", { { {"hardening_modulus_kinematic"}, ""}, "" } });
    mTags.insert({ "elastic_properties_penalty_exponent", { { {"elastic_properties_penalty_exponent"}, ""}, "" } });
    mTags.insert({ "elastic_properties_minimum_ersatz", { { {"elastic_properties_minimum_ersatz"}, ""}, "" } });
    mTags.insert({ "plastic_properties_penalty_exponent", { { {"plastic_properties_penalty_exponent"}, ""}, "" } });
    mTags.insert({ "plastic_properties_minimum_ersatz", { { {"plastic_properties_minimum_ersatz"}, ""}, "" } });
}

void ParseMaterial::allocate()
{
    mTags.clear();

    this->insertCoreProperties();
    this->insertElasticProperties();
    this->insertThermalProperties();
    this->insertPlasticProperties();
    this->insertElectricProperties();
    this->insertElasticOrthoProperties();
}

void ParseMaterial::setCode(XMLGen::Material& aMetadata)
{
    auto tItr = mTags.find("code");
    if (tItr->second.first.second.empty())
    {
        auto tValidCode = XMLGen::check_code_keyword("plato_analyze");
        aMetadata.code(tValidCode);
    }
    else
    {
        auto tValidCode = XMLGen::check_code_keyword(tItr->second.first.second);
        aMetadata.code(tValidCode);
    }
}

void ParseMaterial::setName(XMLGen::Material& aMetadata)
{
    auto tItr = mTags.find("name");
    aMetadata.name(tItr->second.first.second);
}

void ParseMaterial::setMaterialModel(XMLGen::Material& aMetadata)
{
    auto tItr = mTags.find("material_model");
    if (tItr->second.first.second.empty())
    {
        THROWERR("Parse Material: material model is not defined. A unique material model must be defined.")
    }
    else
    {
        auto tValidMatModel = XMLGen::check_material_model_key(tItr->second.first.second);
        aMetadata.materialModel(tValidMatModel);
    }
}

void ParseMaterial::setMaterialProperties(XMLGen::Material& aMetadata)
{
    XMLGen::ValidMaterialPropertyKeys tValidKeys;
    auto tMaterialModel = aMetadata.materialModel();
    auto tPropertyTags = tValidKeys.properties(tMaterialModel);
    for(auto& tKeyword : tPropertyTags)
    {
        auto tItr = mTags.find(tKeyword);
        if(tItr == mTags.end())
        {
            THROWERR(std::string("Parse Material: Material property keyword '") + tKeyword + "' is not a valid keyword.")
        }

        if(!tItr->second.first.second.empty())
        {
            aMetadata.property(tKeyword, tItr->second.first.second);
        }
    }
    this->checkMaterialProperties(aMetadata);
}

void ParseMaterial::setMaterialIdentification(XMLGen::Material& aMetadata)
{
    if(aMetadata.id().empty())
    {
        auto tItr = mTags.find("id");
        if(tItr->second.first.second.empty())
        {
            THROWERR(std::string("Parse Material: material identification number is empty. ")
                + "A unique material identification number must be assigned to a material block.")
        }
        aMetadata.id(tItr->second.first.second);
    }
}

void ParseMaterial::setPenaltyExponent(XMLGen::Material& aMetadata)
{
    auto tItr = mTags.find("penalty_exponent");
    if (tItr->second.first.second.empty())
    {
        if(aMetadata.code().compare("plato_analyze") != 0)
        {
            aMetadata.property("penalty_exponent", "3.0");
        }
    }
    else
    {
        aMetadata.property("penalty_exponent", tItr->second.first.second);
    }
}

void ParseMaterial::setMetadata(XMLGen::Material& aMetadata)
{
    this->setCode(aMetadata);
    this->setName(aMetadata);
    this->setMaterialModel(aMetadata);
    this->setPenaltyExponent(aMetadata);
    this->setMaterialProperties(aMetadata);
    this->setMaterialIdentification(aMetadata);
}

void ParseMaterial::checkUniqueIDs()
{
    std::vector<std::string> tIDs;
    for(auto& tMaterial : mData)
    {
        tIDs.push_back(tMaterial.id());
    }

    if(!XMLGen::unique(tIDs))
    {
        THROWERR("Parse Material: Material block identification numbers, i.e. IDs, are not unique.  Material block IDs must be unique.")
    }
}

void ParseMaterial::checkNames()
{
    for(auto& tMaterial : mData)
    {
        if (tMaterial.name().empty())
        {
            auto tName = "material_" + tMaterial.id();
            tMaterial.name(tName);
        }
    }

    std::vector<std::string> tNames;
    for(auto& tMaterial : mData)
    {
        tNames.push_back(tMaterial.name());
    }

    if(!XMLGen::unique(tNames))
    {
        THROWERR("Parse Material: Material block names are not unique.  Material block names must be unique.")
    }
}

void ParseMaterial::checkMaterialProperties(XMLGen::Material& aMetadata)
{
    if(aMetadata.tags().empty())
    {
        auto tID = aMetadata.id().empty() ? std::string("UNDEFINED") : aMetadata.id();
        THROWERR("Parse Material: Material properties for material block with identification number '" + tID + "' are empty.")
    }
}

std::vector<XMLGen::Material> ParseMaterial::data() const
{
    return mData;
}

void ParseMaterial::parse(std::istream &aInputFile)
{
    mData.clear();
    this->allocate();
    constexpr int MAX_CHARS_PER_LINE = 10000;
    std::vector<char> tBuffer(MAX_CHARS_PER_LINE);
    while (!aInputFile.eof())
    {
        // read an entire line into memory
        std::vector<std::string> tTokens;
        aInputFile.getline(tBuffer.data(), MAX_CHARS_PER_LINE);
        XMLGen::parse_tokens(tBuffer.data(), tTokens);
        XMLGen::to_lower(tTokens);

        std::string tMaterialBlockID;
        if (XMLGen::parse_single_value(tTokens, { "begin", "material" }, tMaterialBlockID))
        {
            XMLGen::Material tMetadata;
            tMetadata.id(tMaterialBlockID);
            XMLGen::erase_tag_values(mTags);
            XMLGen::parse_input_metadata( { "end", "material" }, aInputFile, mTags);
            this->setMetadata(tMetadata);
            mData.push_back(tMetadata);
        }
    }
    this->checkUniqueIDs();
    this->checkNames();
}

}
// namespace XMLGen
