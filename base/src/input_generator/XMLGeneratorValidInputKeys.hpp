/*
 * XMLGeneratorValidInputKeys.hpp
 *
 *  Created on: May 29, 2020
 */

#pragma once


#include <vector>
#include <string>
#include <unordered_map>

#include "XMLG_Macros.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

struct ValidBoolKeys
{
    /*!<
     * \brief Valid plato xml generator parser boolean keys.
     **/
    std::vector<std::string> mKeys = {"true", "false"};
};
// struct ValidCriterionKeys

struct ValidPlatoInputFileMetaDataBlockKeys
{
    /*!<
     * \brief plato input file metadata block names
     **/
    std::vector<std::string> mKeys =
        {"scenario", "objective", "constraint", "material", "block", "uncertainty", "mesh", "output"};
};
// struct ValidPlatoInputFileMetaDataBlockKeys

struct ValidCriterionKeys
{
    /*!<
     * \brief Valid plato input deck criterion keywords.
     **/
    std::vector<std::string> mKeys =
        {"maximize total work", "stress constrained mass minimization", "maximize stiffness", "compliance",
         "volume", "surface area", "minimize thermoelastic energy", "thermoelastic energy", "maximize heat conduction",
         "stress p-norm", "flux p-norm", "effective energy", "minimize effective energy", "minimize stress",
         "minimize flux", "electroelastic energy", "minimize electroelastic energy", "thermal energy"};
};
// struct ValidCriterionKeys

struct ValidRandomCategoryKeys
{
    /*!<
     * \brief Valid plato input deck random categories keywords.
     **/
    std::vector<std::string> mKeys = {"load", "material"};
};
// struct ValidRandomCategoryKeys

struct ValidRandomPropertyKeys
{
    /*!<
     * \brief Valid plato input deck random properties keywords.
     **/
    std::vector<std::string> mKeys = { "angle variation", "youngs_modulus", "poissons_ratio", "mass_density", "youngs_modulus_x", "youngs_modulus_y",
        "youngs_modulus_z", "poissons_ratio_xy", "poissons_ratio_xz", "poissons_ratio_yz", "shear_modulus_xy", "shear_modulus_xz",
        "shear_modulus_yz", "dielectric_permittivity_11", "dielectric_permittivity_33", "piezoelectric_coupling_15", "piezoelectric_coupling_33",
        "piezoelectric_coupling_31", "thermal_conductivity", "specific_heat", "reference_temperature", "thermal_expansivity",
        "yield_stress" };
};
// struct ValidRandomPropertyKeys

struct ValidRandomAttributeKeys
{
    /*!<
     * \brief Valid plato input deck random attributes keywords.
     **/
    std::vector<std::string> mKeys = {"x", "y", "z", "homogeneous"};
};
// struct ValidRandomAttributeKeys

struct ValidRandomIdentificationKeys
{
    /*!<
     * \brief Valid plato input deck identification keywords.
     * Map from random 'category' keyword to identification keyword, i.e. map<category_key, identification_key>.
     **/
    std::unordered_map<std::string, std::string> mKeys = { { "load", "load id" }, { "material", "material id" } };
};
// struct ValidStatisticalDistributionKeys

struct ValidStatisticalDistributionKeys
{
    /*!<
     * \brief Valid plato input deck statistical distribution keywords.
     **/
    std::vector<std::string> mKeys = {"normal", "beta", "uniform"};
};
// struct ValidStatisticalDistributionKeys

struct ValidEssentialBoundaryConditionsKeys
{
    /*!<
     * \brief Valid plato input deck essential boundary condition keywords.
     **/
    std::vector<std::string> mKeys = {"rigid", "fixed", "zero value", "fixed value", "insulated"};
};
// struct ValidCriterionKeys

struct ValidOutputKeys
{
    /*!<
     * \brief Valid plato main output keywords. \n
     *  Map from output keyword to data layout, i.e. map<output_key,data_layout>.
     **/
    std::unordered_map<std::string, std::string> mKeys =
        { {"vonmises","element field"}, {"dispx","nodal field"}, {"dispy","nodal field"}, {"dispz","nodal field"},
          {"temperature","nodal field"}, {"accumulated_plastic_strain","element field"}, {"potential","nodal field"},
          {"objective_gradient","nodal field"}, {"constraint_gradient","nodal field"}, {"topology","nodal field"},
          {"control","nodal field"} };
};
// struct ValidOutputKeys

struct ValidPhysicsKeys
{
    /*!<
     * \brief Valid plato input deck physics keywords.
     **/
    std::vector<std::string> mKeys =
        { "mechanical", "transient mechanics", "plasticity", "stabilized mechanical", "thermal", "heat conduction",
            "electromechanical", "stabilized thermomechanical", "thermomechanical", "coupled heat conduction and mechanics" };
};
// struct ValidPhysicsKeys

struct ValidMaterialPropertyKeys
{
    /*!<
     * \brief Valid plato input deck material property keywords \n
     **/
    std::vector<std::string> mKeys = { "youngs_modulus", "poissons_ratio", "mass_density", "youngs_modulus_x", "youngs_modulus_y", "youngs_modulus_z",
        "poissons_ratio_xy", "poissons_ratio_xz", "poissons_ratio_yz", "shear_modulus_xy", "shear_modulus_xz", "shear_modulus_yz",
        "dielectric_permittivity_11", "dielectric_permittivity_33", "piezoelectric_coupling_15", "piezoelectric_coupling_33", "piezoelectric_coupling_31",
        "thermal_conductivity", "specific_heat", "reference_temperature", "thermal_expansivity" };
};
// struct ValidMaterialPropertyKeys

struct ValidMaterialModelKeys
{
    /*!<
     * \brief Valid plato input deck material model keywords \n
     **/
    std::vector<std::string> mKeys = { "isotropic linear elastic", "orthotropic linear elastic", "isotropic linear electroelastic", "isotropic linear thermal",
        "isotropic linear thermoelastic" };
};
// struct ValidMaterialModelKeys

struct ValidConstraintTargetAbsoluteKeys
{
    /*!<
     * \brief Valid plato input deck constraint absolute target keywords.
     **/
    std::vector<std::string> mKeys = {"volume absolute", "target absolute", "surface_area"};
};
// struct ValidConstraintTargetAbsoluteKeys

struct ValidConstraintTargetNormalizedKeys
{
    /*!<
     * \brief Valid plato deck constraint normalized target keywords.
     **/
    std::vector<std::string> mKeys = {"volume fraction", "target normalized"};
};
// struct ValidConstraintTargetNormalizedKeys

struct ValidAxesKeys
{
    /*!< map from dimension to axis, i.e. map<dimension, axis> */
    std::unordered_map<size_t, std::string> mKeys = { {0, "x"}, {1, "y"}, {2, "z"} };
};
// struct ValidAxesKeys

struct ValidCodeKeys
{
    /*!< valid plato input deck code keywords supported in plato */
    std::vector<std::string> mKeys = {"plato_analyze", "sierra_sd", "platomain"};
};
// struct ValidCodeKeys

struct ValidDiscretizationKeys
{
    /*!< valid abstract design variables supported in plato, i.e. control variables discretization */
    std::vector<std::string> mKeys = {"density", "levelset"};
};
// struct ValidLevelSetInitKeys

struct ValidLevelSetInitKeys
{
    /*!< valid level set initialization methods */
    std::vector<std::string> mKeys = {"primitives", "swiss cheese"};
};
// struct ValidLevelSetInitKeys

struct ValidLayoutKeys
{
    /*!<
     * valid operation field layouts \n
     * \brief map from light-input file key to Plato layout, i.e. map<light_input_file_key, plato_layout_key>
     **/
    std::unordered_map<std::string, std::string> mKeys =
        { {"element field", "Element Field"}, {"nodal field", "Nodal Field"}, {"global", "Global"} };
};
// struct ValidLayoutKeys

struct ValidFilterKeys
{
    /*!<
     * valid filters \n
     * \brief map from light-input file key to Plato main operation XML file key, i.e. map<light_input_file_key,plato_main_operation_file_key>
     **/
    std::unordered_map<std::string, std::string> mKeys = { {"identity", "Identity"},
        {"kernel", "Kernel"}, {"kernel then heaviside", "KernelThenHeaviside"}, {"kernel then tanh", "KernelThenTANH"} };
};
// struct ValidFilterKeys

struct ValidAnalyzeOutputKeys
{
    /*!<
     * valid plato analyze output keywords \n
     * \brief map from light-input file output key to plato analyze key, i.e. map<light_input_file_output key, plato_analyze_output_key>. \n
     * Basically, this maps connects the key use for any quantity of interest inside Plato's light input file to the key used in \n
     * Plato Analyze to identify the quantity of interest.
     **/
    std::unordered_map<std::string, std::string> mKeys = { {"vonmises", "vonmises"}, {"plastic_multiplier_increment", "plastic multiplier increment"},
      {"accumulated_plastic_strain", "accumulated plastic strain"}, {"deviatoric_stress", "deviatoric stress"}, {"elastic_strain", "elastic_strain"},
      {"plastic_strain", "plastic strain"}, {"cauchy_stress", "cauchy stress"}, {"backstress", "backstress"}, {"dispx", "Solution X"},
      {"dispy", "Solution Y"}, {"dispz", "Solution Z"}, {"principal_stresses", "principal stresses"}, {"cauchy_stress", "cauchy stress"} };
};
// struct ValidAnalyzeOutputKeys

struct ValidAnalyzeMaterialPropertyKeys
{
private:
    /*!<
     * valid plato analyze material models and corresponding material property keywords \n
     * \brief map from material model to map from material property tag in plato input file to \n
     * pair of plato analyze input file material property key and its corresponding value type, i.e. \n
     *
     * map< material_model, map<plato_input_file_material_property_tag, pair<plato_analyze_input_file_material_property_tag, value_type>>>.
     *
     **/
    std::unordered_map<std::string, std::unordered_map<std::string, std::pair<std::string, std::string>>> mKeys =
        {
            { "isotropic linear elastic", { { "youngs_modulus", {"Youngs Modulus", "double"} },
                { "poissons_ratio", {"Poissons Ratio", "double"} }, { "mass_density", {"Mass Density", "double"} } }
            },

            { "orthotropic linear elastic", { { "youngs_modulus_x", {"Youngs Modulus X", "double"} },
                { "youngs_modulus_y", {"Youngs Modulus Y", "double"} }, { "youngs_modulus_z", {"Youngs Modulus Z", "double"} },
                { "poissons_ratio_xy", {"Poissons Ratio XY", "double"} }, { "poissons_ratio_xz", {"Poissons Ratio XZ", "double"} },
                { "poissons_ratio_yz", {"Poissons Ratio YZ", "double"} }, { "shear_modulus_xy", {"Shear Modulus XY", "double"} },
                { "shear_modulus_xz", {"Shear Modulus XZ", "double"} }, { "shear_modulus_yz", {"Shear Modulus YZ", "double"} },
                { "mass_density", {"Mass Density", "double"} } }
            },

            { "isotropic linear electroelastic", { { "youngs_modulus", {"Youngs Modulus", "double"} },
                { "poissons_ratio", {"Poissons Ratio", "double"} }, { "dielectric_permittivity_11", {"p11", "double"} },
                { "dielectric_permittivity_33", {"p33", "double"} }, { "piezoelectric_coupling_15", {"e15", "double"} },
                { "piezoelectric_coupling_33", {"e33", "double"} }, { "piezoelectric_coupling_31", {"e31", "double"} },
                { "thermal_expansivity", {"Alpha", "double"} } }
            },

            { "isotropic linear thermal", { { "thermal_conductivity", { "Thermal Conductivity", "double" } },
                { "mass_density", {"Mass Density", "double"} }, { "specific_heat", {"Specific Heat", "double"} } }
            },

            { "isotropic linear thermoelastic", { { "thermal_conductivity", { "Thermal Conductivity", "double" } },
                { "youngs_modulus", {"Youngs Modulus", "double"} }, { "poissons_ratio", {"Poissons Ratio", "double"} },
                { "thermal_expansivity", { "Thermal Expansivity", "double" } }, { "reference_temperature", { "Reference Temperature", "double" } },
                { "mass_density", {"Mass Density", "double"} } }
            },

            { "j2 plasticity",
              {
                { "youngs_modulus", {"Youngs Modulus", "double"} },
                { "poissons_ratio", {"Poissons Ratio", "double"} },
                { "pressure_scaling", { "Pressure Scaling", "double" } },
                { "hardening_modulus_isotropic", { "Hardening Modulus Isotropic", "double" } },
                { "hardening_modulus_kinematic", { "Hardening Modulus Kinematic", "double" } },
                { "initial_yield_stress", {"Initial Yield Stress", "double"} },
                { "elastic_properties_penalty_exponent", {"Elastic Properties Penalty Exponent", "double"} },
                { "elastic_properties_minimum_ersatz", {"Elastic Properties Minimum Ersatz", "double"} },
                { "plastic_properties_penalty_exponent", {"Plastic Properties Penalty Exponent", "double"} },
                { "plastic_properties_minimum_ersatz", {"Plastic Properties Minimum Ersatz", "double"} }
              }
            }
        };

public:
    /******************************************************************************//**
     * \fn tag
     * \brief Return valid Plato Analyze material property tag.
     * \param [in] aMaterialModelTag material model tag
     * \param [in] aPropertyTag      material property tag
     * \return material property tag recognized by Plato Analyze
    **********************************************************************************/
    std::string tag(const std::string& aMaterialModelTag, const std::string& aPropertyTag) const
    {
        auto tLowerMaterialModelTag = XMLGen::to_lower(aMaterialModelTag);
        auto tMaterialModelItr = mKeys.find(tLowerMaterialModelTag);
        if (tMaterialModelItr == mKeys.end())
        {
            THROWERR("Valid Analyze Material Property Keys: Material model '" + tLowerMaterialModelTag + "' is not supported in Plato Analyze.")
        }

        auto tLowerPropertyTag = XMLGen::to_lower(aPropertyTag);
        auto tItr = tMaterialModelItr->second.find(tLowerPropertyTag);
        if (tItr == tMaterialModelItr->second.end())
        {
            THROWERR(std::string("Append Material Properties To Plato Analyze Material Model: Material property with tag '")
                + tLowerPropertyTag + "' is not supported in Plato Analyze by '" + tLowerMaterialModelTag + "' material model.")
        }

        return tItr->second.first;
    }

    /******************************************************************************//**
     * \fn type
     * \brief Return material property data type, e.g. double, int, etc.
     * \param [in] aMaterialModelTag material model tag
     * \param [in] aPropertyTag      material property tag
     * \return material property data type
    **********************************************************************************/
    std::string type(const std::string& aMaterialModelTag, const std::string& aPropertyTag) const
    {
        auto tLowerMaterialModelTag = XMLGen::to_lower(aMaterialModelTag);
        auto tMaterialModelItr = mKeys.find(tLowerMaterialModelTag);
        if (tMaterialModelItr == mKeys.end())
        {
            THROWERR("Valid Analyze Material Property Keys: Material model '" + tLowerMaterialModelTag + "' is not supported in Plato Analyze.")
        }

        auto tLowerPropertyTag = XMLGen::to_lower(aPropertyTag);
        auto tItr = tMaterialModelItr->second.find(tLowerPropertyTag);
        if (tItr == tMaterialModelItr->second.end())
        {
            THROWERR(std::string("Append Material Properties To Plato Analyze Material Model: Material property with tag '")
                + tLowerPropertyTag + "' is not supported in Plato Analyze by '" + tLowerMaterialModelTag + "' material model.")
        }

        return tItr->second.second;
    }

    /******************************************************************************//**
     * \fn pair
     * \brief Return material property tag to material property data type pair, e.g. \n
     *     std::pair<material property tag, material property data>
     * \param [in] aMaterialModelTag material model tag
     * \param [in] aPropertyTag      material property tag
     * \return material property tag to material property data type pair
    **********************************************************************************/
    std::pair<std::string, std::string> pair(const std::string& aMaterialModelTag, const std::string& aPropertyTag) const
    {
        auto tLowerMaterialModelTag = XMLGen::to_lower(aMaterialModelTag);
        auto tMaterialModelItr = mKeys.find(tLowerMaterialModelTag);
        if (tMaterialModelItr == mKeys.end())
        {
            THROWERR("Valid Analyze Material Property Keys: Material model '" + tLowerMaterialModelTag + "' is not supported in Plato Analyze.")
        }

        auto tLowerPropertyTag = XMLGen::to_lower(aPropertyTag);
        auto tItr = tMaterialModelItr->second.find(tLowerPropertyTag);
        if (tItr == tMaterialModelItr->second.end())
        {
            THROWERR(std::string("Append Material Properties To Plato Analyze Material Model: Material property with tag '")
                + tLowerPropertyTag + "' is not supported in Plato Analyze by '" + tLowerMaterialModelTag + "' material model.")
        }

        return tItr->second;
    }
};
// struct ValidAnalyzeMaterialPropertyKeys

struct ValidAnalyzePhysicsKeys
{
private:
    /*!<
     * valid plato analyze physics and corresponding PDE keywords \n
     * \brief map from physics keyword to pair of plato analyze physics and \n
     * partial differential equations (PDE) keywords, i.e. \n
     *
     * map< plato_main_physics_key, pair<plato_analyze_physics_key, plato_analyze_pde_key> >.
     *
     **/
    std::unordered_map<std::string, std::pair<std::string,std::string>> mKeys =
        {
            { "mechanical", {"Mechanical", "Elliptic"} },
            { "transient mechanics", {"Mechanical", "Hyperbolic"} },
            { "transient thermomechanics", {"Thermomechanical", "Parabolic"} },
            { "infinitesimal strain plasticity", {"Mechanical", "Infinitesimal Strain Plasticity"} },
            { "stabilized mechanical", {"Stabilized Mechanical", "Elliptic"} },
            { "thermal", {"Thermal", "Elliptic"} },
            { "heat conduction", {"Thermal", "Parabolic"} },
            { "electromechanical", {"Electromechanical", "Elliptic"} },
            { "stabilized thermomechanical", {"Stabilized Thermomechanical", "Elliptic"} },
            { "thermomechanical", {"Thermomechanical", "Elliptic"} },
            { "coupled heat conduction and mechanics", {"Thermomechanical", "Parabolic"} }
        };

public:
    std::string pde(const std::string& aPhysicsTag) const
    {
        auto tLowerPhysics = XMLGen::to_lower(aPhysicsTag);
        auto tPhysicsItr = mKeys.find(tLowerPhysics);
        if (tPhysicsItr == mKeys.end())
        {
            THROWERR(std::string("Valid Analyze Physics Keys: Physics '") + tLowerPhysics + "' is not supported in Plato Analyze.")
        }
        return (tPhysicsItr->second.second);
    }

    std::string physics(const std::string& aPhysicsTag) const
    {
        auto tLowerPhysics = XMLGen::to_lower(aPhysicsTag);
        auto tPhysicsItr = mKeys.find(tLowerPhysics);
        if (tPhysicsItr == mKeys.end())
        {
            THROWERR(std::string("Valid Analyze Physics Keys: Physics '") + tLowerPhysics + "' is not supported in Plato Analyze.")
        }
        return (tPhysicsItr->second.first);
    }
};
// struct ValidAnalyzePhysicsKeys

struct ValidAnalyzeCriteriaKeys
{
    /*!<
     * valid plato analyze optimization criteria \n
     * \brief map from plato main criterion key to pair of plato analyze criterion \n
     * key and self-adjoint flag, i.e. \n
     *
     * map< plato_main_cirterion_key, pair<plato_analyze_criterion_key, plato_analyze_self_adjoint_key> >.
     *
     **/
    std::unordered_map<std::string, std::pair<std::string, bool>> mKeys =
    {
        { "volume", { "Volume", false } },
        { "elastic work", { "Elastic Work", true } },
        { "plastic work", { "Plastic Work", false } },
        { "compliance", { "Internal Elastic Energy", true } },
        { "maximize stiffness", { "Internal Elastic Energy", true } },
        { "local stress", { "Stress Constraint Quadratic", false } },
        { "stress p-norm", { "Stress P-Norm", false } },
        { "minimize stress", { "Stress P-Norm", false } },
        { "effective energy", { "Effective Energy", true } },
        { "minimize effective energy", { "Effective Energy", true } },
        { "stress constrained mass minimization", { "Stress Constraint General", false } },
        { "thermal energy", { "Internal Thermal Energy", false } },
        { "maximize heat conduction", { "Internal Thermal Energy", false } },
        { "flux p-norm", { "Flux P-Norm", false } },
        { "minimize flux", { "Flux P-Norm", false } },
        { "thermoelastic energy", { "Internal Thermoelastic Energy", false } },
        { "minimize thermoelastic energy", { "Internal Thermoelastic Energy", false } },
        { "electroelastic energy", { "Internal Electroelastic Energy", false } },
        { "minimize electroelastic energy", { "Internal Electroelastic Energy", false } }
    };
};
// ValidAnalyzeCriteriaKeys

struct ValidSpatialDimsKeys
{
    /*!<
     * \brief Valid plato problem spatial dimensions.
     **/
    std::vector<std::string> mKeys = { "3", "2" };
};
// struct ValidSpatialDimsKeys

struct ValidDofsKeys
{
    /*!< map from physics to map from degree of freedom name to degree of freedom index, i.e. \n
     *
     * map<physics, map<dof_name, dof_index>>
     * */
    std::unordered_map<std::string, std::unordered_map<std::string,std::string>> mKeys =
        {
            {"mechanical", { {"dispx", "0"}, {"dispy", "1"}, {"dispz", "2"} } },
            {"thermal", { {"temp", "0"} } }, {"electrostatics", { {"potential", "0"} } },
            {"thermalmechanics", { {"dispx", "0"}, {"dispy", "1"}, {"dispz", "2"}, {"temp", "3"} } },
            {"electromechanics", { {"dispx", "0"}, {"dispy", "1"}, {"dispz", "2"}, {"potential", "3"} } }
        };
};
// struct ValidDofsKeys

}
// namespace XMLGen
