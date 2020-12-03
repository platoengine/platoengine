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

struct ValidKeys
{
    std::vector<std::string> mKeys;
};

struct ValidCriterionParameterKeys
{
    std::vector<std::string> mKeys = 
    {
        "type", 
        "normalize", 
        "normalization_value",
        "stress_p_norm_exponent", 
        "material_penalty_model", 
        "material_penalty_exponent", 
        "minimum_ersatz_material_value"
    };
};

struct ValidBoolKeys
{
    /*!<
     * \brief Valid plato xml generator parser boolean keys.
     **/
    std::vector<std::string> mKeys = {"true", "false"};
};
// struct ValidBoolKeys

struct ValidPlatoInputFileMetaDataBlockKeys
{
    /*!<
     * \brief plato input file metadata block names
     **/
    std::vector<std::string> mKeys =
    {
        "service", 
        "objective", 
        "constraint", 
        "material", 
        "block", 
        "uncertainty", 
        "mesh", 
        "output"
    };
};
// struct ValidPlatoInputFileMetaDataBlockKeys

struct ValidObjectiveTypeKeys
{
    /*!<
     * \brief Valid plato input deck criterion keywords.
     **/
    std::vector<std::string> mKeys = {"single_criterion", "weighted_sum"};
};
// struct ValidObjectiveTypeKeys

struct ValidCriterionTypeKeys
{
    /*!<
     * \brief Valid plato input deck criterion keywords.
     **/
    std::vector<std::string> mKeys =
    {
        "mechanical_compliance", 
        "thermal_compliance",
        "electrical_compliance",
        "thermomechanical_compliance", 
        "electromechanical_compliance", 
        "total_work", 
        "elastic_work",
        "plastic_work",
        "volume", 
        "stress_p-norm", 
        "mass", 
        "CG_x", 
        "CG_y", 
        "CG_z", 
        "Ixx", 
        "Iyy", 
        "Izz", 
        "Ixz", 
        "Iyz", 
        "Ixy", 
        "effective_energy", 
        "surface_area", 
        "flux_p-norm", 
        "stress", 
        "average_temperature", 
        "stress_constraint_quadratic",  
        "flux" 
    };
};
// struct ValidCriterionTypeKeys

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
    std::vector<std::string> mKeys = 
    { 
        "angle variation", 
        "youngs_modulus", 
        "poissons_ratio", 
        "mass_density", 
        "youngs_modulus_x", 
        "youngs_modulus_y",
        "youngs_modulus_z", 
        "poissons_ratio_xy", 
        "poissons_ratio_xz", 
        "poissons_ratio_yz", 
        "shear_modulus_xy", 
        "shear_modulus_xz",
        "shear_modulus_yz", 
        "dielectric_permittivity_11", 
        "dielectric_permittivity_33", 
        "piezoelectric_coupling_15", 
        "piezoelectric_coupling_33",
        "piezoelectric_coupling_31", 
        "thermal_conductivity", 
        "specific_heat", 
        "reference_temperature", 
        "thermal_expansivity",
        "yield_stress" 
    };
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
    std::unordered_map<std::string, std::string> mKeys = { { "load", "load_id" }, { "material", "material_id" } };
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
//
struct ValidLoadKeys
{
    /*!<
     * \brief Valid plato input deck essential boundary condition keywords.
     **/
    std::vector<std::string> mKeys = {"traction", "uniform_surface_flux", "force", "pressure"};
};
// struct ValidEssentialBoundaryConditionsKeys

struct ValidEssentialBoundaryConditionsKeys
{
    /*!<
     * \brief Valid plato input deck essential boundary condition keywords.
     **/
    std::vector<std::string> mKeys = {"rigid", "fixed", "zero_value", "fixed_value", "insulated"};
};
// struct ValidEssentialBoundaryConditionsKeys

struct ValidOutputToLayoutKeys
{
    /*!<
     * \brief Valid plato main output keywords. \n
     *  Map from output keyword to data layout, i.e. map<output_key,data_layout>.
     **/
    std::unordered_map<std::string, std::string> mKeys =
    {
        {"vonmises","element_field"}, 
        {"dispx","nodal_field"}, 
        {"dispy","nodal_field"}, 
        {"dispz","nodal_field"},
        {"temperature","nodal_field"}, 
        {"accumulated_plastic_strain","element_field"}, 
        {"potential","nodal_field"},
        {"objective_gradient","nodal_field"}, 
        {"constraint_gradient","nodal_field"}, 
        {"topology","nodal_field"},
        {"control","nodal_field"}, 
        {"cauchy_stress","element_field"}, 
        {"deviatoric_stress","element_field"},
        {"plastic_multiplier_increment","element_field"}, 
        {"elastic_strain","element_field"},
        {"plastic_strain","element_field"}, 
        {"backstress","element_field"}, 
        {"principal_stresses","element_field"},
        {"stress","element_field"}, 
        {"strain","element_field"}
    };
};
// struct ValidOutputToLayoutKeys

struct ValidPhysicsKeys
{
    /*!<
     * \brief Valid plato input deck physics keywords.
     **/
    std::vector<std::string> mKeys =
    { 
        "steady_state_mechanics", 
        "transient_mechanics", 
        "steady_state_thermal", 
        "transient_thermal", 
        "steady_state_electrical", 
        "steady_state_thermomechanics",
        "transient_thermomechanics",
        "steady_state_electromechanics",
        "plasticity", 
        "thermoplasticity"
    };
};
// struct ValidPhysicsKeys

struct ValidMaterialPropertyKeys
{
    /*!<
     * \brief Valid plato input deck material property keywords \n
     **/
    std::vector<std::string> mKeys = 
    { 
        "youngs_modulus", 
        "poissons_ratio", 
        "mass_density", 
        "youngs_modulus_x", 
        "youngs_modulus_y", 
        "youngs_modulus_z",
        "poissons_ratio_xy", 
        "poissons_ratio_xz", 
        "poissons_ratio_yz", 
        "shear_modulus_xy", 
        "shear_modulus_xz", 
        "shear_modulus_yz",
        "dielectric_permittivity_11", 
        "dielectric_permittivity_33", 
        "piezoelectric_coupling_15", 
        "piezoelectric_coupling_33", 
        "piezoelectric_coupling_31",
        "thermal_conductivity", 
        "specific_heat", 
        "reference_temperature", 
        "thermal_expansivity" 
    };
};
// struct ValidMaterialPropertyKeys

struct ValidMaterialModelKeys
{
    /*!<
     * \brief Valid plato input deck material model keywords \n
     **/
    std::vector<std::string> mKeys = 
    { 
        "isotropic_linear_elastic", 
        "orthotropic_linear_elastic", 
        "isotropic_linear_electroelastic", 
        "isotropic_linear_thermal",
        "isotropic_linear_thermoelastic" };
};
// struct ValidMaterialModelKeys

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
        { {"element_field", "Element Field"}, {"nodal_field", "Nodal Field"}, {"global", "Global"} };
};
// struct ValidLayoutKeys
//
struct ValidLayouts
{
    /*!<
     * valid operation field layouts \n
     * \brief list of valid Plato layouts
     **/
    std::vector<std::string> mKeys =
        { "Element Field", "Nodal Field", "Global" };
};
// struct ValidLayoutKeys

struct ValidFilterKeys
{
    /*!<
     * valid filters \n
     * \brief map from light-input file key to Plato main operation XML file key, i.e. map<light_input_file_key,plato_main_operation_file_key>
     **/
    std::unordered_map<std::string, std::string> mKeys = 
    { 
        {"identity", "Identity"},
        {"kernel", "Kernel"}, 
        {"kernel_then_heaviside", "KernelThenHeaviside"}, 
        {"kernel_then_tanh", "KernelThenTANH"} 
    };
};
// struct ValidFilterKeys

struct ValidAnalyzeOutputKeys
{
    /*!<
     * \brief Map from Plato Main input file output keywords to Plato Analyze plottable option output keywords. \n
     * The plottable feature is used by Plato Analyze to write output data into its global metadata map. Metadata \n
     * saved in Plato Analyze's metadata map can be accessed by Plato Main through the Plato Analyze application \n
     * layer during Multiple Program, Multiple Data (MPMD) runs. Similarly, plottable data can be saved by Plato \n
     * Analyze into an exodus file, foregoing output through Plato Engine, by invoking the Visualization operation \n
     * in Plato Analyze.
     **/
    std::unordered_map<std::string, std::string> mKeys =
    {
        {"vonmises", "Vonmises"}, 
        {"plastic_multiplier_increment", "plastic multiplier increment"}, 
        {"accumulated_plastic_strain", "accumulated plastic strain"},
        {"deviatoric_stress", "deviatoric stress"}, 
        {"elastic_strain", "elastic_strain"}, 
        {"plastic_strain", "plastic strain"}, 
        {"cauchy_stress", "cauchy stress"},
        {"backstress", "backstress"}, 
        {"stress", "stress"}, 
        {"strain", "strain"}
    };
};
// struct ValidAnalyzeOutputKeys

struct ValidPerformerOutputKeys
{
private:
    /*!<
     * \brief Map from Plato Main input file output keywords to performer codes output argument keywords. These \n
     * output argument keywords are recognized by the Plato::Application interface of the performer and are used \n
     * to extract data from the performer. Thus, Plato Main relies on these output argument names to extract data \n
     * from the performer, e.g. Plato Analyze. Plato Main relies on these output argument keywords to output the \n
     * quantities of interests associated with these keywords in the platomain.exo file.
     *
     *      i.e. map<code_keyword, pair<plato_main_output_keyword, code_output_keyword>. \n
     **/
    std::unordered_map<std::string, std::unordered_map<std::string, std::string> > mKeys =
    {
        { "plato_analyze", 
            { 
                {"vonmises", "Vonmises"}, 
                {"plastic_multiplier_increment", "plastic multiplier increment"},
                {"accumulated_plastic_strain", "accumulated plastic strain"},       
                {"deviatoric_stress", "deviatoric stress"},
                {"elastic_strain", "elastic_strain"}, 
                {"plastic_strain", "plastic strain"}, 
                {"cauchy_stress", "cauchy stress"},
                {"backstress", "backstress"}, 
                {"dispx", "Solution X"}, 
                {"dispy", "Solution Y"}, 
                {"dispz", "Solution Z"},
                {"principal_stresses", "principal stresses"}, 
                {"temperature", "Solution"} 
            }
        }
    };

public:
    /******************************************************************************//**
     * \fn argument
     * \brief Return valid output argument name.
     * \param [in] aCode  code name
     * \param [in] aKey   output keyword
     * \return valid output argument name
    **********************************************************************************/
    std::string argument(const std::string& aCode, const std::string& aKey) const
    {
        if(aCode.empty())
        {
            THROWERR("Valid Performer Output Keys: input 'code' argument is empty.")
        }
        if(aKey.empty())
        {
            THROWERR("Valid Performer Output Keys: input 'key' argument is empty.")
        }
        auto tCodeItr = mKeys.find(aCode);
        if(tCodeItr == mKeys.end())
        {
            THROWERR("Valid Performer Output Keys: code '" + aCode + "' is not supported.")
        }
        auto tKeyItr = tCodeItr->second.find(aKey);
        if(tKeyItr == tCodeItr->second.end())
        {
            THROWERR("Valid Performer Output Keys: output keyword '" + aKey + "' is not supported by code '" + aCode + ".")
        }
        return (tKeyItr->second);
    }
};

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
        { "isotropic_linear_elastic", 
            { 
                { "youngs_modulus", {"Youngs Modulus", "double"} },
                { "poissons_ratio", {"Poissons Ratio", "double"} }, 
                { "mass_density", {"Mass Density", "double"} } 
            }
        },

        { "orthotropic_linear_elastic", 
            { 
                { "youngs_modulus_x", {"Youngs Modulus X", "double"} },
                { "youngs_modulus_y", {"Youngs Modulus Y", "double"} }, 
                { "youngs_modulus_z", {"Youngs Modulus Z", "double"} },
                { "poissons_ratio_xy", {"Poissons Ratio XY", "double"} }, 
                { "poissons_ratio_xz", {"Poissons Ratio XZ", "double"} },
                { "poissons_ratio_yz", {"Poissons Ratio YZ", "double"} }, 
                { "shear_modulus_xy", {"Shear Modulus XY", "double"} },
                { "shear_modulus_xz", {"Shear Modulus XZ", "double"} }, 
                { "shear_modulus_yz", {"Shear Modulus YZ", "double"} },
                { "mass_density", {"Mass Density", "double"} } 
            }
        },

        { "isotropic_linear_electroelastic", 
            { 
                { "youngs_modulus", {"Youngs Modulus", "double"} },
                { "poissons_ratio", {"Poissons Ratio", "double"} }, 
                { "dielectric_permittivity_11", {"p11", "double"} },
                { "dielectric_permittivity_33", {"p33", "double"} }, 
                { "piezoelectric_coupling_15", {"e15", "double"} },
                { "piezoelectric_coupling_33", {"e33", "double"} }, 
                { "piezoelectric_coupling_31", {"e31", "double"} },
                { "thermal_expansivity", {"Alpha", "double"} } 
            }
        },

        { "isotropic_linear_thermal", 
            { 
                { "thermal_conductivity", { "Thermal Conductivity", "double" } },
                { "mass_density", {"Mass Density", "double"} }, 
                { "specific_heat", {"Specific Heat", "double"} } 
            }
        },

        { "isotropic_linear_thermoelastic", 
            { 
                { "thermal_conductivity", { "Thermal Conductivity", "double" } },
                { "youngs_modulus", {"Youngs Modulus", "double"} }, 
                { "poissons_ratio", {"Poissons Ratio", "double"} },
                { "thermal_expansivity", { "Thermal Expansivity", "double" } }, 
                { "reference_temperature", { "Reference Temperature", "double" } },
                { "mass_density", {"Mass Density", "double"} } 
            }
        },

        { "j2_plasticity",
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
            { "steady_state_mechanics", {"Mechanical", "Elliptic"} },
            { "transient_mechanics", {"Mechanical", "Hyperbolic"} },
            { "steady_state_thermal", {"Thermal", "Elliptic"} },
            { "transient_thermal", {"Thermal", "Parabolic"} },
            { "steady_state_electrical", {"Electrical", "Elliptic"} },
            { "steady_state_thermomechanics", {"Thermomechanical", "Elliptic"} },
            { "transient_thermomechanics", {"Thermomechanical", "Parabolic"} },
            { "steady_state_electromechanics", {"Electromechanical", "Elliptic"} },
            { "plasticity", {"Plasticity", "Elliptic"} },
            { "thermoplasticity", {"Thermoplasticity", "Elliptic"} },
            { "stabilized_mechanics", {"Stabilized Mechanics", "Elliptic"} }
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
     * \brief map from plato main objective type key to pair of plato analyze criterion \n
     * key and self-adjoint flag, i.e. \n
     *
     * map< plato_main_objective_type_key, pair<plato_analyze_criterion_key, plato_analyze_self_adjoint_key> >.
     *
     **/
    std::unordered_map<std::string, std::pair<std::string, bool>> mKeys =
    {
        { "volume", { "Volume", false } },
        { "elastic_work", { "Elastic Work", true } },
        { "plastic_work", { "Plastic Work", false } },
        { "mechanical_compliance", { "Internal Elastic Energy", true } },
        { "local_stress", { "Stress Constraint Quadratic", false } },
        { "stress_p-norm", { "Stress P-Norm", false } },
        { "effective_energy", { "Effective Energy", true } },
        //{ "stress constrained mass minimization", { "Stress Constraint General", false } },
        { "thermal_compliance", { "Internal Thermal Energy", false } },
        { "flux_p-norm", { "Flux P-Norm", false } },
        { "thermomechanical_compliance", { "Internal Thermoelastic Energy", false } },
        { "electrical_compliance", { "Internal Electroelastic Energy", false } },
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
            {"steady_state_mechanics", { {"dispx", "0"}, {"dispy", "1"}, {"dispz", "2"} } },
            {"transient_mechanics", { {"dispx", "0"}, {"dispy", "1"}, {"dispz", "2"} } },
            {"steady_state_thermal", { {"temp", "0"} } }, 
            {"transient_thermal", { {"temp", "0"} } }, 
            {"steady_state_electrical", { {"potential", "0"} } },
            {"steady_state_thermomechanics", { {"dispx", "0"}, {"dispy", "1"}, {"dispz", "2"}, {"temp", "3"} } },
            {"transient_thermomechanics", { {"dispx", "0"}, {"dispy", "1"}, {"dispz", "2"}, {"temp", "3"} } },
            {"steady_state_electromechanics", { {"dispx", "0"}, {"dispy", "1"}, {"dispz", "2"}, {"potential", "3"} } }
            // not sure of DOFs {"plasticity", { {"dispx", "0"}, {"dispy", "1"}, {"dispz", "2"} } }
            // not sure of DOFs {"thermoplasticity", { {"dispx", "0"}, {"dispy", "1"}, {"dispz", "2"} } }
        };
};
// struct ValidDofsKeys

}
// namespace XMLGen
