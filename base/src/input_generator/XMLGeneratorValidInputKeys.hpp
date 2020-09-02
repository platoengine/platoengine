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
        {"service", "objective", "constraint", "material", "block", "uncertainty", "mesh", "output"};
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
private:
    /*!<
     * \brief Valid plato input deck identification keywords.
     * Map from random 'category' keyword to identification keyword, i.e. map<category_key, identification_key>.
     **/
    std::unordered_map<std::string, std::string> mKeys = { { "load", "load_id" }, { "material", "material_id" } };

public:
    /******************************************************************************//**
     * \fn value
     * \brief Return supported random category keyword.
     * \param [in] aKey input file keyword
     * \return supported random category keyword. If key is not supported, return an empty string.
    **********************************************************************************/
    std::string value(const std::string& aKey) const
    {
        auto tLowerKey = XMLGen::to_lower(aKey);
        auto tItr = mKeys.find(tLowerKey);
        if(tItr == mKeys.end())
        {
            return ("");
        }
        return tItr->second;
    }
};
// struct ValidStatisticalDistributionKeys

struct ValidStatisticalDistributionKeys
{
private:
    /*!<
     * \brief Valid plato input deck statistical distribution keywords.
     **/
    std::vector<std::string> mKeys = {"normal", "beta", "uniform"};

public:
    /******************************************************************************//**
     * \fn value
     * \brief Return supported probability distribution function keyword.
     * \param [in] aKey input file keyword
     * \return supported probability distribution function keyword. If key is not supported, return an empty string.
    **********************************************************************************/
    std::string value(const std::string& aKey) const
    {
        auto tLowerKey = XMLGen::to_lower(aKey);
        auto tItr = std::find(mKeys.begin(), mKeys.end(), tLowerKey);
        if(tItr == mKeys.end())
        {
            return ("");
        }
        return tItr.operator*();
    }
};
// struct ValidStatisticalDistributionKeys

struct ValidEssentialBoundaryConditionsKeys
{
private:
    /*!<
     * \brief Valid plato input deck essential boundary condition keywords.
     **/
    std::vector<std::string> mKeys = {"rigid", "fixed", "zero value", "fixed value", "insulated"};

public:
    /******************************************************************************//**
     * \fn value
     * \brief Return supported essential boundary condition keyword.
     * \param [in] aKey input file keyword
     * \return supported essential boundary condition keyword. If key is not supported, return an empty string.
    **********************************************************************************/
    std::string value(const std::string& aKey) const
    {
        auto tLowerKey = XMLGen::to_lower(aKey);
        auto tItr = std::find(mKeys.begin(), mKeys.end(), tLowerKey);
        if(tItr == mKeys.end())
        {
            return ("");
        }
        return tItr.operator*();
    }
};
// struct ValidCriterionKeys

struct ValidOutputToLayoutKeys
{
private:
    /*!<
     * \brief Valid plato main output keywords. \n
     *  Map from output keyword to data layout, i.e. map<output_key,data_layout>.
     **/
    std::unordered_map<std::string, std::string> mKeys =
        {
          {"vonmises","element field"}, {"dispx","nodal field"}, {"dispy","nodal field"}, {"dispz","nodal field"},
          {"temperature","nodal field"}, {"accumulated_plastic_strain","element field"}, {"potential","nodal field"},
          {"objective_gradient","nodal field"}, {"constraint_gradient","nodal field"}, {"topology","nodal field"},
          {"control","nodal field"}, {"cauchy_stress","element field"}, {"deviatoric_stress","element field"},
          {"plastic_multiplier_increment","element field"}, {"elastic_strain","element field"},
          {"plastic_strain","element field"}, {"backstress","element field"}, {"principal_stresses","element field"},
          {"stress","element field"}, {"strain","element field"}
        };

public:
    /******************************************************************************//**
     * \fn value
     * \brief Return supported output quantity of interest (QoI) data layout keyword.
     * \param [in] aKey input file keyword
     * \return supported output data layout keyword. If key is not supported, return an empty string.
    **********************************************************************************/
    std::string value(const std::string& aKey) const
    {
        auto tLowerKey = XMLGen::to_lower(aKey);
        auto tItr = mKeys.find(tLowerKey);
        if(tItr == mKeys.end())
        {
            return ("");
        }
        return tItr->second;
    }

    /******************************************************************************//**
     * \fn key
     * \brief Return supported output quantity of interest (QoI) key.
     * \param [in] aKey input file keyword
     * \return supported output data layout key. If key is not supported, return an empty string.
    **********************************************************************************/
    std::string key(const std::string& aKey) const
    {
        auto tLowerKey = XMLGen::to_lower(aKey);
        auto tItr = mKeys.find(tLowerKey);
        if(tItr == mKeys.end())
        {
            return ("");
        }
        return tItr->first;
    }
};
// struct ValidOutputToLayoutKeys

struct ValidPhysicsKeys
{
private:
    /*!<
     * \brief Valid plato input deck physics keywords.
     **/
    std::vector<std::string> mKeys =
        { "mechanical", "transient mechanics", "plasticity", "stabilized mechanics", "thermal", "heat conduction",
            "electromechanical", "stabilized thermomechanical", "thermomechanical", "coupled heat conduction and mechanics" };

public:
    /******************************************************************************//**
     * \fn value
     * \brief Return supported physics keyword.
     * \param [in] aKey input file keyword
     * \return supported physics keyword. If key is not supported, return an empty string.
    **********************************************************************************/
    std::string value(const std::string& aKey) const
    {
        auto tLowerKey = XMLGen::to_lower(aKey);
        auto tItr = std::find(mKeys.begin(), mKeys.end(), tLowerKey);
        if(tItr == mKeys.end())
        {
            return ("");
        }
        return tItr.operator*();
    }
};
// struct ValidPhysicsKeys

struct ValidMaterialPropertyKeys
{
private:
    /*!<
     * \brief Valid plato input deck material property keywords \n
     **/
    std::vector<std::string> mKeys = { "youngs_modulus", "poissons_ratio", "mass_density", "youngs_modulus_x", "youngs_modulus_y", "youngs_modulus_z",
        "poissons_ratio_xy", "poissons_ratio_xz", "poissons_ratio_yz", "shear_modulus_xy", "shear_modulus_xz", "shear_modulus_yz",
        "dielectric_permittivity_11", "dielectric_permittivity_33", "piezoelectric_coupling_15", "piezoelectric_coupling_33", "piezoelectric_coupling_31",
        "thermal_conductivity", "specific_heat", "reference_temperature", "thermal_expansivity" };

public:
    /******************************************************************************//**
     * \fn keys
     * \brief Return list of valid keys.
     * \return list of valid keys
    **********************************************************************************/
    const std::vector<std::string> keys() const
    {
        return mKeys;
    }
};
// struct ValidMaterialPropertyKeys

struct ValidMaterialModelKeys
{
private:
    /*!<
     * \brief Valid plato input deck material model keywords \n
     **/
    std::vector<std::string> mKeys = { "isotropic linear elastic", "orthotropic linear elastic", "isotropic linear electroelastic", "isotropic linear thermal",
        "isotropic linear thermoelastic" };

public:
    /******************************************************************************//**
     * \fn value
     * \brief Return supported material model keyword.
     * \param [in] aKey input file keyword
     * \return supported material model keyword. If key is not supported, return an empty string.
    **********************************************************************************/
    std::string value(const std::string& aKey) const
    {
        auto tLowerKey = XMLGen::to_lower(aKey);
        auto tItr = std::find(mKeys.begin(), mKeys.end(), tLowerKey);
        if(tItr == mKeys.end())
        {
            return ("");
        }
        return tItr.operator*();
    }
};
// struct ValidMaterialModelKeys

struct ValidConstraintTargetAbsoluteKeys
{
private:
    /*!<
     * \brief Valid plato input deck constraint absolute target keywords.
     **/
    std::vector<std::string> mKeys = {"volume absolute", "target absolute", "surface_area"};

public:
    /******************************************************************************//**
     * \fn keys
     * \brief Return list of valid keys.
     * \return list of valid keys
    **********************************************************************************/
    const std::vector<std::string> keys() const
    {
        return mKeys;
    }
};
// struct ValidConstraintTargetAbsoluteKeys

struct ValidConstraintTargetNormalizedKeys
{
private:
    /*!<
     * \brief Valid plato deck constraint normalized target keywords.
     **/
    std::vector<std::string> mKeys = {"volume fraction", "target normalized"};

public:
    /******************************************************************************//**
     * \fn keys
     * \brief Return list of valid keys.
     * \return list of valid keys
    **********************************************************************************/
    const std::vector<std::string> keys() const
    {
        return mKeys;
    }
};
// struct ValidConstraintTargetNormalizedKeys

struct ValidAxesKeys
{
private:
    /*!< map from dimension to axis, i.e. map<dimension, axis> */
    std::unordered_map<size_t, std::string> mKeys = { {0u, "x"}, {1u, "y"}, {2u, "z"} };

public:
    /******************************************************************************//**
     * \fn value
     * \brief Return supported dimension axis keyword.
     * \param [in] aKey input file keyword
     * \return supported dimension axis keyword. If key is not supported, return an empty string.
    **********************************************************************************/
    std::string value(const size_t aKey) const
    {
        auto tItr = mKeys.find(aKey);
        if(tItr == mKeys.end())
        {
            return ("");
        }
        return tItr->second;
    }
};
// struct ValidAxesKeys

struct ValidCodeKeys
{
private:
    /*!< valid plato input deck code keywords supported in plato */
    std::vector<std::string> mKeys = {"plato_analyze", "sierra_sd", "platomain"};

public:
    /******************************************************************************//**
     * \fn value
     * \brief Return code keyword supported by Plato Engine.
     * \param [in] aKey input file keyword
     * \return supported code keyword. If key is not supported, return an empty string.
    **********************************************************************************/
    std::string value(const std::string& aKey) const
    {
        auto tLowerKey = XMLGen::to_lower(aKey);
        auto tItr = std::find(mKeys.begin(), mKeys.end(), tLowerKey);
        if(tItr == mKeys.end())
        {
            return ("");
        }
        return tItr.operator*();
    }
};
// struct ValidCodeKeys

struct ValidDiscretizationKeys
{
private:
    /*!< valid abstract design variables supported in plato, i.e. control variables discretization */
    std::vector<std::string> mKeys = {"density", "levelset"};

public:
    /******************************************************************************//**
     * \fn value
     * \brief Return topology discretization keyword supported by Plato Engine.
     * \param [in] aKey input file keyword
     * \return supported topology discretization keyword. If key is not supported, return an empty string.
    **********************************************************************************/
    std::string value(const std::string& aKey) const
    {
        auto tLowerKey = XMLGen::to_lower(aKey);
        auto tItr = std::find(mKeys.begin(), mKeys.end(), tLowerKey);
        if(tItr == mKeys.end())
        {
            return ("");
        }
        return tItr.operator*();
    }
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
private:
    /*!<
     * valid operation field layouts \n
     * \brief map from light-input file key to Plato layout, i.e. map<light_input_file_key, plato_layout_key>
     **/
    std::unordered_map<std::string, std::string> mKeys =
        { {"element field", "Element Field"}, {"nodal field", "Nodal Field"}, {"global", "Global"} };

public:
    /******************************************************************************//**
     * \fn value
     * \brief Return data layout keyword supported by Plato Engine.
     * \param [in] aKey input file keyword
     * \return supported data layout keyword. If key is not supported, return an empty string.
    **********************************************************************************/
    std::string value(const std::string& aKey) const
    {
        auto tLowerKey = XMLGen::to_lower(aKey);
        auto tItr = mKeys.find(tLowerKey);
        if(tItr == mKeys.end())
        {
            return ("");
        }
        return tItr->second;
    }
};
// struct ValidLayoutKeys

struct ValidFilterKeys
{
private:
    /*!<
     * valid filters \n
     * \brief map from light-input file key to Plato main operation XML file key, i.e. map<light_input_file_key,plato_main_operation_file_key>
     **/
    std::unordered_map<std::string, std::string> mKeys = { {"identity", "Identity"},
        {"kernel", "Kernel"}, {"kernel then heaviside", "KernelThenHeaviside"}, {"kernel then tanh", "KernelThenTANH"} };

public:
    /******************************************************************************//**
     * \fn value
     * \brief Return filter keyword supported by Plato Engine.
     * \param [in] aKey input file keyword
     * \return supported filter keyword. If key is not supported, return an empty string.
    **********************************************************************************/
    std::string value(const std::string& aKey) const
    {
        auto tItr = mKeys.find(aKey);
        if(tItr == mKeys.end())
        {
            return ("");
        }
        return tItr->second;
    }
};
// struct ValidFilterKeys

struct ValidAnalyzeOutputKeys
{
private:
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
          {"vonmises", "Vonmises"}, {"plastic_multiplier_increment", "plastic multiplier increment"}, {"accumulated_plastic_strain", "accumulated plastic strain"},
          {"deviatoric_stress", "deviatoric stress"}, {"elastic_strain", "elastic_strain"}, {"plastic_strain", "plastic strain"}, {"cauchy_stress", "cauchy stress"},
          {"backstress", "backstress"}, {"stress", "stress"}, {"strain", "strain"}
        };

public:
    /******************************************************************************//**
     * \fn value
     * \brief Return output quantity of interest (QoI) keyword supported by Plato Analyze.
     * \param [in] aKey input file keyword
     * \return supported output QoI keyword. If key is not supported, return an empty string.
    **********************************************************************************/
    std::string value(const std::string& aKey) const
    {
        auto tItr = mKeys.find(aKey);
        if(tItr == mKeys.end())
        {
            return ("");
        }
        return tItr->second;
    }
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
        { "plato_analyze", { {"vonmises", "Vonmises"}, {"plastic_multiplier_increment", "plastic multiplier increment"},
          {"accumulated_plastic_strain", "accumulated plastic strain"}, {"deviatoric_stress", "deviatoric stress"},
          {"elastic_strain", "elastic_strain"}, {"plastic_strain", "plastic strain"}, {"cauchy_stress", "cauchy stress"},
          {"backstress", "backstress"}, {"dispx", "Solution X"}, {"dispy", "Solution Y"}, {"dispz", "Solution Z"},
          {"principal_stresses", "principal stresses"}, {"temperature", "Solution"} }
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
            { "plasticity", {"Plasticity", "Elliptic"} },
            { "transient mechanics", {"Mechanical", "Hyperbolic"} },
            { "transient thermomechanics", {"Thermomechanical", "Parabolic"} },
            { "stabilized mechanics", {"Stabilized Mechanical", "Elliptic"} },
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
