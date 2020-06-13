/*
 * XMLGeneratorValidInputKeys.hpp
 *
 *  Created on: May 29, 2020
 */

#pragma once


#include <unordered_map>

namespace XMLGen
{

struct ValidAxesKeys
{
    /*!< map from dimension to axis, i.e. map<dimension, axis> */
    std::unordered_map<size_t, std::string> mKeys = { {0, "x"}, {1, "y"}, {2, "z"} };
};
// struct ValidAxesKeys

struct ValidPhysicsPerformerKeys
{
    /*!< valid physics performers supported by plato */
    std::vector<std::string> mKeys = {"plato_analyze", "sierra_sd"};
};
// struct ValidPhysicsPerformerKeys

struct ValidDiscretizationKeys
{
    /*!< valid physical design variables, i.e. discretization */
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
        { {"element field", "Element Field"}, {"nodal field", "Nodal Field"} };
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
     * valid plato analyze output keys \n
     * \brief map from light-input file output key to plato analyze key, i.e. map<light_input_file_output key, plato_analyze_output_key>. \n
     * Basically, this maps connects the key use for any quantity of interest inside Plato's light input file to the key used in \n
     * Plato Analyze to identify the quantity of interest.
     **/
    std::unordered_map<std::string, std::string> mKeys = { {"von_mises", "vonmises"}, {"plastic_multiplier_increment", "plastic multiplier increment"},
      {"accumulated_plastic_strain", "accumulated plastic strain"}, {"deviatoric_stress", "deviatoric stress"}, {"elastic_strain", "elastic_strain"},
      {"plastic_strain", "plastic strain"}, {"cauchy_stress", "cauchy stress"}, {"backstress", "backstress"}, {"dispx", "Solution X"},
      {"dispy", "Solution Y"}, {"dispz", "Solution Z"} };
};
// struct ValidAnalyzeOutputKeys

struct ValidAnalyzeMaterialPropertyKeys
{
    /*!<
     * valid plato analyze material models and corresponding material property keys \n
     * \brief map from material model to map from material property tag in plato input file to \n
     * pair of plato analyze input file material property key and its corresponding value type, i.e. \n
     *
     * map< material_model, map<plato_input_file_material_property_tag, pair<plato_analyze_input_file_material_property_tag, value_type>>>.
     *
     **/
    std::unordered_map<std::string, std::unordered_map<std::string, std::pair<std::string, std::string>>> mKeys =
        {
            { "isotropic linear elastic", { { "youngs modulus", {"Youngs Modulus", "double"} },
                { "poissons ratio", {"Poissons Ratio", "double"} }, { "mass density", {"Mass Density", "double"} } }
            },

            { "orthotropic linear elastic", { { "youngs modulus x", {"Youngs Modulus X", "double"} },
                { "youngs modulus y", {"Youngs Modulus Y", "double"} }, { "youngs modulus z", {"Youngs Modulus Z", "double"} },
                { "poissons ratio xy", {"Poissons Ratio XY", "double"} }, { "poissons ratio xz", {"Poissons Ratio XZ", "double"} },
                { "poissons ratio yz", {"Poissons Ratio YZ", "double"} }, { "shear modulus ratio xy", {"Shear Modulus XY", "double"} },
                { "shear modulus ratio xz", {"Shear Modulus XZ", "double"} }, { "shear modulus ratio yz", {"Shear Modulus YZ", "double"} },
                { "mass density", {"Mass Density", "double"} } }
            },

            { "isotropic linear electroelastic", { { "youngs modulus", {"Youngs Modulus", "double"} },
                { "poissons ratio", {"Poissons Ratio", "double"} }, { "dielectric permittivity 11", {"p11", "double"} },
                { "dielectric permittivity 33", {"p33", "double"} }, { "piezoelectric coupling 15", {"e15", "double"} },
                { "piezoelectric coupling 33", {"e33", "double"} }, { "piezoelectric coupling 31", {"e31", "double"} },
                { "thermal expansion", {"Alpha", "double"} } }
            },

            { "isotropic linear thermal", { { "thermal conductivity coefficient", { "Thermal Conductivity Coefficient", "double" } },
                { "mass density", {"Mass Density", "double"} }, { "specific heat", {"Specific Heat", "double"} } }
            },

            { "isotropic linear thermoelastic", { { "thermal conductivity coefficient", { "Thermal Conductivity Coefficient", "double" } },
                { "youngs modulus", {"Youngs Modulus", "double"} }, { "poissons ratio", {"Poissons Ratio", "double"} },
                { "thermal expansion coefficient", { "Thermal Expansion Coefficient", "double" } },
                { "reference temperature", { "Reference Temperature", "double" } }, { "mass density", {"Mass Density", "double"} } }
            }
        };
};
// struct ValidAnalyzeMaterialPropertyKeys

}
// namespace XMLGen
