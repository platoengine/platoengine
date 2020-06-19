/*
 * XMLGeneratorValidInputKeys.hpp
 *
 *  Created on: May 29, 2020
 */

#pragma once


#include <vector>
#include <string>
#include <unordered_map>

namespace XMLGen
{

struct ValidCriterionKeys
{
    /*!<
     * \brief Valid plato xml generator parser criterion keys.
     **/
    std::vector<std::string> mKeys =
        {"maximize total work", "stress constrained mass minimization", "maximize stiffness", "compliance",
         "volume", "surface area", "minimize thermoelastic energy", "thermoelastic energy", "maximize heat conduction",
         "stress p-norm", "flux p-norm", "effective energy", "minimize effective energy", "minimize stress",
         "minimize flux", "electroelastic energy", "minimize electroelastic energy", "thermal energy"};
};
// struct ValidCriterionKeys

struct ValidOutputKeys
{
    /*!<
     * \brief Valid plato input file output keywords. \n
     *  Map from output keyword to data layout, i.e. map<output_key,data_layout>.
     **/
    std::unordered_map<std::string, std::string> mKeys =
        { {"vonmises","element field"}, {"dispx","nodal field"}, {"dispy","nodal field"}, {"dispz","nodal field"},
          {"temperature","nodal field"}, {"accumulated_plastic_strain","element field"}, {"potential","nodal field"} };
};
// struct ValidOutputKeys

struct ValidPhysicsKeys
{
    /*!<
     * \brief Valid plato input file physics keywords.
     **/
    std::vector<std::string> mKeys =
        { "mechanical", "transient mechanics", "plasticity", "stabilized mechanical", "thermal", "heat conduction",
            "electromechanical", "stabilized thermomechanical", "thermomechanical", "coupled heat conduction and mechanics" };
};
// struct ValidPhysicsKeys

struct ValidConstraintTargetAbsoluteKeys
{
    /*!<
     * \brief Valid plato xml generator parser constraint absolute target keys.
     **/
    std::vector<std::string> mKeys = {"volume absolute", "target absolute", "surface_area"};
};
// struct ValidConstraintTargetAbsoluteKeys

struct ValidConstraintTargetNormalizedKeys
{
    /*!<
     * \brief Valid plato xml generator parser constraint normalized target keys.
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

struct ValidPerformerKeys
{
    /*!< valid physics performers supported by plato */
    std::vector<std::string> mKeys = {"plato_analyze", "sierra_sd", "platomain"};
};
// struct ValidPerformerKeys

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
    std::unordered_map<std::string, std::string> mKeys = { {"vonmises", "vonmises"}, {"plastic_multiplier_increment", "plastic multiplier increment"},
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

struct ValidAnalyzePhysicsKeys
{
    /*!<
     * valid plato analyze physics and corresponding PDE keys \n
     * \brief map from physics keyword to pair of plato analyze physics and \n
     * partial differential equations (PDE) keys, i.e. \n
     *
     * map< plato_main_physics_key, pair<plato_analyze_physics_key, plato_analyze_pde_key> >.
     *
     **/
    std::unordered_map<std::string, std::pair<std::string,std::string>> mKeys =
        {
            { "mechanical", {"Mechanical", "Elliptic"} },
            { "transient mechanics", {"Mechanical", "Hyperbolic"} },
            { "plasticity", {"Mechanical", "Parabolic"} },
            { "stabilized mechanical", {"Stabilized Mechanical", "Elliptic"} },
            { "thermal", {"Thermal", "Elliptic"} },
            { "heat conduction", {"Thermal", "Parabolic"} },
            { "electromechanical", {"Electromechanical", "Elliptic"} },
            { "stabilized thermomechanical", {"Stabilized Thermomechanical", "Elliptic"} },
            { "thermomechanical", {"Thermomechanical", "Elliptic"} },
            { "coupled heat conduction and mechanics", {"Thermomechanical", "Parabolic"} }
        };
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
