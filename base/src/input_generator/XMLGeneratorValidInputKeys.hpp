/*
 * XMLGeneratorValidInputKeys.hpp
 *
 *  Created on: May 29, 2020
 */

#pragma once


#include <unordered_map>

namespace XMLGen
{

struct ValidLoadAxisKeys
{
    /*!< valid load vector axis, i.e. {x,y,z} axes */
    std::vector<std::string> mKeys = {"x", "y", "z"};
};
// struct ValidLoadAxisKeys

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
     * valid plato analyze output keys \n
     * \brief map from plato problem input file material property tag to pair of plato analyze input file material \n
     *   property tag and its value type, i.e. \n
     *
     * map<plato_problem_input_file_material_property_tag, pair<plato_analyze_input_xml_file_material_property_tag, value_type>>.
     *
     **/
    std::unordered_map<std::string, std::pair<std::string, std::string>> mKeys = { { "youngs modulus", { "Youngs Modulus", "double" } }, { "poissons ratio", { "Poissons Ratio", "double" } },
        { "thermal expansion coefficient", { "Thermal Expansion Coefficient", "double" } }, { "thermal conductivity coefficient", { "Thermal Conductivity Coefficient", "double" } },
        { "reference temperature", { "Reference Temperature", "double" } } };
};
// struct ValidAnalyzeMaterialPropertyKeys

}
// namespace XMLGen
