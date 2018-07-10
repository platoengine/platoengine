/*
 * XMLGenerator_UnitTester.cpp
 *
 *  Created on: May 25, 2018
 *
 */

#include "XMLGenerator_UnitTester.hpp"

/******************************************************************************/
XMLGenerator_UnitTester::XMLGenerator_UnitTester()
/******************************************************************************/
{
}

/******************************************************************************/
XMLGenerator_UnitTester::~XMLGenerator_UnitTester()
/******************************************************************************/
{
}

/******************************************************************************/
bool XMLGenerator_UnitTester::publicParseSingleValue(const std::vector<std::string> &aTokens,
                                    const std::vector<std::string> &aInputStrings,
                                    std::string &aReturnStringValue)
/******************************************************************************/
{
    return parseSingleValue(aTokens, aInputStrings, aReturnStringValue);
}

/******************************************************************************/
bool XMLGenerator_UnitTester::publicParseSingleUnLoweredValue(const std::vector<std::string> &aTokens,
                                                              const std::vector<std::string> &aUnLoweredTokens,
                                                              const std::vector<std::string> &aInputStrings,
                                                              std::string &aReturnStringValue)
/******************************************************************************/
{
    return parseSingleUnLoweredValue(aTokens, aUnLoweredTokens, aInputStrings, aReturnStringValue);
}

/******************************************************************************/
bool XMLGenerator_UnitTester::publicParseTokens(char *buffer, std::vector<std::string> &tokens)
/******************************************************************************/
{
    return parseTokens(buffer, tokens);
}

/******************************************************************************/
bool XMLGenerator_UnitTester::publicParseObjectives(std::istream &sin)
/******************************************************************************/
{
    return parseObjectives(sin);
}

/******************************************************************************/
bool XMLGenerator_UnitTester::publicParseBlocks(std::istream &sin)
/******************************************************************************/
{
    return parseBlocks(sin);
}

/******************************************************************************/
bool XMLGenerator_UnitTester::publicParseMaterials(std::istream &sin)
/******************************************************************************/
{
    return parseMaterials(sin);
}

/******************************************************************************/
bool XMLGenerator_UnitTester::publicParseCodePaths(std::istream &sin)
/******************************************************************************/
{
    return parseCodePaths(sin);
}

/******************************************************************************/
bool XMLGenerator_UnitTester::publicParseConstraints(std::istream &sin)
/******************************************************************************/
{
    return parseConstraints(sin);
}

/******************************************************************************/
bool XMLGenerator_UnitTester::publicParseMesh(std::istream &sin)
/******************************************************************************/
{
    return parseMesh(sin);
}

/******************************************************************************/
bool XMLGenerator_UnitTester::publicParseOptimizationParameters(std::istream &sin)
/******************************************************************************/
{
    return parseOptimizationParameters(sin);
}

/******************************************************************************/
void XMLGenerator_UnitTester::clearInputData()
/******************************************************************************/
{
    m_InputData.objectives.clear();
    m_InputData.constraints.clear();
    m_InputData.materials.clear();
    m_InputData.blocks.clear();
    m_InputData.levelset_nodesets.clear();
    m_InputData.fixed_block_ids.clear();
    m_InputData.fixed_sideset_ids.clear();
    m_InputData.fixed_nodeset_ids.clear();
    m_InputData.filter_radius_scale="";
    m_InputData.filter_radius_absolute="";
    m_InputData.num_opt_processors="";
    m_InputData.output_frequency="";
    m_InputData.output_method="";
    m_InputData.max_iterations="";
    m_InputData.discretization="";
    m_InputData.volume_fraction="";
    m_InputData.mesh_name="";
    m_InputData.plato_main_path="";
    m_InputData.lightmp_path="";
    m_InputData.salinas_path="";
    m_InputData.albany_path="";
    m_InputData.optimization_algorithm="";
    m_InputData.check_gradient="";
    m_InputData.check_hessian="";
    m_InputData.restart_iteration="";
    m_InputData.initial_density_value="";
    m_InputData.create_levelset_spheres="";
    m_InputData.levelset_sphere_radius="";
    m_InputData.levelset_sphere_packing_factor="";
    m_InputData.levelset_initialization_method="";
    m_InputData.levelset_material_box_min="";
    m_InputData.levelset_material_box_max="";
    m_InputData.GCMMA_inner_kkt_tolerance="";
    m_InputData.GCMMA_outer_kkt_tolerance="";
    m_InputData.GCMMA_inner_control_stagnation_tolerance="";
    m_InputData.GCMMA_outer_control_stagnation_tolerance="";
    m_InputData.GCMMA_outer_objective_stagnation_tolerance="";
    m_InputData.GCMMA_max_inner_iterations="";
    m_InputData.GCMMA_outer_stationarity_tolerance="";
    m_InputData.GCMMA_initial_moving_asymptotes_scale_factor="";
    m_InputData.KS_max_trust_region_iterations="";
    m_InputData.KS_trust_region_expansion_factor="";
    m_InputData.KS_trust_region_contraction_factor="";
    m_InputData.KS_outer_gradient_tolerance="";
    m_InputData.KS_outer_stationarity_tolerance="";
    m_InputData.KS_outer_stagnation_tolerance="";
    m_InputData.KS_outer_control_stagnation_tolerance="";
    m_InputData.KS_outer_actual_reduction_tolerance="";
}

