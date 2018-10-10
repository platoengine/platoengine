/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
*/

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
bool XMLGenerator_UnitTester::publicParseLoads(std::istream &sin)
/******************************************************************************/
{
    return parseLoads(sin);
}

/******************************************************************************/
bool XMLGenerator_UnitTester::publicParseBCs(std::istream &sin)
/******************************************************************************/
{
    return parseBCs(sin);
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
bool XMLGenerator_UnitTester::publicParseUncertainties(std::istream &sin)
/******************************************************************************/
{
    return parseUncertainties(sin);
}

/******************************************************************************/
bool XMLGenerator_UnitTester::publicExpandUncertaintiesForGenerate()
/******************************************************************************/
{
    return expandUncertaintiesForGenerate();
}

/******************************************************************************/
bool XMLGenerator_UnitTester::publicDistributeObjectivesForGenerate()
/******************************************************************************/
{
    return distributeObjectivesForGenerate();
}

/******************************************************************************/
void XMLGenerator_UnitTester::clearInputData()
/******************************************************************************/
{
    m_InputData.bcs.clear();
    m_InputData.load_cases.clear();
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
    m_InputData.filter_power="";
    m_InputData.num_opt_processors="";
    m_InputData.output_frequency="";
    m_InputData.output_method="";
    m_InputData.max_iterations="";
    m_InputData.discretization="";
    m_InputData.volume_fraction="";
    m_InputData.mesh_name="";
    m_InputData.mesh_name_without_extension="";
    m_InputData.plato_main_path="";
    m_InputData.lightmp_path="";
    m_InputData.sierra_sd_path="";
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
    m_InputData.uncertainties.clear();
}

std::string XMLGenerator_UnitTester::getBCApplicationType(const std::string &aBCID)
{
    for(size_t j=0; j<m_InputData.bcs.size(); ++j)
    {
        if(m_InputData.bcs[j].bc_id == aBCID)
        {
            return m_InputData.bcs[j].app_type;
        }
    }
    return "";
}

std::string XMLGenerator_UnitTester::getBCApplicationID(const std::string &aBCID)
{
    for(size_t j=0; j<m_InputData.bcs.size(); ++j)
    {
        if(m_InputData.bcs[j].bc_id == aBCID)
        {
            return m_InputData.bcs[j].app_id;
        }
    }
    return "";
}

std::string XMLGenerator_UnitTester::getBCApplicationDOF(const std::string &aBCID)
{
    for(size_t j=0; j<m_InputData.bcs.size(); ++j)
    {
        if(m_InputData.bcs[j].bc_id == aBCID)
        {
            return m_InputData.bcs[j].dof;
        }
    }
    return "";
}

std::string XMLGenerator_UnitTester::getLoadType(const std::string &aLoadID, const int &aLoadIndex)
{
    for(size_t j=0; j<m_InputData.load_cases.size(); ++j)
    {
        if(m_InputData.load_cases[j].id == aLoadID)
        {
            return m_InputData.load_cases[j].loads[aLoadIndex].type;
        }
    }
    return "";
}

std::string XMLGenerator_UnitTester::getLoadApplicationType(const std::string &aLoadID, const int &aLoadIndex)
{
    for(size_t j=0; j<m_InputData.load_cases.size(); ++j)
    {
        if(m_InputData.load_cases[j].id == aLoadID)
        {
            return m_InputData.load_cases[j].loads[aLoadIndex].app_type;
        }
    }
    return "";
}

std::string XMLGenerator_UnitTester::getLoadApplicationID(const std::string &aLoadID, const int &aLoadIndex)
{
    for(size_t j=0; j<m_InputData.load_cases.size(); ++j)
    {
        if(m_InputData.load_cases[j].id == aLoadID)
        {
            return m_InputData.load_cases[j].loads[aLoadIndex].app_id;
        }
    }
    return "";
}

std::string XMLGenerator_UnitTester::getLoadDirectionX(const std::string &aLoadID, const int &aLoadIndex)
{
    for(size_t j=0; j<m_InputData.load_cases.size(); ++j)
    {
        if(m_InputData.load_cases[j].id == aLoadID)
        {
            return m_InputData.load_cases[j].loads[aLoadIndex].x;
        }
    }
    return "";
}

std::string XMLGenerator_UnitTester::getLoadDirectionY(const std::string &aLoadID, const int &aLoadIndex)
{
    for(size_t j=0; j<m_InputData.load_cases.size(); ++j)
    {
        if(m_InputData.load_cases[j].id == aLoadID)
        {
            return m_InputData.load_cases[j].loads[aLoadIndex].y;
        }
    }
    return "";
}

std::string XMLGenerator_UnitTester::getLoadDirectionZ(const std::string &aLoadID, const int &aLoadIndex)
{
    for(size_t j=0; j<m_InputData.load_cases.size(); ++j)
    {
        if(m_InputData.load_cases[j].id == aLoadID)
        {
            return m_InputData.load_cases[j].loads[aLoadIndex].z;
        }
    }
    return "";
}



