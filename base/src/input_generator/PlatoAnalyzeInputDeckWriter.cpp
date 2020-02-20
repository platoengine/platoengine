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
 * PlatoAnalyzeInputDeckWriter.cpp
 *
 *  Created on: Nov 19, 2019
 *
 */

#include "PlatoAnalyzeInputDeckWriter.hpp"
#include "XMLGeneratorUtilities.hpp"

#include <exodusII.h>

namespace XMLGen
{


/******************************************************************************/
PlatoAnalyzeInputDeckWriter::PlatoAnalyzeInputDeckWriter(const InputData &aInputData) :
        mInputData(aInputData)
/******************************************************************************/
{
}

/******************************************************************************/
PlatoAnalyzeInputDeckWriter::~PlatoAnalyzeInputDeckWriter()
/******************************************************************************/
{
}

void PlatoAnalyzeInputDeckWriter::generate(std::ostringstream *aStringStream)
{
    for(size_t i=0; i<mInputData.objectives.size(); ++i)
    {
        const XMLGen::Objective& cur_obj = mInputData.objectives[i];
        if(!cur_obj.code_name.compare("plato_analyze"))
        {
            checkForNodesetSidesetNameConflicts();

            char buf[200];
            sprintf(buf, "plato_analyze_input_deck_%s.xml", cur_obj.name.c_str());
            pugi::xml_document doc;

            // Version entry
            pugi::xml_node tmp_node = doc.append_child(pugi::node_declaration);
            tmp_node.set_name("xml");
            pugi::xml_attribute tmp_att = tmp_node.append_attribute("version");
            tmp_att.set_value("1.0");

            pugi::xml_node n1, n2;
            n1 = doc.append_child("ParameterList");
            n1.append_attribute("name") = "Problem";
            addNTVParameter(n1, "Physics", "string", "Plato Driver");
            addNTVParameter(n1, "Spatial Dimension", "int", "3");
            addNTVParameter(n1, "Input Mesh", "string", mInputData.run_mesh_name.c_str());

            n2 = n1.append_child("ParameterList");
            n2.append_attribute("name") = "Plato Problem";
            if(!cur_obj.type.compare("maximize stiffness"))
            {
                buildMaximizeStiffnessParamsForPlatoAnalyze(cur_obj, n2);
            }
            else if(!cur_obj.type.compare("maximize heat conduction"))
            {
                buildMaximizeHeatConductionParamsForPlatoAnalyze(cur_obj, n2);
            }
            else if(!cur_obj.type.compare("minimize thermoelastic energy"))
            {
                buildMinimizeThermoelasticEnergyParamsForPlatoAnalyze(cur_obj, n2);
            }

            // Write the file
            if(aStringStream)
                doc.save(*aStringStream, "\t", pugi::format_default & ~pugi::format_indent);
            else
                doc.save_file(buf, "  ");
        }
    }
}

/******************************************************************************/
bool PlatoAnalyzeInputDeckWriter::checkForNodesetSidesetNameConflicts()
/******************************************************************************/
{
  std::vector<std::string> nodeset_names;
  std::vector<std::string> sideset_names;

  std::vector<XMLGen::LoadCase> load_cases = mInputData.load_cases;

  for(auto load_case:load_cases)
  {
    std::vector<XMLGen::Load> loads = load_case.loads;
    for(auto load:loads)
    {
      if(load.app_type == "nodeset")
        nodeset_names.push_back(load.app_name);
      else if(load.app_type == "sideset")
        sideset_names.push_back(load.app_name);
      else
        std::cout << "WARNING:: Mesh set type found that is not \"nodeset\" or \"sideset\"" << std::endl;
    }
  }

  std::vector<XMLGen::BC> BCs = mInputData.bcs;
  for(auto bc:BCs)
  {
    if(bc.app_type == "nodeset")
      nodeset_names.push_back(bc.app_name);
    else if(bc.app_type == "sideset")
      sideset_names.push_back(bc.app_name);
    else
      std::cout << "WARNING:: Mesh set type found that is not \"nodeset\" or \"sideset\"" << std::endl;
  }

  for(auto ns_name:nodeset_names)
    if(ns_name == "")
    {
      std::cout << "ERROR::XMLGenerator Nodeset name is empty" << std::endl;
      return true;
    }

  for(auto ss_name:sideset_names)
    if(ss_name == "")
    {
      std::cout << "ERROR::XMLGenerator Sideset name is empty" << std::endl;
      return true;
    }

  for(auto ns_name:nodeset_names)
  {
    for(auto ss_name:sideset_names)
    {
      if(ns_name == ss_name)
      {
        std::cout << "ERROR::XMLGenerator Nodeset name is identical to sideset name" << std::endl;
        return true;
      }
    }
  }

  return false;
}

/******************************************************************************/
void PlatoAnalyzeInputDeckWriter::buildMaximizeStiffnessParamsForPlatoAnalyze(const XMLGen::Objective& aObjective, pugi::xml_node aNode)
/******************************************************************************/
{
    pugi::xml_node tPugiNode1, tPugiNode2;

    addNTVParameter(aNode, "Physics", "string", "Mechanical");
    addNTVParameter(aNode, "PDE Constraint", "string", "Elliptic");
    addNTVParameter(aNode, "Constraint", "string", "My Volume");
    addNTVParameter(aNode, "Objective", "string", "My Internal Elastic Energy");
    if(mInputData.optimization_type == "shape")
        addNTVParameter(aNode, "Self-Adjoint", "bool", "false");
    else
        addNTVParameter(aNode, "Self-Adjoint", "bool", "true");

    // Volume Constraint
    addVolumeConstraintForPlatoAnalyze(aNode);

    // Internal Elastic Energy Objective
    addPAObjectiveBlock(aNode, "My Internal Elastic Energy");
    // Elliptic
    addPAPDEConstraintBlock(aNode, "Elliptic");

    // Material model
    addPAMaterialModelBlock(aNode, "Isotropic Linear Elastic");

    int tBCCounter = 1;
    buildMechanicalNBCsForPlatoAnalyze(aObjective, aNode, "Natural Boundary Conditions", tBCCounter);

    tBCCounter = 1;
    tPugiNode1 = aNode.append_child("ParameterList");
    tPugiNode1.append_attribute("name") = "Essential Boundary Conditions";
    buildMechanicalEBCsForPlatoAnalyze(aObjective, tPugiNode1, tBCCounter);
}

/******************************************************************************/
void PlatoAnalyzeInputDeckWriter::buildMinimizeThermoelasticEnergyParamsForPlatoAnalyze(const XMLGen::Objective& aObjective, pugi::xml_node aNode)
/******************************************************************************/
{
    pugi::xml_node tPugiNode1, tPugiNode2;

    addNTVParameter(aNode, "Physics", "string", "Thermomechanical");
    addNTVParameter(aNode, "PDE Constraint", "string", "Elliptic");
    addNTVParameter(aNode, "Constraint", "string", "My Volume");
    addNTVParameter(aNode, "Objective", "string", "My Internal Thermoelastic Energy");
    addNTVParameter(aNode, "Self-Adjoint", "bool", "false");

    // Volume Constraint
    addVolumeConstraintForPlatoAnalyze(aNode);

    // Internal Thermoelastic Energy Objective
    addPAObjectiveBlock(aNode, "My Internal Thermoelastic Energy");

    // Thermoelastostatics
    addPAPDEConstraintBlock(aNode, "Elliptic");

    // Material Model
    addPAMaterialModelBlock(aNode, "Isotropic Linear Thermoelastic");

    int tBCCounter = 1;
    buildMechanicalNBCsForPlatoAnalyze(aObjective, aNode, "Mechanical Natural Boundary Conditions", tBCCounter);
    buildThermalNBCsForPlatoAnalyze(aObjective, aNode, "Thermal Natural Boundary Conditions", tBCCounter);

    tBCCounter = 1;
    tPugiNode1 = aNode.append_child("ParameterList");
    tPugiNode1.append_attribute("name") = "Essential Boundary Conditions";
    buildMechanicalEBCsForPlatoAnalyze(aObjective, tPugiNode1, tBCCounter);
    buildThermalEBCsForPlatoAnalyze(aObjective, tPugiNode1, tBCCounter, "3");
}

/******************************************************************************/
void PlatoAnalyzeInputDeckWriter::buildMaximizeHeatConductionParamsForPlatoAnalyze(const XMLGen::Objective& aObjective, pugi::xml_node aNode)
/******************************************************************************/
{
    pugi::xml_node tPugiNode1, tPugiNode2;

    addNTVParameter(aNode, "Physics", "string", "Thermal");
    addNTVParameter(aNode, "PDE Constraint", "string", "Thermostatics");
    addNTVParameter(aNode, "Constraint", "string", "My Volume");
    addNTVParameter(aNode, "Objective", "string", "My Internal Thermal Energy");
    addNTVParameter(aNode, "Self-Adjoint", "bool", "false");

    // Volume Constraint
    addVolumeConstraintForPlatoAnalyze(aNode);

    // Internal Elastic Energy Objective
    addPAObjectiveBlock(aNode, "My Internal Thermal Energy");
    // Thermostatics
    addPAPDEConstraintBlock(aNode, "Thermostatics");
    // Material Model
    addPAMaterialModelBlock(aNode, "Isotropic Linear Thermal");

    int tBCCounter = 1;
    buildThermalNBCsForPlatoAnalyze(aObjective, aNode, "Natural Boundary Conditions", tBCCounter);

    tBCCounter = 1;
    tPugiNode1 = aNode.append_child("ParameterList");
    tPugiNode1.append_attribute("name") = "Essential Boundary Conditions";
    buildThermalEBCsForPlatoAnalyze(aObjective, tPugiNode1, tBCCounter, "0");
}

void PlatoAnalyzeInputDeckWriter::addVolumeConstraintForPlatoAnalyze(pugi::xml_node aNode)
{
    pugi::xml_node tPugiNode1, tPugiNode2;

    tPugiNode1 = aNode.append_child("ParameterList");
    tPugiNode1.append_attribute("name") = "My Volume";
    addNTVParameter(tPugiNode1, "Type", "string", "Scalar Function");
    addNTVParameter(tPugiNode1, "Scalar Function Type", "string", "Volume");
    tPugiNode2 = tPugiNode1.append_child("ParameterList");
    tPugiNode2.append_attribute("name") = "Penalty Function";
    addNTVParameter(tPugiNode2, "Type", "string", "SIMP");
    addNTVParameter(tPugiNode2, "Exponent", "double", "1.0");
    addNTVParameter(tPugiNode2, "Minimum Value", "double", "0.0");
}

void PlatoAnalyzeInputDeckWriter::addPAObjectiveBlock(pugi::xml_node aNode, const char* aObjectiveName)
{
    pugi::xml_node tPugiNode1, tPugiNode2;

    tPugiNode1 = aNode.append_child("ParameterList");
    tPugiNode1.append_attribute("name") = aObjectiveName;
    addNTVParameter(tPugiNode1, "Type", "string", "Scalar Function");
    tPugiNode2 = tPugiNode1.append_child("ParameterList");
    tPugiNode2.append_attribute("name") = "Penalty Function";
    addNTVParameter(tPugiNode2, "Type", "string", "SIMP");
    addNTVParameter(tPugiNode2, "Exponent", "double", "3.0");
    if(std::strcmp(aObjectiveName, "My Internal Thermal Energy") == 0)
    {
        addNTVParameter(tPugiNode1, "Scalar Function Type", "string", "Internal Thermal Energy");
    }
    else if(std::strcmp(aObjectiveName, "My Internal Thermoelastic Energy") == 0)
    {
        addNTVParameter(tPugiNode1, "Scalar Function Type", "string", "Internal Thermoelastic Energy");
    }
    else if(std::strcmp(aObjectiveName, "My Internal Elastic Energy") == 0)
    {
        addNTVParameter(tPugiNode1, "Scalar Function Type", "string", "Internal Elastic Energy");
        addNTVParameter(tPugiNode2, "Minimum Value", "double", "1.0e-3");
    }
}

void PlatoAnalyzeInputDeckWriter::addPAPDEConstraintBlock(pugi::xml_node aNode, const char* aPDEConstraintName)
{
    pugi::xml_node tPugiNode1, tPugiNode2;

    tPugiNode1 = aNode.append_child("ParameterList");
    tPugiNode1.append_attribute("name") = aPDEConstraintName;
    tPugiNode2 = tPugiNode1.append_child("ParameterList");
    tPugiNode2.append_attribute("name") = "Penalty Function";
    addNTVParameter(tPugiNode2, "Type", "string", "SIMP");
    addNTVParameter(tPugiNode2, "Exponent", "double", "3.0");
    addNTVParameter(tPugiNode2, "Minimum Value", "double", "1e-3");
}

void PlatoAnalyzeInputDeckWriter::addPAMaterialModelBlock(pugi::xml_node aNode, const char* aMaterialModelName)
{
    pugi::xml_node tPugiNode1, tPugiNode2;

    tPugiNode1 = aNode.append_child("ParameterList");
    tPugiNode1.append_attribute("name") = "Material Model";
    tPugiNode2 = tPugiNode1.append_child("ParameterList");
    tPugiNode2.append_attribute("name") = aMaterialModelName;
    if(mInputData.materials.size() > 0)
    {
        if(std::strcmp(aMaterialModelName, "Isotropic Linear Thermal") == 0)
        {
            addNTVParameter(tPugiNode2, "Conductivity Coefficient", "double", mInputData.materials[0].thermal_conductivity);
            addNTVParameter(tPugiNode2, "Mass Density", "double", mInputData.materials[0].density);
            addNTVParameter(tPugiNode2, "Specific Heat", "double", mInputData.materials[0].specific_heat);
        }
        else if(std::strcmp(aMaterialModelName, "Isotropic Linear Thermoelastic") == 0)
        {
            addNTVParameter(tPugiNode2, "Poissons Ratio", "double", mInputData.materials[0].poissons_ratio); // Assuming 1 material!!!
            addNTVParameter(tPugiNode2, "Youngs Modulus", "double", mInputData.materials[0].youngs_modulus);
            addNTVParameter(tPugiNode2, "Thermal Expansion Coefficient", "double", mInputData.materials[0].thermal_expansion);
            addNTVParameter(tPugiNode2, "Thermal Conductivity Coefficient", "double", mInputData.materials[0].thermal_conductivity);
            addNTVParameter(tPugiNode2, "Reference Temperature", "double", mInputData.materials[0].reference_temperature);
        }
        else if(std::strcmp(aMaterialModelName, "Isotropic Linear Elastic") == 0)
        {
            addNTVParameter(tPugiNode2, "Poissons Ratio", "double", mInputData.materials[0].poissons_ratio); // Assuming 1 material!!!
            addNTVParameter(tPugiNode2, "Youngs Modulus", "double", mInputData.materials[0].youngs_modulus);
        }
    }
}

void PlatoAnalyzeInputDeckWriter::buildThermalNBCsForPlatoAnalyze(const XMLGen::Objective& aObjective,
                                                                  pugi::xml_node aNode,
                                                                  const std::string &aTitle,
                                                                  int &aBCCounter)
{
    char tBuffer[200];
    pugi::xml_node tPugiNode1, tPugiNode2;

    tPugiNode1 = aNode.append_child("ParameterList");
    tPugiNode1.append_attribute("name") = aTitle.c_str();
    for(size_t j=0; j<aObjective.load_case_ids.size(); j++)
    {
        bool found = false;
        XMLGen::LoadCase cur_load_case;
        std::string cur_load_id = aObjective.load_case_ids[j];
        for(size_t qq=0; qq<mInputData.load_cases.size(); ++qq)
        {
            if(cur_load_id == mInputData.load_cases[qq].id)
            {
                found = true;
                cur_load_case = mInputData.load_cases[qq];
            }
        }
        if(found)
        {
            for(size_t e=0; e<cur_load_case.loads.size(); e++)
            {
                XMLGen::Load cur_load = cur_load_case.loads[e];
                if(cur_load.type == "heat")
                {
                    tPugiNode2 = tPugiNode1.append_child("ParameterList");
                    sprintf(tBuffer, "Surface Flux Boundary Condition %d", aBCCounter++);
                    tPugiNode2.append_attribute("name") = tBuffer;
                    addNTVParameter(tPugiNode2, "Type", "string", "Uniform");
                    addNTVParameter(tPugiNode2, "Value", "double", cur_load.values[0]);
                    sprintf(tBuffer, "%s", cur_load.app_name.c_str());
                    addNTVParameter(tPugiNode2, "Sides", "string", tBuffer);
                }
            }
        }
    }
}

void PlatoAnalyzeInputDeckWriter::buildThermalEBCsForPlatoAnalyze(const XMLGen::Objective& aObjective, pugi::xml_node aNode,
                                                                  int &aBCCounter,
                                                                  const char* aVariableIndex)
{
    char tBuffer[200];
    pugi::xml_node tPugiNode2;

    for(size_t j=0; j<aObjective.bc_ids.size(); j++)
    {
        bool found = false;
        XMLGen::BC cur_bc;
        std::string cur_bc_id = aObjective.bc_ids[j];
        for(size_t qq=0; qq<mInputData.bcs.size(); ++qq)
        {
            if(cur_bc_id == mInputData.bcs[qq].bc_id)
            {
                found = true;
                cur_bc = mInputData.bcs[qq];
            }
        }
        if(found && cur_bc.type == "temperature")
        {
            tPugiNode2 = aNode.append_child("ParameterList");
            sprintf(tBuffer, "Fixed Temperature Boundary Condition %d", aBCCounter++);
            tPugiNode2.append_attribute("name") = tBuffer;
            addNTVParameter(tPugiNode2, "Index", "int", aVariableIndex);
            sprintf(tBuffer, "%s", cur_bc.app_name.c_str());
            addNTVParameter(tPugiNode2, "Sides", "string", tBuffer);
            if(cur_bc.value.empty())
            {
                addNTVParameter(tPugiNode2, "Type", "string", "Zero Value");
            }
            else
            {
                addNTVParameter(tPugiNode2, "Type", "string", "Fixed Value");
                addNTVParameter(tPugiNode2, "Value", "double", cur_bc.value);
            }
        }
    }
}

void PlatoAnalyzeInputDeckWriter::buildMechanicalEBCsForPlatoAnalyze(const XMLGen::Objective& aObjective, pugi::xml_node aNode,
                                                             int &aBCCounter)
{
    char tBuffer[200];
    pugi::xml_node tPugiNode2;

    for(size_t j=0; j<aObjective.bc_ids.size(); j++)
    {
        bool found = false;
        XMLGen::BC cur_bc;
        std::string cur_bc_id = aObjective.bc_ids[j];
        for(size_t qq=0; qq<mInputData.bcs.size(); ++qq)
        {
            if(cur_bc_id == mInputData.bcs[qq].bc_id)
            {
                found = true;
                cur_bc = mInputData.bcs[qq];
            }
        }
        if(found && cur_bc.type == "displacement")
        {
            if(cur_bc.dof.empty())
            {
                // apply in all 3 directions
                // X Displacement
                tPugiNode2 = aNode.append_child("ParameterList");
                sprintf(tBuffer, "X Fixed Displacement Boundary Condition %d", aBCCounter++);
                tPugiNode2.append_attribute("name") = tBuffer;
                addNTVParameter(tPugiNode2, "Index", "int", "0");
                sprintf(tBuffer, "%s", cur_bc.app_name.c_str());
                addNTVParameter(tPugiNode2, "Sides", "string", tBuffer);
                addNTVParameter(tPugiNode2, "Type", "string", "Zero Value");
                // Y Displacement
                tPugiNode2 = aNode.append_child("ParameterList");
                sprintf(tBuffer, "Y Fixed Displacement Boundary Condition %d", aBCCounter++);
                tPugiNode2.append_attribute("name") = tBuffer;
                addNTVParameter(tPugiNode2, "Index", "int", "1");
                sprintf(tBuffer, "%s", cur_bc.app_name.c_str());
                addNTVParameter(tPugiNode2, "Sides", "string", tBuffer);
                addNTVParameter(tPugiNode2, "Type", "string", "Zero Value");
                // Z Displacement
                tPugiNode2 = aNode.append_child("ParameterList");
                sprintf(tBuffer, "Z Fixed Displacement Boundary Condition %d", aBCCounter++);
                tPugiNode2.append_attribute("name") = tBuffer;
                addNTVParameter(tPugiNode2, "Index", "int", "2");
                sprintf(tBuffer, "%s", cur_bc.app_name.c_str());
                addNTVParameter(tPugiNode2, "Sides", "string", tBuffer);
                addNTVParameter(tPugiNode2, "Type", "string", "Zero Value");
            }
            else
            {
                std::string tBCIndex;
                std::string tBCDOF;
                std::string tBCType;
                if(cur_bc.dof == "x")
                {
                    tBCIndex = "0";
                    tBCDOF = "X";
                }
                else if(cur_bc.dof == "y")
                {
                    tBCIndex = "1";
                    tBCDOF = "Y";
                }
                else if(cur_bc.dof == "z")
                {
                    tBCIndex = "2";
                    tBCDOF = "Z";
                }

                tPugiNode2 = aNode.append_child("ParameterList");
                sprintf(tBuffer, "%s Fixed Displacement Boundary Condition %d", tBCDOF.c_str(), aBCCounter++);
                tPugiNode2.append_attribute("name") = tBuffer;
                addNTVParameter(tPugiNode2, "Index", "int", tBCIndex);
                sprintf(tBuffer, "%s", cur_bc.app_name.c_str());
                addNTVParameter(tPugiNode2, "Sides", "string", tBuffer);
                if(cur_bc.value.empty())
                {
                    addNTVParameter(tPugiNode2, "Type", "string", "Zero Value");
                }
                else
                {
                    addNTVParameter(tPugiNode2, "Type", "string", "Fixed Value");
                    addNTVParameter(tPugiNode2, "Value", "double", cur_bc.value);
                }
            }
        }
    }
}

void PlatoAnalyzeInputDeckWriter::buildMechanicalNBCsForPlatoAnalyze(const XMLGen::Objective& aObjective, pugi::xml_node aNode, const std::string &aTitle,
                                                     int &aBCCounter)
{
    char tBuffer[200];
    pugi::xml_node tPugiNode1, tPugiNode2;

    tPugiNode1 = aNode.append_child("ParameterList");
    tPugiNode1.append_attribute("name") = aTitle.c_str();
    for(size_t j=0; j<aObjective.load_case_ids.size(); j++)
    {
        bool found = false;
        XMLGen::LoadCase cur_load_case;
        std::string cur_load_id = aObjective.load_case_ids[j];
        for(size_t qq=0; qq<mInputData.load_cases.size(); ++qq)
        {
            if(cur_load_id == mInputData.load_cases[qq].id)
            {
                found = true;
                cur_load_case = mInputData.load_cases[qq];
            }
        }
        if(found)
        {
            for(size_t e=0; e<cur_load_case.loads.size(); e++)
            {
                XMLGen::Load cur_load = cur_load_case.loads[e];
                if(cur_load.type == "traction")
                {
                    tPugiNode2 = tPugiNode1.append_child("ParameterList");
                    sprintf(tBuffer, "Traction Vector Boundary Condition %d", aBCCounter++);
                    tPugiNode2.append_attribute("name") = tBuffer;
                    addNTVParameter(tPugiNode2, "Type", "string", "Uniform");
                    double x = std::atof(cur_load.values[0].c_str());
                    double y = std::atof(cur_load.values[1].c_str());
                    double z = std::atof(cur_load.values[2].c_str());
                    sprintf(tBuffer, "{%lf,%lf,%lf}", x, y, z);
                    addNTVParameter(tPugiNode2, "Values", "Array(double)", tBuffer);
                    sprintf(tBuffer, "%s", cur_load.app_name.c_str());
                    addNTVParameter(tPugiNode2, "Sides", "string", tBuffer);
                }
            }
        }
    }
}




}


