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
 * XML_GoldValues.hpp
 *
 *  Created on: Sep 26, 2019
 */


namespace PlatoTestXMLGenerator
{
std::string gMechanicalGoldString =
        "<?xml version=\"1.0\"?>\n"
        "<ParameterList name=\"Problem\">\n"
        "<Parameter name=\"Physics\" type=\"string\" value=\"Plato Driver\" />\n"
        "<Parameter name=\"Spatial Dimension\" type=\"int\" value=\"3\" />\n"
        "<Parameter name=\"Input Mesh\" type=\"string\" value=\"\" />\n"
        "<ParameterList name=\"Plato Problem\">\n"
        "<Parameter name=\"Physics\" type=\"string\" value=\"Mechanical\" />\n"
        "<Parameter name=\"PDE Constraint\" type=\"string\" value=\"Elliptic\" />\n"
        "<Parameter name=\"Constraint\" type=\"string\" value=\"My Volume\" />\n"
        "<Parameter name=\"Objective\" type=\"string\" value=\"My Internal Elastic Energy\" />\n"
        "<Parameter name=\"Self-Adjoint\" type=\"bool\" value=\"true\" />\n"
        "<ParameterList name=\"My Volume\">\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Scalar Function\" />\n"
        "<Parameter name=\"Scalar Function Type\" type=\"string\" value=\"Volume\" />\n"
        "<ParameterList name=\"Penalty Function\">\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"SIMP\" />\n"
        "<Parameter name=\"Exponent\" type=\"double\" value=\"1.0\" />\n"
        "<Parameter name=\"Minimum Value\" type=\"double\" value=\"0.0\" />\n"
        "</ParameterList>\n"
        "</ParameterList>\n"
        "<ParameterList name=\"My Internal Elastic Energy\">\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Scalar Function\" />\n"
        "<ParameterList name=\"Penalty Function\">\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"SIMP\" />\n"
        "<Parameter name=\"Exponent\" type=\"double\" value=\"3.0\" />\n"
        "<Parameter name=\"Minimum Value\" type=\"double\" value=\"1.0e-3\" />\n"
        "</ParameterList>\n"
        "<Parameter name=\"Scalar Function Type\" type=\"string\" value=\"Internal Elastic Energy\" />\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Elliptic\">\n"
        "<ParameterList name=\"Penalty Function\">\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"SIMP\" />\n"
        "<Parameter name=\"Exponent\" type=\"double\" value=\"3.0\" />\n"
        "<Parameter name=\"Minimum Value\" type=\"double\" value=\"1e-3\" />\n"
        "</ParameterList>\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Material Model\">\n"
        "<ParameterList name=\"Isotropic Linear Elastic\">\n"
        "<Parameter name=\"Poissons Ratio\" type=\"double\" value=\"0.3\" />\n"
        "<Parameter name=\"Youngs Modulus\" type=\"double\" value=\"1e8\" />\n"
        "</ParameterList>\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Natural Boundary Conditions\">\n"
        "<ParameterList name=\"Traction Vector Boundary Condition 1\">\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Uniform\" />\n"
        "<Parameter name=\"Values\" type=\"Array(double)\" value=\"{0.000000,-3000.000000,0.000000}\" />\n"
        "<Parameter name=\"Sides\" type=\"string\" value=\"2\" />\n"
        "</ParameterList>\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Essential Boundary Conditions\">\n"
        "<ParameterList name=\"X Fixed Displacement Boundary Condition 1\">\n"
        "<Parameter name=\"Index\" type=\"int\" value=\"0\" />\n"
        "<Parameter name=\"Sides\" type=\"string\" value=\"1\" />\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Zero Value\" />\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Y Fixed Displacement Boundary Condition 2\">\n"
        "<Parameter name=\"Index\" type=\"int\" value=\"1\" />\n"
        "<Parameter name=\"Sides\" type=\"string\" value=\"1\" />\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Zero Value\" />\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Z Fixed Displacement Boundary Condition 3\">\n"
        "<Parameter name=\"Index\" type=\"int\" value=\"2\" />\n"
        "<Parameter name=\"Sides\" type=\"string\" value=\"1\" />\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Zero Value\" />\n"
        "</ParameterList>\n"
        "<ParameterList name=\"X Fixed Displacement Boundary Condition 4\">\n"
        "<Parameter name=\"Index\" type=\"int\" value=\"0\" />\n"
        "<Parameter name=\"Sides\" type=\"string\" value=\"1\" />\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Zero Value\" />\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Y Fixed Displacement Boundary Condition 5\">\n"
        "<Parameter name=\"Index\" type=\"int\" value=\"1\" />\n"
        "<Parameter name=\"Sides\" type=\"string\" value=\"1\" />\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Fixed Value\" />\n"
        "<Parameter name=\"Value\" type=\"double\" value=\"3.0\" />\n"
        "</ParameterList>\n"
        "</ParameterList>\n"
        "</ParameterList>\n"
        "</ParameterList>\n"
;

std::string gThermalGoldString =
        "<?xml version=\"1.0\"?>\n"
        "<ParameterList name=\"Problem\">\n"
        "<Parameter name=\"Physics\" type=\"string\" value=\"Plato Driver\" />\n"
        "<Parameter name=\"Spatial Dimension\" type=\"int\" value=\"3\" />\n"
        "<Parameter name=\"Input Mesh\" type=\"string\" value=\"\" />\n"
        "<ParameterList name=\"Plato Problem\">\n"
        "<Parameter name=\"Physics\" type=\"string\" value=\"Thermal\" />\n"
        "<Parameter name=\"PDE Constraint\" type=\"string\" value=\"Thermostatics\" />\n"
        "<Parameter name=\"Constraint\" type=\"string\" value=\"My Volume\" />\n"
        "<Parameter name=\"Objective\" type=\"string\" value=\"My Internal Thermal Energy\" />\n"
        "<Parameter name=\"Self-Adjoint\" type=\"bool\" value=\"false\" />\n"
        "<ParameterList name=\"My Volume\">\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Scalar Function\" />\n"
        "<Parameter name=\"Scalar Function Type\" type=\"string\" value=\"Volume\" />\n"
        "<ParameterList name=\"Penalty Function\">\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"SIMP\" />\n"
        "<Parameter name=\"Exponent\" type=\"double\" value=\"1.0\" />\n"
        "<Parameter name=\"Minimum Value\" type=\"double\" value=\"0.0\" />\n"
        "</ParameterList>\n"
        "</ParameterList>\n"
        "<ParameterList name=\"My Internal Thermal Energy\">\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Scalar Function\" />\n"
        "<ParameterList name=\"Penalty Function\">\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"SIMP\" />\n"
        "<Parameter name=\"Exponent\" type=\"double\" value=\"3.0\" />\n"
        "</ParameterList>\n"
        "<Parameter name=\"Scalar Function Type\" type=\"string\" value=\"Internal Thermal Energy\" />\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Thermostatics\">\n"
        "<ParameterList name=\"Penalty Function\">\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"SIMP\" />\n"
        "<Parameter name=\"Exponent\" type=\"double\" value=\"3.0\" />\n"
        "<Parameter name=\"Minimum Value\" type=\"double\" value=\"1e-3\" />\n"
        "</ParameterList>\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Material Model\">\n"
        "<ParameterList name=\"Isotropic Linear Thermal\">\n"
        "<Parameter name=\"Conductivity Coefficient\" type=\"double\" value=\"210.0\" />\n"
        "<Parameter name=\"Mass Density\" type=\"double\" value=\"2703\" />\n"
        "<Parameter name=\"Specific Heat\" type=\"double\" value=\"900\" />\n"
        "</ParameterList>\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Natural Boundary Conditions\">\n"
        "<ParameterList name=\"Surface Flux Boundary Condition 1\">\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Uniform\" />\n"
        "<Parameter name=\"Value\" type=\"double\" value=\"-1e2\" />\n"
        "<Parameter name=\"Sides\" type=\"string\" value=\"ss_1\" />\n"
        "</ParameterList>\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Essential Boundary Conditions\">\n"
        "<ParameterList name=\"Fixed Temperature Boundary Condition 1\">\n"
        "<Parameter name=\"Index\" type=\"int\" value=\"0\" />\n"
        "<Parameter name=\"Sides\" type=\"string\" value=\"1\" />\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Zero Value\" />\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Fixed Temperature Boundary Condition 2\">\n"
        "<Parameter name=\"Index\" type=\"int\" value=\"0\" />\n"
        "<Parameter name=\"Sides\" type=\"string\" value=\"2\" />\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Zero Value\" />\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Fixed Temperature Boundary Condition 3\">\n"
        "<Parameter name=\"Index\" type=\"int\" value=\"0\" />\n"
        "<Parameter name=\"Sides\" type=\"string\" value=\"3\" />\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Fixed Value\" />\n"
        "<Parameter name=\"Value\" type=\"double\" value=\"25.0\" />\n"
        "</ParameterList>\n"
        "</ParameterList>\n"
        "</ParameterList>\n"
        "</ParameterList>\n";

std::string gThermomechanicalGoldString =
        "<?xml version=\"1.0\"?>\n"
        "<ParameterList name=\"Problem\">\n"
        "<Parameter name=\"Physics\" type=\"string\" value=\"Plato Driver\" />\n"
        "<Parameter name=\"Spatial Dimension\" type=\"int\" value=\"3\" />\n"
        "<Parameter name=\"Input Mesh\" type=\"string\" value=\"\" />\n"
        "<ParameterList name=\"Plato Problem\">\n"
        "<Parameter name=\"Physics\" type=\"string\" value=\"Thermomechanical\" />\n"
        "<Parameter name=\"PDE Constraint\" type=\"string\" value=\"Elliptic\" />\n"
        "<Parameter name=\"Constraint\" type=\"string\" value=\"My Volume\" />\n"
        "<Parameter name=\"Objective\" type=\"string\" value=\"My Internal Thermoelastic Energy\" />\n"
        "<Parameter name=\"Self-Adjoint\" type=\"bool\" value=\"false\" />\n"
        "<ParameterList name=\"My Volume\">\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Scalar Function\" />\n"
        "<Parameter name=\"Scalar Function Type\" type=\"string\" value=\"Volume\" />\n"
        "<ParameterList name=\"Penalty Function\">\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"SIMP\" />\n"
        "<Parameter name=\"Exponent\" type=\"double\" value=\"1.0\" />\n"
        "<Parameter name=\"Minimum Value\" type=\"double\" value=\"0.0\" />\n"
        "</ParameterList>\n"
        "</ParameterList>\n"
        "<ParameterList name=\"My Internal Thermoelastic Energy\">\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Scalar Function\" />\n"
        "<ParameterList name=\"Penalty Function\">\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"SIMP\" />\n"
        "<Parameter name=\"Exponent\" type=\"double\" value=\"3.0\" />\n"
        "</ParameterList>\n"
        "<Parameter name=\"Scalar Function Type\" type=\"string\" value=\"Internal Thermoelastic Energy\" />\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Elliptic\">\n"
        "<ParameterList name=\"Penalty Function\">\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"SIMP\" />\n"
        "<Parameter name=\"Exponent\" type=\"double\" value=\"3.0\" />\n"
        "<Parameter name=\"Minimum Value\" type=\"double\" value=\"1e-3\" />\n"
        "</ParameterList>\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Material Model\">\n"
        "<ParameterList name=\"Isotropic Linear Thermoelastic\">\n"
        "<Parameter name=\"Poissons Ratio\" type=\"double\" value=\"0.3\" />\n"
        "<Parameter name=\"Youngs Modulus\" type=\"double\" value=\"1e11\" />\n"
        "<Parameter name=\"Thermal Expansion Coefficient\" type=\"double\" value=\"1e-5\" />\n"
        "<Parameter name=\"Thermal Conductivity Coefficient\" type=\"double\" value=\"910.0\" />\n"
        "<Parameter name=\"Reference Temperature\" type=\"double\" value=\"1e-2\" />\n"
        "</ParameterList>\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Mechanical Natural Boundary Conditions\">\n"
        "<ParameterList name=\"Traction Vector Boundary Condition 1\">\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Uniform\" />\n"
        "<Parameter name=\"Values\" type=\"Array(double)\" value=\"{0.000000,100000.000000,0.000000}\" />\n"
        "<Parameter name=\"Sides\" type=\"string\" value=\"ss_1\" />\n"
        "</ParameterList>\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Thermal Natural Boundary Conditions\">\n"
        "<ParameterList name=\"Surface Flux Boundary Condition 2\">\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Uniform\" />\n"
        "<Parameter name=\"Value\" type=\"double\" value=\"0.0\" />\n"
        "<Parameter name=\"Sides\" type=\"string\" value=\"ss_1\" />\n"
        "</ParameterList>\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Essential Boundary Conditions\">\n"
        "<ParameterList name=\"Y Fixed Displacement Boundary Condition 1\">\n"
        "<Parameter name=\"Index\" type=\"int\" value=\"1\" />\n"
        "<Parameter name=\"Sides\" type=\"string\" value=\"1\" />\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Zero Value\" />\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Z Fixed Displacement Boundary Condition 2\">\n"
        "<Parameter name=\"Index\" type=\"int\" value=\"2\" />\n"
        "<Parameter name=\"Sides\" type=\"string\" value=\"1\" />\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Zero Value\" />\n"
        "</ParameterList>\n"
        "<ParameterList name=\"X Fixed Displacement Boundary Condition 3\">\n"
        "<Parameter name=\"Index\" type=\"int\" value=\"0\" />\n"
        "<Parameter name=\"Sides\" type=\"string\" value=\"11\" />\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Zero Value\" />\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Y Fixed Displacement Boundary Condition 4\">\n"
        "<Parameter name=\"Index\" type=\"int\" value=\"1\" />\n"
        "<Parameter name=\"Sides\" type=\"string\" value=\"2\" />\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Zero Value\" />\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Z Fixed Displacement Boundary Condition 5\">\n"
        "<Parameter name=\"Index\" type=\"int\" value=\"2\" />\n"
        "<Parameter name=\"Sides\" type=\"string\" value=\"2\" />\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Zero Value\" />\n"
        "</ParameterList>\n"
        "<ParameterList name=\"X Fixed Displacement Boundary Condition 6\">\n"
        "<Parameter name=\"Index\" type=\"int\" value=\"0\" />\n"
        "<Parameter name=\"Sides\" type=\"string\" value=\"21\" />\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Zero Value\" />\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Fixed Temperature Boundary Condition 7\">\n"
        "<Parameter name=\"Index\" type=\"int\" value=\"3\" />\n"
        "<Parameter name=\"Sides\" type=\"string\" value=\"1\" />\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Zero Value\" />\n"
        "</ParameterList>\n"
        "<ParameterList name=\"Fixed Temperature Boundary Condition 8\">\n"
        "<Parameter name=\"Index\" type=\"int\" value=\"3\" />\n"
        "<Parameter name=\"Sides\" type=\"string\" value=\"2\" />\n"
        "<Parameter name=\"Type\" type=\"string\" value=\"Zero Value\" />\n"
        "</ParameterList>\n"
        "</ParameterList>\n"
        "</ParameterList>\n"
        "</ParameterList>\n";

std::string gDefinesXMLString = "";
} // end PlatoTestXMLGenerator namespace
