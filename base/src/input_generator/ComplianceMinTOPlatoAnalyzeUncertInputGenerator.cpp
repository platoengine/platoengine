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
 * ComplianceMinTOPlatoAnalyzeUncertInputGenerator.cpp
 *
 *  Created on: Apr 16, 2020
 *
 */

#include "XMLGeneratorDefinesFileUtilities.hpp"
#include "XMLGeneratorLaunchScriptUtilities.hpp"
#include "XMLGeneratorAnalyzeUncertaintyLaunchScriptUtilities.hpp"
#include "XMLGeneratorRandomInterfaceFileUtilities.hpp"
#include "XMLGeneratorPlatoMainInputFileUtilities.hpp"
#include "XMLGeneratorPlatoMainOperationFileUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeInputFileUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeOperationsFileUtilities.hpp"

#include "ComplianceMinTOPlatoAnalyzeUncertInputGenerator.hpp"

namespace XMLGen
{

ComplianceMinTOPlatoAnalyzeUncertInputGenerator::ComplianceMinTOPlatoAnalyzeUncertInputGenerator(const InputData &aInputData) :
    DefaultInputGenerator(aInputData)
{
}

ComplianceMinTOPlatoAnalyzeUncertInputGenerator::~ComplianceMinTOPlatoAnalyzeUncertInputGenerator() 
{
}

bool ComplianceMinTOPlatoAnalyzeUncertInputGenerator::generateInputFiles()
{
    XMLGen::write_define_xml_file(m_InputData.mRandomMetaData, m_InputData.m_UncertaintyMetaData);
    XMLGen::write_interface_xml_file_for_nondeterministic_usecase(m_InputData);
    XMLGen::write_plato_main_operations_xml_file_for_nondeterministic_usecase(m_InputData);
    XMLGen::write_plato_main_input_deck_file(m_InputData);
    XMLGen::write_plato_analyze_operation_xml_file_for_nondeterministic_usecase(m_InputData);
    XMLGen::write_amgx_input_file();
    XMLGen::write_plato_analyze_input_deck_file(m_InputData);
    XMLGen::generate_launch_script(m_InputData);

    std::cout << "Successfully wrote XML files." << std::endl;
    return true;
}

}
