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
 * InputXMLChecker.hpp
 *
 *  Created on: Aug 4, 2017
 *
 */

#ifndef SRC_INTERFACEXMLCHECKER_HPP_
#define SRC_INTERFACEXMLCHECKER_HPP_

#include <vector>
#include "Plato_Parser.hpp"

class InputXMLChecker {

public:
  InputXMLChecker(const std::string &interface_xml_filename,
                  const std::string &plato_main_operations_filename,
                  const std::vector<std::string> &physics_app_files);
  ~InputXMLChecker();
  bool check();

private:
  pugi::xml_document mInterface;
  pugi::xml_document mPlatoOperations;
  std::vector<pugi::xml_document*> mPhysicsAppOperations;
  bool check_performers();
  bool check_shared_data();
  bool check_stages();
  bool check_operation(const std::string &stage_name, pugi::xml_node op_node);
  bool definition_exists_in_interface(const std::string &type_name, const std::string &name);

};


#endif /* SRC_INTERFACEXMLCHECKER_HPP_ */
