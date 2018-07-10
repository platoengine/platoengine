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

#include <iostream>
#include "InputXMLChecker.hpp"

/******************************************************************************/
int main(int argc, char *argv[])
/******************************************************************************/
{
    int num_physics_apps = argc-3;
    if(num_physics_apps > 0)
    {
        std::string interface_file = argv[1];
        std::string plato_operations_file = argv[2];
        std::vector<std::string> physics_app_files;
        for(int i=0; i<num_physics_apps; ++i)
        {
            physics_app_files.push_back(argv[3+i]);
        }
        std::cout << "Running checker on the following input files:" << std::endl;
        std::cout << "    Interface file: " << interface_file << std::endl;
        std::cout << "    PlatoMain operations file: " << plato_operations_file << std::endl;
        for(int i=0; i<num_physics_apps; ++i)
        {
            std::cout << "    Physics app operations file: " << physics_app_files[i] << std::endl;
        }
        InputXMLChecker checker(interface_file, plato_operations_file, physics_app_files);
        checker.check();
    }
    else
    {
        std::cout << "Not enough input files were entered." << std::endl;
    }
}



