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
 * XMLG_Macros.hpp
 *
 *  Created on: Jun 21, 2019
 */

#pragma once

#include <iostream>

namespace Plato
{

#define REPORT(msg) \
        std::cout << std::string("\nPLATO WARNING: ") + msg + "\n";
        
#define PRINTERR(msg) \
        std::cout << std::string("\nFILE: ") + __FILE__ \
        + std::string("\nFUNCTION: ") + __PRETTY_FUNCTION__ \
        + std::string("\nLINE:") + std::to_string(__LINE__) \
        + std::string("\nMESSAGE: ") + msg;

#define THROWERR(msg) \
        throw std::runtime_error(std::string("\nFILE: ") + __FILE__ \
        + std::string("\nFUNCTION: ") + __PRETTY_FUNCTION__ \
        + std::string("\nLINE:") + std::to_string(__LINE__) \
        + std::string("\nMESSAGE: ") + msg);

#define PRINTEMPTYINFO(parameter, context) \
        std::cout << std::string("INFO: Parameter \"") + parameter + "\" was not set and there is no default value specified. Downstream applications depending on this parameter will use whatever default they have for it. Context: " + context + "\n";

#define PRINTIGNOREINFO(parameter, context) \
        std::cout << std::string("INFO: Ignoring parameter \"") + parameter + "\" because it was not set. It will take on whatever defaults are in the downstream application. Context: " + context + "\n";

#define PRINTDEFAULTINFO(parameter, value, context) \
        std::cout << std::string("INFO: Using default value of \"") + value + "\" for parameter \"" + parameter + "\" because it was not set. Context: " + context + "\n";

}
// namespace Plato
