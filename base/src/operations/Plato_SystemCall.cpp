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
 * Plato_SystemCall.cpp
 *
 *  Created on: Jun 29, 2019
 */

#include <cstdlib>
#include <sstream>

#include "PlatoApp.hpp"
#include "Plato_Parser.hpp"
#include "Plato_InputData.hpp"
#include "Plato_Exceptions.hpp"
#include "Plato_SystemCall.hpp"
#include "Plato_OperationsUtilities.hpp"

namespace Plato
{
  /******************************************************************************/
  void SystemCall::getArguments(std::vector<Plato::LocalArg> & aLocalArgs)
  /******************************************************************************/
  {
    for(auto& tInputName : mInputNames) {
      aLocalArgs.push_back(Plato::LocalArg(Plato::data::layout_t::SCALAR, tInputName));
    }
  }

  /******************************************************************************/
  SystemCall::SystemCall(PlatoApp* aPlatoApp, Plato::InputData & aNode) :
          Plato::LocalOp(aPlatoApp)
  /******************************************************************************/
  {
    mStringCommand = Plato::Get::String(aNode, "Command");

    for(Plato::InputData tInputNode : aNode.getByName<Plato::InputData>("Input"))
    {
      mInputNames.push_back(Plato::Get::String(tInputNode, "ArgumentName"));
    }
  }

  /******************************************************************************/
  void SystemCall::operator()()
  /******************************************************************************/
  {
    // collect arguments
    std::stringstream commandPlusArgs;
    commandPlusArgs << mStringCommand << " ";
    for(auto& tInputName : mInputNames) {
      auto tInputArgument = mPlatoApp->getValue(tInputName);
      if(tInputArgument->size() > 1){
        throw ParsingException("PlatoApp::SystemCall: input arguments must be than length one.");
      }
      commandPlusArgs << tInputArgument->data()[0];
    }

    // make system call
    std::system(commandPlusArgs.str().c_str());
  }
}

