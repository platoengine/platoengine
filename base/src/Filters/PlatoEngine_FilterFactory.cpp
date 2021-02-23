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

#include "PlatoEngine_FilterFactory.hpp"

#include "PlatoEngine_AbstractFilter.hpp"
#include "PlatoEngine_IdentityFilter.hpp"
#include "PlatoEngine_KernelFilter.hpp"
#include "PlatoEngine_KernelThenHeavisideFilter.hpp"
#include "PlatoEngine_KernelThenTANHFilter.hpp"
#ifdef AMFILTER_ENABLED
#include "PlatoEngine_KernelThenStructuredAMFilter.hpp"
#endif
#include "Plato_Interface.hpp"
#include "data_mesh.hpp"
#include "Plato_Parser.hpp"

#include <memory>
#include <string>
#include <cstdlib>
#include <iostream>

namespace Plato
{

Plato::AbstractFilter* build_filter(InputData aInputData, MPI_Comm& aLocalComm, DataMesh* aMesh)
{
    Plato::AbstractFilter* tResult = nullptr;

    if( aInputData.size<Plato::InputData>("Filter") == 0 )
    {
        // no filter
        tResult = new Plato::IdentityFilter();
    }
    else
    {
        auto tFilterNode = aInputData.get<Plato::InputData>("Filter");
        std::string tNameString = Plato::Get::String(tFilterNode, "Name");
        if(tNameString == "Kernel")
        {
            tResult = new Plato::KernelFilter();
        }
        else if(tNameString == "KernelThenHeaviside")
        {
            tResult = new Plato::KernelThenHeavisideFilter();
        }
        else if(tNameString == "KernelThenTANH")
        {
            tResult = new Plato::KernelThenTANHFilter();
        }
#ifdef AMFILTER_ENABLED
        else if(tNameString == "KernelThenAM")
        {
            // we need to call the custom build function with the AM filter 
            // to extract the mesh data
            Plato::KernelThenStructuredAMFilter* tCustomFilter = new Plato::KernelThenStructuredAMFilter();
            tCustomFilter->build(aInputData, aLocalComm, aMesh);
            return tCustomFilter;
        }
#endif
        else if(tNameString == "Identity")
        {
            tResult = new Plato::IdentityFilter();
        }
        else
        {
            std::cout << "build_filter could not build '" << tNameString << "'" << std::endl;
            std::abort();
        }
    }
    tResult->build(aInputData, aLocalComm, aMesh);
    return tResult;
}

}
