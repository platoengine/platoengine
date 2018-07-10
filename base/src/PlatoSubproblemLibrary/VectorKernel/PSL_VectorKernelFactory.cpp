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

#include "PSL_VectorKernelFactory.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_VectorKernel.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_PureVectorKernel.hpp"
#include "PSL_MixedVectorKernel.hpp"
#include "PSL_CrossVectorKernel.hpp"

#include <cstddef>

namespace PlatoSubproblemLibrary
{

VectorKernel* build_vector_kernel(const vector_kernel_t::vector_kernel_t& type,
                                  const int& vector_kernel_parameter,
                                  AbstractInterface::GlobalUtilities* util)
{
    VectorKernel* result = NULL;

    switch(type)
    {
        case vector_kernel_t::vector_kernel_t::pure_kernel:
        {
            result = new PureVectorKernel(vector_kernel_parameter);
            break;
        }
        case vector_kernel_t::vector_kernel_t::mixed_kernel:
        {
            result = new MixedVectorKernel(vector_kernel_parameter);
            break;
        }
        case vector_kernel_t::vector_kernel_t::cross_kernel:
        {
            result = new CrossVectorKernel(vector_kernel_parameter);
            break;
        }
        default:
        {
            util->fatal_error("PlatoSubproblemLibrary could not match enum to VectorKernel. Aborting.\n\n");
            break;
        }
    }

    return result;
}

}


