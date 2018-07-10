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

#include "PSL_PureVectorKernel.hpp"

#include "PSL_VectorKernel.hpp"

#include <vector>
#include <cassert>
#include <cmath>
#include <math.h>

namespace PlatoSubproblemLibrary
{

PureVectorKernel::PureVectorKernel(int projection_power) :
        VectorKernel(vector_kernel_t::vector_kernel_t::pure_kernel),
        m_projection_power(projection_power)
{
    assert(0 < m_projection_power);
}
PureVectorKernel::~PureVectorKernel()
{
}

int PureVectorKernel::projection_length(const int& source_length)
{
    return m_projection_power * source_length;
}
void PureVectorKernel::project(const std::vector<double>& source, std::vector<double>& destination)
{
    // allocate
    const int source_length = source.size();
    const int destination_length = projection_length(source_length);
    destination.resize(destination_length);

    // fill
    for(int p = 1; p <= m_projection_power; p++)
    {
        for(int i = 0; i < source_length; i++)
        {
            const int source_index = i;
            const int destination_index = (p - 1) * source_length + i;
            destination[destination_index] = std::pow(source[source_index], p);
        }
    }
}

}


