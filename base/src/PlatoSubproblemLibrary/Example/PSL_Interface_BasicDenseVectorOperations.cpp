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

// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Interface_BasicDenseVectorOperations.hpp"

#include "PSL_Abstract_DenseVectorOperations.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace example
{

Interface_BasicDenseVectorOperations::Interface_BasicDenseVectorOperations() :
        AbstractInterface::DenseVectorOperations()
{
}

Interface_BasicDenseVectorOperations::~Interface_BasicDenseVectorOperations()
{
}

double Interface_BasicDenseVectorOperations::dot(const std::vector<double>& x, const std::vector<double>& y)
{
    const size_t num_entries = x.size();

    double result = 0.;
    for(size_t index = 0; index < num_entries; index++)
    {
        result += x[index] * y[index];
    }
    return (result);
}

void Interface_BasicDenseVectorOperations::axpy(double alpha, const std::vector<double>& x, std::vector<double>& y)
{
    const size_t num_entries = x.size();

    for(size_t index = 0; index < num_entries; ++index)
    {
        y[index] = alpha * x[index] + y[index];
    }
}

void Interface_BasicDenseVectorOperations::scale(double alpha, std::vector<double>& x)
{
    const size_t num_entries = x.size();

    for(size_t index = 0; index < num_entries; ++index)
    {
        x[index] = alpha * x[index];
    }
}

void Interface_BasicDenseVectorOperations::multiply(const std::vector<double>& x,
                                                    const std::vector<double>& y,
                                                    std::vector<double>& z)
{
    const size_t num_entries = x.size();
    z.resize(num_entries);
    for(size_t i = 0; i < num_entries; ++i)
    {
        z[i] = x[i] * y[i];
    }
}

void Interface_BasicDenseVectorOperations::multiply(const std::vector<double>& x, std::vector<double>& y)
{
    const size_t num_entries = x.size();
    for(size_t i = 0; i < num_entries; ++i)
    {
        y[i] *= x[i];
    }
}

}
}
