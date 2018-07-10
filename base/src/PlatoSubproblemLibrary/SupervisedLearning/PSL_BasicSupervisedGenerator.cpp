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

#include "PSL_BasicSupervisedGenerator.hpp"

#include "PSL_Random.hpp"
#include "PSL_SupervisedGenerator.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{

BasicSupervisedGenerator::BasicSupervisedGenerator(const double& output_lower_transition, const double& output_upper_transition) :
        SupervisedGenerator(),
        m_output_lower_transition(output_lower_transition),
        m_output_upper_transition(output_upper_transition)
{
}

BasicSupervisedGenerator::~BasicSupervisedGenerator()
{
}

void BasicSupervisedGenerator::get_sizes(int& output_enum_size, int& num_input_double_scalars, std::vector<int>& input_enum_sizes)
{
    output_enum_size = 2;
    num_input_double_scalars = 1;
    input_enum_sizes.assign(1u, 2);
}

void BasicSupervisedGenerator::generate(std::vector<double>& input_double_scalars,
                                        std::vector<int>& input_enums,
                                        int& actual_output_enum)
{
    // define ranges
    const double sep = m_output_upper_transition - m_output_lower_transition;
    const double lower_range = m_output_lower_transition - 2. * sep;
    const double upper_range = m_output_upper_transition + 1. * sep;

    // sample
    const double scalar = uniform_rand_double(lower_range, upper_range);
    input_double_scalars.assign(1u, scalar);
    const int input_enum = rand_int(0, 2);
    input_enums.assign(1u, input_enum);

    // determine actual output
    const bool below_lower = (scalar < m_output_lower_transition);
    const bool above_upper = (m_output_upper_transition < scalar);
    if((input_enum == 1) && (below_lower || above_upper))
    {
        actual_output_enum = 1;
    }
    else
    {
        actual_output_enum = 0;
    }
}

}
