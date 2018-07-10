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

#include "PSL_DistanceVoter.hpp"
#include "PSL_AbstractAuthority.hpp"

#include "PSL_ParameterDataEnums.hpp"

namespace PlatoSubproblemLibrary
{

DistanceVoter::DistanceVoter(AbstractAuthority* authority, const distance_voter_t::distance_voter_t& type) :
        m_authority(authority),
        m_type(type)
{
}
DistanceVoter::~DistanceVoter()
{
}
distance_voter_t::distance_voter_t DistanceVoter::get_distance_voter_type()
{
    return m_type;
}

int DistanceVoter::choose_output(const int& output_class_size,
                                 const std::vector<double>& distances_per_instance,
                                 const std::vector<int>& classes_per_instance)
{
    const int num_input = distances_per_instance.size();
    assert(num_input == int(classes_per_instance.size()));

    // count each class
    std::vector<int> class_count(output_class_size, 0);
    for(int i = 0; i < num_input; i++)
    {
        assert(0 <= classes_per_instance[i]);
        assert(classes_per_instance[i] < output_class_size);
        class_count[classes_per_instance[i]]++;
    }

    return choose_output(class_count, distances_per_instance, classes_per_instance);
}
int DistanceVoter::choose_output(const std::vector<int>& instances_per_class,
                                         const std::vector<double>& distances_per_instance,
                                         const std::vector<int>& classes_per_instance)
{
    return choose_output(instances_per_class.size(), distances_per_instance, classes_per_instance);
}

}
