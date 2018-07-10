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

#include "PSL_RadixHashByPrime.hpp"

#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <cstddef>
#include <cassert>

namespace PlatoSubproblemLibrary
{

RadixHashByPrime::RadixHashByPrime(AbstractAuthority* authority) :
        m_authority(authority),
        m_radix_multipliers(),
        m_hash_size()
{
}
RadixHashByPrime::~RadixHashByPrime()
{
}

void RadixHashByPrime::initialize(const int& maximum_hash_size, const std::vector<int>& radix_sizes)
{
    const int length = radix_sizes.size();

    m_hash_size = maximum_hash_size;
    while(!is_prime(m_hash_size))
    {
        m_hash_size--;
    }

    // compute multipliers
    m_radix_multipliers.assign(length, 1);
    for(int i = 0; i + 1 < length; i++)
    {
        m_radix_multipliers[i + 1] = (m_radix_multipliers[i] * radix_sizes[i]) % maximum_hash_size;
    }
}

int RadixHashByPrime::get_hash_size()
{
    return m_hash_size;
}

int RadixHashByPrime::hash(const std::vector<int>& this_radix)
{
    const int length = m_radix_multipliers.size();
    assert(length == int(this_radix.size()));

    // hash it up
    int result = 0;
    for(int i = 0; i < length; i++)
    {
        result = (result + this_radix[i] * m_radix_multipliers[i]) % m_hash_size;
    }
    return result;
}

}
