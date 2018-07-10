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

#include "PSL_LinearCongruentialGenerator.hpp"

#include <cassert>

namespace PlatoSubproblemLibrary
{

LinearCongruentialGenerator::LinearCongruentialGenerator() :
        m_modulus(0),
        m_multiplier(0),
        m_increment(0),
        m_current_state(0)
{
}
LinearCongruentialGenerator::~LinearCongruentialGenerator()
{
}
void LinearCongruentialGenerator::build(const unsigned long int& modulus,
                                        const unsigned long int& multiplier,
                                        const unsigned long int& increment,
                                        const unsigned long int& seed)
{
    m_modulus = modulus;
    m_multiplier = multiplier;
    m_increment = increment;
    m_current_state = seed;
}
unsigned long int LinearCongruentialGenerator::get_modulus()
{
    return m_modulus;
}

unsigned long int LinearCongruentialGenerator::generate()
{
    assert(0 < m_modulus);
    m_current_state = (m_increment + ((m_multiplier * m_current_state) % m_modulus)) % m_modulus;
    return m_current_state;
}

LinearCongruentialGenerator psl_global_lcg;
void globalLcg_deterministic_seed()
{
    const unsigned long int modulus = 2147483647;
    const unsigned long int multiplier = 48271;
    const unsigned long int increment = 0;
    const unsigned long int seed = 42;
    psl_global_lcg.build(modulus, multiplier, increment, seed);
}
unsigned long int globalLcg_getRandInt()
{
    return psl_global_lcg.generate();
}
double globalLcg_getRandDouble()
{
    const long double numerator = psl_global_lcg.generate();
    const long double denominator = psl_global_lcg.get_modulus();
    return numerator / denominator;
}

}
