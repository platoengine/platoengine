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
 * Plato_SromXML.cpp
 *
 *  Created on: June 18, 2019
 */
#include <iostream>

#include "Plato_SromXML.hpp"
#include "Plato_SromXMLUtils.hpp"

namespace Plato
{

bool generate_load_sroms(const Plato::srom::InputMetaData & aInput, Plato::srom::OutputMetaData & aOutput)
{
    aOutput.mLoadCases.clear();
    std::vector<Plato::srom::Load> tRandomLoads, tDeterministicLoads;
    if(Plato::expand_random_and_deterministic_loads(aInput.mLoads, tRandomLoads, tDeterministicLoads) == false)
    {
        PRINTERR("FAILED TO GENERATE THE SETS OF RANDOM AND DETERMINISTIC LOADS.\n");
        return (false);
    }

    for(size_t tLoadIndex = 0; tLoadIndex < tRandomLoads.size(); tLoadIndex++)
    {
        std::vector<Plato::srom::RandomVariable> tMySampleProbPairs;
        if(Plato::compute_sample_probability_pairs(tRandomLoads[tLoadIndex].mRandomVars, tMySampleProbPairs) == false)
        {
            std::ostringstream tMsg;
            tMsg << "FAILED TO COMPUTE THE SAMPLE-PROBABILITY PAIRS FOR LOAD #" << tLoadIndex << ".\n";
            PRINTERR(tMsg.str().c_str());
            return (false);
        }

        std::vector<Plato::srom::RandomRotations> tMySetRandomRotation;
        if(Plato::generate_set_random_rotations(tMySampleProbPairs, tMySetRandomRotation) == false)
        {
            std::ostringstream tMsg;
            tMsg << "FAILED TO GENERATE SET OF RANDOM ROTATIONS FOR LOAD #" << tLoadIndex << ".\n";
            PRINTERR(tMsg.str().c_str());
            return (false);
        }

        std::vector<Plato::srom::RandomLoad> tMySetRandomLoads;
        if(Plato::generate_set_random_loads(tRandomLoads[tLoadIndex], tMySetRandomRotation, tMySetRandomLoads) == false)
        {
            std::ostringstream tMsg;
            tMsg << "FAILED TO GENERATE SET OF RANDOM LOADS FOR LOAD #" << tLoadIndex << ".\n";
            PRINTERR(tMsg.str().c_str());
            return (false);
        }

        if(Plato::expand_random_load_cases(tMySetRandomLoads, aOutput.mLoadCases) == false)
        {
            std::ostringstream tMsg;
            tMsg << "FAILED TO EXPAND RANDOM LOAD CASES FOR LOAD #" << tLoadIndex << ".\n";
            PRINTERR(tMsg.str().c_str());
            return (false);
        }
    }

    if(Plato::generate_output_random_load_cases(tDeterministicLoads, aOutput.mLoadCases) == false)
    {
        PRINTERR("FAILED TO GENERATE SET OF OUTPUT RANDOM LOAD CASES.\n");
        return (false);
    }

    return(true);
}

} // namespace Plato

