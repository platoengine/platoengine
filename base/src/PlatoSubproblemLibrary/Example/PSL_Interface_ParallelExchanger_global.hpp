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

// PlatoSubproblemLibraryVersion(6): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Interface_ParallelExchanger_localAndNonlocal.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class ParallelVector;
}
class AbstractAuthority;

namespace example
{

class Interface_ParallelExchanger_global : public Interface_ParallelExchanger_localAndNonlocal
{
public:
    Interface_ParallelExchanger_global(AbstractAuthority* authority);
    virtual ~Interface_ParallelExchanger_global();

    // put global identifiers for local indexes
    void put_globals(const std::vector<size_t>& globals);

    virtual void build();

protected:

    void build_send_first(size_t num_local,
                          size_t proc,
                          const std::vector<int>& global_minimum_indexes,
                          const std::vector<int>& global_maximum_indexes,
                          const std::vector<std::pair<size_t, size_t> >& global_and_local_pair,
                          std::vector<std::vector<std::pair<size_t, size_t> > >& shared_local_and_nonlocal_pairs);
    void build_receive_first(size_t num_local,
                             size_t proc,
                             const std::vector<std::pair<size_t, size_t> >& global_and_local_pair,
                             std::vector<std::vector<std::pair<size_t, size_t> > >& shared_local_and_nonlocal_pairs);

    bool intersect(int a_lower, int a_upper, int b_lower, int b_upper);

    // received data
    std::vector<size_t> m_globals;

};

}
}
