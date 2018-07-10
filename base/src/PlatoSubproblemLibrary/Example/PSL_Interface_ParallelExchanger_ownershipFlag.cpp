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
#include "PSL_Interface_ParallelExchanger_ownershipFlag.hpp"

#include "PSL_Abstract_ParallelExchanger_Managed.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace example
{

Interface_ParallelExchanger_ownershipFlag::Interface_ParallelExchanger_ownershipFlag(AbstractAuthority* authority) :
        AbstractInterface::ParallelExchanger_Managed(authority),
        m_is_owned()
{
}

Interface_ParallelExchanger_ownershipFlag::~Interface_ParallelExchanger_ownershipFlag()
{
    m_is_owned.clear();
}

void Interface_ParallelExchanger_ownershipFlag::put_is_owned(const std::vector<bool>& is_owned)
{
    m_is_owned = is_owned;
}

void Interface_ParallelExchanger_ownershipFlag::build()
{
    const size_t num_locations = m_is_owned.size();

    // count local contracted
    size_t num_contracted = 0;
    for(size_t local_index = 0u; local_index < num_locations; local_index++)
    {
        num_contracted += m_is_owned[local_index];
    }

    // allocate
    m_contracted_to_local.resize(num_contracted, 0u);

    // fill
    size_t contracted_counter = 0;
    for(size_t local_index = 0u; local_index < num_locations; local_index++)
    {
        if(m_is_owned[local_index])
        {
            m_contracted_to_local[contracted_counter++] = local_index;
        }
    }

    // no communication
    size_t size = m_authority->mpi_wrapper->get_size();
    m_local_index_to_send.resize(size);
    m_local_index_to_recv.resize(size);
}

}
}
