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

#include "PSL_UnitTestingHelper.hpp"

#include "PSL_DataSequence.hpp"
#include "PSL_DataOrganizerFactory.hpp"
#include "PSL_Interface_BasicDenseVectorOperations.hpp"
#include "PSL_Interface_DenseMatrixBuilder.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Interface_MpiWrapper.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_DataFlow.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_Random.hpp"

#include <mpi.h>
#include <vector>

namespace PlatoSubproblemLibrary
{
namespace TestingDataOrganizer
{

PSL_TEST(DataOrganizer, buildDataOrganizerFromFactory)
{
    set_rand_seed();
    // allocate
    AbstractAuthority authority;

    // for each type to try
    DataFlow* flow = NULL;
    const int num_types = data_flow_t::data_flow_t::TOTAL_NUM_DATA_FLOWS;
    for(int t = 0; t < num_types; t++)
    {
        const data_flow_t::data_flow_t this_type = data_flow_t::data_flow_t(t);
        flow = build_data_flow(this_type, &authority);
        EXPECT_EQ(flow->get_data_sequence()->get_data_flow_type(), this_type);
        safe_free(flow);
    }
}

}
}
