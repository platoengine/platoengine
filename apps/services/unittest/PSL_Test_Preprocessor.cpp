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

#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Interface_MpiWrapper.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_PreprocessorFactory.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_Preprocessor.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_DataSequence.hpp"
#include "PSL_Random.hpp"

#include <mpi.h>
#include <vector>

namespace PlatoSubproblemLibrary
{
namespace TestingPreprocessor
{

PSL_TEST(Preprocessor,buildPreprocessorFromFactory)
{
    set_rand_seed();
    // allocate
    AbstractAuthority authority;
    DataSequence sequence(&authority, data_flow_t::data_flow_t::inert_data_flow);

    // for each type to try
    Preprocessor* preprocessor = NULL;
    const int num_types = preprocessor_t::preprocessor_t::TOTAL_NUM_PREPROCESSORS;
    for(int t = 0; t < num_types; t++)
    {
        const preprocessor_t::preprocessor_t this_type = preprocessor_t::preprocessor_t(t);
        preprocessor = build_preprocessor(this_type, &authority, &sequence);
        EXPECT_EQ(preprocessor->get_preprocessor_type(), this_type);
        safe_free(preprocessor);
    }
}

}
}
