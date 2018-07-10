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
