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
