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

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_DecisionMetric.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_GiniImpurityMetric.hpp"
#include "PSL_EntropyMetric.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_DecisionMetricFactory.hpp"
#include "PSL_Classifier.hpp"
#include "PSL_DecisionTreeNode.hpp"
#include "PSL_DecisionMetricFactory.hpp"
#include "PSL_RandomForest.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_BasicSupervisedGenerator.hpp"
#include "PSL_SupervisedGeneratorTester.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_CrossValidationErrorDiscreteObjective.hpp"
#include "PSL_IterativeSelection.hpp"
#include "PSL_Random.hpp"

#include <vector>
#include <string>

namespace PlatoSubproblemLibrary
{
namespace TestingRandomForest
{

PSL_TEST(RandomForest,basicExample)
{
    set_rand_seed();
    PSL_SerialOnlyTest;
    AbstractAuthority authority;

    // sampler
    BasicSupervisedGenerator sampler(.01, 5.1);

    // tester
    SupervisedGeneratorTester tester(&authority);

    // parameters for training
    ParameterData parameter_data;
    parameter_data.defaults_for_randomForest();
    parameter_data.set_requested_num_trees(5);

    // test
    const int num_train = 256;
    const int num_test = 99;
    const double testing_accuracy = tester.get_accuracy(&sampler, num_train, &parameter_data, num_test);
    EXPECT_NEAR(testing_accuracy, .97, .041);
}

class RFErrorObjective : public CrossValidationErrorDiscreteObjective
{
public:
    RFErrorObjective(AbstractAuthority* authority, const std::vector<datasets_t::datasets_t>& datasets) :
            CrossValidationErrorDiscreteObjective(authority, datasets)
    {
    }
    virtual ~RFErrorObjective()
    {
    }

    virtual void get_domain(std::vector<double>& inclusive_lower,
                            std::vector<double>& inclusive_upper,
                            std::vector<int>& num_values)
    {
        //                 0   1   2   3    4   5   6
        inclusive_lower = {.4, .3, .1, .0,  1., 1., 0.};
        inclusive_upper = {1., 1., 1., .08, 3., 3., 2.5};
        num_values =      {7,  8,  9,  5,   3,  3,  6};
    }

protected:
    virtual void get_parameters(const std::vector<double>& parameters, ParameterData* parameter_data)
    {
        assert(parameters.size() == 7u);
        parameter_data->defaults_for_randomForest();

        // parameter 0
        parameter_data->set_dataset_bagging_fraction(parameters[0]);
        // parameter 1
        parameter_data->set_feature_subspace_power(parameters[1]);
        // parameter 2
        parameter_data->set_feature_subspace_scale(parameters[2]);
        // parameter 3
        parameter_data->set_scaled_stop_splitting_metric_value(parameters[3]);
        // parameter 4
        parameter_data->set_minimum_leaf_size(std::pow(2., parameters[4]));
        // parameter 5
        parameter_data->set_minimum_features_considered(std::pow(2., parameters[5]));
        // parameter 6
        parameter_data->set_max_tree_depth_per_total_features(parameters[6]);
    }

    virtual double compute_result(const std::vector<double>& results)
    {
        return sum(results);
    }

};
PSL_TEST(RandomForest, errorMinimization)
{
    set_rand_seed();
    return;
    PSL_SerialOnlyTest

    // allocate
    AbstractAuthority authority;

    // define objective
    /* std::vector<datasets_t::datasets_t> datasets = {datasets_t::datasets_t::iris_dataset,
                                                    datasets_t::datasets_t::pop_failures_dataset,
                                                    datasets_t::datasets_t::nicotine_dataset,
                                                    datasets_t::datasets_t::lsvt_dataset,
                                                    datasets_t::datasets_t::frogs_MFCCs_dataset,
                                                    datasets_t::datasets_t::biodeg_dataset,
                                                    datasets_t::datasets_t::wilt_dataset}; */
    std::vector<datasets_t::datasets_t> datasets = {datasets_t::datasets_t::pop_failures_dataset,
                                                    datasets_t::datasets_t::nicotine_dataset,
                                                    datasets_t::datasets_t::lsvt_dataset,
                                                    datasets_t::datasets_t::biodeg_dataset,
                                                    datasets_t::datasets_t::waveform_dataset,
                                                    datasets_t::datasets_t::wine_quality_dataset,
                                                    datasets_t::datasets_t::cervical_cancer_dataset};
    RFErrorObjective obj(&authority, datasets);

    // define searcher
    IterativeSelection optimizer(&authority);
    optimizer.set_verbose(true);
    optimizer.set_objective(&obj);

    // find a minimum
    std::vector<double> best_parameters;
    const double found_min = optimizer.find_min(best_parameters);
    EXPECT_GT(found_min, .0);

    // report min
    best_parameters.push_back(found_min);
    authority.utilities->print(best_parameters, true);
}

}
}
