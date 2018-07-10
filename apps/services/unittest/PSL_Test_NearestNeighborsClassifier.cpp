#include "PSL_UnitTestingHelper.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_Classifier.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_BasicSupervisedGenerator.hpp"
#include "PSL_NearestNeighborsClassifier.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_SupervisedGeneratorTester.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_FindDataset.hpp"
#include "PSL_CrossValidationEstimator.hpp"
#include "PSL_ClassificationAssessor.hpp"
#include "PSL_SplitTrainTestEstimator.hpp"
#include "PSL_Random.hpp"

#include <vector>
#include <string>

namespace PlatoSubproblemLibrary
{
namespace TestingNearestNeighborsClassifier
{

PSL_TEST(NearestNeighborsClassifier,basicExample)
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
    parameter_data.defaults_for_nearestNeighborClassifier();

    // test
    const int num_train = 512;
    const int num_test = 199;
    const double testing_accuracy = tester.get_accuracy(&sampler, num_train, &parameter_data, num_test);
    EXPECT_NEAR(testing_accuracy, .96, .041);
}

bool passed_NearestNeighborsClassifier_CrossValidation_iris()
{
    // allocate needed interfaces
    AbstractAuthority authority;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::iris_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_nearestNeighborClassifier();
    const int num_fold = 5;
    const double expected_accuracy = .92;

    // write dataset
    FindDataset finder(&authority);
    const std::string archive_filename = finder.find(dataset);

    // assess
    CrossValidationEstimator estimator(&authority);
    ClassificationAssessor assessor(&authority);
    estimator.initialize(archive_filename, false, num_fold);
    estimator.estimate_accuracy(&parameter_data, &assessor);
    const double actual_accuracy = assessor.get_classification_accuracy();

    return (actual_accuracy >= expected_accuracy);
}
PSL_TEST(NearestNeighborsClassifier, passCrossValidationIris)
{
    set_rand_seed();
    PSL_SerialOnlyTest
    stocastic_test(passed_NearestNeighborsClassifier_CrossValidation_iris, 1, 3, .32);
}

double accuracy_NearestNeighborsClassifier_germanCredit()
{
    // allocate needed interfaces
    AbstractAuthority authority;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::german_credit_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_nearestNeighborClassifier();
    parameter_data.set_num_neighbors(5);
    parameter_data.set_should_select_nearest_neighbors_by_validation(false);
    const double testing_holdout = .1;

    // write dataset
    FindDataset finder(&authority);
    const std::string archive_filename = finder.find(dataset);

    // assess
    SplitTrainTestEstimator estimator(&authority);
    ClassificationAssessor assessor(&authority);
    estimator.initialize(archive_filename, false, testing_holdout);
    estimator.estimate_accuracy(&parameter_data, &assessor);
    const double accuracy = assessor.get_classification_accuracy();

    return accuracy;
}
PSL_TEST(NearestNeighborsClassifier, germanCreditAccuracy)
{
    set_rand_seed();
    PSL_SerialOnlyTest
    stocastic_test(accuracy_NearestNeighborsClassifier_germanCredit, 4, .72, .08);
}

bool improve_NearestNeighborsClassifier_lsvt()
{
    // allocate needed interfaces
    AbstractAuthority authority;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::lsvt_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_nearestNeighborClassifier();
    parameter_data.set_num_neighbors(60);
    const int num_fold = 8;

    // write dataset
    FindDataset finder(&authority);
    const std::string archive_filename = finder.find(dataset);

    // assess with validation
    CrossValidationEstimator estimator_wv(&authority);
    ClassificationAssessor assessor_wv(&authority);
    estimator_wv.initialize(archive_filename, false, num_fold);
    estimator_wv.estimate_accuracy(&parameter_data, &assessor_wv);
    const double accuracy_wv = assessor_wv.get_classification_accuracy();

    // assess without validation
    parameter_data.set_should_select_nearest_neighbors_by_validation(false);
    CrossValidationEstimator estimator_wov(&authority);
    ClassificationAssessor assessor_wov(&authority);
    estimator_wov.initialize(archive_filename, false, num_fold);
    estimator_wov.estimate_accuracy(&parameter_data, &assessor_wov);
    const double accuracy_wov = assessor_wov.get_classification_accuracy();

    // did validation improve?
    const bool did_improve = (accuracy_wov <= accuracy_wv);
    return did_improve;
}
PSL_TEST(NearestNeighborsClassifier, lsvtImprove)
{
    set_rand_seed();
    PSL_SerialOnlyTest

    // ensure dataset is found
    AbstractAuthority authority;
    FindDataset finder(&authority);
    datasets_t::datasets_t dataset = datasets_t::datasets_t::lsvt_dataset;
    const std::string archive_filename = finder.find(dataset);
    if(0u < archive_filename.size())
    {
        stocastic_test(improve_NearestNeighborsClassifier_lsvt, 5, 15, .59);
    }
}

}
}

