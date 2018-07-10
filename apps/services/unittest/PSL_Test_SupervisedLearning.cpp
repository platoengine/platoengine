#include "PSL_UnitTestingHelper.hpp"

#include "PSL_ClassificationArchive.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Interface_DenseMatrixBuilder.hpp"
#include "PSL_FindDataset.hpp"
#include "PSL_ClassificationAccuracyEstimator.hpp"
#include "PSL_CrossValidationEstimator.hpp"
#include "PSL_SplitTrainTestEstimator.hpp"
#include "PSL_FreeStandardization.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_ClassificationAssessor.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_Random.hpp"

#include <string>
#include <iostream>
#include <cmath>
#include <math.h>
#include <utility>

namespace PlatoSubproblemLibrary
{
namespace TestingSupervisedLearning
{

void get_testCrossValidation(const datasets_t::datasets_t& dataset, ParameterData& parameter_data, double& accuracy, double& time)
{
    //        return;
    PSL_SerialOnlyTest
    // allocate needed interfaces
    AbstractAuthority authority;

    const int num_fold = 10;

    // write dataset
    FindDataset finder(&authority);
    const std::string archive_filename = finder.find(dataset);

    // allocate for assess
    CrossValidationEstimator estimator(&authority);
    ClassificationAssessor assessor(&authority);
    estimator.initialize(archive_filename, false, num_fold);

    // assess
    time = -authority.mpi_wrapper->get_time();
    estimator.estimate_accuracy(&parameter_data, &assessor);
    time += authority.mpi_wrapper->get_time();
    accuracy = assessor.get_classification_accuracy();
}
void test_testCrossValidation(const datasets_t::datasets_t& dataset,
                              ParameterData& parameter_data,
                              const double expected_accuracy,
                              const double accuracy_tolerance)
{
    double actual_accuracy = -1.;
    double time = -1.;
    get_testCrossValidation(dataset, parameter_data, actual_accuracy, time);
    std::cout << "accuracy fraction=" << actual_accuracy << "; time=" << time << std::endl;
    EXPECT_NEAR(actual_accuracy, expected_accuracy, accuracy_tolerance);
}

PSL_TEST(SupervisedLearning,makeTable)
{
    set_rand_seed();
    return;

    // set datasets table
    const int num_repeat = 8;
#define PSL_SupervisedLearningMakeTable_Tokenize(_name) #_name
#define PSL_SupervisedLearningMakeTable_VectorizeDataset(_name) \
        std::make_pair(datasets_t::datasets_t::_name##_dataset,PSL_SupervisedLearningMakeTable_Tokenize(_name))
    std::vector<std::pair<datasets_t::datasets_t, std::string> > dataset_and_names = {
    PSL_SupervisedLearningMakeTable_VectorizeDataset(pop_failures),
    PSL_SupervisedLearningMakeTable_VectorizeDataset(online_news),
    PSL_SupervisedLearningMakeTable_VectorizeDataset(nicotine),
    PSL_SupervisedLearningMakeTable_VectorizeDataset(lsvt),
    PSL_SupervisedLearningMakeTable_VectorizeDataset(htru2),
    PSL_SupervisedLearningMakeTable_VectorizeDataset(frogs_MFCCs),
    PSL_SupervisedLearningMakeTable_VectorizeDataset(dota2_test),
    PSL_SupervisedLearningMakeTable_VectorizeDataset(default_credit),
    PSL_SupervisedLearningMakeTable_VectorizeDataset(biodeg),
    PSL_SupervisedLearningMakeTable_VectorizeDataset(wilt),
    PSL_SupervisedLearningMakeTable_VectorizeDataset(waveform),
    PSL_SupervisedLearningMakeTable_VectorizeDataset(wine_quality),
    PSL_SupervisedLearningMakeTable_VectorizeDataset(cervical_cancer),
    PSL_SupervisedLearningMakeTable_VectorizeDataset(connect_4),
    PSL_SupervisedLearningMakeTable_VectorizeDataset(pen_digits),
    PSL_SupervisedLearningMakeTable_VectorizeDataset(seizure),
    PSL_SupervisedLearningMakeTable_VectorizeDataset(sensorless_drive)
    };
    const int num_datasets = dataset_and_names.size();

    // set classifiers
    const int num_classifiers = 4;
    std::vector<std::pair<ParameterData, std::string> > parameters_and_names(num_classifiers);
    parameters_and_names[0].first.defaults_for_feedForwardNeuralNetwork();
    parameters_and_names[0].second = "FFNN";
    parameters_and_names[1].first.defaults_for_supportVectorMachine();
    parameters_and_names[1].second = "SVM";
    parameters_and_names[2].first.defaults_for_randomForest();
    parameters_and_names[2].second = "RF";
    parameters_and_names[3].first.defaults_for_nearestNeighborClassifier();
    parameters_and_names[3].second = "kNN";

    // allocate
    std::vector<std::vector<std::pair<std::vector<double>,std::vector<double> > > > dataset_classifier_and_accuracy_then_time(
    num_datasets, std::vector<std::pair<std::vector<double>,std::vector<double> > >(
    num_classifiers, std::pair<std::vector<double>,std::vector<double> >(
    std::vector<double>(num_repeat, -500.), std::vector<double>(num_repeat, -500.))));
    assert(dataset_classifier_and_accuracy_then_time.size() == size_t(num_datasets));

    // for verbose percent complete printing
    double run_counter = 0;
    double expected_runs = num_repeat * num_datasets * num_classifiers;

    // fill, for each repeat
    for(int repeat = 0; repeat < num_repeat; repeat++)
    {
        // for each dataset
        std::vector<int> dataset_order;
        random_permutation(num_datasets, dataset_order);
        assert(dataset_order.size() == size_t(num_datasets));
        for(int abstract_d = 0; abstract_d < num_datasets; abstract_d++)
        {
            const int d = dataset_order[abstract_d];
            assert(0 <= d);
            assert(d < num_datasets);
            assert(dataset_classifier_and_accuracy_then_time[d].size() == size_t(num_classifiers));

            // for each classifier
            std::vector<int> classifier_order;
            random_permutation(num_classifiers, classifier_order);
            assert(classifier_order.size() == size_t(num_classifiers));
            for(int abstract_c = 0; abstract_c < num_classifiers; abstract_c++)
            {
                const int c = classifier_order[abstract_c];
                assert(0 <= c);
                assert(c < num_classifiers);

                // print percent complete
                std::cout << 100. * (run_counter++) / expected_runs << "%..." << std::endl;

                assert(dataset_classifier_and_accuracy_then_time[d][c].first.size() == size_t(num_repeat));
                assert(dataset_classifier_and_accuracy_then_time[d][c].second.size() == size_t(num_repeat));
                get_testCrossValidation(dataset_and_names[d].first,
                                        parameters_and_names[c].first,
                                        dataset_classifier_and_accuracy_then_time[d][c].first[repeat],
                                        dataset_classifier_and_accuracy_then_time[d][c].second[repeat]);
            }
        }
    }

    // print
    std::cout << ",";
    for(int c = 0; c < num_classifiers; c++)
    {
        for(int r = 0; r < 4; r++)
        {
            std::cout << parameters_and_names[c].second << ",";
        }
    }
    std::cout << std::endl;
    for(int d = 0; d < num_datasets; d++)
    {
        std::cout << dataset_and_names[d].second << ",";
        for(int c = 0; c < num_classifiers; c++)
        {
            double accuracy_mean = -5.;
            double accuracy_std = -5.;
            get_mean_and_std(dataset_classifier_and_accuracy_then_time[d][c].first, accuracy_mean, accuracy_std);
            double time_mean = -5.;
            double time_std = -5.;
            get_mean_and_std(dataset_classifier_and_accuracy_then_time[d][c].second, time_mean, time_std);
            std::cout << accuracy_mean << "," << accuracy_std << "," << time_mean << "," << time_std << ",";
        }
        std::cout << std::endl;
    }
}

PSL_TEST(SupervisedLearning,TestCrossValidation_popFailures_FFNN)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::pop_failures_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(4);
    parameter_data.set_validation_holdout(.05);
    const double expected_accuracy = .90;
    const double accuracy_tolerance = .1;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_popFailures_SVM)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::pop_failures_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    const double expected_accuracy = .90;
    const double accuracy_tolerance = .1;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_popFailures_RF)
{
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::pop_failures_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_randomForest();
    const double expected_accuracy = .90;
    const double accuracy_tolerance = .1;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_nicotine_FFNN)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::nicotine_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(4);
    parameter_data.set_validation_holdout(.05);
    const double expected_accuracy = .45;
    const double accuracy_tolerance = .1;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_nicotine_SVM)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::nicotine_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    const double expected_accuracy = .42;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_nicotine_RF)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::nicotine_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_randomForest();
    const double expected_accuracy = .45;
    const double accuracy_tolerance = .1;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_nicotine_kNN)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::nicotine_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_nearestNeighborClassifier();
    parameter_data.set_num_neighbors(25);
    parameter_data.set_should_select_nearest_neighbors_by_validation(true);
    const double expected_accuracy = .41;
    const double accuracy_tolerance = .1;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_lsvt_FFNN)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::lsvt_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(4);
    parameter_data.set_validation_holdout(.02);
    const double expected_accuracy = .80;
    const double accuracy_tolerance = .1;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_lsvt_SVM)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::lsvt_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    const double expected_accuracy = .80;
    const double accuracy_tolerance = .1;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_lsvt_RF)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::lsvt_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_randomForest();
    const double expected_accuracy = .80;
    const double accuracy_tolerance = .1;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_frogsMFCCs_FFNN)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::frogs_MFCCs_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(4);
    parameter_data.set_validation_holdout(.05);
    const double expected_accuracy = .90;
    const double accuracy_tolerance = .1;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_frogsMFCCs_SVM)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::frogs_MFCCs_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    const double expected_accuracy = .95;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_frogsMFCCs_RF)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::frogs_MFCCs_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_randomForest();
    const double expected_accuracy = .95;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_biodeg_FFNN)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::biodeg_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(4);
    parameter_data.set_validation_holdout(.04);
    const double expected_accuracy = .85;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_biodeg_SVM)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::biodeg_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    const double expected_accuracy = .85;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_biodeg_RF)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::biodeg_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_randomForest();
    const double expected_accuracy = .85;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_biodeg_kNN)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::biodeg_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_nearestNeighborClassifier();
    parameter_data.set_num_neighbors(25);
    parameter_data.set_should_select_nearest_neighbors_by_validation(true);
    const double expected_accuracy = .87;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_wilt_FFNN)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::wilt_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(4);
    parameter_data.set_validation_holdout(.04);
    const double expected_accuracy = .95;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_wilt_SVM)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::wilt_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    const double expected_accuracy = .97;
    const double accuracy_tolerance = .03;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_wilt_RF)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::wilt_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_randomForest();
    const double expected_accuracy = .97;
    const double accuracy_tolerance = .03;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_onlineNews_FFNN)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::online_news_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(4);
    parameter_data.set_validation_holdout(.04);
    const double expected_accuracy = .76;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_onlineNews_SVM)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::online_news_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    const double expected_accuracy = .76;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_onlineNews_RF)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::online_news_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_randomForest();
    const double expected_accuracy = .77;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_htru2_FFNN)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::htru2_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(4);
    parameter_data.set_validation_holdout(.04);
    const double expected_accuracy = .95;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_htru2_SVM)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::htru2_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    const double expected_accuracy = .96;
    const double accuracy_tolerance = .04;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_htru2_RF)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::htru2_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_randomForest();
    const double expected_accuracy = .96;
    const double accuracy_tolerance = .04;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_dota2Test_FFNN)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::dota2_test_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(4);
    parameter_data.set_validation_holdout(.04);
    const double expected_accuracy = 0.55;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_dota2Test_SVM)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::dota2_test_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    const double expected_accuracy = .55;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_dota2Test_RF)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::dota2_test_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_randomForest();
    const double expected_accuracy = .55;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_defaultCredit_FFNN)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::default_credit_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(4);
    parameter_data.set_validation_holdout(.04);
    const double expected_accuracy = .82;
    const double accuracy_tolerance = .1;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_defaultCredit_SVM)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::default_credit_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    const double expected_accuracy = .78;
    const double accuracy_tolerance = .1;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_defaultCredit_RF)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::default_credit_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_randomForest();
    const double expected_accuracy = .78;
    const double accuracy_tolerance = .1;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_waveform_FFNN)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::waveform_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(4);
    parameter_data.set_validation_holdout(.04);
    const double expected_accuracy = .85;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_waveform_SVM)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::waveform_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    const double expected_accuracy = .85;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_waveform_RF)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::waveform_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_randomForest();
    const double expected_accuracy = .85;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_wineQuality_FFNN)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::wine_quality_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(4);
    parameter_data.set_validation_holdout(.04);
    const double expected_accuracy = .55;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_wineQuality_SVM)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::wine_quality_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    const double expected_accuracy = .55;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_wineQuality_RF)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::wine_quality_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_randomForest();
    const double expected_accuracy = .64;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_legacyMLTR_FFNN)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::legacy_mltr_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(4);
    parameter_data.set_validation_holdout(.04);
    const double expected_accuracy = .90;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_legacyMLTR_SVM)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::legacy_mltr_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    const double expected_accuracy = .85;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_legacyMLTR_RF)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::legacy_mltr_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_randomForest();
    const double expected_accuracy = .85;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_cervicalCancer_FFNN)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::cervical_cancer_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(4);
    parameter_data.set_validation_holdout(.04);
    const double expected_accuracy = .76;
    const double accuracy_tolerance = .06;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_cervicalCancer_SVM)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::cervical_cancer_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    const double expected_accuracy = .76;
    const double accuracy_tolerance = .06;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_cervicalCancer_RF)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::cervical_cancer_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_randomForest();
    const double expected_accuracy = .76;
    const double accuracy_tolerance = .06;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_connect4_FFNN)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::connect_4_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(4);
    parameter_data.set_validation_holdout(.04);
    const double expected_accuracy = .78;
    const double accuracy_tolerance = .06;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_connect4_SVM)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::connect_4_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    const double expected_accuracy = .74;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_connect4_RF)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::connect_4_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_randomForest();
    const double expected_accuracy = .83;
    const double accuracy_tolerance = .06;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_pendigits_FFNN)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::pen_digits_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(4);
    parameter_data.set_validation_holdout(.04);
    const double expected_accuracy = .93;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_pendigits_SVM)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::pen_digits_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    const double expected_accuracy = .98;
    const double accuracy_tolerance = .03;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_pendigits_RF)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::pen_digits_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_randomForest();
    const double expected_accuracy = .98;
    const double accuracy_tolerance = .03;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_seizure_FFNN)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::seizure_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(4);
    parameter_data.set_validation_holdout(.04);
    const double expected_accuracy = .82;
    const double accuracy_tolerance = .08;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_seizure_SVM)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::seizure_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    const double expected_accuracy = .95;
    const double accuracy_tolerance = .06;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_seizure_RF)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::seizure_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_randomForest();
    const double expected_accuracy = .97;
    const double accuracy_tolerance = .04;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_sensorlessDrive_FFNN)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::sensorless_drive_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(4);
    parameter_data.set_validation_holdout(.04);
    const double expected_accuracy = .94;
    const double accuracy_tolerance = .05;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_sensorlessDrive_SVM)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::sensorless_drive_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    const double expected_accuracy = .84;
    const double accuracy_tolerance = .06;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}
PSL_TEST(SupervisedLearning,TestCrossValidation_sensorlessDrive_RF)
{
    set_rand_seed();
    return;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::sensorless_drive_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_randomForest();
    const double expected_accuracy = .99;
    const double accuracy_tolerance = .07;

    test_testCrossValidation(dataset, parameter_data, expected_accuracy, accuracy_tolerance);
}

}
}
