#include "PSL_UnitTestingHelper.hpp"

#include "PSL_FreeHelpers.hpp"
#include "PSL_GradientCheck.hpp"
#include "PSL_FindDataset.hpp"
#include "PSL_ClassificationArchive.hpp"
#include "PSL_Interface_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_SupportVectorMachine.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_SplitTrainTestEstimator.hpp"
#include "PSL_CrossValidationEstimator.hpp"
#include "PSL_DiscreteObjective.hpp"
#include "PSL_IterativeSelection.hpp"
#include "PSL_ClassificationAccuracyEstimator.hpp"
#include "PSL_CrossValidationErrorDiscreteObjective.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_ClassificationAssessor.hpp"
#include "PSL_InputOutputManager.hpp"
#include "PSL_Random.hpp"

#include <string>
#include <iostream>
#include <cmath>
#include <math.h>

namespace PlatoSubproblemLibrary
{
namespace TestingSupportVectorMachine
{

class SVMErrorObjective : public CrossValidationErrorDiscreteObjective
{
public:
    SVMErrorObjective(AbstractAuthority* authority,
                      const std::vector<datasets_t::datasets_t>& datasets) :
            CrossValidationErrorDiscreteObjective(authority, datasets)
    {
    }
    virtual ~SVMErrorObjective()
    {
    }

    virtual void get_domain(std::vector<double>& inclusive_lower,
                            std::vector<double>& inclusive_upper,
                            std::vector<int>& num_values)
    {
        inclusive_lower = {.05, 0., 0., 0.};
        inclusive_upper = {.25, 7., 1., 5.};
        num_values =      {9,   8,  8,  6};
    }

protected:
    virtual void get_parameters(const std::vector<double>& parameters, ParameterData* parameter_data)
    {
        parameter_data->defaults_for_supportVectorMachine();

        // parameter 0
        parameter_data->set_empirical_risk(parameters[0]);
        // parameter 1
        switch(int(round(parameters[1])))
        {
            case 0:
            {
                parameter_data->set_preprocessor(preprocessor_t::preprocessor_t::standardization_preprocessor);
                break;
            }
            case 1:
            {
                parameter_data->set_preprocessor(preprocessor_t::preprocessor_t::bipolar_normalization_preprocessor);
                break;
            }
            case 2:
            {
                parameter_data->set_preprocessor(preprocessor_t::preprocessor_t::skew_normalization_preprocessor);
                break;
            }
            case 3:
            {
                parameter_data->set_preprocessor(preprocessor_t::preprocessor_t::PCA_preprocessor);
                break;
            }
            case 4:
            {
                std::vector<preprocessor_t::preprocessor_t> types =
                {
                    preprocessor_t::preprocessor_t::standardization_preprocessor,
                    preprocessor_t::preprocessor_t::PCA_preprocessor
                };
                parameter_data->set_preprocessor(types);
                break;
            }
            case 5:
            {
                std::vector<preprocessor_t::preprocessor_t> types =
                {
                    preprocessor_t::preprocessor_t::bipolar_normalization_preprocessor,
                    preprocessor_t::preprocessor_t::PCA_preprocessor
                };
                parameter_data->set_preprocessor(types);
                break;
            }
            case 6:
            {
                std::vector<preprocessor_t::preprocessor_t> types =
                {
                    preprocessor_t::preprocessor_t::skew_normalization_preprocessor,
                    preprocessor_t::preprocessor_t::PCA_preprocessor
                };
                parameter_data->set_preprocessor(types);
                break;
            }
            case 7:
            {
                std::vector<preprocessor_t::preprocessor_t> types =
                {
                    preprocessor_t::preprocessor_t::skew_normalization_preprocessor,
                    preprocessor_t::preprocessor_t::bipolar_normalization_preprocessor
                };
                parameter_data->set_preprocessor(types);
                break;
            }
            default:
            {
                m_authority->utilities->fatal_error("unmatched parameter\n\n");
                break;
            }
        }
        // parameter 2
        parameter_data->set_snapshot_experience_rewind(parameters[2]);
        // parameter 3
        parameter_data->set_PCA_upper_variation_fraction(1.-(std::pow(2., parameters[3])/100.));
    }

    virtual double compute_result(const std::vector<double>& results)
    {
        return sum(results);
    }

};

PSL_TEST(SupportVectorMachine, errorMinimization)
{
    set_rand_seed();
    return;
    PSL_SerialOnlyTest

    // allocate
    AbstractAuthority authority;

    // define objective
    /*std::vector<datasets_t::datasets_t> datasets = {datasets_t::datasets_t::iris_dataset,
                                                    datasets_t::datasets_t::german_credit_dataset,
                                                    datasets_t::datasets_t::pop_failures_dataset,
                                                    datasets_t::datasets_t::nicotine_dataset,
                                                    datasets_t::datasets_t::lsvt_dataset,
                                                    datasets_t::datasets_t::frogs_MFCCs_dataset,
                                                    datasets_t::datasets_t::biodeg_dataset,
                                                    datasets_t::datasets_t::wilt_dataset};*/
    std::vector<datasets_t::datasets_t> datasets = {datasets_t::datasets_t::iris_dataset,
                                                    datasets_t::datasets_t::german_credit_dataset,
                                                    datasets_t::datasets_t::pop_failures_dataset,
                                                    datasets_t::datasets_t::nicotine_dataset,
                                                    datasets_t::datasets_t::lsvt_dataset,
                                                    datasets_t::datasets_t::biodeg_dataset,
                                                    datasets_t::datasets_t::waveform_dataset,
                                                    datasets_t::datasets_t::wine_quality_dataset,
                                                    datasets_t::datasets_t::cervical_cancer_dataset};
    SVMErrorObjective obj(&authority, datasets);

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

bool passed_SupportVectorMachine_Split_iris()
{
    // allocate needed interfaces
    AbstractAuthority authority;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::iris_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    const double testing_holdout = .2;
    const double expected_accuracy = .9;

    // write dataset
    FindDataset finder(&authority);
    const std::string archive_filename = finder.find(dataset);

    // assess
    SplitTrainTestEstimator estimator(&authority);
    ClassificationAssessor assessor(&authority);
    estimator.initialize(archive_filename, false, testing_holdout);
    estimator.estimate_accuracy(&parameter_data, &assessor);
    const double actual_accuracy = assessor.get_classification_accuracy();

    return (actual_accuracy >= expected_accuracy);
}
PSL_TEST(SupportVectorMachine, passSplitIris)
{
    set_rand_seed();
    PSL_SerialOnlyTest
    stocastic_test(passed_SupportVectorMachine_Split_iris, 3, 10, .79);
}

bool passed_SupportVectorMachine_CrossValidation_iris()
{
    // allocate needed interfaces
    AbstractAuthority authority;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::iris_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
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
PSL_TEST(SupportVectorMachine, passCrossValidationIris)
{
    set_rand_seed();
    PSL_SerialOnlyTest
    stocastic_test(passed_SupportVectorMachine_CrossValidation_iris, 2, 8, .74);
}

double accuracy_SupportVectorMachine_germanCredit(const preprocessor_t::preprocessor_t& preprocessor)
{
    // allocate needed interfaces
    AbstractAuthority authority;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::german_credit_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    parameter_data.set_preprocessor(preprocessor);
    const double testing_holdout = .2;

    // write dataset
    FindDataset finder(&authority);
    const std::string archive_filename = finder.find(dataset);

    // assess
    SplitTrainTestEstimator estimator(&authority);
    ClassificationAssessor assessor(&authority);
    estimator.initialize(archive_filename, false, testing_holdout);
    estimator.estimate_accuracy(&parameter_data, &assessor);
    const double classacc = assessor.get_classification_accuracy();
    return classacc;
}
double accuracy_SupportVectorMachine_germanCredit_standardization()
{
    return accuracy_SupportVectorMachine_germanCredit(preprocessor_t::preprocessor_t::standardization_preprocessor);
}
double accuracy_SupportVectorMachine_germanCredit_bipolarNormalization()
{
    return accuracy_SupportVectorMachine_germanCredit(preprocessor_t::preprocessor_t::bipolar_normalization_preprocessor);
}
double accuracy_SupportVectorMachine_germanCredit_skewNormalization()
{
    return accuracy_SupportVectorMachine_germanCredit(preprocessor_t::preprocessor_t::skew_normalization_preprocessor);
}
double accuracy_SupportVectorMachine_germanCredit_PCA()
{
    return accuracy_SupportVectorMachine_germanCredit(preprocessor_t::preprocessor_t::PCA_preprocessor);
}
PSL_TEST(SupportVectorMachine, germanCreditAccuracy_standardization)
{
    set_rand_seed();
    PSL_SerialOnlyTest
    stocastic_test(accuracy_SupportVectorMachine_germanCredit_standardization, 8, .737, .05);
}
PSL_TEST(SupportVectorMachine, germanCreditAccuracy_bipolarNormalization)
{
    set_rand_seed();
    PSL_SerialOnlyTest
    stocastic_test(accuracy_SupportVectorMachine_germanCredit_bipolarNormalization, 8, .736, .05);
}
PSL_TEST(SupportVectorMachine, germanCreditAccuracy_skewNormalization)
{
    set_rand_seed();
    PSL_SerialOnlyTest
    stocastic_test(accuracy_SupportVectorMachine_germanCredit_skewNormalization, 8, .735, .05);
}
PSL_TEST(SupportVectorMachine, germanCreditAccuracy_PCA)
{
    set_rand_seed();
    PSL_SerialOnlyTest
    stocastic_test(accuracy_SupportVectorMachine_germanCredit_PCA, 8, .71, .05);
}

bool passed_SupportVectorMachine_simple()
{
    bool passed = true;

    // allocate needed
    AbstractAuthority authority;

    // write dataset
    FindDataset finder(&authority);
    const std::string archive_filename = finder.find(datasets_t::datasets_t::simple_dataset);

    // fill parameters
    ParameterData parameter_data;
    parameter_data.defaults_for_supportVectorMachine();
    parameter_data.set_archive_filename(archive_filename);

    // get from archive
    ClassificationArchive archive(&authority);
    archive.initialize(&parameter_data);

    // train network
    SupportVectorMachine svm(&authority);
    svm.initialize(&parameter_data, &archive);
    svm.train(&parameter_data, &archive);

    // define testing problems
    const std::vector<std::vector<double> > testing_input_scalars = { {-0.2, 0.1}, {-0.2, -0.2}, {0.3, -0.3}, {0.3, 0.4}};
    const std::vector<std::vector<int> > testing_input_ints = { {}, {}, {}, {}};
    const std::vector<int> testing_output_ints = {0, 0, 1, 1};

    // test
    const size_t num_testing = testing_output_ints.size();
    for(size_t t = 0u; t < num_testing; t++)
    {
        const int actual = svm.classify(testing_input_scalars[t], testing_input_ints[t]);
        passed &= (testing_output_ints[t] == actual);
    }

    // clean-up archive file
    archive.get_io_manager()->delete_file();

    return passed;
}

PSL_TEST(SupportVectorMachine, simple)
{
    set_rand_seed();
    PSL_SerialOnlyTest
    stocastic_test(passed_SupportVectorMachine_simple, 5, 25, .9);
}

}
}
