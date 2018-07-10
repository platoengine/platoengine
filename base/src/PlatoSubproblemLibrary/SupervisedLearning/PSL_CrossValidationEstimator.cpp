#include "PSL_CrossValidationEstimator.hpp"

#include "PSL_ClassificationAccuracyEstimator.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_Classifier.hpp"
#include "PSL_ClassificationAssessor.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_ClassificationArchive.hpp"
#include "PSL_InputOutputManager.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_DataOrganizerFactory.hpp"
#include "PSL_DataFlow.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <string>
#include <cassert>

namespace PlatoSubproblemLibrary
{

CrossValidationEstimator::CrossValidationEstimator(AbstractAuthority* authority) :
        ClassificationAccuracyEstimator(authority),
        m_archive_filename(),
        m_delete_archive(),
        m_num_folds(-1)
{
}
CrossValidationEstimator::~CrossValidationEstimator()
{
}

void CrossValidationEstimator::initialize(const std::string& archive_filename, const bool& delete_archive, const int& num_folds)
{
    m_archive_filename = archive_filename;
    m_delete_archive = delete_archive;
    m_num_folds = num_folds;
}

void CrossValidationEstimator::estimate_accuracy(ParameterData* parameter_data,
                                                 ClassificationAssessor* assessor)
{
    // if not initialized, abort
    if(m_archive_filename.size() == 0u)
    {
        m_authority->utilities->fatal_error("CrossValidationEstimator: estimate_accuracy invoked before initialize. Aborting.\n\n");
        return;
    }

    // fill filename
    parameter_data->set_archive_filename(m_archive_filename);

    // split archive
    ClassificationArchive splitting_archive(m_authority);
    splitting_archive.initialize(parameter_data);
    std::vector<std::string> fold_filenames;
    std::vector<double> fold_percents(m_num_folds, 1. / double(m_num_folds));
    splitting_archive.split_dataset(fold_percents, fold_filenames);

    // omit one fold for testing
    for(int omit = 0; omit < m_num_folds; omit++)
    {
        // training archive by unite
        std::vector<std::string> unite_filenames(m_num_folds - 1);
        for(int f = 0; f < m_num_folds; f++)
        {
            if(f < omit)
            {
                unite_filenames[f] = fold_filenames[f];
            }
            else if(omit < f)
            {
                unite_filenames[f - 1] = fold_filenames[f];
            }
        }

        // build training
        ClassificationArchive training_archive(m_authority);
        std::string training_filename;
        training_archive.unite_dataset(unite_filenames, training_filename);
        parameter_data->set_archive_filename(training_filename);
        training_archive.initialize(parameter_data);

        // build classifier
        Classifier* classifier = build_classifier(parameter_data->get_classifier(), m_authority);

        // train
        classifier->initialize(parameter_data, &training_archive);
        classifier->train(parameter_data, &training_archive);

        // testing archive
        parameter_data->set_archive_filename(fold_filenames[omit]);
        ClassificationArchive testing_archive(m_authority);
        testing_archive.initialize(parameter_data);

        // if first time, set assessor sizes
        if(omit == 0)
        {
            std::vector<int> input_enum_sizes;
            int output_enum_size = -1;
            testing_archive.get_enum_size(input_enum_sizes, output_enum_size);
            assessor->initialize(parameter_data, output_enum_size);
        }

        // assess
        assessor->assess(classifier, &testing_archive);

        // clean-up
        safe_free(classifier);
        training_archive.get_io_manager()->delete_file();
    }

    // clean-up archives
    for(int f = 0; f < m_num_folds; f++)
    {
        ClassificationArchive fold_archive(m_authority);
        parameter_data->set_archive_filename(fold_filenames[f]);
        fold_archive.initialize(parameter_data);
        fold_archive.get_io_manager()->delete_file();
    }
    if(m_delete_archive)
    {
        splitting_archive.get_io_manager()->delete_file();
    }

    // conditionally announce accuracy
    if(parameter_data->get_verbose())
    {
        m_authority->utilities->print(std::string("testing accuracy: ") + std::to_string(assessor->get_classification_accuracy())
                                      + std::string("\n"));
    }
}

}
