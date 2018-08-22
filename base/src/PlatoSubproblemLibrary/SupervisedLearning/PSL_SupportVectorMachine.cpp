#include "PSL_SupportVectorMachine.hpp"

#include "PSL_Classifier.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_ClassificationArchive.hpp"
#include "PSL_VectorCoding.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_Random.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_ClassificationAssessor.hpp"
#include "PSL_Preprocessor.hpp"
#include "PSL_PreprocessorFactory.hpp"
#include "PSL_VectorKernel.hpp"
#include "PSL_VectorKernelFactory.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_DataSequence.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <cstddef>
#include <string>
#include <algorithm>
#include <cmath>
#include <math.h>
#include <iostream>

namespace PlatoSubproblemLibrary
{

SupportVectorMachine::SupportVectorMachine(AbstractAuthority* authority) :
        Classifier(authority),
        m_output_enum_size(NULL),
        m_kernel_function_enum(NULL),
        m_kernel_function_parameter(NULL),
        m_num_snapshots(NULL),
        m_input_enum_sizes(NULL),
        m_support_vectors(),
        m_kernel_function(NULL)
{
    m_data_sequence->set_data_flow_type(data_flow_t::data_flow_t::support_vector_machine_flow);
}

SupportVectorMachine::~SupportVectorMachine()
{
    m_output_enum_size = NULL;
    m_kernel_function_enum = NULL;
    m_kernel_function_parameter = NULL;
    m_num_snapshots = NULL;
    m_input_enum_sizes = NULL;
    m_support_vectors.clear();
    clear_built_data();
}

void SupportVectorMachine::clear_built_data()
{
    safe_free(m_kernel_function);
    safe_free(m_preprocessor);
}

void SupportVectorMachine::allocate_classifier_data(ParameterData* parameters, ClassificationArchive* archive)
{
    const int num_snapshots = parameters->get_num_snapshots();

    // allocate integers
    m_data_sequence->enqueue_int(parameters->get_vector_kernel());
    m_data_sequence->enqueue_int(parameters->get_vector_kernel_parameter());
    m_data_sequence->enqueue_int(num_snapshots);

    // allocate doubles

    // allocate integer vectors
    std::vector<int>* preprocessors = m_data_sequence->get_int_vector(m_data_sequence->enqueue_int_vector());
    transfer_vector_types(*preprocessors, parameters->get_preprocessor());
    // get sizes
    std::vector<int>* input_enum_sizes = m_data_sequence->get_int_vector(m_data_sequence->enqueue_int_vector());
    int output_enum_size;
    archive->get_enum_size(*input_enum_sizes, output_enum_size);
    m_data_sequence->enqueue_int(output_enum_size);

    // allocate double vectors
    const int local_needed_double_vector = choose(output_enum_size, 2) * num_snapshots;
    m_data_sequence->enqueue_double_vector(local_needed_double_vector);

    // allocate matrices

    // allocate preprocessor
    allocate_preprocessor(parameters);
}

void SupportVectorMachine::unpack_data()
{
    clear_built_data();

    // unpack integer scalars
    m_kernel_function_enum = m_data_sequence->get_int_ptr(m_data_sequence->dequeue_int());
    m_kernel_function_parameter = m_data_sequence->get_int_ptr(m_data_sequence->dequeue_int());
    m_num_snapshots = m_data_sequence->get_int_ptr(m_data_sequence->dequeue_int());
    m_output_enum_size = m_data_sequence->get_int_ptr(m_data_sequence->dequeue_int());

    // build kernel function
    safe_free(m_kernel_function);
    m_kernel_function = build_vector_kernel(vector_kernel_t::vector_kernel_t(*m_kernel_function_enum),
                                            *m_kernel_function_parameter,
                                            m_authority->utilities);

    // unpack double scalars

    // unpack integer vectors
    m_preprocessor_enum = m_data_sequence->get_int_vector(m_data_sequence->dequeue_int_vector());
    m_input_enum_sizes = m_data_sequence->get_int_vector(m_data_sequence->dequeue_int_vector());

    // unpack double vectors
    // unpack support vectors
    const int num_support_vectors_needed = choose(*m_output_enum_size, 2) * (*m_num_snapshots);
    m_support_vectors.resize(num_support_vectors_needed);
    for(int sv = 0; sv < num_support_vectors_needed; sv++)
    {
        m_support_vectors[sv] = m_data_sequence->get_double_vector(m_data_sequence->dequeue_double_vector());
    }

    // unpack matrices

    // unpack preprocessor
    std::vector<preprocessor_t::preprocessor_t> enums;
    transfer_vector_types(enums, *m_preprocessor_enum);
    unpack_preprocessor(enums);
}

void SupportVectorMachine::initialize(ParameterData* parameters, ClassificationArchive* archive)
{
    check_parameters(parameters);

    // allocate support vector machine
    allocate_classifier_data(parameters, archive);

    // associate pointers to underlying data
    unpack_data();

    // build preprocessor
    int processed_length = -1;
    m_preprocessor->begin_initialize(parameters, archive, processed_length);

    // build vectors
    initialize_vectors(parameters, processed_length);
}

void SupportVectorMachine::train(ParameterData* parameters, ClassificationArchive* archive)
{
    // if verbose, announce
    if(parameters->get_verbose())
    {
        m_authority->utilities->print("--- training new support vector machine ---\n");
    }

    // read data
    AbstractInterface::DenseMatrix* dataset = archive->get_all_rows_onehot_encoded();

    // determine rows
    std::vector<std::vector<int> > rows_of_each_output_enum;
    onehot_decode_all_rows(dataset, *m_output_enum_size, rows_of_each_output_enum);

    // for each one-vs-one pattern
    int problem_counter = 0;
    for(int lower_output_enum = 0; lower_output_enum < *m_output_enum_size; lower_output_enum++)
    {
        for(int upper_output_enum = lower_output_enum + 1; upper_output_enum < *m_output_enum_size; upper_output_enum++)
        {
            std::vector<std::vector<double>*> support_vector_snapshots(m_support_vectors.begin() + problem_counter,
                                                                       m_support_vectors.begin() + problem_counter
                                                                       + *m_num_snapshots);

            // train lower versus upper
            train_1v1(parameters,
                      dataset,
                      rows_of_each_output_enum[lower_output_enum],
                      rows_of_each_output_enum[upper_output_enum],
                      support_vector_snapshots);

            // advance problem
            problem_counter += *m_num_snapshots;
        }
    }

    // finalize
    safe_free(dataset);
}

void SupportVectorMachine::initialize_vectors(ParameterData* parameters, const int& input_dimension)
{
    // get length
    assert(m_kernel_function);
    const int support_vector_dimension = m_kernel_function->projection_length(input_dimension) + 1;

    // for each vector
    const size_t num_support_vectors = m_support_vectors.size();
    for(size_t sv = 0u; sv < num_support_vectors; sv++)
    {
        // fill with zeros
        m_support_vectors[sv]->assign(support_vector_dimension, 0.);
    }
}

int SupportVectorMachine::classify(const std::vector<double>& input_scalars, const std::vector<int>& input_enums)
{
    // note: classifications are sometimes non-deterministic; that may be undesirable

    // encode
    std::vector<double> raw_input;
    onehot_encode(input_scalars, input_enums, *m_input_enum_sizes, raw_input);
    m_preprocessor->preprocess(raw_input);

    // augment
    std::vector<double> augmented_input;
    m_kernel_function->project(raw_input, augmented_input);
    augmented_input.push_back(1.);

    // allocate for recording instances
    std::vector<int> occurrences(*m_output_enum_size, 0);

    // for each 1v1 problem
    int problem_counter = 0;
    for(int lower_output_enum = 0; lower_output_enum < *m_output_enum_size; lower_output_enum++)
    {
        for(int upper_output_enum = lower_output_enum + 1; upper_output_enum < *m_output_enum_size; upper_output_enum++)
        {
            for(int snapshot = 0; snapshot < *m_num_snapshots; snapshot++)
            {
                const double dot_value = m_authority->dense_vector_operations->dot(augmented_input,
                                                                                   *m_support_vectors[problem_counter]);

                if(dot_value < 0.)
                {
                    occurrences[lower_output_enum]++;
                }
                else
                {
                    occurrences[upper_output_enum]++;
                }

                // advance problem
                problem_counter++;
            }
        }
    }

    const int classification = rand_max_index(occurrences);
    return classification;
}

void SupportVectorMachine::train_1v1(ParameterData* parameters,
                                     AbstractInterface::DenseMatrix* dataset,
                                     const std::vector<int>& negative_class_rows,
                                     const std::vector<int>& positive_class_rows,
                                     std::vector<std::vector<double>*>& support_vector_snapshots)
{
    // count rows
    const int num_negative_rows = negative_class_rows.size();
    const int num_positive_rows = positive_class_rows.size();
    const int num_rows_in_1v1 = num_negative_rows + num_positive_rows;

    // compute parameters
    const double empirical_risk_parameter = parameters->get_empirical_risk();
    const double structural_risk_parameter = 1. / (empirical_risk_parameter * double(num_rows_in_1v1));

    // for each initial epoch
    const int num_initial_epoch = parameters->get_initial_stocastic_epochs();
    double experience = 0.;
    for(int initial_epoch = 0; initial_epoch < num_initial_epoch; initial_epoch++)
    {
        train_an_epoch(structural_risk_parameter,
                       dataset,
                       negative_class_rows,
                       positive_class_rows,
                       support_vector_snapshots[0],
                       experience);
    }

    // for each snapshot
    const int epochs_per_snapshot = 1 + parameters->get_snapshot_separating_epochs();
    const double experience_rewind = parameters->get_snapshot_experience_rewind();
    for(int snapshot = 0; snapshot < *m_num_snapshots; snapshot++)
    {
        // do epochs
        for(int snapshot_epoch = 0; snapshot_epoch < epochs_per_snapshot; snapshot_epoch++)
        {
            train_an_epoch(structural_risk_parameter,
                           dataset,
                           negative_class_rows,
                           positive_class_rows,
                           support_vector_snapshots[snapshot],
                           experience);
        }

        // load current vector into next snapshot
        const bool has_next_shapshot = (snapshot + 1 < *m_num_snapshots);
        if(has_next_shapshot)
        {
            *support_vector_snapshots[snapshot + 1] = *support_vector_snapshots[snapshot];
        }

        // rewind experience
        experience -= experience_rewind * double(epochs_per_snapshot * num_rows_in_1v1);
    }
}

void SupportVectorMachine::train_an_epoch(const double& structural_risk_parameter,
                                          AbstractInterface::DenseMatrix* dataset,
                                          const std::vector<int>& negative_class_rows,
                                          const std::vector<int>& positive_class_rows,
                                          std::vector<double>* support_vector,
                                          double& experience)
{
    // count rows
    const int num_negative_rows = negative_class_rows.size();
    const int num_positive_rows = positive_class_rows.size();
    const int num_rows_in_1v1 = num_negative_rows + num_positive_rows;

    std::vector<int> abstract_rows;
    random_permutation(num_rows_in_1v1, abstract_rows);
    assert(abstract_rows.size() == size_t(num_rows_in_1v1));

    // for each abstract row
    for(int ari = 0; ari < num_rows_in_1v1; ari++)
    {
        const int abstract_row = abstract_rows[ari];

        int acutal_row_index = -1;
        int class_label = 0;
        if(abstract_row < num_negative_rows)
        {
            // negative class
            class_label = -1;
            acutal_row_index = negative_class_rows[abstract_row];
        }
        else
        {
            // positive class
            class_label = 1;
            acutal_row_index = positive_class_rows[abstract_row - num_negative_rows];
        }

        // get row
        std::vector<double> full_row;
        dataset->get_row(acutal_row_index, full_row);

        // TODO: performance optimization, do this decode once by submatrix
        // get only input
        std::vector<double> row_input;
        onehot_decode(full_row, *m_output_enum_size, row_input);
        m_preprocessor->preprocess(row_input);

        // kernel the input
        std::vector<double> augmented_and_reflected_pattern;
        m_kernel_function->project(row_input, augmented_and_reflected_pattern);

        // augment
        augmented_and_reflected_pattern.push_back(1.);

        // reflect
        m_authority->dense_vector_operations->scale(class_label, augmented_and_reflected_pattern);
        const double dot_value = m_authority->dense_vector_operations->dot(augmented_and_reflected_pattern, *support_vector);
        m_authority->dense_vector_operations->scale(experience / (experience + 1.), *support_vector);
        if(dot_value <= 1.)
        {
            const double scale_factor = 1. / (structural_risk_parameter * (experience + 1.));
            m_authority->dense_vector_operations->axpy(scale_factor, augmented_and_reflected_pattern, *support_vector);
        }

        // increment experience
        experience++;
    }
}

#define PSL_SupportVectorMachine_ParameterWarning(name) \
        if(!parameters->didUserInput_##name()) \
        util->fatal_error("SupportVectorMachine: missing parameter "#name"\n");

void SupportVectorMachine::check_parameters(ParameterData* parameters)
{
    AbstractInterface::GlobalUtilities* util = m_authority->utilities;

    PSL_SupportVectorMachine_ParameterWarning(num_snapshots)
    if(parameters->get_num_snapshots() < 1)
    {
        util->fatal_error("SupportVectorMachine: invalid num_snapshots parameter\n");
    }
    PSL_SupportVectorMachine_ParameterWarning(preprocessor)
    PSL_SupportVectorMachine_ParameterWarning(vector_kernel)
    PSL_SupportVectorMachine_ParameterWarning(vector_kernel_parameter)
    PSL_SupportVectorMachine_ParameterWarning(verbose)
    PSL_SupportVectorMachine_ParameterWarning(empirical_risk)
    if(parameters->get_empirical_risk() <= 0.)
    {
        util->fatal_error("SupportVectorMachine: invalid empirical_risk parameter\n");
    }
    PSL_SupportVectorMachine_ParameterWarning(initial_stocastic_epochs)
    if(parameters->get_initial_stocastic_epochs() < 0)
    {
        util->fatal_error("SupportVectorMachine: invalid initial_stocastic_epochs parameter\n");
    }
    PSL_SupportVectorMachine_ParameterWarning(snapshot_separating_epochs)
    if(parameters->get_snapshot_separating_epochs() < 0)
    {
        util->fatal_error("SupportVectorMachine: invalid snapshot_separating_epochs parameter\n");
    }
    PSL_SupportVectorMachine_ParameterWarning(snapshot_experience_rewind)
    if((parameters->get_snapshot_experience_rewind() < 0.) || (1. < parameters->get_snapshot_experience_rewind()))
    {
        util->fatal_error("SupportVectorMachine: invalid snapshot_experience_rewind parameter\n");
    }
}

}

