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

#include "PSL_NearestNeighborsClassifier.hpp"

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
#include "PSL_DataSequence.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_VectorCoding.hpp"
#include "PSL_DistanceVoterFactory.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_DistanceVoter.hpp"
#include "PSL_KNearestNeighborsSearcher.hpp"
#include "PSL_KNearestNeighborsFactory.hpp"

#include <vector>
#include <cstddef>
#include <string>
#include <algorithm>
#include <iostream>
#include <cassert>

namespace PlatoSubproblemLibrary
{

NearestNeighborsClassifier::NearestNeighborsClassifier(AbstractAuthority* authority) :
        Classifier(authority),
        m_output_enum_size(NULL),
        m_num_neighbors(NULL),
        m_processed_length(NULL),
        m_distance_voter_enum(NULL),
        m_input_enum_sizes(NULL),
        m_distance_voter(NULL),
        m_knn_searcher(NULL)
{
    m_data_sequence->set_data_flow_type(data_flow_t::data_flow_t::nearest_neighbors_classifier_flow);
}

NearestNeighborsClassifier::~NearestNeighborsClassifier()
{
    clear_derived_data();
    clear_knnSearcher();
}

void NearestNeighborsClassifier::allocate_classifier_data(ParameterData* parameters, ClassificationArchive* archive)
{
    // allocate integers
    m_data_sequence->enqueue_int(parameters->get_num_neighbors());
    m_data_sequence->enqueue_int(-1);
    m_data_sequence->enqueue_int(parameters->get_distance_voter());

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

    // allocate matrices

    // allocate preprocessor
    allocate_preprocessor(parameters);

    // allocate searcher
    ensureBuilt_knnSearcher();
    m_knn_searcher->extract_parameters(parameters);

    // TODO: efficiency improvement only need size information
    AbstractInterface::DenseMatrix* training_matrix = archive->get_all_rows_onehot_encoded();
    const int num_answers = training_matrix->get_num_rows();
    m_knn_searcher->before_allocate(num_answers);
    safe_free(training_matrix);

    m_knn_searcher->allocate_data();
}

void NearestNeighborsClassifier::unpack_data()
{
    // unpack integer scalars
    m_num_neighbors = m_data_sequence->get_int_ptr(m_data_sequence->dequeue_int());
    m_processed_length = m_data_sequence->get_int_ptr(m_data_sequence->dequeue_int());
    m_distance_voter_enum = m_data_sequence->get_int_ptr(m_data_sequence->dequeue_int());
    m_output_enum_size = m_data_sequence->get_int_ptr(m_data_sequence->dequeue_int());

    // unpack double scalars

    // unpack integer vectors
    m_preprocessor_enum = m_data_sequence->get_int_vector(m_data_sequence->dequeue_int_vector());
    m_input_enum_sizes = m_data_sequence->get_int_vector(m_data_sequence->dequeue_int_vector());

    // unpack double vectors

    // unpack matrices

    // derived data
    build_derived_data();

    // unpack preprocessor
    std::vector<preprocessor_t::preprocessor_t> enums;
    transfer_vector_types(enums, *m_preprocessor_enum);
    unpack_preprocessor(enums);

    // unpack searcher
    ensureBuilt_knnSearcher();
    m_knn_searcher->unpack_data();
}

void NearestNeighborsClassifier::initialize(ParameterData* parameters, ClassificationArchive* archive)
{
    check_parameters(parameters);

    // allocate support vector machine
    allocate_classifier_data(parameters, archive);

    // associate pointers to underlying data
    unpack_data();

    // build preprocessor
    m_preprocessor->begin_initialize(parameters, archive, *m_processed_length);
}

void NearestNeighborsClassifier::train(ParameterData* parameters, ClassificationArchive* archive)
{
    // if verbose, announce
    if(parameters->get_verbose())
    {
        m_authority->utilities->print("--- training new nearest neighbors classifier ---\n");
    }

    // get training data
    AbstractInterface::DenseMatrix* training_matrix = archive->get_all_rows_onehot_encoded();
    const int num_rows = training_matrix->get_num_rows();

    // allocate processed training data
    const int processed_length = *m_processed_length;
    AbstractInterface::DenseMatrix* processed_train_dataset = m_authority->dense_builder->build_by_fill(num_rows, processed_length, 0.);
    std::vector<int> output_enum_of_train_dataset(num_rows);

    // fill processed data
    for(int r = 0; r < num_rows; r++)
    {
        // get full row
        std::vector<double> full_row;
        training_matrix->get_row(r, full_row);

        // store output
        onehot_decode(full_row, *m_output_enum_size, output_enum_of_train_dataset[r]);

        // remove output
        std::vector<double> input_row;
        onehot_decode(full_row, *m_output_enum_size, input_row);

        // pre-process
        m_preprocessor->preprocess(input_row);
        assert(int(input_row.size()) == processed_length);

        // transfer
        for(int c = 0; c < processed_length; c++)
        {
            processed_train_dataset->set_value(r, c, input_row[c]);
        }
    }
    safe_free(training_matrix);

    // transfer processed data
    m_knn_searcher->after_unpack(processed_train_dataset, output_enum_of_train_dataset);
    m_knn_searcher->initialize();

    // validate to actual neighbors size
    if(parameters->get_should_select_nearest_neighbors_by_validation())
    {
        // do hold-one-out validation

        // build voters
        const int num_voter_types = distance_voter_t::distance_voter_t::TOTAL_NUM_DISTANCE_VOTERS;
        std::vector<DistanceVoter*> voters(num_voter_types, NULL);
        for(int v = 0; v < num_voter_types; v++)
        {
            distance_voter_t::distance_voter_t vote_type = distance_voter_t::distance_voter_t(v);
            voters[v] = build_distance_voter(vote_type, m_authority);
        }

        // allocate for results
        const int max_num_neighbors = *m_num_neighbors;
        std::vector<int> accurate_at_this_neighbor_and_type(max_num_neighbors * num_voter_types, 0);

        // for validation, plan to remove one
        const int searcher_exposed_num_neighbors = max_num_neighbors + 1;

        // for each row
        for(int r1 = 0; r1 < num_rows; r1++)
        {
            std::vector<double> this_row;
            processed_train_dataset->get_row(r1, this_row);
            const int expected_output = output_enum_of_train_dataset[r1];

            // get neighbors
            std::vector<int> output_worst_to_best;
            std::vector<double> distance_worst_to_best;
            m_knn_searcher->get_neighbors(searcher_exposed_num_neighbors, this_row, distance_worst_to_best, output_worst_to_best);

            // for validation, remove closest neighbor
            const int inital_num_neighbors = distance_worst_to_best.size();
            if(1 < inital_num_neighbors)
            {
                distance_worst_to_best.resize(inital_num_neighbors - 1);
                output_worst_to_best.resize(inital_num_neighbors - 1);
            }

            // get class counts
            std::vector<int> class_counts(*m_output_enum_size, 0);
            const int actual_num_neighbors_received = output_worst_to_best.size();
            assert(actual_num_neighbors_received == int(distance_worst_to_best.size()));
            for(int n = 0; n < actual_num_neighbors_received; n++)
            {
                class_counts[output_worst_to_best[n]]++;
            }

            // consider various numbers of neighbors
            for(int neighbors = max_num_neighbors; 1 <= neighbors; neighbors--)
            {
                // get distances and class outputs
                const int actual_neighbors_removed = std::max(actual_num_neighbors_received - neighbors, 0);
                std::vector<double> this_distances(distance_worst_to_best.begin() + actual_neighbors_removed,
                                                   distance_worst_to_best.end());
                std::vector<int> this_class_output(output_worst_to_best.begin() + actual_neighbors_removed,
                                                   output_worst_to_best.end());

                // for each voter
                for(int v = 0; v < num_voter_types; v++)
                {
                    // if accurate, count it
                    const int computed_output = voters[v]->choose_output(class_counts, this_distances, this_class_output);
                    accurate_at_this_neighbor_and_type[(neighbors - 1) * num_voter_types + v] += (expected_output
                                                                                                  == computed_output);
                }

                // update class counts
                const int theortical_neighbors_removed = max_num_neighbors - neighbors;
                class_counts[output_worst_to_best[theortical_neighbors_removed]]--;
            }
        }

        // clean up
        safe_free(voters);

        // choose best
        const int neighbor_and_type = rand_max_index(accurate_at_this_neighbor_and_type);
        *m_distance_voter_enum = neighbor_and_type % num_voter_types;
        *m_num_neighbors = 1 + (neighbor_and_type - *m_distance_voter_enum) / num_voter_types;
        build_derived_data();

        // if verbose, announce
        if(parameters->get_verbose())
        {
            m_authority->utilities->print("hold one out validation accurate counts:");
            m_authority->utilities->print(accurate_at_this_neighbor_and_type, true);
            m_authority->utilities->print(std::string("max neighbors:") + std::to_string(max_num_neighbors)
                                          + std::string(", selected neighbors:") + std::to_string(*m_num_neighbors)
                                          + std::string(", distance voter enum:") + std::to_string(*m_distance_voter_enum)
                                          + std::string("\n"));
        }
    }

    // if verbose, announce
    if(parameters->get_verbose())
    {
        m_authority->utilities->print("--- done training nearest neighbors classifier ---\n");
    }
}

int NearestNeighborsClassifier::classify(const std::vector<double>& input_scalars, const std::vector<int>& input_enums)
{
    // encode
    std::vector<double> input;
    onehot_encode(input_scalars, input_enums, *m_input_enum_sizes, input);
    m_preprocessor->preprocess(input);

    // get neighbors
    std::vector<int> output_worst_to_best;
    std::vector<double> distance_worst_to_best;
    m_knn_searcher->get_neighbors(*m_num_neighbors, input, distance_worst_to_best, output_worst_to_best);

    // vote for output
    const int output_result = m_distance_voter->choose_output(*m_output_enum_size, distance_worst_to_best, output_worst_to_best);
    return output_result;
}

#define PSL_NearestNeighborsClassifier_ParameterWarning(name) \
        if(!parameters->didUserInput_##name()) \
        util->fatal_error("NearestNeighborsClassifier: missing parameter "#name"\n");

void NearestNeighborsClassifier::check_parameters(ParameterData* parameters)
{
    AbstractInterface::GlobalUtilities* util = m_authority->utilities;

    PSL_NearestNeighborsClassifier_ParameterWarning(preprocessor)
    PSL_NearestNeighborsClassifier_ParameterWarning(num_neighbors)
    if(parameters->get_num_neighbors() < 1)
    {
        util->fatal_error("NearestNeighborsClassifier: invalid num_neighbors parameter\n");
    }
    PSL_NearestNeighborsClassifier_ParameterWarning(verbose)
    PSL_NearestNeighborsClassifier_ParameterWarning(should_select_nearest_neighbors_by_validation)
}

void NearestNeighborsClassifier::clear_derived_data()
{
    safe_free(m_distance_voter);
}
void NearestNeighborsClassifier::build_derived_data()
{
    clear_derived_data();
    const distance_voter_t::distance_voter_t voter_enum = distance_voter_t::distance_voter_t(*m_distance_voter_enum);
    m_distance_voter = build_distance_voter(voter_enum, m_authority);
}

void NearestNeighborsClassifier::clear_knnSearcher()
{
    safe_free(m_knn_searcher);
}
void NearestNeighborsClassifier::ensureBuilt_knnSearcher()
{
    if(m_knn_searcher == NULL)
    {
        clear_knnSearcher();
        const k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t searcher_type =
                k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t::random_projection_forest_searcher;
        m_knn_searcher = build_knn_searcher(searcher_type, m_authority, m_data_sequence);
    }
}

}

