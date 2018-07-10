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

#include "PSL_DataOrganizerFactory.hpp"

#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_DataSequence.hpp"
#include "PSL_InputOutputManager.hpp"
#include "PSL_MultiLayoutVector.hpp"
#include "PSL_FeedForwardNeuralNetwork.hpp"
#include "PSL_SupportVectorMachine.hpp"
#include "PSL_RandomForest.hpp"
#include "PSL_NearestNeighborsClassifier.hpp"

#include <string>
#include <cassert>

namespace PlatoSubproblemLibrary
{

DataSequence* load_data_sequence(const std::string& filename,
                                 AbstractAuthority* authority)
{
    // load from file
    DataSequence* result = new DataSequence(authority,data_flow_t::data_flow_t::inert_data_flow);
    result->load_from_file(filename);
    return result;
}

Classifier* build_classifier(const data_flow_t::data_flow_t& flow_type,
                             AbstractAuthority* authority)
{
    DataFlow* flow = build_data_flow(flow_type, authority);
    Classifier* classifier = dynamic_cast<Classifier*>(flow);
    assert(classifier);
    return classifier;
}
DataFlow* build_data_flow(const data_flow_t::data_flow_t& flow_type,
                          AbstractAuthority* authority)
{
    DataFlow* result = NULL;

    switch(flow_type)
    {
        case data_flow_t::data_flow_t::inert_data_flow:
        {
            result = new DataFlow(authority);
            break;
        }
        case data_flow_t::data_flow_t::feed_forward_neural_network_flow:
        {
            result = new FeedForwardNeuralNetwork(authority);
            break;
        }
        case data_flow_t::data_flow_t::support_vector_machine_flow:
        {
            result = new SupportVectorMachine(authority);
            break;
        }
        case data_flow_t::data_flow_t::random_forest_flow:
        {
            result = new RandomForest(authority);
            break;
        }
        case data_flow_t::data_flow_t::nearest_neighbors_classifier_flow:
        {
            result = new NearestNeighborsClassifier(authority);
            break;
        }
        case data_flow_t::data_flow_t::TOTAL_NUM_DATA_FLOWS:
        default:
        {
            authority->utilities->fatal_error("Failed to match flow type in build_data_flow. Aborting.\n\n");
            break;
        }
    }

    return result;
}

DataFlow* load_data_flow(const std::string& filename,
                         AbstractAuthority* authority)
{
    // load
    DataSequence* sequence = load_data_sequence(filename, authority);
    DataFlow* flow = build_data_flow(sequence->get_data_flow_type(), authority);
    flow->set_data_sequence(sequence, true);
    flow->unpack_data();

    return flow;
}
Classifier* load_classifier(const std::string& filename,
                            AbstractAuthority* authority)
{
    DataFlow* flow = load_data_flow(filename, authority);
    Classifier* classifier = dynamic_cast<Classifier*>(flow);
    assert(classifier);
    return classifier;
}

}
