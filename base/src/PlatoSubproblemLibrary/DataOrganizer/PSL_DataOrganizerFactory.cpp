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
