#include "PSL_BruteForceKNNSearcher.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_KNearestNeighborsSearcher.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_ParameterData.hpp"

#include <vector>
#include <queue>
#include <functional>
#include <utility>
#include <cmath>
#include <math.h>

namespace PlatoSubproblemLibrary
{

BruteForceKNNSearcher::BruteForceKNNSearcher(AbstractAuthority* authority) :
        KNearestNeighborsSearcher(authority, k_nearest_neighbors_searchers_t::brute_force_searcher)
{
}
BruteForceKNNSearcher::~BruteForceKNNSearcher()
{
}

bool BruteForceKNNSearcher::is_exact()
{
    return true;
}
void BruteForceKNNSearcher::extract_parameters(ParameterData* parameters)
{
}

// for data flow
void BruteForceKNNSearcher::unpack_data()
{
    KNearestNeighborsSearcher::unpack_data();
}
void BruteForceKNNSearcher::allocate_data()
{
    KNearestNeighborsSearcher::allocate_data();
}

// for searching
void BruteForceKNNSearcher::initialize()
{
    // nothing to do
}
void BruteForceKNNSearcher::get_neighbors(const int& num_neighbors,
                                          const std::vector<double>& input_to_search,
                                          std::vector<double>& worst_to_best_distances,
                                          std::vector<int>& worst_to_best_class_outputs)
{
    // allocate queue
    std::priority_queue<std::pair<double, int>,std::vector<std::pair<double, int> >,std::less<std::pair<double, int> > > distance_then_outputClass;

    // for each row
    const int num_rows = (*m_answer_points)->get_num_rows();
    for(int r = 0; r < num_rows; r++)
    {
        // determine distance
        std::vector<double> this_row;
        (*m_answer_points)->get_row(r, this_row);
        const double dist = std::sqrt(m_authority->dense_vector_operations->delta_squared(input_to_search, this_row));

        bool should_enqueue = false;

        if(int(distance_then_outputClass.size()) < num_neighbors)
        {
            // if not enough neighbors, add
            should_enqueue = true;
        }
        else if(dist < distance_then_outputClass.top().first)
        {
            // if better than top, add
            should_enqueue = true;
            distance_then_outputClass.pop();
        }

        // if adding, enqueue
        if(should_enqueue)
        {
            distance_then_outputClass.push(std::make_pair(dist, (*m_answer_class_outputs)[r]));
        }
    }

    worst_to_best_distances.resize(num_neighbors);
    worst_to_best_class_outputs.resize(num_neighbors);
    int counter = 0;
    while(!distance_then_outputClass.empty())
    {
        // count
        worst_to_best_distances[counter] = distance_then_outputClass.top().first;
        worst_to_best_class_outputs[counter] = distance_then_outputClass.top().second;
        // dequeue
        distance_then_outputClass.pop();
        // advance
        counter++;
    }
    worst_to_best_distances.resize(counter);
    worst_to_best_class_outputs.resize(counter);
}

}
