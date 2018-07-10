// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
namespace AbstractInterface
{
class MpiWrapper;
class ParallelVector;

class ParallelExchanger
{
public:
    ParallelExchanger(AbstractAuthority* authority);
    virtual ~ParallelExchanger();

    virtual MpiWrapper* get_mpi();

    // get indexes of local points that are locally owned
    virtual std::vector<size_t> get_local_contracted_indexes() = 0;
    // convert data vector in parallel format to vector of values for locally owned points
    virtual std::vector<double> get_contraction_to_local_indexes(ParallelVector* input_data_vector) = 0;
    // communicate between processors to expand the locally owned data to parallel format with some values shared on processors
    virtual void get_expansion_to_parallel_vector(const std::vector<double>& input_data_vector, ParallelVector* output_data_vector) = 0;
    // determine maximum absolute parallel error
    virtual double get_maximum_absolute_parallel_error(ParallelVector* input_data_vector) = 0;

protected:
    AbstractAuthority* m_authority;

};

}

}
