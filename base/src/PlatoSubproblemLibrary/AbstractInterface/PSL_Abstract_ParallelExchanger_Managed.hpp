// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_ParallelExchanger.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
namespace AbstractInterface
{
class ParallelVector;

class ParallelExchanger_Managed : public AbstractInterface::ParallelExchanger
{
public:
    ParallelExchanger_Managed(AbstractAuthority* authority);
    virtual ~ParallelExchanger_Managed();

    virtual void build() = 0;

    // get indexes of local points that are locally owned
    virtual std::vector<size_t> get_local_contracted_indexes();
    // convert data vector in parallel format to vector of values for locally owned points
    virtual std::vector<double> get_contraction_to_local_indexes(ParallelVector* input_data_vector);
    // communicate between processors to expand the locally owned data to parallel format with some values shared on processors
    virtual void get_expansion_to_parallel_vector(const std::vector<double>& input_data_vector, ParallelVector* output_data_vector);
    // determine maximum absolute parallel error
    virtual double get_maximum_absolute_parallel_error(ParallelVector* input_data_vector);

protected:
    // for contraction
    std::vector<size_t> m_contracted_to_local;

    // for expansion
    std::vector<std::vector<size_t> > m_local_index_to_send;
    std::vector<std::vector<size_t> > m_local_index_to_recv;

};

}

}
