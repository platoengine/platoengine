// PlatoSubproblemLibraryVersion(6): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_ParallelExchanger_Managed.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class ParallelVector;
}
class AbstractAuthority;

namespace example
{

class Interface_ParallelExchanger_localAndNonlocal : public AbstractInterface::ParallelExchanger_Managed
{
public:
    Interface_ParallelExchanger_localAndNonlocal(AbstractAuthority* authority);
    virtual ~Interface_ParallelExchanger_localAndNonlocal();

    // put local and nonlocal shared pairs
    void put_shared_pairs(std::vector<std::vector<std::pair<size_t, size_t> > >& shared_local_and_nonlocal_pairs);
    void put_num_local_locations(size_t num_locations);

    virtual void build();

protected:

    virtual void build_node_ownership(std::vector<int>& node_ownership);

    // received data
    std::vector<std::vector<std::pair<size_t, size_t> > > m_processor_then_vector_of_local_and_nonlocal_pairs;
    size_t m_num_locations;

};

}
}
