// PlatoSubproblemLibraryVersion(6): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Interface_ParallelExchanger_localAndNonlocal.hpp"

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

class Interface_ParallelExchanger_global : public Interface_ParallelExchanger_localAndNonlocal
{
public:
    Interface_ParallelExchanger_global(AbstractAuthority* authority);
    virtual ~Interface_ParallelExchanger_global();

    // put global identifiers for local indexes
    void put_globals(const std::vector<size_t>& globals);

    virtual void build();

protected:

    void build_send_first(size_t num_local,
                          size_t proc,
                          const std::vector<int>& global_minimum_indexes,
                          const std::vector<int>& global_maximum_indexes,
                          const std::vector<std::pair<size_t, size_t> >& global_and_local_pair,
                          std::vector<std::vector<std::pair<size_t, size_t> > >& shared_local_and_nonlocal_pairs);
    void build_receive_first(size_t num_local,
                             size_t proc,
                             const std::vector<std::pair<size_t, size_t> >& global_and_local_pair,
                             std::vector<std::vector<std::pair<size_t, size_t> > >& shared_local_and_nonlocal_pairs);

    bool intersect(int a_lower, int a_upper, int b_lower, int b_upper);

    // received data
    std::vector<size_t> m_globals;

};

}
}
