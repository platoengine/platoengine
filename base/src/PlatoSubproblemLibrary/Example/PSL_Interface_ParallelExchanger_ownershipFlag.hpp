// PlatoSubproblemLibraryVersion(6): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_ParallelExchanger_Managed.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;

namespace example
{

class Interface_ParallelExchanger_ownershipFlag : public AbstractInterface::ParallelExchanger_Managed
{
public:
    Interface_ParallelExchanger_ownershipFlag(AbstractAuthority* authority);
    virtual ~Interface_ParallelExchanger_ownershipFlag();

    // put ownership
    void put_is_owned(const std::vector<bool>& is_owned);

    virtual void build();

protected:

    // received data
    std::vector<bool> m_is_owned;

};

}
}
