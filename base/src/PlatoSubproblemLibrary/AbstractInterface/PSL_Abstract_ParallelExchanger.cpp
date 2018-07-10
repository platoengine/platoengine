// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Abstract_ParallelExchanger.hpp"

#include "PSL_Abstract_ParallelVector.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_AbstractAuthority.hpp"

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{

ParallelExchanger::ParallelExchanger(AbstractAuthority* authority) :
        m_authority(authority)
{

}

ParallelExchanger::~ParallelExchanger()
{

}

MpiWrapper* ParallelExchanger::get_mpi()
{
    return m_authority->mpi_wrapper;
}

}
}
