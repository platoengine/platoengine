// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_NodeSet.hpp"

namespace PlatoSubproblemLibrary
{

NodeSet::NodeSet() :
        m_nodes(),
        m_index()
{
}

NodeSet::NodeSet(size_t index, const std::set<size_t>& others) :
        m_nodes(others),
        m_index(index)
{
}

NodeSet::~NodeSet()
{
}

size_t NodeSet::size() const
{
    return m_nodes.size();
}

void NodeSet::clear()
{
    m_nodes.clear();
}

void NodeSet::insert(size_t other)
{
    m_nodes.insert(other);
}

size_t NodeSet::get_index() const
{
    return m_index;
}

void NodeSet::set_index(size_t index)
{
    m_index = index;
}

}
