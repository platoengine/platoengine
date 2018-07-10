// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

#include <cstddef>
#include <set>

namespace PlatoSubproblemLibrary
{
class NodeSet
{
public:
    NodeSet();
    NodeSet(size_t index, const std::set<size_t>& others);
    ~NodeSet();
    size_t size() const;
    void clear();
    void insert(size_t other);
    std::set<size_t> m_nodes;
    size_t get_index() const;
    void set_index(size_t index);
private:
    size_t m_index;
};
}
