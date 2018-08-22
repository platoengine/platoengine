// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_Triangle.hpp"

#include "PSL_Point.hpp"

#include <cstddef>
#include <vector>
#include <cmath>
#include <cassert>

namespace PlatoSubproblemLibrary
{

Triangle::Triangle() :
                m_index()
{
    for(short local_index=0;local_index<PSL_CONSTANT_TRIANGLE_VERTS;local_index++) {
        m_vertex[local_index] = NULL;
    }
}

Triangle::~Triangle()
{
    for(short local_index=0;local_index<PSL_CONSTANT_TRIANGLE_VERTS;local_index++) {
        m_vertex[local_index] = NULL;
    }
}

// construct
Triangle::Triangle(size_t index, Point* vertex0, Point* vertex1, Point* vertex2) :
        m_index(index)
{
    m_vertex[0] = vertex0;
    m_vertex[1] = vertex1;
    m_vertex[2] = vertex2;
}

// set data
void Triangle::set_vertex(short local_index, Point* vertex)
{
    m_vertex[local_index] = vertex;
}
void Triangle::set_index(size_t index)
{
    m_index = index;
}

// get data
Point* Triangle::get_vertex(short local_index)
{
    return m_vertex[local_index];
}
size_t Triangle::get_index() const
{
    return m_index;
}

}
