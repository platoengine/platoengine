// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

/* A Triangle of three points.
 */

#include "PSL_Point.hpp"
#include "PSL_Constants.hpp"

#include <cstddef>
#include <vector>

namespace PlatoSubproblemLibrary
{

class Triangle {
public:
    Triangle();
    ~Triangle();

    // construct
    Triangle(size_t index, Point* vertex0, Point* vertex1, Point* vertex2);

    // set data
    void set_vertex(short local_index, Point* vertex);
    void set_index(size_t index);

    // get data
    Point* get_vertex(short local_index);
    size_t get_index() const;

private:
    size_t m_index;
    Point* m_vertex[PSL_CONSTANT_TRIANGLE_VERTS];
};

}
