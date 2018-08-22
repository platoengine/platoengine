// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

/* An ordered collection of points in an arbitrary dimension.
 *
 * Point are accessed in this class by an indexing order in the cloud.
 * This ordering may not reflect the individual indexes of the points (be careful).
 */

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
class AxisAlignedBoundingBox;
class Point;

class PointCloud {
public:
    PointCloud();
    PointCloud(PointCloud* other, const std::vector<size_t>& indexes);
    ~PointCloud();

    void resize(size_t num_points);
    void assign(std::vector<Point>& points);
    void assign(size_t index, const Point& point);
    void push_back(const Point& point);
    size_t get_num_points() const;
    Point* get_point(size_t index);

    AxisAlignedBoundingBox get_bound();

protected:
    std::vector<Point> m_points;

};

}
