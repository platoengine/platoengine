// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_SymmetryPlaneAgent.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
enum symPlane_side_t
{
    positive, negative, both, no_information
};

namespace AbstractInterface
{
class PointCloud;
}
class AbstractAuthority;
class ParameterData;
class Point;

class ByNarrowClone_SymmetryPlaneAgent : public Abstract_SymmetryPlaneAgent
{
public:
    ByNarrowClone_SymmetryPlaneAgent(AbstractAuthority* authority);
    virtual ~ByNarrowClone_SymmetryPlaneAgent();

    virtual PointCloud* build_kernel_points(double support_distance,
                                            double point_resolution_tolerance,
                                            ParameterData* input_data,
                                            AbstractInterface::PointCloud* all_points,
                                            const std::vector<size_t>& indexes_of_local_points);
    virtual std::vector<double> expand_with_symmetry_points(const std::vector<double>& contracted_data_vector);
    virtual std::vector<double> contract_by_symmetry_points(const std::vector<double>& expanded_data_vector);

protected:
    void read_input_data(ParameterData* input_data);

    void determine_num_reflection_points_and_local_plane_side(const std::vector<Point>& kernel_point_vector,
                                                              symPlane_side_t& local_plane_side,
                                                              size_t& num_symmetric_points);
    void determine_global_plane_side(const std::vector<int>& all_symPlane_sides, symPlane_side_t& global_plane_side);
    void check_allowable_global_plane_side(symPlane_side_t global_plane_side);
    double get_symmetry_axial_value(const Point& point);
    bool is_reflection_point(double axial_value);
    void reflect_point(Point& to_reflect);
    double is_on_symmetry_plane(double this_point_axial_value);

    size_t m_num_contracted_points;
    std::vector<size_t> m_indexes_to_clone;

    // for build
    bool m_sym_plane_x;
    bool m_sym_plane_y;
    bool m_sym_plane_z;
    double m_symmetry_plane_axial_offset;
    double m_symmetric_buffer_distance;
    double m_point_resolution_tolerance;

};

}
