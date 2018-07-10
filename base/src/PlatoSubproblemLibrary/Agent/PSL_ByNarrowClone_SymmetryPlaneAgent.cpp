// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#include "PSL_ByNarrowClone_SymmetryPlaneAgent.hpp"

#include "PSL_Abstract_SymmetryPlaneAgent.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_Abstract_PointCloud.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_PointCloud.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Point.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <cstddef>
#include <algorithm>
#include <cmath>

namespace PlatoSubproblemLibrary
{

ByNarrowClone_SymmetryPlaneAgent::ByNarrowClone_SymmetryPlaneAgent(AbstractAuthority* authority) :
        Abstract_SymmetryPlaneAgent(symmetry_plane_agent_t::by_narrow_clone, authority),
        m_num_contracted_points(),
        m_indexes_to_clone()
{
}

ByNarrowClone_SymmetryPlaneAgent::~ByNarrowClone_SymmetryPlaneAgent()
{
}

PointCloud* ByNarrowClone_SymmetryPlaneAgent::build_kernel_points(double support_distance,
                                                                  double point_resolution_tolerance,
                                                                  ParameterData* input_data,
                                                                  AbstractInterface::PointCloud* all_points,
                                                                  const std::vector<size_t>& indexes_of_local_points)
{
    // read parameters
    read_input_data(input_data);
    const double multiplier_for_buffer = 2.01;
    m_symmetric_buffer_distance = support_distance * multiplier_for_buffer;
    m_point_resolution_tolerance = point_resolution_tolerance;

    // allocate
    m_num_contracted_points = indexes_of_local_points.size();
    m_indexes_to_clone.resize(0u);
    PointCloud* kernel_points = new PointCloud;
    std::vector<Point> kernel_point_vector(m_num_contracted_points);

    // fill kernel_point_vector
    for(size_t local_index = 0u; local_index < m_num_contracted_points; local_index++)
    {
        kernel_point_vector[local_index] = all_points->get_point(indexes_of_local_points[local_index]);
        kernel_point_vector[local_index].set_index(local_index);
    }

    // if no symmetry plane, do nothing
    if(!m_sym_plane_x && !m_sym_plane_y && !m_sym_plane_z)
    {
        kernel_points->assign(kernel_point_vector);
        return kernel_points;
    }

    // count symmetry points, and determine local plane side
    symPlane_side_t local_plane_side = symPlane_side_t::no_information;
    size_t num_reflection_points = 0;
    determine_num_reflection_points_and_local_plane_side(kernel_point_vector,
                                                         local_plane_side,
                                                         num_reflection_points);

    // get plane side from all processors
    const size_t mpi_size = m_authority->mpi_wrapper->get_size();
    std::vector<int> all_symPlane_sides(mpi_size, symPlane_side_t::both);
    m_authority->mpi_wrapper->all_gather(int(local_plane_side), all_symPlane_sides);

    // get global plane side
    symPlane_side_t global_plane_side = symPlane_side_t::no_information;
    determine_global_plane_side(all_symPlane_sides, global_plane_side);
    check_allowable_global_plane_side(global_plane_side);

    // create symmetric points
    m_indexes_to_clone.assign(num_reflection_points, 0u);
    kernel_point_vector.resize(m_num_contracted_points + num_reflection_points);
    size_t symmetric_counter_for_allocation = 0;
    for(size_t local_index = 0u; local_index < m_num_contracted_points; local_index++)
    {
        const double this_axial_value = get_symmetry_axial_value(kernel_point_vector[local_index]);

        if(is_reflection_point(this_axial_value))
        {
            // set symmetric point
            const size_t combined_index = m_num_contracted_points + symmetric_counter_for_allocation;
            kernel_point_vector[combined_index] = kernel_point_vector[local_index];
            reflect_point(kernel_point_vector[combined_index]);
            kernel_point_vector[combined_index].set_index(combined_index);

            // prepare clone
            m_indexes_to_clone[symmetric_counter_for_allocation] = local_index;

            symmetric_counter_for_allocation++;
        }
    }

    kernel_points->assign(kernel_point_vector);
    return kernel_points;
}

std::vector<double> ByNarrowClone_SymmetryPlaneAgent::expand_with_symmetry_points(const std::vector<double>& contracted_data_vector)
{
    // allocate
    const size_t num_clones = m_indexes_to_clone.size();
    std::vector<double> expanded_data_vector(m_num_contracted_points + num_clones);

    // fill contracted
    std::copy(contracted_data_vector.begin(), contracted_data_vector.end(), expanded_data_vector.begin());

    // fill clones
    for(size_t clone_index = 0u; clone_index < num_clones; clone_index++)
    {
        expanded_data_vector[m_num_contracted_points + clone_index] = contracted_data_vector[m_indexes_to_clone[clone_index]];
    }

    return expanded_data_vector;
}

std::vector<double> ByNarrowClone_SymmetryPlaneAgent::contract_by_symmetry_points(const std::vector<double>& expanded_data_vector)
{
    // contract
    std::vector<double> contracted_data_vector(&expanded_data_vector[0], &expanded_data_vector[m_num_contracted_points]);
    return contracted_data_vector;
}

void ByNarrowClone_SymmetryPlaneAgent::read_input_data(ParameterData* input_data)
{
    m_sym_plane_x = input_data->didUserInput_symPlane_X();
    m_sym_plane_y = input_data->didUserInput_symPlane_Y();
    m_sym_plane_z = input_data->didUserInput_symPlane_Z();

    // if multiple symmetry planes, abort
    if(int(m_sym_plane_x) + int(m_sym_plane_y) + int(m_sym_plane_z) > 1)
    {
        m_authority->utilities->fatal_error("ByNarrowClone_SymmetryPlaneAgent: multiple symmetry planes set. Aborting.\n\n");
    }

    if(input_data->didUserInput_symPlane_X())
    {
        m_symmetry_plane_axial_offset = input_data->get_symPlane_X();
    }
    else if(input_data->didUserInput_symPlane_Y())
    {
        m_symmetry_plane_axial_offset = input_data->get_symPlane_Y();
    }
    else if(input_data->didUserInput_symPlane_Z())
    {
        m_symmetry_plane_axial_offset = input_data->get_symPlane_Z();
    }
}

void ByNarrowClone_SymmetryPlaneAgent::determine_num_reflection_points_and_local_plane_side(const std::vector<Point>& kernel_point_vector,
                                                                                            symPlane_side_t& local_plane_side,
                                                                                            size_t& num_symmetric_points)
{
    const size_t num_points = kernel_point_vector.size();

    // for each point
    for(size_t index = 0; index < num_points; index++)
    {
        const double this_axial_value = get_symmetry_axial_value(kernel_point_vector[index]);

        symPlane_side_t this_point_plane_side = symPlane_side_t::no_information;

        num_symmetric_points += is_reflection_point(this_axial_value);

        // if node not on symmetry plane, determine node's side
        if(!is_on_symmetry_plane(this_axial_value))
        {
            if(this_axial_value < m_symmetry_plane_axial_offset)
            {
                this_point_plane_side = symPlane_side_t::negative;
            }
            else
            {
                this_point_plane_side = symPlane_side_t::positive;
            }
        }

        // determine this_point_plane_side's affect on local_plane_side
        switch(local_plane_side)
        {
            case symPlane_side_t::no_information:
            {
                local_plane_side = this_point_plane_side;
                break;
            }
            case symPlane_side_t::both:
            {
                break;
            }
            case symPlane_side_t::positive:
            {
                if(this_point_plane_side == symPlane_side_t::negative)
                {
                    local_plane_side = symPlane_side_t::both;
                }
                else
                {
                    local_plane_side = symPlane_side_t::positive;
                }
                break;
            }
            case symPlane_side_t::negative:
            {
                if(this_point_plane_side == symPlane_side_t::positive)
                {
                    local_plane_side = symPlane_side_t::both;
                }
                else
                {
                    local_plane_side = symPlane_side_t::negative;
                }
                break;
            }
            default:
            {
                m_authority->utilities->fatal_error("ByNarrowClone_SymmetryPlaneAgent: could not match local plane side. Aborting.\n\n");
                break;
            }
        }
    }
}

void ByNarrowClone_SymmetryPlaneAgent::determine_global_plane_side(const std::vector<int>& all_symPlane_sides,
                                                                   symPlane_side_t& global_plane_side)
{
    const size_t mpi_size = all_symPlane_sides.size();

    global_plane_side = symPlane_side_t(all_symPlane_sides[0]);
    for(size_t proc = 1; proc < mpi_size; proc++)
    {
        symPlane_side_t proc_symPlane_side = symPlane_side_t(all_symPlane_sides[proc]);
        switch(global_plane_side)
        {
            case symPlane_side_t::no_information:
            {
                global_plane_side = proc_symPlane_side;
                break;
            }
            case symPlane_side_t::both:
            {
                break;
            }
            case symPlane_side_t::positive:
            {
                // global == positive
                switch(proc_symPlane_side)
                {
                    case symPlane_side_t::no_information:
                    {
                        break;
                    }
                    case symPlane_side_t::both:
                    {
                        global_plane_side = proc_symPlane_side;
                        break;
                    }
                    case symPlane_side_t::positive:
                    {
                        break;
                    }
                    case symPlane_side_t::negative:
                    {
                        global_plane_side = symPlane_side_t::both;
                        break;
                    }
                    default:
                    {
                        m_authority->utilities->fatal_error("ByNarrowClone_SymmetryPlaneAgent: symmetry plane side case not handled. Aborting.\n\n");
                        break;
                    }
                }
                break;
            }
            case symPlane_side_t::negative:
            {
                // global == negative
                switch(proc_symPlane_side)
                {
                    case symPlane_side_t::no_information:
                    {
                        break;
                    }
                    case symPlane_side_t::both:
                    {
                        global_plane_side = proc_symPlane_side;
                        break;
                    }
                    case symPlane_side_t::positive:
                    {
                        global_plane_side = symPlane_side_t::both;
                        break;
                    }
                    case symPlane_side_t::negative:
                    {
                        break;
                    }
                    default:
                    {
                        m_authority->utilities->fatal_error("ByNarrowClone_SymmetryPlaneAgent: symmetry plane side case not handled. Aborting.\n\n");
                        break;
                    }
                }
                break;
            }
            default:
            {
                m_authority->utilities->fatal_error("ByNarrowClone_SymmetryPlaneAgent: symmetry plane side case not handled. Aborting.\n\n");
                break;
            }
        }
    }
}

void ByNarrowClone_SymmetryPlaneAgent::check_allowable_global_plane_side(symPlane_side_t global_plane_side)
{
    switch(global_plane_side)
    {
        case symPlane_side_t::no_information:
        {
            m_authority->utilities->fatal_error("ByNarrowClone_SymmetryPlaneAgent: unable to determine mesh side for symmetry plane. Aborting.\n\n");
            break;
        }
        case symPlane_side_t::both:
        {
            m_authority->utilities->fatal_error("ByNarrowClone_SymmetryPlaneAgent: determined that mesh exists on both sides of smoother symmetry plane; see documentation. Aborting.\n\n");
            break;
        }
        case symPlane_side_t::positive:
        case symPlane_side_t::negative:
        {
            break;
        }
        default:
        {
            m_authority->utilities->fatal_error("ByNarrowClone_SymmetryPlaneAgent: symmetry plane case not handled. Aborting.\n\n");
            break;
        }
    }
}

double ByNarrowClone_SymmetryPlaneAgent::get_symmetry_axial_value(const Point& point)
{
    if(m_sym_plane_x)
    {
        return point(0u);
    }
    else if(m_sym_plane_y)
    {
        return point(1u);
    }
    else if(m_sym_plane_z)
    {
        return point(2u);
    }
    m_authority->utilities->fatal_error("ByNarrowClone_SymmetryPlaneAgent: could not match symmetry plane axis. Aborting.\n\n");
    return 0.;
}

bool ByNarrowClone_SymmetryPlaneAgent::is_reflection_point(double axial_value)
{
    // if node not on symmetry plane
    if(!is_on_symmetry_plane(axial_value))
    {
        // if node within symmetry buffer
        if(std::fabs(axial_value - m_symmetry_plane_axial_offset) <= m_point_resolution_tolerance + m_symmetric_buffer_distance)
        {
            return true;
        }
    }

    return false;
}

void ByNarrowClone_SymmetryPlaneAgent::reflect_point(Point& to_reflect)
{
    if(m_sym_plane_x)
    {
        to_reflect.set(0u, 2 * m_symmetry_plane_axial_offset - to_reflect(0u));
    }
    else if(m_sym_plane_y)
    {
        to_reflect.set(1u, 2 * m_symmetry_plane_axial_offset - to_reflect(1u));
    }
    else if(m_sym_plane_z)
    {
        to_reflect.set(2u, 2 * m_symmetry_plane_axial_offset - to_reflect(2u));
    }
    else
    {
        m_authority->utilities->fatal_error("ByNarrowClone_SymmetryPlaneAgent: could not match symmetry plane axis. Aborting.\n\n");
    }
}

double ByNarrowClone_SymmetryPlaneAgent::is_on_symmetry_plane(double this_point_axial_value)
{
    return (std::fabs(this_point_axial_value - m_symmetry_plane_axial_offset) < m_point_resolution_tolerance);
}

}
