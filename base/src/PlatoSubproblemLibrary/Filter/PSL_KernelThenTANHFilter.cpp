// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_KernelThenTANHFilter.hpp"

#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Abstract_ParallelExchanger.hpp"
#include "PSL_Abstract_ParallelVector.hpp"
#include "PSL_Abstract_PointCloud.hpp"
#include "PSL_Abstract_MatrixAssemblyAgent.hpp"
#include "PSL_ByRow_MatrixAssemblyAgent.hpp"
#include "PSL_Abstract_SymmetryPlaneAgent.hpp"
#include "PSL_ByNarrowClone_SymmetryPlaneAgent.hpp"
#include "PSL_Abstract_MeshScaleAgent.hpp"
#include "PSL_ByOptimizedElementSide_MeshScaleAgent.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_PointCloud.hpp"
#include "PSL_Abstract_MatrixNormalizationAgent.hpp"
#include "PSL_Default_MatrixNormalizationAgent.hpp"
#include "PSL_Abstract_SparseMatrixBuilder.hpp"
#include "PSL_Abstract_FixedRadiusNearestNeighborsSearcher.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_SparseMatrix.hpp"
#include "PSL_Abstract_PositiveDefiniteLinearSolver.hpp"
#include "PSL_Abstract_PointGhostingAgent.hpp"
#include "PSL_ByNarrowShare_PointGhostingAgent.hpp"
#include "PSL_Abstract_BoundedSupportFunction.hpp"
#include "PSL_BoundedSupportFunctionFactory.hpp"
#include "PSL_Point.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_KernelFilter.hpp"

#include <cassert>
#include <vector>
#include <cstddef>
#include <sstream>
#include <cmath>
#include <algorithm>

namespace PlatoSubproblemLibrary
{

KernelThenTANHFilter::KernelThenTANHFilter(AbstractAuthority* authority,
                           ParameterData* data,
                           AbstractInterface::PointCloud* points,
                           AbstractInterface::ParallelExchanger* exchanger) :
        Filter(),
        m_built(false),
        m_announce_radius(false),
        m_authority(authority),
        m_input_data(data),
        m_original_points(points),
        m_parallel_exchanger(exchanger),
        m_kernel(NULL),
        m_current_heaviside_parameter(-1.),
        m_heaviside_parameter_continuation_scale(-1.),
        m_max_heaviside_parameter(-1.)
{
}

KernelThenTANHFilter::~KernelThenTANHFilter()
{
    m_authority = NULL;
    m_input_data = NULL;
    m_original_points = NULL;
    m_parallel_exchanger = NULL;
    safe_free(m_kernel);
}

void KernelThenTANHFilter::set_authority(AbstractAuthority* authority)
{
    m_authority = authority;
}
void KernelThenTANHFilter::set_input_data(ParameterData* data)
{
    m_input_data = data;
}
void KernelThenTANHFilter::set_points(AbstractInterface::PointCloud* points)
{
    m_original_points = points;
}
void KernelThenTANHFilter::set_parallel_exchanger(AbstractInterface::ParallelExchanger* exchanger)
{
    m_parallel_exchanger = exchanger;
}

void KernelThenTANHFilter::announce_radius()
{
    m_announce_radius = true;
    if(m_kernel)
    {
        m_kernel->announce_radius();
    }
}

void KernelThenTANHFilter::build()
{
    assert(m_authority);
    assert(m_authority->utilities);
    if(m_built)
    {
        m_authority->utilities->print("KernelThenTANHFilter attempted to be built multiple times. Warning.\n\n");
        return;
    }
    m_built = true;

    // check parameters
    check_input_data();

    // build kernel
    safe_free(m_kernel);
    m_kernel = new KernelFilter(m_authority, m_input_data, m_original_points, m_parallel_exchanger);
    m_kernel->build();
    if(m_announce_radius)
    {
        m_kernel->announce_radius();
    }

    // transfer parameters
    m_current_heaviside_parameter =  m_input_data->get_min_heaviside_parameter();
    m_heaviside_parameter_continuation_scale =  m_input_data->get_heaviside_continuation_scale();
    m_max_heaviside_parameter =  m_input_data->get_max_heaviside_parameter();
}

void KernelThenTANHFilter::apply(AbstractInterface::ParallelVector* field)
{
    m_kernel->apply(field);

    // apply tanh projection
    const size_t dimension = field->get_length();
    for(size_t i = 0u; i < dimension; i++)
    {
        field->set_value(i, tanh_apply(m_current_heaviside_parameter, field->get_value(i)));
    }
}
void KernelThenTANHFilter::apply(AbstractInterface::ParallelVector* base_field, AbstractInterface::ParallelVector* gradient)
{
    assert(m_kernel);

    // stash base field
    std::vector<double> base;
    base_field->get_values(base);

    // kernel filtered control
    m_kernel->apply(base_field);

    // scale gradient by tanh projection contribution
    const size_t num_controls = base.size();
    for(size_t control_index = 0u; control_index < num_controls; control_index++)
    {
        double initial_gradient_value = gradient->get_value(control_index);
        double kerneled_value = base_field->get_value(control_index);
        double heaviside_derivative_value = tanh_gradient(m_current_heaviside_parameter, kerneled_value);
        gradient->set_value(control_index, initial_gradient_value * heaviside_derivative_value);
    }

    // finish projected gradient calculation by applying kernel filter
    base_field->set_values(base);
    m_kernel->apply(base_field, gradient);
}
void KernelThenTANHFilter::advance_continuation()
{
    m_current_heaviside_parameter = std::min(m_current_heaviside_parameter * m_heaviside_parameter_continuation_scale,
                                             m_max_heaviside_parameter);
}

void KernelThenTANHFilter::check_input_data()
{
    if(!m_input_data->didUserInput_min_heaviside_parameter())
    {
        m_authority->utilities->fatal_error("KernelThenTANHFilter: min_heaviside_parameter not set in input. Aborting.\n\n");
    }
    if(!m_input_data->didUserInput_heaviside_continuation_scale())
    {
        m_authority->utilities->fatal_error("KernelThenTANHFilter: heaviside_continuation_scale not set in input. Aborting.\n\n");
    }
    if(!m_input_data->didUserInput_max_heaviside_parameter())
    {
        m_authority->utilities->fatal_error("KernelThenTANHFilter: max_heaviside_parameter not set in input. Aborting.\n\n");
    }
}

}
