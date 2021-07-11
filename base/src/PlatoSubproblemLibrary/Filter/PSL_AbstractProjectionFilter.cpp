// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.

#include "PSL_AbstractProjectionFilter.hpp"

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
#include "PSL_AbstractAuthority.hpp"

#include <cassert>
#include <vector>
#include <cstddef>
#include <sstream>
#include <cmath>
#include <algorithm>


namespace PlatoSubproblemLibrary
{

namespace AbstractInterface
{
class PointCloud;
class ParallelVector;
class ParallelExchanger;
}
class ParameterData;
class AbstractAuthority;

AbstractProjectionFilter::AbstractProjectionFilter(AbstractAuthority* authority,
                           ParameterData* data,
                           AbstractInterface::PointCloud* points,
                           AbstractInterface::ParallelExchanger* exchanger) :
        Filter(),
        m_built(false),
        m_authority(authority),
        m_input_data(data),
        m_original_points(points),
        m_parallel_exchanger(exchanger),
        m_current_heaviside_parameter(-1.),
        m_heaviside_parameter_continuation_scale(-1.),
        m_max_heaviside_parameter(-1.)
{
}

AbstractProjectionFilter::~AbstractProjectionFilter()
{
    m_authority = NULL;
    m_input_data = NULL;
    m_original_points = NULL;
    m_parallel_exchanger = NULL;
}

void AbstractProjectionFilter::set_authority(AbstractAuthority* authority)
{
    m_authority = authority;
}

void AbstractProjectionFilter::set_input_data(ParameterData* data)
{
    m_input_data = data;
}

void AbstractProjectionFilter::set_points(AbstractInterface::PointCloud* points)
{
    m_original_points = points;
}

void AbstractProjectionFilter::set_parallel_exchanger(AbstractInterface::ParallelExchanger* exchanger)
{
    m_parallel_exchanger = exchanger;
}

void AbstractProjectionFilter::build()
{
    assert(m_authority);
    assert(m_authority->utilities);
    if(m_built)
    {
        m_authority->utilities->print("AbstractProjectionFilter attempted to be built multiple times. Warning.\n\n");
        return;
    }
    m_built = true;

    // check parameters
    check_input_data();

    // transfer parameters
    m_current_heaviside_parameter =  m_input_data->get_min_heaviside_parameter();
    m_heaviside_parameter_continuation_scale =  m_input_data->get_heaviside_continuation_scale();
    m_max_heaviside_parameter =  m_input_data->get_max_heaviside_parameter();
}

void AbstractProjectionFilter::apply(AbstractInterface::ParallelVector* field)
{
    // apply heaviside
    const size_t dimension = field->get_length();
    for(size_t i = 0u; i < dimension; i++)
    {
        field->set_value(i, projection_apply(m_current_heaviside_parameter, field->get_value(i)));
    }
}

void AbstractProjectionFilter::apply(AbstractInterface::ParallelVector* base_field, AbstractInterface::ParallelVector* gradient)
{
    // stash base field
    std::vector<double> base;
    base_field->get_values(base);
    
    // scale gradient by heaviside projection contribution
    const size_t num_controls = base.size();
    for(size_t control_index = 0u; control_index < num_controls; control_index++)
    {
        double initial_gradient_value = gradient->get_value(control_index);
        double filtered_value = base_field->get_value(control_index);
        double heaviside_derivative_value = projection_gradient(m_current_heaviside_parameter, filtered_value);
        gradient->set_value(control_index, initial_gradient_value * heaviside_derivative_value);
    }
}

void AbstractProjectionFilter::advance_continuation()
{
    m_current_heaviside_parameter = std::min(m_current_heaviside_parameter * m_heaviside_parameter_continuation_scale,
                                             m_max_heaviside_parameter);
}

void AbstractProjectionFilter::additive_advance_continuation()
{
    m_current_heaviside_parameter = std::min(m_current_heaviside_parameter + m_heaviside_parameter_continuation_scale,
                                             m_max_heaviside_parameter);
}

void AbstractProjectionFilter::check_input_data()
{
    if(!m_input_data->didUserInput_min_heaviside_parameter())
    {
        m_authority->utilities->fatal_error("Filter: min_heaviside_parameter not set in input. Aborting.\n\n");
    }
    if(!m_input_data->didUserInput_heaviside_continuation_scale())
    {
        m_authority->utilities->fatal_error("Filter: heaviside_continuation_scale not set in input. Aborting.\n\n");
    }
    if(!m_input_data->didUserInput_max_heaviside_parameter())
    {
        m_authority->utilities->fatal_error("Filter: max_heaviside_parameter not set in input. Aborting.\n\n");
    }
}

}
