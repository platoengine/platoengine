// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.

#include "PSL_AbstractKernelThenProjection.hpp"

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
#include "PSL_KernelFilter.hpp"

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
class KernelFilter;

AbstractKernelThenProjection::AbstractKernelThenProjection(AbstractAuthority* authority,
                           ParameterData* data,
                           AbstractInterface::PointCloud* points,
                           AbstractInterface::ParallelExchanger* exchanger) :
        AbstractKernelThenFilter(authority, data, points, exchanger),
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

AbstractKernelThenProjection::~AbstractKernelThenProjection()
{
    m_authority = NULL;
    m_input_data = NULL;
    m_original_points = NULL;
    m_parallel_exchanger = NULL;
    safe_free(m_kernel);
}

void AbstractKernelThenProjection::build()
{
    AbstractKernelThenFilter::build();

    // transfer parameters
    m_current_heaviside_parameter =  m_input_data->get_min_heaviside_parameter();
    m_heaviside_parameter_continuation_scale =  m_input_data->get_heaviside_continuation_scale();
    m_max_heaviside_parameter =  m_input_data->get_max_heaviside_parameter();
}

void AbstractKernelThenProjection::advance_continuation()
{
    m_current_heaviside_parameter = std::min(m_current_heaviside_parameter * m_heaviside_parameter_continuation_scale,
                                             m_max_heaviside_parameter);
}
void AbstractKernelThenProjection::additive_advance_continuation()
{
    m_current_heaviside_parameter = std::min(m_current_heaviside_parameter + m_heaviside_parameter_continuation_scale,
                                             m_max_heaviside_parameter);
}

void AbstractKernelThenProjection::check_input_data()
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

void AbstractKernelThenProjection::internal_apply(AbstractInterface::ParallelVector* field)
{
    projection_apply(m_current_heaviside_parameter, field);
}

void AbstractKernelThenProjection::internal_gradient(AbstractInterface::ParallelVector* const field, AbstractInterface::ParallelVector* gradient) const
{
        projection_gradient(m_current_heaviside_parameter, field, gradient);
}

}
