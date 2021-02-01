// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.

#include "PSL_AbstractKernelThenFilter.hpp"

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
#include <iostream>


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

AbstractKernelThenFilter::AbstractKernelThenFilter(AbstractAuthority* authority,
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
        m_kernel(NULL)
{
}

AbstractKernelThenFilter::~AbstractKernelThenFilter()
{
    m_authority = NULL;
    m_input_data = NULL;
    m_original_points = NULL;
    m_parallel_exchanger = NULL;
    safe_free(m_kernel);
}

void AbstractKernelThenFilter::set_authority(AbstractAuthority* authority)
{
    m_authority = authority;
}
void AbstractKernelThenFilter::set_input_data(ParameterData* data)
{
    m_input_data = data;
}
void AbstractKernelThenFilter::set_points(AbstractInterface::PointCloud* points)
{
    m_original_points = points;
}
void AbstractKernelThenFilter::set_parallel_exchanger(AbstractInterface::ParallelExchanger* exchanger)
{
    m_parallel_exchanger = exchanger;
}
void AbstractKernelThenFilter::announce_radius()
{
    m_announce_radius = true;
    if(m_kernel)
    {
        m_kernel->announce_radius();
    }
}
void AbstractKernelThenFilter::build()
{
    assert(m_authority);
    assert(m_authority->utilities);
    if(m_built)
    {
        m_authority->utilities->print("AbstractKernelThenFilter attempted to be built multiple times. Warning.\n\n");
        return;
    }
    m_built = true;

    // build kernel
    safe_free(m_kernel);
    m_kernel = new KernelFilter(m_authority, m_input_data, m_original_points, m_parallel_exchanger);
    m_kernel->build();
    if(m_announce_radius)
    {
        m_kernel->announce_radius();
    }
}

void AbstractKernelThenFilter::apply(AbstractInterface::ParallelVector* field)
{
    m_kernel->apply(field);

    // apply post filter
    internal_apply(field);
}

void AbstractKernelThenFilter::apply(AbstractInterface::ParallelVector* base_field, AbstractInterface::ParallelVector* gradient)
{
    assert(m_kernel);

    // stash base field
    std::vector<double> base;
    base_field->get_values(base);

    // kernel filtered control
    m_kernel->apply(base_field);

    // apply post filter gradient 
    internal_gradient(base_field, gradient);

    // finish gradient calculation by applying kernel filter
    base_field->set_values(base);
    m_kernel->apply(base_field, gradient);
}

}
