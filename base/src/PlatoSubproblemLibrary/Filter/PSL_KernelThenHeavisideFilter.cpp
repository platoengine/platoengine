/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
*/

// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_KernelThenHeavisideFilter.hpp"

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

KernelThenHeavisideFilter::KernelThenHeavisideFilter(AbstractAuthority* authority,
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

KernelThenHeavisideFilter::~KernelThenHeavisideFilter()
{
    m_authority = NULL;
    m_input_data = NULL;
    m_original_points = NULL;
    m_parallel_exchanger = NULL;
    safe_free(m_kernel);
}

void KernelThenHeavisideFilter::set_authority(AbstractAuthority* authority)
{
    m_authority = authority;
}
void KernelThenHeavisideFilter::set_input_data(ParameterData* data)
{
    m_input_data = data;
}
void KernelThenHeavisideFilter::set_points(AbstractInterface::PointCloud* points)
{
    m_original_points = points;
}
void KernelThenHeavisideFilter::set_parallel_exchanger(AbstractInterface::ParallelExchanger* exchanger)
{
    m_parallel_exchanger = exchanger;
}

void KernelThenHeavisideFilter::announce_radius()
{
    m_announce_radius = true;
    if(m_kernel)
    {
        m_kernel->announce_radius();
    }
}

void KernelThenHeavisideFilter::build()
{
    assert(m_authority);
    assert(m_authority->utilities);
    if(m_built)
    {
        m_authority->utilities->print("KernelThenHeavisideFilter attempted to be built multiple times. Warning.\n\n");
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

void KernelThenHeavisideFilter::apply(AbstractInterface::ParallelVector* field)
{
    m_kernel->apply(field);

    // apply heaviside
    const size_t dimension = field->get_length();
    for(size_t i = 0u; i < dimension; i++)
    {
        field->set_value(i, heaviside_apply(m_current_heaviside_parameter, 0.5, field->get_value(i)));
    }
}
void KernelThenHeavisideFilter::apply(AbstractInterface::ParallelVector* base_field, AbstractInterface::ParallelVector* gradient)
{
    assert(m_kernel);

    // stash base field
    std::vector<double> base;
    base_field->get_values(base);

    // kernel filtered control
    m_kernel->apply(base_field);

    // scale gradient by heaviside projection contribution
    const size_t num_controls = base.size();
    for(size_t control_index = 0u; control_index < num_controls; control_index++)
    {
        double initial_gradient_value = gradient->get_value(control_index);
        double kerneled_value = base_field->get_value(control_index);
        double heaviside_derivative_value = heaviside_gradient(m_current_heaviside_parameter, 0.5, kerneled_value);
        gradient->set_value(control_index, initial_gradient_value * heaviside_derivative_value);
    }

    // finish projected gradient calculation by applying kernel filter
    base_field->set_values(base);
    m_kernel->apply(base_field, gradient);
}
void KernelThenHeavisideFilter::advance_continuation()
{
    m_current_heaviside_parameter = std::min(m_current_heaviside_parameter * m_heaviside_parameter_continuation_scale,
                                             m_max_heaviside_parameter);
}

void KernelThenHeavisideFilter::check_input_data()
{
    if(!m_input_data->didUserInput_min_heaviside_parameter())
    {
        m_authority->utilities->fatal_error("KernelThenHeavisideFilter: min_heaviside_parameter not set in input. Aborting.\n\n");
    }
    if(!m_input_data->didUserInput_heaviside_continuation_scale())
    {
        m_authority->utilities->fatal_error("KernelThenHeavisideFilter: heaviside_continuation_scale not set in input. Aborting.\n\n");
    }
    if(!m_input_data->didUserInput_max_heaviside_parameter())
    {
        m_authority->utilities->fatal_error("KernelThenHeavisideFilter: max_heaviside_parameter not set in input. Aborting.\n\n");
    }
}

}
