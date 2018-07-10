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

#include "PlatoEngine_KernelFilter.hpp"

#include "PSL_KernelFilter.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Interface_MpiWrapper.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Interface_ParallelVector.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_Abstract_PointCloud.hpp"
#include "PSL_Abstract_ParallelExchanger.hpp"
#include "PSL_Abstract_SparseMatrixBuilder.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_Abstract_PositiveDefiniteLinearSolver.hpp"
#include "PSL_Interface_SparseMatrixBuilder.hpp"
#include "PSL_BoundingBoxMortonHierarchy.hpp"
#include "PSL_Interface_DenseMatrixBuilder.hpp"
#include "PSL_Interface_BasicDenseVectorOperations.hpp"
#include "PSL_Interface_ConjugateGradient.hpp"
#include "PSL_InterfaceToEngine_ParameterDataBuilder.hpp"
#include "PSL_InterfaceToEngine_OptimizationMesh.hpp"
#include "PSL_Interface_ParallelExchanger_global.hpp"
#include "Plato_Interface.hpp"
#include "data_mesh.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <iostream>
#include <algorithm>

namespace Plato
{

KernelFilter::KernelFilter() :
        AbstractFilter(),
        m_comm(),
        m_kernel(NULL),
        m_input_data(NULL),
        m_points(NULL),
        m_parallel_exchanger(NULL),
        m_maximum_absolute_parallel_error_tolerance(1e-3),
        m_validate_interface(false)
{
}

#define PlatoEngine_KernelFilter_SafeFree(_1) \
    if(_1) { \
        delete _1; \
        _1 = NULL; \
    }

KernelFilter::~KernelFilter()
{
    PlatoEngine_KernelFilter_SafeFree(m_kernel);
    PlatoEngine_KernelFilter_SafeFree(m_input_data);
    PlatoEngine_KernelFilter_SafeFree(m_points);
    PlatoEngine_KernelFilter_SafeFree(m_parallel_exchanger);
    PlatoEngine_KernelFilter_SafeFree(m_authority);
}

void KernelFilter::build(InputData aInputData, MPI_Comm& aLocalComm, DataMesh* aMesh)
{
    m_comm = aLocalComm;

    // build and allocate needed functionalities
    m_authority = new PlatoSubproblemLibrary::AbstractAuthority(&m_comm);
    build_input_data(aInputData);
    build_points(aMesh);
    build_parallel_exchanger(aMesh);

    // allocate filter
    m_kernel = new PlatoSubproblemLibrary::KernelFilter(m_authority,
                                                        m_input_data,
                                                        m_points,
                                                        m_parallel_exchanger);

    // enable announcing chosen radius
    m_kernel->announce_radius();

    // build kernel
    m_kernel->build();

    if(m_validate_interface)
    {
        // confirm validate
        const size_t local_num_nodes = m_points->get_num_points();
        PlatoSubproblemLibrary::example::Interface_ParallelVector pv(std::vector<double>(local_num_nodes, 0.));
        if(!m_kernel->is_valid(&pv))
        {
            m_authority->utilities->fatal_error("KernelFilter::build invalid specification to kernel. Aborting. \n\n");
        }
        else
        {
            m_authority->utilities->print("KernelFilter::build is_valid confirmed.\n");
        }
    }
}

void KernelFilter::apply_on_field(size_t length, double* field_data)
{
    std::vector<double> input(field_data, field_data + length);
    PlatoSubproblemLibrary::example::Interface_ParallelVector pv(input);

    if(m_validate_interface)
    {
        const double initial_parallel_error = m_parallel_exchanger->get_maximum_absolute_parallel_error(&pv);
        if(m_maximum_absolute_parallel_error_tolerance < initial_parallel_error)
        {
            m_authority->utilities->fatal_error("KernelFilter::apply_on_field high parallel error before apply_on_field. Aborting. \n\n");
        }
    }

    m_kernel->apply(&pv);

    if(m_validate_interface)
    {
        const double final_parallel_error = m_parallel_exchanger->get_maximum_absolute_parallel_error(&pv);
        if(m_maximum_absolute_parallel_error_tolerance < final_parallel_error)
        {
            m_authority->utilities->fatal_error("KernelFilter::apply_on_field high parallel error after apply_on_field. Aborting. \n\n");
        }
    }

    std::copy(pv.m_data.begin(), pv.m_data.end(), field_data);
}

void KernelFilter::apply_on_gradient(size_t length, double* gradient_data)
{
    std::vector<double> input(gradient_data, gradient_data + length);
    PlatoSubproblemLibrary::example::Interface_ParallelVector pv(input);

    if(m_validate_interface)
    {
        const double initial_parallel_error = m_parallel_exchanger->get_maximum_absolute_parallel_error(&pv);
        if(m_maximum_absolute_parallel_error_tolerance < initial_parallel_error)
        {
            m_authority->utilities->fatal_error("KernelFilter::apply_on_gradient high parallel error before apply_on_gradient. Aborting. \n\n");
        }
    }

    m_kernel->apply(NULL, &pv);

    if(m_validate_interface)
    {
        const double final_parallel_error = m_parallel_exchanger->get_maximum_absolute_parallel_error(&pv);
        if(m_maximum_absolute_parallel_error_tolerance < final_parallel_error)
        {
            m_authority->utilities->fatal_error("KernelFilter::apply_on_gradient high parallel error after apply_on_gradient. Aborting. \n\n");
        }
    }

    std::copy(pv.m_data.begin(), pv.m_data.end(), gradient_data);
}

void KernelFilter::build_input_data(InputData aInputData)
{
    Plato::InterfaceToEngine_ParameterDataBuilder builder(aInputData);
    m_input_data = builder.build();
}

void KernelFilter::build_points(DataMesh* mesh)
{
    m_points = new Plato::InterfaceToEngine_OptimizationMesh(mesh);
}

void KernelFilter::build_parallel_exchanger(DataMesh* mesh)
{
    // get globals
    const size_t num_nodes = mesh->getNumNodes();
    std::vector<size_t> globals(mesh->nodeGlobalIds,mesh->nodeGlobalIds+num_nodes);

    // build
    m_parallel_exchanger = new PlatoSubproblemLibrary::example::Interface_ParallelExchanger_global(m_authority);
    ((PlatoSubproblemLibrary::example::Interface_ParallelExchanger_global*) m_parallel_exchanger)->put_globals(globals);
    ((PlatoSubproblemLibrary::example::Interface_ParallelExchanger_global*) m_parallel_exchanger)->build();

}

}
