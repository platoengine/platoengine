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

#include "PlatoEngine_AbstractProjectionFilter.hpp"

#include "PSL_AbstractProjectionFilter.hpp"
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

AbstractProjectionFilter::AbstractProjectionFilter() :
        AbstractFilter(),
        m_comm(),
        mAdvanceContinuationIteration(0),
        mStartIteration(0),
        mUpdateInterval(1),
        mAdditiveContinuation(false),
        m_filter(NULL),
        m_authority(NULL),
        m_input_data(NULL),
        m_points(NULL),
        m_parallel_exchanger(NULL)
{
}

#define PlatoEngine_AbstractProjectionFilter_SafeFree(_1) \
    if(_1) { \
        delete _1; \
        _1 = NULL; \
    }

AbstractProjectionFilter::~AbstractProjectionFilter()
{
    PlatoEngine_AbstractProjectionFilter_SafeFree(m_filter);
    PlatoEngine_AbstractProjectionFilter_SafeFree(m_input_data);
    PlatoEngine_AbstractProjectionFilter_SafeFree(m_points);
    PlatoEngine_AbstractProjectionFilter_SafeFree(m_parallel_exchanger);
    PlatoEngine_AbstractProjectionFilter_SafeFree(m_authority);
}

void AbstractProjectionFilter::build(InputData aInputData, MPI_Comm& aLocalComm, DataMesh* aMesh)
{
    m_comm = aLocalComm;

    // build and allocate needed functionalities
    m_authority = new PlatoSubproblemLibrary::AbstractAuthority(&m_comm);
    build_input_data(aInputData);
    build_points(aMesh);
    build_parallel_exchanger(aMesh);

    //set variables for continuation update
    auto tFilterNode = aInputData.get<Plato::InputData>("Filter");
    if(tFilterNode.size<std::string>("UpdateInterval") > 0)
    {
        mUpdateInterval = Plato::Get::Double(tFilterNode, "UpdateInterval");
    }
    if(tFilterNode.size<std::string>("StartIteration") > 0)
    {
        mStartIteration = Plato::Get::Double(tFilterNode, "StartIteration");
    }
    if(tFilterNode.size<std::string>("UseAdditiveContinuation") > 0)
    {
        mAdditiveContinuation = Plato::Get::Bool(tFilterNode, "UseAdditiveContinuation");
    }

    allocateFilter();

    // build filter
    m_filter->build();
}

void AbstractProjectionFilter::apply_on_field(size_t length, double* field_data)
{
    // build parallel vector
    std::vector<double> input(field_data, field_data + length);
    PlatoSubproblemLibrary::example::Interface_ParallelVector pv(input);

    // do filter
    m_filter->apply(&pv);

    // transfer field data back
    std::copy(pv.m_data.begin(), pv.m_data.end(), field_data);
}

void AbstractProjectionFilter::apply_on_gradient(size_t length, double* base_field_data, double* gradient_data)
{
    // build parallel vectors
    std::vector<double> input_base_field(base_field_data, base_field_data + length);
    PlatoSubproblemLibrary::example::Interface_ParallelVector pv_base_field(input_base_field);
    std::vector<double> input_gradient(gradient_data, gradient_data + length);
    PlatoSubproblemLibrary::example::Interface_ParallelVector pv_gradient(input_gradient);

    // do filter
    m_filter->apply(&pv_base_field, &pv_gradient);

    // transfer gradient data back
    std::copy(pv_gradient.m_data.begin(), pv_gradient.m_data.end(), gradient_data);
}

void AbstractProjectionFilter::advance_continuation()
{
    if(mAdvanceContinuationIteration >= mStartIteration &&(mAdvanceContinuationIteration - mStartIteration) % mUpdateInterval == 0)
    {
      if(mAdditiveContinuation)
        m_filter->additive_advance_continuation();
      else
        m_filter->advance_continuation();
    }

    mAdvanceContinuationIteration++;
}

void AbstractProjectionFilter::build_input_data(InputData aInputData)
{
    Plato::InterfaceToEngine_ParameterDataBuilder builder(aInputData);
    m_input_data = builder.build();
}

void AbstractProjectionFilter::build_points(DataMesh* mesh)
{
    m_points = new Plato::InterfaceToEngine_OptimizationMesh(mesh);
}

void AbstractProjectionFilter::build_parallel_exchanger(DataMesh* mesh)
{
    // get globals
    const size_t num_nodes = mesh->getNumNodes();
    std::vector<size_t> globals(mesh->nodeGlobalIds, mesh->nodeGlobalIds + num_nodes);

    // build
    m_parallel_exchanger = new PlatoSubproblemLibrary::example::Interface_ParallelExchanger_global(m_authority);
    ((PlatoSubproblemLibrary::example::Interface_ParallelExchanger_global*) m_parallel_exchanger)->put_globals(globals);
    ((PlatoSubproblemLibrary::example::Interface_ParallelExchanger_global*) m_parallel_exchanger)->build();

}

}
