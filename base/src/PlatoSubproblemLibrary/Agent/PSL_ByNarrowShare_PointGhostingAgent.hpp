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
#pragma once

#include "PSL_Abstract_PointGhostingAgent.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class OverlapSearcher;
}
class AbstractAuthority;
class ParameterData;
class PointCloud;
class AxisAlignedBoundingBox;

class ByNarrowShare_PointGhostingAgent : public Abstract_PointGhostingAgent
{
public:
    ByNarrowShare_PointGhostingAgent(AbstractAuthority* authority);
    virtual ~ByNarrowShare_PointGhostingAgent();

    virtual void share(double support_distance,
                       PointCloud* local_kernel_points,
                       std::vector<PointCloud*>& nonlocal_kernel_points,
                       std::vector<size_t>& processor_neighbors_below,
                       std::vector<size_t>& processor_neighbors_above);

protected:
    void determine_processor_bounds_and_neighbors(PointCloud* local_kernel_points,
                                                  std::vector<AxisAlignedBoundingBox>& processor_bounds,
                                                  std::vector<size_t>& processor_neighbors_below,
                                                  std::vector<size_t>& processor_neighbors_above);
    void share_points_with_below_processor(const std::vector<size_t>& processor_neighbors_below,
                                           std::vector<AxisAlignedBoundingBox>& processor_bounds,
                                           PointCloud* local_kernel_points,
                                           std::vector<PointCloud*>& nonlocal_kernel_points);
    void share_points_with_above_processor(const std::vector<size_t>& processor_neighbors_above,
                                           std::vector<AxisAlignedBoundingBox>& processor_bounds,
                                           PointCloud* local_kernel_points,
                                           std::vector<PointCloud*>& nonlocal_kernel_points);
    void share_points_with_processor_send(size_t other_proc_id,
                                          std::vector<AxisAlignedBoundingBox>& processor_bounds,
                                          PointCloud* local_kernel_points);
    void share_points_with_processor_receive(size_t other_proc_id, std::vector<PointCloud*>& nonlocal_kernel_points);

    AbstractInterface::OverlapSearcher* m_overlap_searcher;
    double m_support_distance;

};

}
