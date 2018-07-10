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
#include "PSL_Selector.hpp"

#include "PSL_NodeSet.hpp"
#include "PSL_Abstract_PointCloud.hpp"
#include "PSL_Point.hpp"

#include <cstddef>
#include <cmath>

namespace PlatoSubproblemLibrary
{

Selector::Selector() :
        m_tolerance(1e-6),
        m_dimension(-1),
        m_value(0.)
{
}
Selector::~Selector()
{
}

void Selector::set_selection_tolerance(double tol)
{
    m_tolerance = tol;
}
void Selector::set_selection_criteria_x(double value)
{
    m_dimension = 0;
    m_value = value;
}
void Selector::set_selection_criteria_y(double value)
{
    m_dimension = 1;
    m_value = value;
}
void Selector::set_selection_criteria_z(double value)
{
    m_dimension = 2;
    m_value = value;
}

void Selector::select(AbstractInterface::PointCloud* nodes, NodeSet& result)
{
    if(m_dimension != 0 && m_dimension != 1 && m_dimension != 2)
    {
        return;
    }

    // for each node
    size_t num_nodes = nodes->get_num_points();
    for(size_t node_index = 0u; node_index < num_nodes; node_index++)
    {
        Point node = nodes->get_point(node_index);

        // if within tolerance, select
        if(fabs(node(m_dimension) - m_value) <= m_tolerance)
        {
            result.insert(node.get_index());
        }
    }
}

}
