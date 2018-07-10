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

#include "PSL_GiniImpurityMetric.hpp"

#include "PSL_DecisionMetric.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_FreeHelpers.hpp"

#include <vector>
#include <cstddef>
#include <algorithm>

namespace PlatoSubproblemLibrary
{

GiniImpurityMetric::GiniImpurityMetric() :
        DecisionMetric(decision_metric_t::decision_metric_t::gini_impurity_metric)
{
}

GiniImpurityMetric::~GiniImpurityMetric()
{
}

double GiniImpurityMetric::measure(const std::vector<int>& num_class_members)
{
    const size_t num_classes = num_class_members.size();
    const double population_size = sum(num_class_members);
    if(population_size < 1.)
    {
        return 0.;
    }

    std::vector<double> squared_class_probabilities(num_classes);
    for(size_t c = 0u; c < num_classes; c++)
    {
        const double prob = double(num_class_members[c]) / population_size;
        squared_class_probabilities[c] = prob * prob;
    }

    const double impurity = 1. - sum(squared_class_probabilities);
    return impurity;
}

double GiniImpurityMetric::lower_cutoff_upscale()
{
    return 1.;
}

}
