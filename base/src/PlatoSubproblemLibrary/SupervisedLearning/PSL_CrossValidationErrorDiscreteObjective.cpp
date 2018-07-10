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

#include "PSL_CrossValidationErrorDiscreteObjective.hpp"

#include "PSL_DiscreteObjective.hpp"
#include "PSL_FindDataset.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_InputOutputManager.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_CrossValidationEstimator.hpp"
#include "PSL_ClassificationAssessor.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <string>

namespace PlatoSubproblemLibrary
{

CrossValidationErrorDiscreteObjective::CrossValidationErrorDiscreteObjective(AbstractAuthority* authority,
                                                                             const std::vector<datasets_t::datasets_t>& dataset) :
        DiscreteObjective(),
        m_authority(authority),
        m_archive_filename(),
        m_finder(authority)
{
    // write each dataset
    const size_t num_datasets = dataset.size();
    m_archive_filename.resize(num_datasets);
    for(size_t d = 0u; d < num_datasets; d++)
    {
        m_archive_filename[d] = m_finder.find(dataset[d]);
    }
}
CrossValidationErrorDiscreteObjective::~CrossValidationErrorDiscreteObjective()
{
}

double CrossValidationErrorDiscreteObjective::evaluate(const std::vector<double>& parameters)
{
    // allocate for results
    const size_t num_datasets = m_archive_filename.size();
    std::vector<double> results(num_datasets);

    // for each dataset
    const int num_fold = 8;
    for(size_t d = 0u; d < num_datasets; d++)
    {
        // get parameters
        ParameterData parameter_data;
        get_parameters(parameters, &parameter_data);

        // assess
        CrossValidationEstimator estimator(m_authority);
        ClassificationAssessor assessor(m_authority);
        estimator.initialize(m_archive_filename[d], false, num_fold);
        estimator.estimate_accuracy(&parameter_data, &assessor);

        // contribute to result
        results[d] = 1. - assessor.get_classification_accuracy();
    }

    return compute_result(results);
}

}
