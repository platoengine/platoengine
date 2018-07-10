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
