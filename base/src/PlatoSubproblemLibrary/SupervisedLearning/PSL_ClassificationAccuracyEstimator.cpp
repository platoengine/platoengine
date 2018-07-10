#include "PSL_ClassificationAccuracyEstimator.hpp"

#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_AbstractAuthority.hpp"

namespace PlatoSubproblemLibrary
{

ClassificationAccuracyEstimator::ClassificationAccuracyEstimator(AbstractAuthority* authority) :
        m_authority(authority)
{
}

ClassificationAccuracyEstimator::~ClassificationAccuracyEstimator()
{
}

}
