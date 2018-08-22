#include "PSL_ClassifierInterface.hpp"

#include "PSL_ParameterData.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

ClassifierInterface::ClassifierInterface(AbstractAuthority* authority) :
        m_authority(authority)
{
}

ClassifierInterface::~ClassifierInterface()
{
}

}
