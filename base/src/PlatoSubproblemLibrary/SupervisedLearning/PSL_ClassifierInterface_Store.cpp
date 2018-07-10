#include "PSL_ClassifierInterface_Store.hpp"

#include "PSL_ClassifierInterface.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_ClassificationArchive.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

ClassifierInterface_Store::ClassifierInterface_Store(AbstractAuthority* authority) :
        ClassifierInterface(authority)
{
}
ClassifierInterface_Store::~ClassifierInterface_Store()
{
}

void ClassifierInterface_Store::initialize(ParameterData* parameters,
                                           int num_scalars,
                                           const std::vector<int>& input_enum_sizes,
                                           int output_enum_size)
{
    m_archive = new ClassificationArchive(m_authority);
    m_archive->initialize(parameters);
    m_archive->set_enum_size(input_enum_sizes, output_enum_size);
}

int ClassifierInterface_Store::predict_classification(const std::vector<double>& input_scalars,
                                                      const std::vector<int>& input_enums)
{
    return -1;
}

void ClassifierInterface_Store::store_accurate_classification(const std::vector<double>& input_scalars,
                                                              const std::vector<int>& input_enums,
                                                              int accurate_output)
{
    m_archive->add_row(input_scalars, input_enums, accurate_output);
}

}
