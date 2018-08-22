#include "PSL_ClassifierInterface_Predict.hpp"

#include "PSL_ClassifierInterface.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

ClassifierInterface_Predict::ClassifierInterface_Predict(AbstractAuthority* authority) :
                ClassifierInterface(authority)
{
}
ClassifierInterface_Predict::~ClassifierInterface_Predict()
{
}

void ClassifierInterface_Predict::initialize(ParameterData* parameters,
                                                    int num_scalars,
                                                    const std::vector<int>& enum_sizes,
                                                    int output_enum_size)
{
    // TODO
}

int ClassifierInterface_Predict::predict_classification(const std::vector<double>& input_scalars,
                                                               const std::vector<int>& input_enums)
{
    // TODO
    return 0;
}

void ClassifierInterface_Predict::store_accurate_classification(const std::vector<double>& input_scalars,
                                                                       const std::vector<int>& input_enums,
                                                                       int accurate_output)
{
}

}
