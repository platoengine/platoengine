#include "PSL_PreprocessorFactory.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Preprocessor.hpp"
#include "PSL_PreprocessStandardization.hpp"
#include "PSL_PreprocessBipolarNormalization.hpp"
#include "PSL_PreprocessSkewNormalization.hpp"
#include "PSL_PreprocessorPCA.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_MultistagePreprocessor.hpp"
#include "PSL_DataSequence.hpp"

#include <cstddef>

namespace PlatoSubproblemLibrary
{

Preprocessor* build_preprocessor(const preprocessor_t::preprocessor_t& type,
                                 AbstractAuthority* authority,
                                 DataSequence* sequence)
{
    Preprocessor* result = NULL;

    switch(type)
    {
        case preprocessor_t::preprocessor_t::no_preprocessor:
        {
            result = new Preprocessor(authority);
            break;
        }
        case preprocessor_t::preprocessor_t::standardization_preprocessor:
        {
            result = new Standardization(authority);
            break;
        }
        case preprocessor_t::preprocessor_t::bipolar_normalization_preprocessor:
        {
            result = new BipolarNormalization(authority);
            break;
        }
        case preprocessor_t::preprocessor_t::skew_normalization_preprocessor:
        {
            result = new SkewNormalization(authority);
            break;
        }
        case preprocessor_t::preprocessor_t::PCA_preprocessor:
        {
            result = new PreprocessorPCA(authority);
            break;
        }
        case preprocessor_t::preprocessor_t::TOTAL_NUM_PREPROCESSORS:
        default:
        {
            authority->utilities->fatal_error("PlatoSubproblemLibrary could not match enum to preprocessor. Aborting.\n\n");
            break;
        }
    }

    // if sequence == NULL, preprocessor will built its own data sequence
    if(sequence != NULL)
    {
        result->set_data_sequence(sequence, false);
    }

    return result;
}

}
