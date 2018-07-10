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
