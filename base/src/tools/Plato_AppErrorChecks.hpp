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

/*
 * Plato_AppErrorChecks.hpp
 *
 *  Created on: Sep 28, 2018
 */

#pragma once

#include <map>
#include <string>
#include <vector>
#include <sstream>
#include <stdexcept>

namespace Plato
{

/******************************************************************************//**
 * @brief Check if argument name is defined
 * @param [in] aName name of input operation
 * @param [in] aMap valid name map
**********************************************************************************/
template<typename DataType>
void check_shared_data_argument_definition(const std::string & aArgumentName,
                                           const std::map<std::string, DataType> & aMap)
{
    try
    {
        if(aMap.find(aArgumentName) == aMap.end())
        {
            std::ostringstream tErrorMsg;
            tErrorMsg << "\n\n ******** MESSAGE: ARGUMENT NAME = '" << aArgumentName.c_str()
                    << "' IS NOT DEFINE IN MAP. ABORT! ********* \n\n";
            throw std::invalid_argument(tErrorMsg.str().c_str());
        }
    }
    catch(const std::invalid_argument & tErrorMsg)
    {
        throw tErrorMsg;
    }
}

/******************************************************************************//**
 * @brief Check if operation name is defined
 * @param [in] aName name of input operation
 * @param [in] aNames valid name list
**********************************************************************************/
void check_operation_definition(const std::string & aOperationName, const std::vector<std::string> & aNames);

/******************************************************************************//**
 * @brief Check if shared data and application layout are not similar
 * @param [in] aAppLayout application data layout
 * @param [in] aSharedDataLayout shared data data layout
**********************************************************************************/
void check_data_layout(const Plato::data::layout_t & aAppLayout, const Plato::data::layout_t & aSharedDataLayout);

} // namespace Plato
