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
 * Plato_OptimizersIO_Utilities.hpp
 *
 *  Created on: Sep 1, 2019
 */

#pragma once

#include <vector>
#include <string>

#include "Plato_Vector.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Return memory space
 * @param [in] aInput memory space keyword
 * @return memory space, default = HOST
**********************************************************************************/
Plato::MemorySpace::type_t get_memory_space(const std::string& aInput);

namespace error
{

/******************************************************************************//**
 * @brief Check if output file is open. An error is thrown if output file is not open.
 * @param [in] aOutputFile output file
**********************************************************************************/
template<typename Type>
void is_file_open(const Type& aOutputFile)
{
    try
    {
        if(aOutputFile.is_open() == false)
        {
            throw std::invalid_argument("OUTPUT FILE IS NOT OPEN.\n");
        }
    }
    catch(const std::invalid_argument & tError)
    {
        throw tError;
    }
}
/******************************************************************************//**
 * @brief Specialization that permits stringstreams.
 * @param [in] aOutputStream stringstream pretending to be a file.
**********************************************************************************/
template<>
void is_file_open(const std::stringstream& aOutputStream);

/******************************************************************************//**
 * @brief Check if vector is empty. An error is thrown if vector is empty.
 * @param [in] aInput a vector
**********************************************************************************/
template<typename ScalarType>
void is_vector_empty(const std::vector<ScalarType>& aInput)
{
    try
    {
        if(aInput.empty() == true)
        {
            throw std::invalid_argument("CONTAINER SIZE IS 0.\n");
        }
    }
    catch(const std::invalid_argument & tError)
    {
        throw tError;
    }
}

}
// namespace error

}
// namespace Plato
