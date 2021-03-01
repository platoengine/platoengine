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
 * Plato_FreeFunctions.hpp
 *
 */

#pragma once

#include <vector>
#include <cfloat>
#include <string>
#include <locale>
#include <sstream>
#include <cstddef>

namespace Plato
{

template<typename T>
T free_sum(const std::vector<T>& in_)
{
    T result = T(0.);
    const size_t length = in_.size();
    for(size_t i = 0u; i < length; i++)
    {
        result += in_[i];
    }
    return result;
}

std::string to_string(const double d);
std::string to_string(const float f);
std::string to_string(const int d);
std::string to_string(const size_t d);

/******************************************************************************//**
 * \fn equal
 * \brief Return true if the two floating point numbers are the same.
 * \param [in] aA value one
 * \param [in] aB value two
 * \return Flag
**********************************************************************************/
bool equal(const double& aA, const double& aB);

/******************************************************************************//**
 * \fn transform_tokens
 * \brief Convert uppercase word to lowercase.
 * \param [in] aInput word
 * \return lowercase word
**********************************************************************************/
std::string transform_tokens(const std::vector<std::string>& aTokens);

/******************************************************************************//**
 * \fn tolower
 * \brief Convert uppercase word to lowercase.
 * \param [in] aInput word
 * \return lowercase word
**********************************************************************************/
inline std::string tolower(const std::string& aInput)
{
    std::locale tLocale;
    std::ostringstream tOutput;
    for (auto& tChar : aInput)
    {
        tOutput << std::tolower(tChar,tLocale);
    }
    return (tOutput.str());
}
// function tolower

/******************************************************************************//**
 * \fn toupper
 * \brief Convert string to upper case.
 * \param [in] aInput word
 * \return upper case word
**********************************************************************************/
std::string toupper(const std::string& aInput);

void system(const char* aString);

} // end namespace Plato
