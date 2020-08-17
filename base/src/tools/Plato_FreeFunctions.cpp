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
 * Plato_FreeFunctions.cpp
 *
 */

#include "Plato_FreeFunctions.hpp"

#include <cmath>
#include <vector>
#include <cstddef>
#include <iterator>
#include <algorithm>

namespace Plato
{

/**********************************************************************************/
std::string to_string(const double d)
{
  int sig_digits = DECIMAL_DIG;
  std::string tString;
  char buf[50];
  sprintf(buf, "%.*e", sig_digits, d);
  tString = buf;
  return tString;
}
/**********************************************************************************/

/**********************************************************************************/
std::string to_string(const float f)
{
  int sig_digits = DECIMAL_DIG;
  std::string tString;
  char buf[50];
  sprintf(buf, "%.*e", sig_digits, f);
  tString = buf;
  return tString;
}
/**********************************************************************************/

/**********************************************************************************/
std::string to_string(const int d)
{
  return std::to_string(d);
}
/**********************************************************************************/

/**********************************************************************************/
std::string to_string(const size_t d)
{
  return std::to_string(d);
}
/**********************************************************************************/

/**********************************************************************************/
std::string toupper(const std::string& aInput)
{
    std::locale tLocale;
    std::ostringstream tOutput;
    for (auto& tChar : aInput)
    {
        tOutput << std::toupper(tChar,tLocale);
    }
    return (tOutput.str());
}
/**********************************************************************************/

/**********************************************************************************/
bool equal(const double& aA, const double& aB)
{
  return std::nextafter(aA, std::numeric_limits<double>::lowest()) <= aB
    && std::nextafter(aA, std::numeric_limits<double>::max()) >= aB;
}
/**********************************************************************************/

/**********************************************************************************/
std::string transform_tokens(const std::vector<std::string>& aTokens)
{
    if(aTokens.empty())
    {
        return std::string("");
    }

    std::string tOutput;
    auto tEndIndex = aTokens.size() - 1u;
    auto tEndIterator = std::next(aTokens.begin(), tEndIndex);
    for(auto tItr = aTokens.begin(); tItr != tEndIterator; ++tItr)
    {
        auto tIndex = std::distance(aTokens.begin(), tItr);
        tOutput += aTokens[tIndex] + " ";
    }
    tOutput += aTokens[tEndIndex];

    return tOutput;
}
// function transform_tokens
/**********************************************************************************/

/**********************************************************************************/
void system(const char* aString)
{
  int return_val = std::system(aString);
  int temp = return_val;
  return_val = temp;
}
/**********************************************************************************/

} // end namespace Plato
