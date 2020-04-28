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
 * Plato_SromHelpers.hpp
 *
 *  Created on: Apr 28, 2020
 */

#pragma once

#include <cmath>
#include <sstream>
#include <iomanip>

#include "Plato_Macros.hpp"

namespace Plato
{

namespace srom
{

/******************************************************************************//**
 * \fn to_string
 * \brief Convert double to string.
 * \param [in] aInput     floating-point number
 * \param [in] aPrecision precision (default = 16 digits)
 * \return converted floating-point number as a value of type string
 * \note If precision < 0, set precision to its default value, which is '6' digits.
 * \note If precision is infinite or NaN, set precision to its default value, which is '0' digits.
**********************************************************************************/
inline std::string to_string(const double& aInput, int aPrecision = 16)
{
    if(std::isfinite(aInput) == false)
    {
        THROWERR("Convert double to string: detected a non-finite floating-point number.")
    }

    std::ostringstream tValueString;
    tValueString << std::fixed; // forces fix-point notation instead of default scientific notation
    tValueString << std::setprecision(aPrecision); // sets precision for fixed-point notation
    tValueString << aInput;
    return (tValueString.str());
}
// function to_string

}
// namespace srom

}
// namespace Plato
