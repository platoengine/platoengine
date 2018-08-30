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
 * Plato_CircularCylinder.hpp
 *
 *  Created on: Aug 29, 2018
 */

#pragma once

#define _USE_MATH_DEFINES

#include <map>
#include <string>
#include <math.h>
#include <vector>
#include <cassert>
#include <cstddef>

#include "Plato_GeometryModel.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Circular cylinder geometry model class
**********************************************************************************/
template<typename ScalarType = double>
class CircularCylinder : public Plato::GeometryModel<ScalarType>
{
public:
    /******************************************************************************//**
     * @brief Default constructor
    **********************************************************************************/
    CircularCylinder() :
            mParameters()
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~CircularCylinder()
    {
    }

    /******************************************************************************//**
     * @brief compute the area of a circular cylinder.
    **********************************************************************************/
    ScalarType area()
    {
        const ScalarType tRadius = mParameters.find("Radius")->second;
        const ScalarType tLength = mParameters.find("Length")->second;
        const ScalarType tArea = static_cast<ScalarType>(2) * M_PI * tRadius * tLength;
        return (tArea);
    }

    /******************************************************************************//**
     * @brief compute the gradient of a circular cylinder with respect to parameters that define geometry.
     * @param aOutput gradient with respect to the parameters that defined a geometry
    **********************************************************************************/
    void gradient(std::vector<ScalarType>& aOutput)
    {
        assert(aOutput.size() == static_cast<size_t>(2));

        const ScalarType tRadius = mParameters.find("Radius")->second;
        const ScalarType tLength = mParameters.find("Length")->second;
        aOutput[0] = static_cast<ScalarType>(2) * M_PI * tLength;
        aOutput[1] = static_cast<ScalarType>(2) * M_PI * tRadius;
    }

    /******************************************************************************//**
     * @brief set parameters that define a circular cylinder.
    **********************************************************************************/
    void set(const std::map<std::string, ScalarType>& aParam)
    {
        mParameters = aParam;
    }

private:
    std::map<std::string, ScalarType> mParameters;
};
// class CircularCylinder

} // namespace Plato
