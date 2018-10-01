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
 * Plato_GeometryModel.hpp
 *
 *  Created on: Aug 29, 2018
 */

#pragma once

namespace Plato
{

/******************************************************************************//**
 * @brief Defines options for geometry. 1) INITIAL = initial geometry. 2) DYNAMIC = geometry changing in time
**********************************************************************************/
struct Configuration
{
    enum type_t
    {
        INITIAL = 0,
        DYNAMIC = 1
    };
};
// struct Configuration

/******************************************************************************//**
 * @brief Abstract geometry model class
**********************************************************************************/
template<typename ScalarType = double>
class GeometryModel
{
public:
    /******************************************************************************//**
     * @brief Default destructor
    **********************************************************************************/
    virtual ~GeometryModel()
    {
    }

    /******************************************************************************//**
     * @brief Compute the area of a user-defined geometry.
    **********************************************************************************/
    virtual ScalarType area() = 0;

    /******************************************************************************//**
     * @brief Compute the gradient with respect to the parameters that define the geometry.
     * @param aOutput gradient with respect to the parameters that define the geometry
    **********************************************************************************/
    virtual void gradient(std::vector<ScalarType>& aOutput) = 0;

    /******************************************************************************//**
     * @brief Set parameters that define the geometry.
     * @param aParam parameters that define the geometry
    **********************************************************************************/
    virtual void update(const std::map<std::string, ScalarType>& aParam) = 0;
};
// class GeometryModel

}
// namespace Plato
