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

#pragma once

#include <map>
#include <vector>
#include <cstddef>

namespace Plato
{

/******************************************************************************//**
 * @brief Efficiently store and retrieve the next unassigned index in the sequence
 * 0, 1, ..., N. Note, indices are zero based.
**********************************************************************************/
class UniqueCounter
{
public:
    /******************************************************************************//**
     * @brief Constructor
    **********************************************************************************/
    UniqueCounter();

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~UniqueCounter();

    /******************************************************************************//**
     * @brief Mark an index as assigned
     * @param [in] aIndex index to assigned
     * @return true if already assigned, false if unassigned before
    **********************************************************************************/
    bool mark(const size_t& aIndex);

    /******************************************************************************//**
     * @brief Retrieve smallest unassigned index
     * @return smallest unassigned index
    **********************************************************************************/
    size_t assignNextUnique();

    /******************************************************************************//**
     * @brief Retrieve list of assigned indices
     * @return list of assigned indices
    **********************************************************************************/
    std::vector<size_t> list();

private:
    size_t mUnassignedIndex; /*!< smallest unassigned index */
    std::map<size_t, bool> mIsAssigned; /*!< map from assigned index to assigned flag */
    size_t mMaxUnassignedIndex; /*!< maximum unassigned index, i.e. upper bound */
};
// class UniqueCounter

}
// namespace Plato
