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

#include "Plato_UniqueCounter.hpp"

#include <cstddef>
#include <map>
#include <stdlib.h>
#include <cstdlib>

namespace Plato
{

UniqueCounter::UniqueCounter() :
        mUnassignedIndex(0u),
        mIsAssigned(),
        mMaxUnassignedIndex(5000000u)
{
}
UniqueCounter::~UniqueCounter()
{
}

bool UniqueCounter::mark(const size_t& aIndex)
{
    // true if already assigned, false if unassigned before
    const bool tValueBefore = mIsAssigned[aIndex];
    mIsAssigned[aIndex] = true;
    return (tValueBefore);
}

size_t UniqueCounter::assignNextUnique()
{
    // linear scan until find unassigned
    while(true)
    {
        // if unassigned
        if(!mIsAssigned[mUnassignedIndex])
        {
            // do assign
            const size_t tToReturn = mUnassignedIndex;
            mIsAssigned[mUnassignedIndex] = true;
            mUnassignedIndex++;
            return tToReturn;
        }

        // continue scan
        mUnassignedIndex++;

        // avoid infinite loop by max
        if(mMaxUnassignedIndex < mUnassignedIndex)
        {
            break;
        }
    }

    // should never reach here
    std::abort();
    return (0u);
}

std::vector<size_t> UniqueCounter::list()
{
    const size_t tNumIndices = mIsAssigned.size();
    std::vector<size_t> tList(tNumIndices);

    size_t tIndex = 0;
    std::map<size_t, bool>::iterator tIterator;
    for (tIterator = mIsAssigned.begin(); tIterator != mIsAssigned.end(); ++tIterator)
    {
        tList[tIndex] = tIterator->first;
        tIndex++;
    }

    return (tList);
}

}
