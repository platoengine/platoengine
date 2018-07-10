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
 * Plato_StageInputDataMng.cpp
 *
 *  Created on: Oct 10, 2017
 */

#include <cassert>

#include "Plato_StageInputDataMng.hpp"
#include "Plato_OperationInputDataMng.hpp"

namespace Plato
{

StageInputDataMng::StageInputDataMng() :
        mStageNames(),
        mOperationInputs(),
        mSharedDataMap()
{
}

StageInputDataMng::~StageInputDataMng()
{
}

int StageInputDataMng::getNumStages() const
{
    return (mSharedDataMap.size());
}

const std::string & StageInputDataMng::getStageName() const
{
    assert(mStageNames.empty() == false);
    assert(1u == mStageNames.size());
    const int tSTAGE_INDEX = 0;
    return (mStageNames[tSTAGE_INDEX]);
}

const std::string & StageInputDataMng::getStageName(const int & aStageIndex) const
{
    assert(mStageNames.empty() == false);
    assert(aStageIndex < static_cast<int>(mStageNames.size()));
    return (mStageNames[aStageIndex]);
}

int StageInputDataMng::getNumInputs() const
{
    assert(mSharedDataMap.empty() == false);
    assert(1u == mStageNames.size());
    const int tSTAGE_INDEX = 0;
    const std::string & tStageName = mStageNames[tSTAGE_INDEX];
    std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>>::const_iterator tIterator;
    tIterator = mSharedDataMap.find(tStageName);
    return (tIterator->second.first.size());
}

int StageInputDataMng::getNumInputs(const int & aStageIndex) const
{
    assert(mSharedDataMap.empty() == false);
    assert(aStageIndex >= static_cast<int>(0));
    assert(aStageIndex < static_cast<int>(mStageNames.size()));
    const std::string & tStageName = mStageNames[aStageIndex];
    std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>>::const_iterator tIterator;
    tIterator = mSharedDataMap.find(tStageName);
    return (tIterator->second.first.size());
}

int StageInputDataMng::getNumInputs(const std::string & aStageName) const
{
    assert(mSharedDataMap.empty() == false);
    std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>>::const_iterator tIterator;
    tIterator = mSharedDataMap.find(aStageName);
    assert(tIterator != mSharedDataMap.end());
    return (tIterator->second.first.size());
}

const std::vector<std::string> & StageInputDataMng::getInputs(const int & aStageIndex) const
{
    assert(mSharedDataMap.empty() == false);
    assert(aStageIndex >= static_cast<int>(0));
    assert(aStageIndex < static_cast<int>(mStageNames.size()));
    const std::string & tStageName = mStageNames[aStageIndex];
    std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>>::const_iterator tIterator;
    tIterator = mSharedDataMap.find(tStageName);
    return (tIterator->second.first);
}

const std::string & StageInputDataMng::getInput(const int & aStageIndex, const int & aInputIndex) const
{
    assert(mSharedDataMap.empty() == false);
    assert(aStageIndex >= static_cast<int>(0));
    assert(aStageIndex < static_cast<int>(mStageNames.size()));
    const std::string & tStageName = mStageNames[aStageIndex];
    std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>>::const_iterator tIterator;
    tIterator = mSharedDataMap.find(tStageName);
    assert(aInputIndex >= static_cast<int>(0));
//    assert(aInputIndex < static_cast<int>(tIterator->second.second.size()));
    assert(aInputIndex < static_cast<int>(tIterator->second.first.size()));
    return (tIterator->second.first[aInputIndex]);
}

const std::vector<std::string> & StageInputDataMng::getInputs(const std::string & aStageName) const
{
    assert(mSharedDataMap.empty() == false);
    std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>>::const_iterator tIterator;
    tIterator = mSharedDataMap.find(aStageName);
    assert(tIterator != mSharedDataMap.end());
    return (tIterator->second.first);
}

const std::string & StageInputDataMng::getInput(const std::string & aStageName, const int & aInputIndex) const
{
    assert(mSharedDataMap.empty() == false);
    std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>>::const_iterator tIterator;
    tIterator = mSharedDataMap.find(aStageName);
    assert(tIterator != mSharedDataMap.end());
    assert(aInputIndex >= static_cast<int>(0));
//    assert(aInputIndex < static_cast<int>(tIterator->second.second.size()));
    assert(aInputIndex < static_cast<int>(tIterator->second.first.size()));
    return (tIterator->second.first[aInputIndex]);
}

int StageInputDataMng::getNumOutputs() const
{
    assert(mSharedDataMap.empty() == false);
    assert(1u == mStageNames.size());
    const int tSTAGE_INDEX = 0;
    const std::string & tStageName = mStageNames[tSTAGE_INDEX];
    std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>>::const_iterator tIterator;
    tIterator = mSharedDataMap.find(tStageName);
    return (tIterator->second.second.size());
}

int StageInputDataMng::getNumOutputs(const int & aStageIndex) const
{
    assert(mSharedDataMap.empty() == false);
    assert(aStageIndex >= static_cast<int>(0));
    assert(aStageIndex < static_cast<int>(mStageNames.size()));
    const std::string & tStageName = mStageNames[aStageIndex];
    std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>>::const_iterator tIterator;
    tIterator = mSharedDataMap.find(tStageName);
    return (tIterator->second.second.size());
}

int StageInputDataMng::getNumOutputs(const std::string & aStageName) const
{
    assert(mSharedDataMap.empty() == false);
    std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>>::const_iterator tIterator;
    tIterator = mSharedDataMap.find(aStageName);
    assert(tIterator != mSharedDataMap.end());
    return (tIterator->second.second.size());
}

const std::vector<std::string> & StageInputDataMng::getOutputs(const int & aStageIndex) const
{
    assert(mSharedDataMap.empty() == false);
    assert(aStageIndex >= static_cast<int>(0));
    assert(aStageIndex < static_cast<int>(mStageNames.size()));
    const std::string & tStageName = mStageNames[aStageIndex];
    std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>>::const_iterator tIterator;
    tIterator = mSharedDataMap.find(tStageName);
    return (tIterator->second.second);
}

const std::string & StageInputDataMng::getOutput(const int & aStageIndex, const int & aOutputIndex) const
{
    assert(mSharedDataMap.empty() == false);
    assert(aStageIndex >= static_cast<int>(0));
    assert(aStageIndex < static_cast<int>(mStageNames.size()));
    const std::string & tStageName = mStageNames[aStageIndex];
    std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>>::const_iterator tIterator;
    tIterator = mSharedDataMap.find(tStageName);
    assert(aOutputIndex >= static_cast<int>(0));
    assert(aOutputIndex < static_cast<int>(tIterator->second.second.size()));
    return (tIterator->second.second[aOutputIndex]);
}

const std::vector<std::string> & StageInputDataMng::getOutputs(const std::string & aStageName) const
{
    assert(mSharedDataMap.empty() == false);
    std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>>::const_iterator tIterator;
    tIterator = mSharedDataMap.find(aStageName);
    assert(tIterator != mSharedDataMap.end());
    return (tIterator->second.second);
}

const std::string & StageInputDataMng::getOutput(const std::string & aStageName, const int & aOutputIndex) const
{
    assert(mSharedDataMap.empty() == false);
    std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>>::const_iterator tIterator;
    tIterator = mSharedDataMap.find(aStageName);
    assert(tIterator != mSharedDataMap.end());
    assert(aOutputIndex >= static_cast<int>(0));
    assert(aOutputIndex < static_cast<int>(tIterator->second.second.size()));
    return (tIterator->second.second[aOutputIndex]);
}

void StageInputDataMng::add(const std::string & aStageName)
{
    assert(mSharedDataMap.find(aStageName) == mSharedDataMap.end());
    mStageNames.push_back(aStageName);
    std::vector<std::string> tInputs;
    std::vector<std::string> tOutputs;
    mSharedDataMap[aStageName] = std::make_pair(tInputs, tOutputs);
}

void StageInputDataMng::add(const std::string & aStageName, const std::vector<std::string> & aInputs, const std::vector<std::string> & aOutputs)
{
    mStageNames.push_back(aStageName);
    assert(mSharedDataMap.find(aStageName) == mSharedDataMap.end());
    mSharedDataMap[aStageName] = std::make_pair(aInputs, aOutputs);
}

void StageInputDataMng::addInputs(const std::string & aStageName, const std::vector<std::string> & aInputs)
{
    std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>>::iterator tIterator;
    tIterator = mSharedDataMap.find(aStageName);
    if(tIterator != mSharedDataMap.end())
    {
        tIterator->second.first = aInputs;
    }
    else
    {
        mStageNames.push_back(aStageName);
        std::vector<std::string> tOutputs;
        mSharedDataMap[aStageName] = std::make_pair(aInputs, tOutputs);
    }
}

void StageInputDataMng::addOutputs(const std::string & aStageName, const std::vector<std::string> & aOutputs)
{
    std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>>::iterator tIterator;
    tIterator = mSharedDataMap.find(aStageName);
    if(tIterator != mSharedDataMap.end())
    {
        tIterator->second.second = aOutputs;
    }
    else
    {
        mStageNames.push_back(aStageName);
        std::vector<std::string> tInputs;
        mSharedDataMap[aStageName] = std::make_pair(tInputs, aOutputs);
    }
}

int StageInputDataMng::getNumOperations() const
{
    std::map<std::string, std::vector<Plato::OperationInputDataMng>>::const_iterator tIterator;
    const std::string & tStageName = this->getStageName();
    tIterator = mOperationInputs.find(tStageName);
    int ret = 0;
    if(tIterator != mOperationInputs.end())
        ret = tIterator->second.size();
    return ret;
}

int StageInputDataMng::getNumOperations(const int & aStageIndex) const
{
    std::map<std::string, std::vector<Plato::OperationInputDataMng>>::const_iterator tIterator;
    const std::string & tStageName = this->getStageName(aStageIndex);
    tIterator = mOperationInputs.find(tStageName);
    int ret = 0;
    if(tIterator != mOperationInputs.end())
        ret = tIterator->second.size();
    return ret;
}

int StageInputDataMng::getNumOperations(const std::string & aStageName) const
{
    std::map<std::string, std::vector<Plato::OperationInputDataMng>>::const_iterator tIterator;
    tIterator = mOperationInputs.find(aStageName);
    int ret = 0;
    if(tIterator != mOperationInputs.end())
        ret = tIterator->second.size();
    return ret;
}

void StageInputDataMng::addOperationInputData(const std::string & aStageName,
                                              const Plato::OperationInputDataMng & aOperationInputData)
{
    std::map<std::string, std::vector<Plato::OperationInputDataMng>>::iterator tIterator;
    tIterator = mOperationInputs.find(aStageName);
    if(tIterator != mOperationInputs.end())
    {
        tIterator->second.push_back(aOperationInputData);
    }
    else
    {
        mOperationInputs[aStageName].push_back(aOperationInputData);
    }
}

void StageInputDataMng::addOperationInputData(const std::string & aStageName,
                                              const std::vector<Plato::OperationInputDataMng> & aOperationInputData)
{
    std::map<std::string, std::vector<Plato::OperationInputDataMng>>::iterator tIterator;
    tIterator = mOperationInputs.find(aStageName);
    if(tIterator != mOperationInputs.end())
    {
        tIterator->second = aOperationInputData;
    }
    else
    {
        mOperationInputs[aStageName] = aOperationInputData;
    }
}

const Plato::OperationInputDataMng & StageInputDataMng::getOperationInputData(const std::string & aStageName,
                                                                              const int & aOperationIndex) const
{
    assert(mOperationInputs.empty() == false);
    assert(aOperationIndex >= static_cast<int>(0));
    std::map<std::string, std::vector<Plato::OperationInputDataMng>>::const_iterator tIterator;
    tIterator = mOperationInputs.find(aStageName);
    assert(tIterator != mOperationInputs.end());
    assert(aOperationIndex < static_cast<int>(tIterator->second.size()));
    return (tIterator->second[aOperationIndex]);
}

const Plato::OperationInputDataMng & StageInputDataMng::getOperationInputData(const int & aStageIndex,
                                                                              const int & aOperationIndex) const
{
    assert(mOperationInputs.empty() == false);
    assert(aStageIndex >= static_cast<int>(0));
    assert(aStageIndex < static_cast<int>(mStageNames.size()));
    const std::string & tStageName = mStageNames[aStageIndex];
    std::map<std::string, std::vector<Plato::OperationInputDataMng>>::const_iterator tIterator;
    tIterator = mOperationInputs.find(tStageName);
    assert(tIterator != mOperationInputs.end());
    assert(aOperationIndex < static_cast<int>(tIterator->second.size()));
    return (tIterator->second[aOperationIndex]);
}

}
