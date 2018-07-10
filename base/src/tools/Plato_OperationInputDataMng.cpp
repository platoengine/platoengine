/*
 * Plato_OperationInputDataMng.cpp
 *
 *  Created on: Oct 10, 2017
 */

#include <cassert>

#include "Plato_OperationInputDataMng.hpp"

namespace Plato
{

/*****************************************************************************/
OperationInputDataMng::OperationInputDataMng() :
        mPerformerName(),
        mOperationMap(),
        mInputDataMap(),
        mOutputDataMap()
/*****************************************************************************/
{
}

/*****************************************************************************/
OperationInputDataMng::~OperationInputDataMng()
/*****************************************************************************/
{
}

/*****************************************************************************/
int OperationInputDataMng::getNumPerformers() const
/*****************************************************************************/
{
    assert(mPerformerName.size() == mOperationMap.size());
    return (mPerformerName.size());
}

/*****************************************************************************/
int OperationInputDataMng::getNumOperations() const
/*****************************************************************************/
{
    return (mOperationMap.size());
}

/*****************************************************************************/
bool OperationInputDataMng::hasSubOperations() const
/*****************************************************************************/
{
    bool tHasSubOperations = false;
    if(mPerformerName.size() > 1u)
    {
        tHasSubOperations = true;
    }
    return (tHasSubOperations);
}
/*****************************************************************************/
const std::string & OperationInputDataMng::getPerformerName() const
/*****************************************************************************/
{
    assert(mPerformerName.empty() == false);
    assert(1u == mPerformerName.size());
    const size_t tOPERATION_INDEX = 0;
    return (mPerformerName[tOPERATION_INDEX]);
}

/*****************************************************************************/
const std::string & OperationInputDataMng::getPerformerName(const int & aOperationIndex) const
/*****************************************************************************/
{
    assert(mPerformerName.empty() == false);
    assert(aOperationIndex >= static_cast<int>(0));
    assert(aOperationIndex < static_cast<int>(mPerformerName.size()));
    return (mPerformerName[aOperationIndex]);
}

/*****************************************************************************/
const std::string & OperationInputDataMng::getOperationName(const int & aOperationIndex) const
/*****************************************************************************/
{
    assert(mPerformerName.empty() == false);
    assert(aOperationIndex >= static_cast<int>(0));
    assert(aOperationIndex < static_cast<int>(mPerformerName.size()));
    const std::string & tPerformerName = mPerformerName[aOperationIndex];
    std::map<std::string, std::string>::const_iterator tIterator;
    tIterator = mOperationMap.find(tPerformerName);
    return (tIterator->second);
}

/*****************************************************************************/
const std::string & OperationInputDataMng::getOperationName(const std::string & aPerformerName) const
/*****************************************************************************/
{
    assert(mOperationMap.empty() == false);
    std::map<std::string, std::string>::const_iterator tIterator;
    tIterator = mOperationMap.find(aPerformerName);
    assert(tIterator != mOperationMap.end());
    return (tIterator->second);
}

/*****************************************************************************/
int OperationInputDataMng::getNumInputs(const int & aOperationIndex) const
/*****************************************************************************/
{
    assert(aOperationIndex >= static_cast<int>(0));
    assert(aOperationIndex < static_cast<int>(mPerformerName.size()));
    const std::string & tOperationName = mPerformerName[aOperationIndex];
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>::const_iterator tIterator;
    tIterator = mInputDataMap.find(tOperationName);
    return (tIterator->second.size());
}

/*****************************************************************************/
int OperationInputDataMng::getNumInputs(const std::string & aPerformerName) const
/*****************************************************************************/
{
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>::const_iterator tIterator;
    tIterator = mInputDataMap.find(aPerformerName);
    return (tIterator->second.size());
}

/*****************************************************************************/
const std::vector<std::pair<std::string, std::string>> & OperationInputDataMng::getInputs(const int & aOperationIndex) const
/*****************************************************************************/
{
    assert(mInputDataMap.empty() == false);
    assert(aOperationIndex >= static_cast<int>(0));
    assert(aOperationIndex < static_cast<int>(mPerformerName.size()));
    const std::string & tOperationName = mPerformerName[aOperationIndex];
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>::const_iterator tIterator;
    tIterator = mInputDataMap.find(tOperationName);
    return (tIterator->second);
}

/*****************************************************************************/
const std::vector<std::pair<std::string, std::string>> & OperationInputDataMng::getInputs(const std::string & aPerformerName) const
/*****************************************************************************/
{
    assert(mInputDataMap.empty() == false);
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>::const_iterator tIterator;
    tIterator = mInputDataMap.find(aPerformerName);
    assert(tIterator != mInputDataMap.end());
    return (tIterator->second);
}

/*****************************************************************************/
const std::string & OperationInputDataMng::getInputSharedData(const int & aOperationIndex, const int & aInputDataIndex) const
/*****************************************************************************/
{
    assert(mInputDataMap.empty() == false);
    assert(aOperationIndex >= static_cast<int>(0));
    assert(aOperationIndex < static_cast<int>(mPerformerName.size()));
    const std::string & tOperationName = mPerformerName[aOperationIndex];
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>::const_iterator tIterator;
    tIterator = mInputDataMap.find(tOperationName);
    assert(aInputDataIndex >= static_cast<int>(0));
    assert(aInputDataIndex < static_cast<int>(tIterator->second.size()));
    return (tIterator->second[aInputDataIndex].first);
}

/*****************************************************************************/
const std::string & OperationInputDataMng::getInputSharedData(const std::string & aPerformerName, const int & aInputDataIndex) const
/*****************************************************************************/
{
    assert(mInputDataMap.empty() == false);
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>::const_iterator tIterator;
    tIterator = mInputDataMap.find(aPerformerName);
    assert(tIterator != mInputDataMap.end());
    assert(aInputDataIndex >= static_cast<int>(0));
    assert(aInputDataIndex < static_cast<int>(tIterator->second.size()));
    return (tIterator->second[aInputDataIndex].first);
}

/*****************************************************************************/
const std::string & OperationInputDataMng::getInputArgument(const int & aOperationIndex, const int & aInputDataIndex) const
/*****************************************************************************/
{
    assert(mInputDataMap.empty() == false);
    assert(aOperationIndex >= static_cast<int>(0));
    assert(aOperationIndex < static_cast<int>(mPerformerName.size()));
    const std::string & tOperationName = mPerformerName[aOperationIndex];
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>::const_iterator tIterator;
    tIterator = mInputDataMap.find(tOperationName);
    assert(aInputDataIndex >= static_cast<int>(0));
    assert(aInputDataIndex < static_cast<int>(tIterator->second.size()));
    return (tIterator->second[aInputDataIndex].second);
}
/*****************************************************************************/
const std::string & OperationInputDataMng::getInputArgument(const std::string & aPerformerName, const int & aInputDataIndex) const
/*****************************************************************************/
{
    assert(mInputDataMap.empty() == false);
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>::const_iterator tIterator;
    tIterator = mInputDataMap.find(aPerformerName);
    assert(tIterator != mInputDataMap.end());
    assert(aInputDataIndex >= static_cast<int>(0));
    assert(aInputDataIndex < static_cast<int>(tIterator->second.size()));
    return (tIterator->second[aInputDataIndex].second);
}

/*****************************************************************************/
int OperationInputDataMng::getNumOutputs(const int & aOperationIndex) const
/*****************************************************************************/
{
    assert(aOperationIndex >= static_cast<int>(0));
    assert(aOperationIndex < static_cast<int>(mPerformerName.size()));
    const std::string & tOperationName = mPerformerName[aOperationIndex];
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>::const_iterator tIterator;
    tIterator = mOutputDataMap.find(tOperationName);
    return (tIterator->second.size());
}

/*****************************************************************************/
int OperationInputDataMng::getNumOutputs(const std::string & aPerformerName) const
/*****************************************************************************/
{
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>::const_iterator tIterator;
    tIterator = mOutputDataMap.find(aPerformerName);
    return (tIterator->second.size());
}

/*****************************************************************************/
const std::vector<std::pair<std::string, std::string>> & OperationInputDataMng::getOutputs(const int & aOperationIndex) const
/*****************************************************************************/
{
    assert(mOutputDataMap.empty() == false);
    assert(aOperationIndex >= static_cast<int>(0));
    assert(aOperationIndex < static_cast<int>(mPerformerName.size()));
    const std::string & tOperationName = mPerformerName[aOperationIndex];
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>::const_iterator tIterator;
    tIterator = mOutputDataMap.find(tOperationName);
    return (tIterator->second);
}

/*****************************************************************************/
const std::vector<std::pair<std::string, std::string>> & OperationInputDataMng::getOutputs(const std::string & aPerformerName) const
/*****************************************************************************/
{
    assert(mOutputDataMap.empty() == false);
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>::const_iterator tIterator;
    tIterator = mOutputDataMap.find(aPerformerName);
    assert(tIterator != mOutputDataMap.end());
    return (tIterator->second);
}

/*****************************************************************************/
const std::string & OperationInputDataMng::getOutputSharedData(const int & aOperationIndex, const int & aInputDataIndex) const
/*****************************************************************************/
{
    assert(mOutputDataMap.empty() == false);
    assert(aOperationIndex >= static_cast<int>(0));
    assert(aOperationIndex < static_cast<int>(mPerformerName.size()));
    const std::string & tOperationName = mPerformerName[aOperationIndex];
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>::const_iterator tIterator;
    tIterator = mOutputDataMap.find(tOperationName);
    assert(aInputDataIndex >= static_cast<int>(0));
    assert(aInputDataIndex < static_cast<int>(tIterator->second.size()));
    return (tIterator->second[aInputDataIndex].first);
}

/*****************************************************************************/
const std::string & OperationInputDataMng::getOutputSharedData(const std::string & aPerformerName, const int & aInputDataIndex) const
/*****************************************************************************/
{
    assert(mOutputDataMap.empty() == false);
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>::const_iterator tIterator;
    tIterator = mOutputDataMap.find(aPerformerName);
    assert(tIterator != mOutputDataMap.end());
    assert(aInputDataIndex >= static_cast<int>(0));
    assert(aInputDataIndex < static_cast<int>(tIterator->second.size()));
    return (tIterator->second[aInputDataIndex].first);
}

/*****************************************************************************/
const std::string & OperationInputDataMng::getOutputArgument(const int & aOperationIndex, const int & aInputDataIndex) const
/*****************************************************************************/
{
    assert(mOutputDataMap.empty() == false);
    assert(aOperationIndex >= static_cast<int>(0));
    assert(aOperationIndex < static_cast<int>(mPerformerName.size()));
    const std::string & tOperationName = mPerformerName[aOperationIndex];
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>::const_iterator tIterator;
    tIterator = mOutputDataMap.find(tOperationName);
    assert(aInputDataIndex >= static_cast<int>(0));
    assert(aInputDataIndex < static_cast<int>(tIterator->second.size()));
    return (tIterator->second[aInputDataIndex].second);
}

/*****************************************************************************/
const std::string & OperationInputDataMng::getOutputArgument(const std::string & aPerformerName, const int & aInputDataIndex) const
/*****************************************************************************/
{
    assert(mOutputDataMap.empty() == false);
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>::const_iterator tIterator;
    tIterator = mOutputDataMap.find(aPerformerName);
    assert(tIterator != mOutputDataMap.end());
    assert(aInputDataIndex >= static_cast<int>(0));
    assert(aInputDataIndex < static_cast<int>(tIterator->second.size()));
    return (tIterator->second[aInputDataIndex].second);
}

/*****************************************************************************/
void OperationInputDataMng::addInput(const std::string & aPerformerName,
                                     const std::string & aOperationName,
                                     const std::string & aSharedDataName,
                                     const std::string & aArgumentName)
/*****************************************************************************/
{
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>::iterator tIterator;
    tIterator = mInputDataMap.find(aPerformerName);
    if(tIterator == mInputDataMap.end())
    {
        mPerformerName.push_back(aPerformerName);
        mOperationMap[aPerformerName] = aOperationName;

        tIterator->second.push_back(std::make_pair(aSharedDataName, aArgumentName));

        std::vector<std::pair<std::string, std::string>> tOutputData;
        mOutputDataMap[aPerformerName] = tOutputData;
    }
    else
    {
        tIterator->second.push_back(std::make_pair(aSharedDataName, aArgumentName));
    }
}

/*****************************************************************************/
void OperationInputDataMng::addInputs(const std::string & aPerformerName,
                                      const std::string & aOperationName,
                                      const std::vector<std::string> & aSharedDataNames,
                                      const std::vector<std::string> & aArgumentNames)
/*****************************************************************************/
{
    assert(aSharedDataNames.size() == aArgumentNames.size());

    std::map<std::string, std::vector<std::pair<std::string, std::string>>>::iterator tIterator;
    tIterator = mInputDataMap.find(aPerformerName);
    if(tIterator == mInputDataMap.end())
    {
        mPerformerName.push_back(aPerformerName);
        mOperationMap[aPerformerName] = aOperationName;

        std::vector<std::pair<std::string, std::string>> tInputData;
        for(size_t tDataIndex = 0; tDataIndex < aArgumentNames.size(); tDataIndex++)
        {
            const std::string & tArgumentName = aArgumentNames[tDataIndex];
            const std::string & tSharedDataName = aSharedDataNames[tDataIndex];
            tInputData.push_back(std::make_pair(tSharedDataName, tArgumentName));
        }
        mInputDataMap[aPerformerName] = tInputData;

        std::vector<std::pair<std::string, std::string>> tOutputData;
        mOutputDataMap[aPerformerName] = tOutputData;
    }
    else
    {
        for(size_t tDataIndex = 0; tDataIndex < aArgumentNames.size(); tDataIndex++)
        {
            const std::string & tArgumentName = aArgumentNames[tDataIndex];
            const std::string & tSharedDataName = aSharedDataNames[tDataIndex];
            tIterator->second.push_back(std::make_pair(tSharedDataName, tArgumentName));
        }
    }
}

/*****************************************************************************/
void OperationInputDataMng::addOutput(const std::string & aPerformerName,
                                      const std::string & aOperationName,
                                      const std::string & aSharedDataName,
                                      const std::string & aArgumentName)
/*****************************************************************************/
{
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>::iterator tIterator;
    tIterator = mOutputDataMap.find(aPerformerName);
    if(tIterator == mOutputDataMap.end())
    {
        mPerformerName.push_back(aPerformerName);
        mOperationMap[aPerformerName] = aOperationName;

        tIterator->second.push_back(std::make_pair(aSharedDataName, aArgumentName));

        std::vector<std::pair<std::string, std::string>> tInputData;
        mInputDataMap[aPerformerName] = tInputData;
    }
    else
    {
        tIterator->second.push_back(std::make_pair(aSharedDataName, aArgumentName));
    }
}

/*****************************************************************************/
void OperationInputDataMng::addOutputs(const std::string & aPerformerName,
                                       const std::string & aOperationName,
                                       const std::vector<std::string> & aSharedDataNames,
                                       const std::vector<std::string> & aArgumentNames)
/*****************************************************************************/
{
    assert(aSharedDataNames.size() == aArgumentNames.size());

    std::map<std::string, std::vector<std::pair<std::string, std::string>>>::iterator tIterator;
    tIterator = mOutputDataMap.find(aPerformerName);
    if(tIterator == mOutputDataMap.end())
    {
        mPerformerName.push_back(aPerformerName);
        mOperationMap[aPerformerName] = aOperationName;

        std::vector<std::pair<std::string, std::string>> tOutputData;
        for(size_t tDataIndex = 0; tDataIndex < aArgumentNames.size(); tDataIndex++)
        {
            const std::string & tArgumentName = aArgumentNames[tDataIndex];
            const std::string & tSharedDataName = aSharedDataNames[tDataIndex];
            tOutputData.push_back(std::make_pair(tSharedDataName, tArgumentName));
        }
        mOutputDataMap[aPerformerName] = tOutputData;

        std::vector<std::pair<std::string, std::string>> tInputData;
        mInputDataMap[aPerformerName] = tInputData;
    }
    else
    {
        for(size_t tDataIndex = 0; tDataIndex < aArgumentNames.size(); tDataIndex++)
        {
            const std::string & tArgumentName = aArgumentNames[tDataIndex];
            const std::string & tSharedDataName = aSharedDataNames[tDataIndex];
            tIterator->second.push_back(std::make_pair(tSharedDataName, tArgumentName));
        }
    }
}

}
