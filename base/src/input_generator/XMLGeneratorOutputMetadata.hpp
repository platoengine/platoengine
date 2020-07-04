/*
 * XMLGeneratorOutputMetadata.hpp
 *
 *  Created on: Jul 4, 2020
 */

#pragma once

#include <map>
#include <string>
#include <unordered_map>

namespace XMLGen
{

/******************************************************************************//**
 * \struct Output
 * \brief The Output metadata structure owns the random and deterministic output \n
 * Quantities of Interests (QoIs) defined by the user in the plato input file.
**********************************************************************************/
struct Output
{
private:
    /******************************************************************************//**
     * \var QoI maps
     * \brief Maps from QoIs identifier (ID) to map from attribute name to attribute \n
     * value, i.e. map<ID, map<attribute, value>>, where attributes are quantities \n
     * used to describe the data.  For instance, valid attributes are 'ArgumentName', \n
     * 'SharedDataName', and \n 'DataLayout'.
    **********************************************************************************/
    std::map<std::string, std::unordered_map<std::string, std::string>> mRandomQoIs;
    std::map<std::string, std::unordered_map<std::string, std::string>> mDeterministicQoIs;

    bool mEnableOutputStage = false;

public:
    bool outputData() const;
    void outputData(const bool& aOutputData);

    void appendRandomQoI(const std::string& aID, const std::string& aDataLayout);
    void appendDeterminsiticQoI(const std::string& aID, const std::string& aDataLayout);

    std::string randomLayout(const std::string& aID) const;
    std::string deterministicLayout(const std::string& aID) const;

    std::string randomArgumentName(const std::string& aID) const;
    std::string deterministicArgumentName(const std::string& aID) const;

    std::string randomSharedDataName(const std::string& aID) const;
    std::string deterministicSharedDataName(const std::string& aID) const;

    std::vector<std::string> randomIDs() const;
    std::vector<std::string> deterministicIDs() const;

    bool isRandomMapEmpty() const;
    bool isDeterministicMapEmpty() const;
};
// struct Output

}
// namespace XMLGen
