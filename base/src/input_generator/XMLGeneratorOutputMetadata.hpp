/*
 * XMLGeneratorOutputMetadata.hpp
 *
 *  Created on: Jul 4, 2020
 */

#pragma once

#include <map>
#include <vector>
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

    /*!< output data is associated with a scenario; thus, this member data is used to denote the scenario id */
    std::string mScenarioID;
    /*!< if true, output data to file (e.g. platomain.exo) */
    bool mOutputDataToFile = false;

public:
    /******************************************************************************//**
     * \fn outputData
     * \brief Return true if user enabled output of quantities of interest to file, \n
     * e.g. output to platomain.exo.
    **********************************************************************************/
    bool outputData() const;

    /******************************************************************************//**
     * \fn outputData
     * \brief Set flag used to enable output of quantities of interest to file, \n
     * e.g. output to platomain.exo.
    **********************************************************************************/
    void outputData(const bool& aOutputData);

    /******************************************************************************//**
     * \fn scenarioID
     * \brief Set scenario identifier associated with output quantities of interest.
     * \param [in] aID scenario identifier (id)
    **********************************************************************************/
    void scenarioID(const std::string& aID);

    /******************************************************************************//**
     * \fn scenarioID
     * \brief Return scenario identifier associated with output quantities of interest.
     * \return scenario identifier (id)
    **********************************************************************************/
    std::string scenarioID() const;

    /******************************************************************************//**
     * \fn appendRandomQoI
     * \brief Append random Quantities of Interest (QoI) to list of outputs. Quantities \n
     * are saved in output file, e.g. platomain.exo.
     * \param [in] aID         quantity of interest identifier
     * \param [in] aDataLayout shared data layout
    **********************************************************************************/
    void appendRandomQoI(const std::string& aID, const std::string& aDataLayout);

    /******************************************************************************//**
     * \fn appendDeterminsiticQoI
     * \brief Append deterministic Quantities of Interest (QoI) to list of outputs. \n
     * Quantities are saved in output file, e.g. platomain.exo.
     * \param [in] aID         quantity of interest identifier
     * \param [in] aDataLayout shared data layout
    **********************************************************************************/
    void appendDeterminsiticQoI(const std::string& aID, const std::string& aDataLayout);

    /******************************************************************************//**
     * \fn randomLayout
     * \brief Return shared data layout of requested random quantities of interest.
     * \param [in] aID quantity of interest identifier
     * \return shared data layout
    **********************************************************************************/
    std::string randomLayout(const std::string& aID) const;

    /******************************************************************************//**
     * \fn deterministicLayout
     * \brief Return shared data layout of requested deterministic quantities of interest.
     * \param [in] aID quantity of interest identifier
     * \return shared data layout
    **********************************************************************************/
    std::string deterministicLayout(const std::string& aID) const;

    /******************************************************************************//**
     * \fn randomArgumentName
     * \brief Return argument name of requested random quantities of interest.
     * \param [in] aID quantity of interest identifier
     * \return argument name
    **********************************************************************************/
    std::string randomArgumentName(const std::string& aID) const;

    /******************************************************************************//**
     * \fn deterministicArgumentName
     * \brief Return argument name of requested deterministic quantities of interest.
     * \param [in] aID quantity of interest identifier
     * \return argument name
    **********************************************************************************/
    std::string deterministicArgumentName(const std::string& aID) const;

    /******************************************************************************//**
     * \fn randomSharedDataName
     * \brief Return shared data name of requested random quantities of interest.
     * \param [in] aID quantity of interest identifier
     * \return shared data name
    **********************************************************************************/
    std::string randomSharedDataName(const std::string& aID) const;

    /******************************************************************************//**
     * \fn randomSharedDataName
     * \brief Return shared data name of requested deterministic quantities of interest.
     * \param [in] aID quantity of interest identifier
     * \return shared data name
    **********************************************************************************/
    std::string deterministicSharedDataName(const std::string& aID) const;

    /******************************************************************************//**
     * \fn randomIDs
     * \brief Return random quantities of interest identifiers.
     * \return random quantities of interest identifiers
    **********************************************************************************/
    std::vector<std::string> randomIDs() const;

    /******************************************************************************//**
     * \fn deterministicIDs
     * \brief Return deterministic quantities of interest identifiers.
     * \return deterministic quantities of interest identifiers
    **********************************************************************************/
    std::vector<std::string> deterministicIDs() const;

    /******************************************************************************//**
     * \fn isRandomMapEmpty
     * \brief Returns true if container of random quantities of interest is empty.
     * \return flag
    **********************************************************************************/
    bool isRandomMapEmpty() const;

    /******************************************************************************//**
     * \fn isDeterministicMapEmpty
     * \brief Returns true if container of deterministic quantities of interest is empty.
     * \return flag
    **********************************************************************************/
    bool isDeterministicMapEmpty() const;
};
// struct Output

}
// namespace XMLGen
