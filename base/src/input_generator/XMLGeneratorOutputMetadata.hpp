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

    std::unordered_map<std::string, std::string> mMetaData; /*!< service metadata, map< tag, value > */

private:
    /******************************************************************************//**
     * \fn getValue
     * \brief Return string value for property with input tag; else, throw an error if \n
     * property is not defined in the metadata.
     * \param [in] aTag property tag
     * \return property string value (default = "")
     **********************************************************************************/
    std::string getValue(const std::string &aTag) const;

    /******************************************************************************//**
     * \fn getBool
     * \brief Return bool value for property with input tag; else, throw an error if \n
     * property is not defined in the metadata.
     * \param [in] aTag property tag
     * \return property bool value (default = false)
     **********************************************************************************/
    bool getBool(const std::string &aTag) const;

public:
    /******************************************************************************//**
     * \fn value
     * \brief Return value for property with input tag; else, throw an error if \n
     * property is not defined in the metadata.
     * \param [in] aTag property tag
     * \return property string value
    **********************************************************************************/
    std::string value(const std::string& aTag) const;

    /******************************************************************************//**
     * \fn tags
     * \brief Return list of parameter tags.
     * \return parameter tags
    **********************************************************************************/
    std::vector<std::string> tags() const;

    /******************************************************************************//**
     * \fn appendParam
     * \brief Append POD parameters to metadata.
     * \param [in] aTag   parameter tag
     * \param [in] aValue parameter value
    **********************************************************************************/
    void appendParam(const std::string& aTag, const std::string& aValue);

    /******************************************************************************//**
     * \fn isOutputDisabled
     * \brief Return 'disable' keyword value.
     * \return value
    **********************************************************************************/
    bool isOutputDisabled() const;

    /******************************************************************************//**
     * \fn disableOutput
     * \brief Disable output to file.
    **********************************************************************************/
    void disableOutput();

    /******************************************************************************//**
     * \fn outputSamples
     * \brief Return 'output_samples' keyword value.
     * \return value
    **********************************************************************************/
    bool outputSamples() const;

    /******************************************************************************//**
     * \fn outputSamples
     * \brief Set 'output_samples' keyword.
     * \param [in] aInput keyword value
     * \return value
    **********************************************************************************/
    void outputSamples(const std::string& aInput);

    /******************************************************************************//**
     * \fn serviceID
     * \brief Set service identifier associated with output quantities of interest.
     * \param [in] aInput service identifier (id)
    **********************************************************************************/
    void serviceID(const std::string& aInput);

    /******************************************************************************//**
     * \fn serviceID
     * \brief Return service identifier associated with output quantities of interest.
     * \return service identifier (id)
    **********************************************************************************/
    std::string serviceID() const;

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
     * \fn argumentName
     * \brief Return argument name of requested random quantities of interest. The \n
     * function searches in the random and deterministic lists, if a match is found, \n
     * the argument name is returned; else, an error is thrown since the requested
     * quantity of interest is not in any list.
     * \param [in] aID quantity of interest identifier
     * \return argument name
    **********************************************************************************/
    std::string argumentName(const std::string& aID) const;

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
     * \fn sharedDataName
     * \brief Return shared data name of requested random quantities of interest. The \n
     * function searches in the random and deterministic lists, if a match is found, \n
     * the shared data name is returned; else, an error is thrown since the requested
     * quantity of interest is not in any list.
     * \param [in] aID quantity of interest identifier
     * \return shared data name
    **********************************************************************************/
    std::string sharedDataName(const std::string& aID) const;

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
     * \fn output_quanitites_of_interests
     * \brief Return list of output quantities of interests (QoIs). Function returns \n
     * both random and deterministic QoIs.
     * \return list of output quantities of interests.
    **********************************************************************************/
    std::vector<std::string> outputIDs() const;

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
