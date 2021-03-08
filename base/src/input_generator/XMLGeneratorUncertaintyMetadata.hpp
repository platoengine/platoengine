/*
 * XMLGeneratorUncertaintyMetadata.hpp
 *
 *  Created on: Jul 26, 2020
 */

#pragma once

#include <string>
#include <unordered_map>

#include "XMLG_Macros.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

struct Uncertainty
{
private:
    std::unordered_map<std::string, std::string> mMetaData; /*!< service metadata, map< tag, value > */

private:
    /******************************************************************************//**
     * \fn getValue
     * \brief Return string value for property with input tag; else, throw an error if \n
     * property is not defined in the metadata.
     * \param [in] aTag property tag
     * \return property string value
    **********************************************************************************/
    std::string getValue(const std::string& aTag) const;

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
     * \fn append
     * \brief Append parameter to metadata.
     * \param [in] aTag   parameter tag
     * \param [in] aValue parameter value
    **********************************************************************************/
    void append(const std::string& aTag, const std::string& aValue);

    /******************************************************************************//**
     * \fn seed
     * \brief Return string value for keyword 'seed'.
     * \return random seed
    **********************************************************************************/
    std::string seed() const;

    /******************************************************************************//**
     * \fn seed
     * \brief Set string value for keyword 'seed'.
     * \param [in] aInput random seed
    **********************************************************************************/
    void seed(const std::string& aInput);

    /******************************************************************************//**
     * \fn dimensions
     * \brief Return string value for keyword 'dimensions'.
     * \return random vector dimensions
    **********************************************************************************/
    std::string dimensions() const;

    /******************************************************************************//**
     * \fn dimensions
     * \brief Set string value for keyword 'dimensions'.
     * \param [in] aInput random vector dimensions
    **********************************************************************************/
    void dimensions(const std::string& aInput);

    /******************************************************************************//**
     * \fn category
     * \brief Return string value for keyword 'category'.
     * \return non-deterministic parameter category, e.g. load, material
    **********************************************************************************/
    std::string category() const;

    /******************************************************************************//**
     * \fn category
     * \brief Set string value for keyword 'category'.
     * \param [in] aInput string value
    **********************************************************************************/
    void category(const std::string& aInput);

    /******************************************************************************//**
     * \fn tag
     * \brief Return string value for keyword 'tag'.
     * \return non-deterministic parameter tag
    **********************************************************************************/
    std::string tag() const;

    /******************************************************************************//**
     * \fn tag
     * \brief Set string value for keyword 'tag'.
     * \param [in] aInput string value
    **********************************************************************************/
    void tag(const std::string& aInput);

    /******************************************************************************//**
     * \fn id
     * \brief Return string value for keyword 'id'.
     * \return identification
    **********************************************************************************/
    std::string id() const;

    /******************************************************************************//**
     * \fn id
     * \brief Set string value for keyword 'id'.
     * \param [in] aInput string value
    **********************************************************************************/
    void id(const std::string& aInput);

    /******************************************************************************//**
     * \fn attribute
     * \brief Return string value for keyword 'attribute'.
     * \return value
    **********************************************************************************/
    std::string attribute() const;

    /******************************************************************************//**
     * \fn attribute
     * \brief Set string value for keyword 'attribute'.
     * \param [in] aInput string value
    **********************************************************************************/
    void attribute(const std::string& aInput);

    /******************************************************************************//**
     * \fn distribution
     * \brief Return string value for keyword 'distribution'.
     * \return distribution type
    **********************************************************************************/
    std::string distribution() const;

    /******************************************************************************//**
     * \fn distribution
     * \brief Set string value for keyword 'distribution'.
     * \param [in] aInput string value
    **********************************************************************************/
    void distribution(const std::string& aInput);

    /******************************************************************************//**
     * \fn samples
     * \brief Return string value for keyword 'number_samples'.
     * \return number of samples
    **********************************************************************************/
    std::string samples() const;

    /******************************************************************************//**
     * \fn samples
     * \brief Set string value for keyword 'number_samples'.
     * \param [in] aInput string value
    **********************************************************************************/
    void samples(const std::string& aInput);

    /******************************************************************************//**
     * \fn filename
     * \brief Return string value for keyword 'filename'.
     * \return filename
    **********************************************************************************/
    std::string filename() const;

    /******************************************************************************//**
     * \fn filename
     * \brief Set string value for keyword 'filename'.
     * \param [in] aInput string value
    **********************************************************************************/
    void filename(const std::string& aInput);

    /******************************************************************************//**
     * \fn std
     * \brief Return string value for keyword 'standard_deviation'.
     * \return standard deviation
    **********************************************************************************/
    std::string std() const;

    /******************************************************************************//**
     * \fn std
     * \brief Set string value for keyword 'standard_deviation'.
     * \param [in] aInput string value
    **********************************************************************************/
    void std(const std::string& aInput);

    /******************************************************************************//**
     * \fn mean
     * \brief Return string value for keyword 'mean'.
     * \return mean
    **********************************************************************************/
    std::string mean() const;

    /******************************************************************************//**
     * \fn mean
     * \brief Set string value for keyword 'mean'.
     * \param [in] aInput string value
    **********************************************************************************/
    void mean(const std::string& aInput);

    /******************************************************************************//**
     * \fn guess
     * \brief Return string value for keyword 'initial_guess'.
     * \return mean
    **********************************************************************************/
    std::string guess() const;

    /******************************************************************************//**
     * \fn guess
     * \brief Set string value for keyword 'initial_guess'.
     * \param [in] aInput string value
    **********************************************************************************/
    void guess(const std::string& aInput);

    /******************************************************************************//**
     * \fn lower
     * \brief Return string value for keyword 'lower_bound'.
     * \return lower bound
    **********************************************************************************/
    std::string lower() const;

    /******************************************************************************//**
     * \fn lower
     * \brief Set string value for keyword 'lower_bound'.
     * \param [in] aInput string value
    **********************************************************************************/
    void lower(const std::string& aInput);

    /******************************************************************************//**
     * \fn upper
     * \brief Return string value for keyword 'upper_bound'.
     * \return upper bound
    **********************************************************************************/
    std::string upper() const;

    /******************************************************************************//**
     * \fn upper
     * \brief Set string value for keyword 'upper_bound'.
     * \param [in] aInput string value
    **********************************************************************************/
    void upper(const std::string& aInput);

    /******************************************************************************//**
     * \fn correlationFilename
     * \brief Return string value for keyword 'correlation_filename'.
     * \return correlation matrix filename
    **********************************************************************************/
    std::string correlationFilename() const;

    /******************************************************************************//**
     * \fn correlationFilename
     * \brief Set string value for keyword 'correlation_filename'.
     * \param [in] aInput correlation matrix filename
    **********************************************************************************/
    void correlationFilename(const std::string& aInput);
};
// struct Uncertainty

}
// namespace XMLGen
