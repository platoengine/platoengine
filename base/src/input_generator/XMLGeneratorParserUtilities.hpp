/*
 * XMLGeneratorParserUtilities.hpp
 *
 *  Created on: Jun 16, 2020
 */

#pragma once

#include <map>
#include <vector>
#include <cstring>
#include <unordered_map>

namespace XMLGen
{

/*!< map from xml file input keyword string to pair< pair<tokens,input_keyword_value>, input_keyword_default_value> >, \n
 * i.e. map< input_keyword_string, pair< pair<tokens,input_keyword_value>, input_keyword_default_value > > */
using MetaDataTags = std::unordered_map<std::string, std::pair<std::pair<std::vector<std::string>,std::string>, std::string>>;

// a "concretized criterion" is a criterion paired with a service and a scenario.
// each concretized criterion will require a shared data, and can be used in
// stage operations
// order is criterionID, serviceID, scenarioID
using ConcretizedCriterion = std::tuple<std::string,std::string,std::string>;

/******************************************************************************//**
 * \fn get_concretized_criterion_identifier_string
 * \brief Returns the identifier for a concretized criterion.
 * \param [in] aConcretizedCriterion input concretized criterion 
 * \return identifier string
**********************************************************************************/
std::string get_concretized_criterion_identifier_string(ConcretizedCriterion aConcretizedCriterion);

/******************************************************************************//**
 * \fn unique
 * \brief Returns true if each string element is unique.
 * \param [in] aInput string container
 * \return Flag
**********************************************************************************/
bool unique(const std::vector<std::string>& aInput);

/******************************************************************************//**
 * \fn erase_tag_values
 * \brief Erase input tag values in map.
 * \param [in/out] aTags map from property tag to pair< pair<token,input_value>, default> >
**********************************************************************************/
void erase_tag_values(XMLGen::MetaDataTags& aTags);

/******************************************************************************//**
 * \fn parse_input_metadata
 * \brief Parse input metadata and set token-value pairs in XMLGen::MetaDataTags map.
 * \param [in]  aStopKeys   keys used to denote end of the parameter block to be parsed
 * \param [in]  aInputFile  input file string
 * \param [out] aTags       map from property tag to pair< pair<token,input_value>, default> >
**********************************************************************************/
void parse_input_metadata
(const std::vector<std::string>& aStopKeys,
 std::istream& aInputFile,
 XMLGen::MetaDataTags& aTags);

/******************************************************************************//**
 * \fn parse_single_value
 * \brief Return matching keyword if input token matches target token.
 * \param [in]  aTokens   input token
 * \param [in]  aTarget   target token
 * \param [out] aKeyword  matching keyword
**********************************************************************************/
bool parse_single_value
(const std::vector<std::string> &aTokens,
 const std::vector<std::string> &aTarget,
 std::string &aKeyword);

/******************************************************************************//**
 * \fn to_lower
 * \brief Convert uppercase word to lowercase.
 * \param [in]  aInput  uppercase keyword
 * \return lowercase keyword
**********************************************************************************/
std::string to_lower(const std::string &aInput);

/******************************************************************************//**
 * \fn to_lower
 * \brief Convert uppercase token to lowercase.
 * \param [in\out]  aInput  uppercase token
**********************************************************************************/
void to_lower(std::vector<std::string>& aInput);

/******************************************************************************//**
 * \fn parse_tokens
 * \brief Parse tokens from buffer.
 * \param [in]  aBuffer token buffer
 * \param [out] aTokens parsed tokens
**********************************************************************************/
bool parse_tokens(char *aBuffer, std::vector<std::string> &aTokens);

/******************************************************************************//**
 * \fn transform_keyword_values
 * \brief Convert tokens into string.
 * \param [in] aTokens tokens
 * \return transformed tokens, i.e. tag
**********************************************************************************/
std::string transform_keyword_values(const std::vector<std::string>& aTokens);

/******************************************************************************//**
 * \fn is_input_keyword_empty
 * \brief Throws error if input keyword is empty.
 * \param [in] aInputTokens input tokens, i.e. tokens parsed
 * \param [in] aTargetKey   target keyword
**********************************************************************************/
void is_input_keyword_empty
(const std::vector<std::string>& aInputTokens,
 const std::vector<std::string>& aTargetKey);

/******************************************************************************//**
 * \fn tokens_match
 * \brief Returns true if input tokens match target key.
 * \param [in] aInputTokens input tokens, i.e. tokens parsed
 * \param [in] aTargetKey   target keyword
 * \return flag
**********************************************************************************/
bool tokens_match
(const std::vector<std::string>& aInputTokens,
 const std::vector<std::string>& aTargetKey);

/******************************************************************************//**
 * \fn parse_tag_values
 * \brief Parse tag values associated with input tokens.
 * \param [in]  aTokens list of tokens
 * \param [out] aTags   map from property tag to pair< pair<token,input_value>, default> >
**********************************************************************************/
void parse_tag_values(const std::vector<std::string>& aTokens, XMLGen::MetaDataTags& aTags);

/******************************************************************************//**
 * \fn is_integer
 * \brief Return false if input string is not a finite number.
 * \param [in]  aInput input token
 * \return flag
**********************************************************************************/
bool is_integer(const std::string& aInput);

/******************************************************************************//**
 * \fn split
 * \brief Split input string into list of tokens
 * \param [in]      aInput   input string
 * \param [in/out]  aOutput  list of tokens
 * \param [in]      aToLower transform tokens to lowercase if flag is set to true
**********************************************************************************/
void split(const std::string& aInput, std::vector<std::string>& aOutput, bool aToLower = true);

/******************************************************************************//**
 * \fn check_data_layout_keyword
 * \brief Throw error if 'data layout' keyword value is not supported.
 * \param [in] aInput 'data layout' keyword
 * \return valid 'data layout' keyword
**********************************************************************************/
std::string check_data_layout_keyword(const std::string& aInput);

/******************************************************************************//**
 * \fn check_output_keyword
 * \brief Throw error if 'output' keyword value is not supported.
 * \param [in] aInput 'output' keyword
 * \return valid 'output' keyword
**********************************************************************************/
std::string check_output_keyword(const std::string& aInput);

/******************************************************************************//**
 * \fn return_output_qoi_data_layout
 * \brief Return data layout for supported output quantity of interest (QoI). \n
 * Throw error if QoI keyword is not supported.
 * \param [in] aInput quantity of interest keyword
 * \return valid data layout
**********************************************************************************/
std::string return_output_qoi_data_layout(const std::string& aInput);

/******************************************************************************//**
 * \fn check_data_layout
 * \brief Throw error if 'data layout' keyword value is not supported.
 * \param [in] aInput 'data layout' keyword
 * \return valid 'data layout' keyword
**********************************************************************************/
std::string check_data_layout(const std::string& aInput);

/******************************************************************************//**
 * \fn check_code_keyword
 * \brief Throw error if 'code' keyword value is not supported.
 * \param [in] aInput 'code' keyword
 * \return valid 'code' keyword
**********************************************************************************/
std::string check_code_keyword(const std::string& aInput);

/******************************************************************************//**
 * \fn transform_boolean_key
 * \brief Throw error if conversion from string to boolean fails.
 * \param [in] aInput input string
 * \return boolean flag
**********************************************************************************/
bool transform_boolean_key(const std::string& aInput);

/******************************************************************************//**
 * \fn check_physics_keyword
 * \brief Throw error if 'physics' keyword value is not supported.
 * \param [in] aInput 'physics' keyword
 * \return valid 'physics' keyword value
**********************************************************************************/
std::string check_physics_keyword(const std::string& aInput);

/******************************************************************************//**
 * \fn check_spatial_dimensions_keyword
 * \brief Throw error if 'dimensions' keyword value is not supported.
 * \param [in] aInput 'dimensions' keyword
 * \return valid 'dimensions' keyword value
**********************************************************************************/
std::string check_spatial_dimensions_keyword(const std::string& aInput);

/******************************************************************************//**
 * \fn is_metadata_block_id_valid
 * \brief Return error if metadata block identifier (id) is invalid.
 * \param [in] aTokens list of tokens
**********************************************************************************/
void is_metadata_block_id_valid(const std::vector<std::string>& aTokens);

/******************************************************************************//**
 * \fn parseTokens
 * \brief split a string up into space delimited tokens
 * \param [in] buffer input string
 * \param [in] aTokens list of tokens
**********************************************************************************/
bool parseTokens(char *buffer, std::vector<std::string> &tokens);

}
// namespace XMLGen
