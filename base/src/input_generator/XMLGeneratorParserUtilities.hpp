/*
 * XMLGeneratorParserUtilities.hpp
 *
 *  Created on: Jun 16, 2020
 */

#pragma once

#include <map>
#include <vector>

namespace XMLGen
{

/*!< map from tag to container of token-value pairs, i.e. map<tag, pair<tokens,value> > */
using UseCaseTags = std::map<std::string, std::pair<std::vector<std::string>,std::string>>;

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
 * \fn transform_tag_values
 * \brief Convert tokens into string.
 * \param [in] aTokens tokens
 * \return transformed tokens, i.e. tag
**********************************************************************************/
std::string transform_tag_values(const std::vector<std::string>& aTokens);

/******************************************************************************//**
 * \fn parse_tag_values
 * \brief Parse tag values associated with input tokens.
 * \param [in]  aTokens list of tokens
 * \param [out] aTags   map from valid tags to valid tokens-value pairs
**********************************************************************************/
void parse_tag_values(const std::vector<std::string>& aTokens, XMLGen::UseCaseTags& aTags);

}
// namespace XMLGen
