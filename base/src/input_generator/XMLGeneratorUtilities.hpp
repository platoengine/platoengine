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
 * XMLGeneratorUtilities.hpp
 *
 *  Created on: Nov 19, 2019
 *
 */

#pragma once

#include "Plato_Parser.hpp"
#include "XMLGeneratorDataStruct.hpp"

#include "XMLGeneratorDefinesFileUtilities.hpp"
#include "XMLGeneratorRandomInterfaceFileUtilities.hpp"
#include "XMLGeneratorPlatoMainInputFileUtilities.hpp"
#include "XMLGeneratorPlatoMainOperationFileUtilities.hpp"
#include "XMLGeneratorLaunchScriptUtilities.hpp"

namespace XMLGen
{
    void append_version_entry(pugi::xml_document& aDocument);

    bool addNTVParameter(pugi::xml_node parent_node,
                     const std::string &name,
                     const std::string &type,
                     const std::string &value);
    bool addChild(pugi::xml_node parent_node, 
              const std::string &name, 
              const std::string &value);

    pugi::xml_node createSingleUserNodalSharedData(pugi::xml_document &aDoc,
                                                   const std::string &aName,
                                                   const std::string &aType,
                                                   const std::string &aOwner,
                                                   const std::string &aUser);
    pugi::xml_node createSingleUserNodalSharedData(pugi::xml_node &aNode,
                                                   const std::string &aName,
                                                   const std::string &aType,
                                                   const std::string &aOwner,
                                                   const std::string &aUser);
    pugi::xml_node createSingleUserElementSharedData(pugi::xml_document &aDoc,
                                                     const std::string &aName,
                                                     const std::string &aType,
                                                     const std::string &aOwner,
                                                     const std::string &aUser);
    pugi::xml_node createSingleUserElementSharedData(pugi::xml_node &aNode,
                                                     const std::string &aName,
                                                     const std::string &aType,
                                                     const std::string &aOwner,
                                                     const std::string &aUser);
    pugi::xml_node createSingleUserGlobalSharedData(pugi::xml_document &aDoc,
                                                    const std::string &aName,
                                                    const std::string &aType,
                                                    const std::string &aSize,
                                                    const std::string &aOwner,
                                                    const std::string &aUser);
    pugi::xml_node createSingleUserGlobalSharedData(pugi::xml_node &aNode,
                                                    const std::string &aName,
                                                    const std::string &aType,
                                                    const std::string &aSize,
                                                    const std::string &aOwner,
                                                    const std::string &aUser);
    pugi::xml_node createMultiUserGlobalSharedData(pugi::xml_document &aDoc,
                                                   const std::string &aName,
                                                   const std::string &aType,
                                                   const std::string &aSize,
                                                   const std::string &aOwner,
                                                   const std::vector<std::string> &aUsers);

    /******************************************************************************//**
     * \fn compute_greatest_divisor
     * \brief Find greatest divisor, i.e. value which yields modulus = 0.
     * \param [in]     aDividend dividend
     * \param [in/out] aDivisor  divisor
    **********************************************************************************/
    size_t compute_greatest_divisor(const size_t& aDividend, size_t aDivisor);

    /******************************************************************************//**
     * \fn append_children
     * \brief Append children to a PUGI XML node.
     * \param [in]     aKeys        keys to append
     * \param [in]     aValues      values to append
     * \param [in/out] aParentNode  PUGI XML node
    **********************************************************************************/
    void append_children
    (const std::vector<std::string>& aKeys,
     const std::vector<std::string>& aValues,
     pugi::xml_node& aParentNode);

    /******************************************************************************//**
     * \fn append_attributes
     * \brief Append attributes, i.e. keys and values, to a PUGI XML document.
     * \param [in]     aNodeName  pugi::node name
     * \param [in]     aKeywords  keywords to append
     * \param [in]     aValues    values to append
     * \param [in/out] aDocument  pugi::xml_document
    **********************************************************************************/
    void append_attributes
    (const std::string& aNodeName,
     const std::vector<std::string>& aKeywords,
     const std::vector<std::string>& aValues,
     pugi::xml_document& aDocument);

    /******************************************************************************//**
     * \fn append_attributes
     * \brief Append attributes, i.e. keys and values, to a PUGI XML node.
     * \param [in]     aKeywords   keys to append
     * \param [in]     aValues     values to append
     * \param [in/out] aParentNode PUGI XML node
    **********************************************************************************/
    void append_attributes
    (const std::vector<std::string>& aKeywords,
     const std::vector<std::string>& aValues,
     pugi::xml_node& aParentNode);

    /******************************************************************************//**
     * \fn transform_tokens
     * \brief Transform list of tokens into string and return its string value.
     * \param [in] aTokens list of tokens
     * \return string value
    **********************************************************************************/
    std::string transform_tokens(const std::vector<std::string>& aTokens);

    /******************************************************************************//**
     * \fn read_data_from_file
     * \brief Read data from text file and return string metadata.
     * \param [in] aFilename text filename
     * \return string metadata
    **********************************************************************************/
    std::stringstream read_data_from_file(const std::string& aFilename);

    /******************************************************************************//**
     * \fn set_key_value
     * \brief Set the value associated with the input key.
     * \param [in]     aKey           parameter key
     * \param [in]     aValue         parameter value
     * \param [in/out] aKeyToValueMap parameter key to value map, i.e. map<key,value>
    **********************************************************************************/
    void set_key_value
    (const std::string& aKey,
     const std::string& aValue,
     std::unordered_map<std::string, std::string>& aKeyToValueMap);

    /******************************************************************************//**
     * \fn set_value_keyword_to_ignore_if_empty
     * \brief If input value keyword is empty, set value keyword to 'IGNORE'.
     * \param [in/out] aValues list of value keywords
    **********************************************************************************/
    void set_value_keyword_to_ignore_if_empty(std::vector<std::string>& aValues);

    /******************************************************************************//**
     * \fn transform_key_tokens
     * \brief Return list of key keywords in key to value map.
     * \param [in] aKeyToValueMap key to value map
     * \return list of key keywords
    **********************************************************************************/
    std::vector<std::string> transform_key_tokens
    (const std::unordered_map<std::string, std::string> &aKeyToValueMap);

    /******************************************************************************//**
     * \fn transform_value_tokens
     * \brief Return list of value keywords in key to value map.
     * \param [in] aKeyToValueMap key to value map
     * \return list of value keywords
    **********************************************************************************/
    std::vector<std::string> transform_value_tokens
    (const std::unordered_map<std::string, std::string> &aKeyToValueMap);
}
// namespace XMLGen
