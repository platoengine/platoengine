/*
 * XMLGeneratorParseMetadata.hpp
 *
 *  Created on: Jun 18, 2020
 */

#pragma once

#include <iostream>

namespace XMLGen
{

/******************************************************************************//**
 * \class ParseMetadata
 * \tparam MetadataType output metadata type
 * \brief Blueprint used by derived instances responsible for parsing input metadata.
**********************************************************************************/
template<typename MetadataType>
class ParseMetadata
{
public:
    /******************************************************************************//**
     * \brief Virtual destructor.
    **********************************************************************************/
    virtual ~ParseMetadata(){}

    /******************************************************************************//**
     * \fn data
     * \brief Return metadata.
     * \return container of objective functions and corresponding metadata
    **********************************************************************************/
    virtual MetadataType data() const = 0;

    /******************************************************************************//**
     * \fn parse
     * \brief Parse input metadata.
     * \param [in] aInputFile input metadata stream
    **********************************************************************************/
    virtual void parse(std::istream& aInputFile) = 0;
};
// class ParseMetadata

}
// namespace XMLGen
