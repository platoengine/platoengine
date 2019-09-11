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
 //S
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
 * Plato_RestartFileUtilities.hpp
 *
 *  Created on: Jul 20, 2019
 */

#pragma once

#include <cmath>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>

#include "Plato_Macros.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Check if restart file is opened.
 * @tparam FileType file type, options: std::ofstream or std::ifstream
 * @param [in] aRestartFile restart file
**********************************************************************************/
template<typename FileType>
inline void is_restart_file_opened(const FileType& aRestartFile)
{
    if(aRestartFile.is_open() == false)
    {
        THROWERR("RESTART FILE IS NOT OPENED.\n")
    }
}
// function is_restart_file_opened

/******************************************************************************//**
 * @brief Check if restart data identifier is not empty.
 * @param [in] aDataID data identifier
**********************************************************************************/
inline void is_restart_data_identifier_defined(const std::string& aDataID)
{
    if(aDataID.empty() == true)
    {
        THROWERR("RESTART DATA IDENTIFIER IS NOT DEFINED.\n")
    }
}
// function is_restart_data_identifier_defined

/******************************************************************************//**
 * @brief Check if restart multi-vector container is allocated.
 * @tparam ScalarType scalar value type
 * @tparam OrdinalType ordinal value type
 * @param [in] aDataID data identifier
 * @param [in] aData 2D array
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void is_restart_multivector_allocated(const std::string& aDataID, const Plato::MultiVector<ScalarType, OrdinalType>& aData)
{
    if(aData.getNumVectors() <= static_cast<OrdinalType>(0))
    {
        THROWERR("RESTART MULTI-VECTOR CONTAINER WITH ID '" + aDataID + "' IS EMPTY.\n")
    }
}
// function is_restart_multivector_allocated

/******************************************************************************//**
 * @brief Check if restart vector container is allocated.
 * @tparam ScalarType scalar value type
 * @tparam OrdinalType ordinal value type
 * @param [in] aDataID data identifier
 * @param [in] aData 1D array
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void is_restart_vector_allocated(const std::string& aDataID, const Plato::Vector<ScalarType, OrdinalType>& aData)
{
    if(aData.size() <= static_cast<OrdinalType>(0))
    {
        THROWERR("RESTART VECTOR CONTAINER WITH ID '" + aDataID + "' IS EMPTY.\n")
    }
}
// function is_restart_vector_allocated

/******************************************************************************//**
 * @brief Check if restart data in multi-vector is finite.
 * @tparam ScalarType scalar value type
 * @tparam OrdinalType ordinal value type
 * @param [in] aDataID data identifier
 * @param [in] aData 2D array
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void is_restart_multivector_data_finite(const std::string& aDataID, const Plato::MultiVector<ScalarType, OrdinalType>& aData)
{
    const ScalarType tDotValue = Plato::dot(aData, aData);
    if(std::isfinite(tDotValue) == false)
    {
        THROWERR("RESTART MULTI-VECTOR CONTAINER WITH ID '" + aDataID + "' HAS CORRUPTED DATA, I.E. NON-FINITE NUMBERS.\n")
    }
}
// function is_restart_multivector_data_finite

/******************************************************************************//**
 * @brief Check if restart data in vector is finite.
 * @tparam ScalarType scalar value type
 * @tparam OrdinalType ordinal value type
 * @param [in] aDataID data identifier
 * @param [in] aData 1D array
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void is_restart_vector_data_finite(const std::string& aDataID, const Plato::Vector<ScalarType, OrdinalType>& aData)
{
    const ScalarType tDotValue = aData.dot(aData);
    if(std::isfinite(tDotValue) == false)
    {
        THROWERR("RESTART VECTOR CONTAINER WITH ID '" + aDataID + "' HAS CORRUPTED DATA, I.E. NON-FINITE NUMBERS.\n")
    }
}
// function is_restart_vector_data_finite

/******************************************************************************//**
 * @brief Check if restart data is finite.
 * @tparam DataType data type, e.g. double, int, size_t
 * @param [in] aDataID data identifier
 * @param [in] aData value
**********************************************************************************/
template<typename DataType>
inline void is_restart_data_finite(const std::string& aDataID, const DataType& aData)
{

    if(std::isfinite(aData) == false)
    {
        THROWERR("RESTART DATA WITH ID '" + aDataID + "' IS NOT A FINITE NUMBER.\n")
    }
}
// function is_restart_data_finite

/******************************************************************************//**
 * @brief Check if restart identifier is valid.
 * @param [in] aDataID data identifier
 * @param [in] aFoundData boolean flag
**********************************************************************************/
inline void is_restart_data_id_defined(const std::string& aDataID, const bool& aFoundData)
{

    if(aFoundData == false)
    {
        THROWERR("RESTART DATA WITH ID '" + aDataID + "' IS NOT DEFINED IN THE RESTART FILE.\n")
    }
}
// function is_restart_data_id_defined

/******************************************************************************//**
 * @brief Print data in multi-vector to restart file.
 * @tparam ScalarType scalar value type
 * @tparam OrdinalType ordinal value type
 * @param [in] aData 2D array
 * @param [in] aDataID data identifier
 * @param [in/out] aRestartFile output file
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void output_restart_data_multivector(const Plato::MultiVector<ScalarType, OrdinalType>& aData,
                                            const std::string& aDataID,
                                            std::ofstream& aRestartFile)
{
    Plato::is_restart_file_opened(aRestartFile);
    Plato::is_restart_data_identifier_defined(aDataID);
    Plato::is_restart_multivector_allocated(aDataID, aData);
    Plato::is_restart_multivector_data_finite(aDataID, aData);

    aRestartFile << std::setprecision(16) << aDataID.c_str() << "\n";
    for(OrdinalType tVecIndex = 0; tVecIndex < aData.getNumVectors(); tVecIndex++)
    {
        const Plato::Vector<ScalarType, OrdinalType> & tVector = aData[tVecIndex];
        for(OrdinalType tDataIndex = 0; tDataIndex < tVector.size(); tDataIndex++)
        {
            aRestartFile << tVector[tDataIndex] << " ";
        }
        aRestartFile << "\n";
    }
    aRestartFile << "\n";
}
// function output_restart_data_multivector

/******************************************************************************//**
 * @brief Print data in vector to restart file.
 * @tparam ScalarType scalar value type
 * @tparam OrdinalType ordinal value type
 * @param [in] aData 1D array
 * @param [in] aDataID data identifier
 * @param [in/out] aRestartFile output file
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void output_restart_data_vector(const Plato::Vector<ScalarType, OrdinalType>& aData,
                                       const std::string& aDataID,
                                       std::ofstream& aRestartFile)
{
    Plato::is_restart_file_opened(aRestartFile);
    Plato::is_restart_data_identifier_defined(aDataID);
    Plato::is_restart_vector_allocated(aDataID, aData);
    Plato::is_restart_vector_data_finite(aDataID, aData);

    aRestartFile << std::setprecision(16) << aDataID.c_str() << "\n";
    for(OrdinalType tDataIndex = 0; tDataIndex < aData.size(); tDataIndex++)
    {
        aRestartFile << aData[tDataIndex] << " ";
    }
    aRestartFile << "\n\n";
}
// function output_restart_data_vector

/******************************************************************************//**
 * @brief Print data in multi-vector to restart file.
 * @tparam DataType value type, e.g. double, int, size_t
 * @param [in] aData value
 * @param [in] aDataID data identifier
 * @param [in/out] aRestartFile output file
**********************************************************************************/
template<typename DataType>
inline void output_restart_data_value(const DataType& aData, const std::string& aDataID, std::ofstream& aRestartFile)
{
    Plato::is_restart_file_opened(aRestartFile);
    Plato::is_restart_data_identifier_defined(aDataID);
    Plato::is_restart_data_finite(aDataID, aData);

    aRestartFile << std::setprecision(16) << aDataID.c_str() << "\n";
    aRestartFile << aData << "\n\n";
}
// function output_restart_data_value

/******************************************************************************//**
 * @brief Read restart vector data from restart file.
 * @tparam ScalarType scalar value type
 * @tparam OrdinalType ordinal value type
 * @param [in] aDataID data identifier
 * @param [in/out] aRestartFile output file
 * @param [in/out] aData 1D array
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void read_restart_data_vector(const std::string& aDataID,
                                     std::ifstream& aRestartFile,
                                     Plato::Vector<ScalarType, OrdinalType>& aData)
{
    Plato::is_restart_file_opened(aRestartFile);
    Plato::is_restart_data_identifier_defined(aDataID);
    Plato::is_restart_vector_allocated(aDataID, aData);

    std::string tLine;
    bool tFoundData = false;

    while(!aRestartFile.eof())
    {
        std::getline(aRestartFile, tLine);
        int tOffset = tLine.find(aDataID, 0);
        if(std::size_t(tOffset) != std::string::npos)
        {
            tFoundData = true;
            std::string tNextLine;
            std::getline(aRestartFile, tNextLine);

            std::string tToken;
            std::istringstream tTokenStream(tNextLine);

            size_t tValueIndex = 0;
            while(std::getline(tTokenStream, tToken, ' '))
            {
                aData[tValueIndex] = std::stod(tToken);
                tValueIndex++;
            }

            break;
        }
    }

    Plato::is_restart_data_id_defined(aDataID, tFoundData);
    Plato::is_restart_vector_data_finite(aDataID, aData);
}
// function read_restart_data_vector

/******************************************************************************//**
 * @brief Read restart multi-vector data from restart file.
 * @tparam ScalarType scalar value type
 * @tparam OrdinalType ordinal value type
 * @param [in] aDataID data identifier
 * @param [in/out] aRestartFile output file
 * @param [in/out] aData 2D array
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void read_restart_data_multivector(const std::string& aDataID,
                                          std::ifstream& aRestartFile,
                                          Plato::MultiVector<ScalarType, OrdinalType>& aData)
{
    Plato::is_restart_file_opened(aRestartFile);
    Plato::is_restart_data_identifier_defined(aDataID);
    Plato::is_restart_multivector_allocated(aDataID, aData);

    std::string tLine;
    bool tFoundData = false;

    while(!aRestartFile.eof())
    {
        std::getline(aRestartFile, tLine);
        int tOffset = tLine.find(aDataID, 0);
        if(std::size_t(tOffset) != std::string::npos)
        {
            tFoundData = true;
            for(OrdinalType tVecIndex = 0; tVecIndex < aData.getNumVectors(); tVecIndex++)
            {
                std::string tNextLine;
                std::getline(aRestartFile, tNextLine);

                std::string tToken;
                OrdinalType tValueIndex = 0;
                std::istringstream tTokenStream(tNextLine);
                while(std::getline(tTokenStream, tToken, ' '))
                {
                    aData(tVecIndex, tValueIndex) = std::stod(tToken);
                    tValueIndex++;
                }
            }
            break;
        }
    }

    Plato::is_restart_data_id_defined(aDataID, tFoundData);
    Plato::is_restart_multivector_data_finite(aDataID, aData);
}
// function read_restart_data_multivector

/******************************************************************************//**
 * @brief Read restart multi-vector data from restart file.
 * @tparam DataType value type, e.g. double, int, size_t
 * @param [in] aDataID data identifier
 * @param [in/out] aRestartFile output file
 * @param [in/out] aData value
**********************************************************************************/
template<typename DataType>
inline void read_restart_data_value(const std::string &aDataID,
                                    std::ifstream &aRestartFile,
                                    DataType &aData)
{
    Plato::is_restart_file_opened(aRestartFile);
    Plato::is_restart_data_identifier_defined(aDataID);

    std::string tLine;
    bool tFoundData = false;

    while (!aRestartFile.eof())
    {
        std::getline(aRestartFile, tLine);
        int tOffset = tLine.find(aDataID, 0);
        if (std::size_t(tOffset) != std::string::npos)
        {
            tFoundData = true;
            std::string tNextLine;
            std::getline(aRestartFile, tNextLine);
            aData = std::stod(tNextLine);
            break;
        }
    }

    Plato::is_restart_data_id_defined(aDataID, tFoundData);
    Plato::is_restart_data_finite(aDataID, aData);
}
// function read_restart_data_value

}
// namespace Plato
