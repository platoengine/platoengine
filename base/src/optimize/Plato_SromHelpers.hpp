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
 * Plato_SromHelpers.hpp
 *
 *  Created on: Apr 28, 2020
 */

#pragma once

#include <cmath>
#include <cfloat>
#include <vector>
#include <locale>
#include <utility>
#include <sstream>
#include <iomanip>
#include <fstream>

#include "Plato_Macros.hpp"

namespace Plato
{

namespace srom
{

/******************************************************************************//**
 * \fn tolower
 * \brief Convert uppercase word to lowercase.
 * \param [in] aInput word
 * \return lowercase word
**********************************************************************************/
inline std::string tolower(const std::string& aInput)
{
    std::locale tLocale;
    std::ostringstream tOutput;
    for (auto& tChar : aInput)
    {
        tOutput << std::tolower(tChar,tLocale);
    }
    return (tOutput.str());
}
// function tolower

/******************************************************************************//**
 * \fn to_string
 * \brief Convert double to string.
 * \param [in] aInput     floating-point number
 * \param [in] aPrecision precision (default = 16 digits)
 * \return converted floating-point number as a value of type string
 * \note If precision < 0, set precision to its default value, which is '6' digits.
 * \note If precision is infinite or NaN, set precision to its default value, which is '0' digits.
**********************************************************************************/
inline std::string to_string(const double& aInput, int aPrecision = 16)
{
    if(std::isfinite(aInput) == false)
    {
        THROWERR("Convert double to string: detected a non-finite floating-point number.")
    }

    std::ostringstream tValueString;
    tValueString << std::fixed; // forces fix-point notation instead of default scientific notation
    tValueString << std::setprecision(aPrecision); // sets precision for fixed-point notation
    tValueString << aInput;
    return (tValueString.str());
}

// function to_string
/******************************************************************************//**
 * \fn write_data
 *
 * \brief Make a CSV file with one or more columns of floating-point values \n
 *   Each column of data is represented by the pair <column name, column data> \n
 *   as std::pair<std::string, std::vector<double>> (i.e. Plato::srom::SampleProbPairs). \n
 *
 * \param [in] aFilename   filename with sample-probability pair information
 * \param [in] aDataset    sample-probability pairs
 * \param [in] aPrecision  floating-point number precision (default = 16 digits)
 *
 * \note All columns should have the same size, i.e. the same number of rows.
**********************************************************************************/
using DataPairs = std::pair<std::string, std::vector<double>>;

inline void write_data
(const std::string &aFilename,
 const std::vector<DataPairs> &aDataset,
 int aPrecision = 16)
{
    // Create an output filestream object
    std::ofstream tMyFile(aFilename);
    tMyFile << std::fixed;
    tMyFile << std::setprecision(aPrecision);

    // Send column names to the stream
    for (size_t tColumn = 0; tColumn < aDataset.size(); ++tColumn)
    {
        tMyFile << aDataset.at(tColumn).first;
        if (tColumn != aDataset.size() - 1) { tMyFile << ","; } // No comma at end of line
    }
    tMyFile << "\n";

    // Send data to the stream
    for (size_t tRow = 0; tRow < aDataset.at(0).second.size(); ++tRow)
    {
        for (size_t tColumn = 0; tColumn < aDataset.size(); ++tColumn)
        {
            tMyFile << aDataset.at(tColumn).second.at(tRow);
            if (tColumn != aDataset.size() - 1) { tMyFile << ","; } // No comma at end of line
        }
        tMyFile << "\n";
    }

    // Close the file
    tMyFile.close();
}
// function write_data

/******************************************************************************//**
 * \fn read_column_names
 * \brief Read the column names from the CSV file.
 * \param [in] aMyFile CSV file
 * \param [in] aResult sample probability pairs
**********************************************************************************/
inline void read_column_names
(std::ifstream& aMyFile,
 std::vector<DataPairs>& aResult)
{
    std::string tLine, tColumnName;
    if(aMyFile.good())
    {
        // Extract the first line in the file
        std::getline(aMyFile, tLine);

        // Create a stringstream from line
        std::stringstream tStringStream(tLine);

        // Extract each column name
        while(std::getline(tStringStream, tColumnName, ','))
        {
            // Initialize and add <colname, int vector> pairs to result
            auto tLowerCaseName = Plato::srom::tolower(tColumnName);
            aResult.push_back( { tLowerCaseName, std::vector<double>{} } );
        }
    }
    else
    {
        THROWERR("Read Column Names: Input file stream is corrupted.")
    }
}
// function read_column_names

/******************************************************************************//**
 * \fn read_data
 * \brief Read data from CSV file line by line
 * \param [in] aMyFile CSV file
 * \param [in] aResult sample probability pairs
**********************************************************************************/
inline void read_data
(std::ifstream& aMyFile,
 std::vector<DataPairs>& aResult)
{
    if (aMyFile.good())
    {
        std::string tLine;
        double tValue = 0;
        while (std::getline(aMyFile, tLine))
        {
            // Create a stringstream of the current line
            std::stringstream tStringStream(tLine);

            // Keep track of the current column index
            size_t tColIdx = 0;

            // Extract each integer
            while (tStringStream >> tValue)
            {
                // Add the current integer to the 'colIdx' column's values vector
                aResult.at(tColIdx).second.push_back(tValue);

                // If the next token is a comma, ignore it and move on
                if (tStringStream.peek() == ',')
                {
                    tStringStream.ignore();
                }

                // Increment the column index
                tColIdx++;
            }
        }
    }
    else
    {
        THROWERR("Read Data: Input file stream is corrupted.")
    }
}
// function read_data

/******************************************************************************//**
 * \fn read_sample_probability_pairs
 * \brief Read a CSV file into a vector of <DataPairs> where each pair represents \n
 *   <column name, column values>
 * \param [in] aFilename filename with sample-probability pair information
**********************************************************************************/
inline std::vector<DataPairs>
read_sample_probability_pairs
(const std::string &aFilename)
{
    // Create an input filestream
    std::ifstream tMyFile(aFilename);

    // Make sure the file is open
    if (!tMyFile.is_open())
    {
        std::ostringstream tMsg;
        tMsg << "Read Sample Probability Pairs: Could not open file with tag '" << aFilename << "'.";
        THROWERR(tMsg.str().c_str())
    }

    // Create a vector of <string, double vector> pairs to store the result
    std::vector<DataPairs> tResult;

    Plato::srom::read_column_names(tMyFile, tResult);
    Plato::srom::read_data(tMyFile, tResult);

    // Close file
    tMyFile.close();

    return tResult;
}
// funciton read_sample_probability_pairs

}
// namespace srom

}
// namespace Plato
