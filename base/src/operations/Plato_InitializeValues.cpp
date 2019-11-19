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
 * Plato_InitializeValues.cpp
 *
 *  Created on: Jun 30, 2019
 */

#include "PlatoApp.hpp"
#include "Plato_Parser.hpp"
#include "Plato_InputData.hpp"
#include "Plato_InitializeValues.hpp"

namespace Plato
{

const int MAX_CHARS_PER_LINE = 10000;

InitializeValues::InitializeValues(PlatoApp* aPlatoAppp, Plato::InputData& aNode) :
        Plato::LocalOp(aPlatoAppp),
        mValuesName("Values"),
        mUpperBoundsName("Upper Bounds"),
        mLowerBoundsName("Lower Bounds"),
        mStringMethod(""),
        mCSMFileName("")
{
    mStringMethod = Plato::Get::String(aNode, "Method");
    if(mStringMethod == "UniformValue")
    {
        mValues.clear();
        mValues.push_back(Plato::Get::Double(aNode, "InitialValue"));
    }
    else if(mStringMethod == "ReadFromCSMFile")
    {
        mCSMFileName = Plato::Get::String(aNode, "CSMFileName");
        getValuesFromCSMFile();
    }
}

void InitializeValues::operator()()
{
    // Output: Values

    std::vector<double>* tData = mPlatoApp->getValue(mValuesName);
    *tData = mValues;

    if(mStringMethod == "ReadFromCSMFile")
    {
        tData = mPlatoApp->getValue(mLowerBoundsName);
        *tData = mLowerBounds;
        tData = mPlatoApp->getValue(mUpperBoundsName);
        *tData = mUpperBounds;
    }
}

void InitializeValues::getArguments(std::vector<Plato::LocalArg>& aLocalArgs)
{
    if(mStringMethod == "UniformValue")
    {
        aLocalArgs.push_back(Plato::LocalArg {Plato::data::layout_t::SCALAR, mValuesName, 1});
    }
    else if(mStringMethod == "ReadFromCSMFile")
    {
        int tSize = mValues.size();
        aLocalArgs.push_back(Plato::LocalArg {Plato::data::layout_t::SCALAR, mValuesName, tSize});
        aLocalArgs.push_back(Plato::LocalArg {Plato::data::layout_t::SCALAR, mLowerBoundsName, tSize});
        aLocalArgs.push_back(Plato::LocalArg {Plato::data::layout_t::SCALAR, mUpperBoundsName, tSize});
    }
}

void InitializeValues::getValuesFromCSMFile()
{
    std::ifstream tInputStream;
    tInputStream.open(mCSMFileName.c_str());
    if(tInputStream.good())
    {
        getValuesFromStream(tInputStream);
        tInputStream.close();
    }
}

void InitializeValues::getValuesFromStream(std::istream &aStream)
{
    mValues.clear();
    mLowerBounds.clear();
    mUpperBounds.clear();

    char tBuffer[MAX_CHARS_PER_LINE];

    // read each line of the file (could optimize this to not read the whole file)
    while(!aStream.eof())
    {
        // read an entire line into memory
        aStream.getline(tBuffer, MAX_CHARS_PER_LINE);

        char *tCharPointer = std::strtok(tBuffer, " ");

        // skip comments
        if(tCharPointer && tCharPointer[0] == '#')
            continue;

        if(tCharPointer && std::strcmp(tCharPointer, "despmtr") == 0)
        {
            // Get the variable name
            tCharPointer = std::strtok(0, " ");
            if(!tCharPointer)
                break;
            // Get the variable value
            tCharPointer = std::strtok(0, " ");
            if(!tCharPointer)
                break;
            mValues.push_back(std::atof(tCharPointer));
            // Get the lower bound keyword
            tCharPointer = std::strtok(0, " ");
            // Get the lower bound value
            tCharPointer = std::strtok(0, " ");
            if(!tCharPointer)
                break;
            mLowerBounds.push_back(std::atof(tCharPointer));
            // Get the upper bound keyword
            tCharPointer = std::strtok(0, " ");
            // Get the upper bound value
            tCharPointer = std::strtok(0, " ");
            if(!tCharPointer)
                break;
            mUpperBounds.push_back(std::atof(tCharPointer));
        }
    }
}

}
// namespace Plato

