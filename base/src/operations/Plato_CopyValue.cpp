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
 * Plato_CopyValue.cpp
 *
 *  Created on: Jun 28, 2019
 */

#include "PlatoApp.hpp"
#include "Plato_Parser.hpp"
#include "Plato_CopyValue.hpp"
#include "Plato_InputData.hpp"

namespace Plato
{

CopyValue::CopyValue(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
        Plato::LocalOp(aPlatoApp),
        mInputName("InputValue"),
        mOutputName("OutputValue")
{
}

CopyValue::~CopyValue()
{
}

void CopyValue::operator()()
{
    if(mPlatoApp->getTimersTree())
    {
        mPlatoApp->getTimersTree()->begin_partition(Plato::timer_partition_t::timer_partition_t::filter);
    }

/*
    std::vector<double>& tToData = *(mPlatoApp->getValue(mOutputName));
    std::vector<double>* tMyValue = mPlatoApp->getValue(mInputName);
    tToData.resize(1);
    tToData[0] = tMyValue->data()[0];

    if(mPlatoApp->getTimersTree())
    {
        mPlatoApp->getTimersTree()->end_partition();
    }
*/

    std::vector<double>& tToData = *(mPlatoApp->getValue(mOutputName));

    unsigned int tDataLength = 0;
    //int tNvals = tMyAggStruct.mInputNames.size();
    int tNvals = 1;
    std::vector<double*> tFromData(tNvals);

    // read first input value
    std::vector<double>* tMyValue = mPlatoApp->getValue(mInputName);
    tFromData[0] = tMyValue->data();
    tDataLength = tMyValue->size();

    // read remaining input values
    /*
    for(int tIval = 1; tIval < tNvals; tIval++)
    {
        tMyValue = mPlatoApp->getValue(tMyAggStruct.mInputNames[tIval]);
        tFromData[tIval] = tMyValue->data();
        if(tMyValue->size() != tDataLength)
        {
            throw ParsingException("PlatoApp::Aggregator: attempted to aggregate vectors of differing lengths.");
        }
    }
*/

    tToData.resize(tDataLength);
    for(unsigned int tIndex = 0; tIndex < tDataLength; tIndex++)
    {
        tToData[tIndex] = 0.0;
        for(int j = 0; j < tNvals; j++)
        {
            tToData[tIndex] += tFromData[j][tIndex];
        }
    }
}

void CopyValue::getArguments(std::vector<Plato::LocalArg>& aLocalArgs)
{
    aLocalArgs.push_back(Plato::LocalArg
        { Plato::data::layout_t::SCALAR, mInputName });
    aLocalArgs.push_back(Plato::LocalArg
        { Plato::data::layout_t::SCALAR, mOutputName });
}

}
// namespace Plato
