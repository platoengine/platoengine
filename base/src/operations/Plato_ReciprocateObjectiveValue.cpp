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
 * Plato_ReciprocateObjectiveValue.cpp
 *
 *  Created on: Jun 27, 2019
 */

#include "PlatoApp.hpp"
#include "Plato_InputData.hpp"
#include "Plato_Exceptions.hpp"
#include "Plato_ReciprocateObjectiveValue.hpp"

namespace Plato
{

ReciprocateObjectiveValue::ReciprocateObjectiveValue(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
        Plato::LocalOp(aPlatoApp)
{
    Plato::InputData tOutputNode = Plato::Get::InputData(aNode, "Output");
    if(aNode.size<Plato::InputData>("Output") > 1)
    {
        throw ParsingException("PlatoApp::ReciprocateObjectiveValue: more than one Output specified.");
    }
    mOutputName = Plato::Get::String(tOutputNode, "ArgumentName");

    Plato::InputData tInputNode = Plato::Get::InputData(aNode, "Input");
    if(aNode.size<Plato::InputData>("Input") > 1)
    {
        throw ParsingException("PlatoApp::ReciprocateObjectiveValue: more than one Input specified.");
    }
    mInputName = Plato::Get::String(tInputNode, "ArgumentName");
}

void ReciprocateObjectiveValue::operator()()
{
    std::vector<double>& tToData = *(mPlatoApp->getValue(mOutputName));
    std::vector<double>& tFromData = *(mPlatoApp->getValue(mInputName));

    for(unsigned int tIndex = 0; tIndex < tToData.size(); tIndex++)
    {
        tToData[tIndex] = tFromData[tIndex] != 0 ? 1.0 / tFromData[tIndex] : 0.0;
    }
}

void ReciprocateObjectiveValue::getArguments(std::vector<Plato::LocalArg> & aLocalArgs)
{
    aLocalArgs.push_back(Plato::LocalArg {Plato::data::layout_t::SCALAR, mOutputName,/*length=*/1});
    aLocalArgs.push_back(Plato::LocalArg {Plato::data::layout_t::SCALAR, mInputName, /*length=*/1});
}

}
