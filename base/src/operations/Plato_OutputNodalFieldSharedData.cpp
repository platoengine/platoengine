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
 * Plato_OutputNodalFieldSharedData.cpp
 *
 *  Created on: October 10, 2020
 */

#include <string>
#include <cstdio>
#include <cstdlib>

#include "PlatoApp.hpp"
#include "Plato_OutputNodalFieldSharedData.hpp"
#include "Plato_OperationsUtilities.hpp"

namespace Plato
{

OutputNodalFieldSharedData::OutputNodalFieldSharedData(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
        Plato::LocalOp(aPlatoApp)
{
    mIndex = 0;
    for(Plato::InputData tInputNode : aNode.getByName<Plato::InputData>("Input"))
    {
        mInputNames.push_back(Plato::Get::String(tInputNode, "ArgumentName"));
    }
}

OutputNodalFieldSharedData::~OutputNodalFieldSharedData()
{
}

void OutputNodalFieldSharedData::getArguments(std::vector<Plato::LocalArg>& aLocalArgs)
{
    for(auto& tInputName : mInputNames) {
        aLocalArgs.push_back(Plato::LocalArg(Plato::data::layout_t::SCALAR_FIELD, tInputName));
    }
}

void OutputNodalFieldSharedData::operator()()
{
    int tMyRank = 0;
    MPI_Comm_rank(mPlatoApp->getComm(), &tMyRank);
    if(tMyRank == 0)
    {
        mIndex++;
        for(size_t i=0; i<mInputNames.size(); ++i)
        {
            auto tInputName = mInputNames[i];
            auto tFileName = tInputName;
            
            tFileName += std::to_string(mIndex);
            FILE *fp=fopen(tFileName.c_str(), "w");
            if(fp)
            {
                // get input data
                auto tInfield = mPlatoApp->getNodeField(tInputName);
                Real* tInputField;
                tInfield->ExtractView(&tInputField);
                const int tLength = tInfield->MyLength();
                for(int j=0; j<tLength; ++j)
                {
                    fprintf(fp, "%lf\n", tInputField[j]);
                }
                fclose(fp);
            }
        }
    }
}

}
