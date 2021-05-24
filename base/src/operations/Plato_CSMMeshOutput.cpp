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
 * Plato_CSMMeshOutput.cpp
 *
 *  Created on: Nov 19, 2019
 */

#include <string>
#include <cstdio>
#include <cstdlib>

#include "PlatoApp.hpp"
#include "Plato_CSMMeshOutput.hpp"
#include "Plato_OperationsUtilities.hpp"

namespace Plato
{

CSMMeshOutput::CSMMeshOutput(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
        Plato::LocalOp(aPlatoApp)
{
    mBaseMeshName = Plato::Get::String(aNode, "BaseMeshName");
    mOutputFrequency = 5;
    if(aNode.size<std::string>("OutputFrequency"))
        mOutputFrequency = Plato::Get::Int(aNode, "OutputFrequency");
    mMaxIterations = 100;
    if(aNode.size<std::string>("MaxIterations"))
        mMaxIterations = Plato::Get::Int(aNode, "MaxIterations");
    mCurIteration = 1;
}

CSMMeshOutput::~CSMMeshOutput()
{
}

void CSMMeshOutput::getArguments(std::vector<Plato::LocalArg>& aLocalArgs)
{
    aLocalArgs.clear();
}

void CSMMeshOutput::buildIterationNumberString(std::string &aString)
{
    aString = "";
    std::string tMaxNumIterationsString = std::to_string(mMaxIterations);
    int tNumDigitsInMaxString = tMaxNumIterationsString.length();
    std::string tCurIterationString = std::to_string(mCurIteration);
    int tNumDigitsInCurString = tCurIterationString.length();
    int tNumZeros = tNumDigitsInMaxString - tNumDigitsInCurString;
    for(int i=0; i<tNumZeros; ++i)
        aString += "0";
    aString += tCurIterationString;
}

void CSMMeshOutput::operator()()
{
    int tMyRank = 0;
    MPI_Comm_rank(mPlatoApp->getComm(), &tMyRank);
    if(mOutputFrequency > 0 && mCurIteration % mOutputFrequency == 0)
    {
        if(tMyRank == 0)
        {
            std::string tIterationString;
            this->buildIterationNumberString(tIterationString);
            std::ostringstream tTheCommand;
            tTheCommand << "cp " << this->mBaseMeshName << " Iteration" << tIterationString << ".exo";
         //   std::cout << "\nExecuting system call: " << tTheCommand.str() << "\n";
            Plato::system(tTheCommand.str().c_str());

            FILE *fp = fopen("last_time_step.txt", "w");
            if(fp)
            {
                fprintf(fp, "%s\n", tIterationString.c_str());
                fclose(fp);
                Plato::system("ls Iteration*.exo >> last_time_step.txt");
            }
        }
    }

    ++mCurIteration;
}

}
