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

#include "PlatoESPApp.hpp"
#include "Plato_Parser.hpp"
#include "Plato_Exceptions.hpp"
#include "Plato_SharedData.hpp"
#include "Plato_TimersTree.hpp"

void PlatoESPApp::importData(const std::string & aArgumentName, const Plato::SharedData & aImportData)
{
    this->importDataT(aArgumentName, aImportData);
}

void PlatoESPApp::exportData(const std::string & aArgumentName, Plato::SharedData & aExportData)
{
    this->exportDataT(aArgumentName, aExportData);
}

void PlatoESPApp::exportDataMap(const Plato::data::layout_t & aDataLayout, std::vector<int> & aMyOwnedGlobalIDs)
{
    aMyOwnedGlobalIDs.clear();
}

void PlatoESPApp::initialize()
{
    // conditionally begin timers
    if(mAppfileData.size<Plato::InputData>("Timers"))
    {
        auto tTimersNode = mAppfileData.get<Plato::InputData>("Timers");
        if(tTimersNode.size<std::string>("time") > 0)
        {
            const bool do_time = Plato::Get::Bool(tTimersNode, "time");
            if(do_time)
            {
                mTimersTree = new Plato::TimersTree(mLocalComm);
            }
        }
    }

    if(!mAppfileData.empty())
    {
    }
}

void PlatoESPApp::compute(const std::string & aOperationName)
{
    if(hasChanged())
    {
        mESP.reset( new ESPType(mModelFileName, mTessFileName, mParameterIndex) );
    }
    mLocalData = mESP->getSensitivities()[0];
}

bool PlatoESPApp::hasChanged()
{
    if( mPrevParameters.size() == 0 )
    {
        mPrevParameters = mParameters;
        return true;
    }

    if( mPrevParameters == mParameters )
    {
        return false;
    }
    else
    {
        mPrevParameters = mParameters;
        return true;
    }
}

void PlatoESPApp::finalize()
{
}

PlatoESPApp::~PlatoESPApp()
{
    if(mTimersTree)
    {
        mTimersTree->print_results();
        delete mTimersTree;
        mTimersTree = nullptr;
    }
}

const MPI_Comm& PlatoESPApp::getComm() const
{
    return mLocalComm;
}

Plato::TimersTree* PlatoESPApp::getTimersTree()
{
    return mTimersTree;
}

PlatoESPApp::PlatoESPApp(int aArgc, char **aArgv, MPI_Comm& aLocalComm) :
        mLocalComm(aLocalComm),
        mAppfileData("Appfile Data"),
        mInputfileData("Inputfile Data"),
        mTimersTree(nullptr)
{
    const char* input_char = getenv("PLATO_APP_FILE");
    Plato::Parser* parser = new Plato::PugiParser();
    mAppfileData = parser->parseFile(input_char);

    std::string tInputfile;
    if(aArgc == 2)
    {
        tInputfile = aArgv[1];
    }
    else
    {
        tInputfile = "platoesp.xml";
    }
    mInputfileData = parser->parseFile(tInputfile.c_str());

    auto tESPNode = mInputfileData.get<Plato::InputData>("ESP");
    mModelFileName = Plato::Get::String(tESPNode, "ModelFileName");
    mTessFileName  = Plato::Get::String(tESPNode, "TessFileName");

    if (parser)
    {
        delete parser;
        parser = nullptr;
    }
}
