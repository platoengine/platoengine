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
 * Plato_PlatoMainOutput.cpp
 *
 *  Created on: Jun 27, 2019
 */

#include <string>
#include <cstdio>
#include <cstdlib>

#ifdef ENABLE_ISO
#include "STKExtract.hpp"
#endif

#include "lightmp.hpp"

#include "PlatoApp.hpp"
#include "Plato_PlatoMainOutput.hpp"
#include "Plato_OperationsUtilities.hpp"
#include <Plato_FreeFunctions.hpp>

namespace Plato
{

PlatoMainOutput::PlatoMainOutput(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
        Plato::LocalOp(aPlatoApp)
{
    bool tPlotTable = true;
    for(auto tInputNode : aNode.getByName<Plato::InputData>("Input"))
    {
        std::string tName = Plato::Get::String(tInputNode, "ArgumentName");
        auto tInputLayout = Plato::getLayout(tInputNode, /*default=*/Plato::data::layout_t::SCALAR_FIELD);
        mOutputData.push_back(Plato::LocalArg {tInputLayout, tName, 0, tPlotTable});
    }

    // configure iso surface output
    //
    mMaxIterations = 100;
    mOutputFrequency = 5;
    mOutputMethod = 2;
    mWriteRestart = Plato::Get::Bool(aNode, "WriteRestart");
    Plato::InputData tSurfaceExtractionNode = Plato::Get::InputData(aNode, "SurfaceExtraction");
    mRestartFieldName = "control";
    if(aNode.size<std::string>("RestartFieldName"))
        mRestartFieldName = Plato::Get::String(aNode, "RestartFieldName");
    if(aNode.size<std::string>("OutputFrequency"))
        mOutputFrequency = Plato::Get::Int(aNode, "OutputFrequency");
    if(aNode.size<std::string>("MaxIterations"))
        mMaxIterations = Plato::Get::Int(aNode, "MaxIterations");
    if(tSurfaceExtractionNode.size<std::string>("OutputMethod"))
    {
        std::string tMethod = Plato::Get::String(tSurfaceExtractionNode, "OutputMethod");
        if(!tMethod.compare("epu"))
        {
            mOutputMethod = 2;
        }
        else if(!tMethod.compare("parallel_write"))
        {
            mOutputMethod = 1;
        }
        else
        {
            mOutputMethod = 2;
        }
    }

#ifdef ENABLE_ISO
    iso::STKExtract ex;
    auto tAvailableFormats = ex.availableFormats();
    for(auto tNode : tSurfaceExtractionNode.getByName<Plato::InputData>("Output"))
    {
        auto tFormat = Plato::Get::String(tNode, "Format", /*asUpperCase=*/true);
        if( std::count(tAvailableFormats.begin(), tAvailableFormats.end(), tFormat) )
        {
            mRequestedFormats.push_back(tFormat);
        }
    }
#endif

    mDiscretization = Plato::Get::String(tSurfaceExtractionNode, "Discretization");
    std::string tDefaultName("Iteration");
    mBaseName = Plato::Get::String(tSurfaceExtractionNode, "BaseName", tDefaultName);

    mAppendIterationCount = Plato::Get::Bool(tSurfaceExtractionNode, "AppendIterationCount", /*defaultValue=*/true);
}

PlatoMainOutput::~PlatoMainOutput()
{
}

void PlatoMainOutput::getArguments(std::vector<Plato::LocalArg>& aLocalArgs)
{
    aLocalArgs = mOutputData;
}

void PlatoMainOutput::buildIterationNumberString(const int &aCurIteration,
                                                 std::string &aString)
{
    aString = "";
    std::string tMaxNumIterationsString = std::to_string(mMaxIterations);
    int tNumDigitsInMaxString = tMaxNumIterationsString.length();
    std::string tCurIterationString = std::to_string(aCurIteration);
    int tNumDigitsInCurString = tCurIterationString.length();
    int tNumZeros = tNumDigitsInMaxString - tNumDigitsInCurString;
    for(int i=0; i<tNumZeros; ++i)
        aString += "0";
    aString += tCurIterationString;
}

void PlatoMainOutput::extractIsoSurface(int aIteration)
{
#ifdef ENABLE_ISO
    std::string output_filename = "";

    std::string tIterationNumberString = "";
    buildIterationNumberString(aIteration, tIterationNumberString);

    output_filename = mBaseName;
    if( mAppendIterationCount )
    {
        output_filename += tIterationNumberString;
    }
    output_filename += ".exo";
    iso::STKExtract ex;
    std::string input_filename = "platomain.exo";
    int num_procs = 0;
    MPI_Comm_size(mPlatoApp->getComm(), &num_procs);
    if(num_procs == 1)
    {
        input_filename += ".1.0";
    }

    std::string tOutputFields = "";
    for(size_t i=0; i<mOutputData.size(); ++i)
    {
        tOutputFields += mOutputData[i].mName;
        if(i < (mOutputData.size()-1))
        {
            tOutputFields += ",";
        }
    }
    if(ex.create_mesh_apis_read_from_file((stk::ParallelMachine*)(&(mPlatoApp->getComm())), // MPI_Comm
                    input_filename,// input filename
                    output_filename,// output filename
                    "Topology",// iso field name
                    tOutputFields,// names of fields to output
                    mRequestedFormats,// names of formats to write
                    1e-5,// min edge length
                    0.5,// iso value
                    0,// level_set data?
                    mOutputMethod,// epu results
                    1,// iso_only
                    1,// read spread file
                    aIteration))// time step/iteration
    {
        ex.run_extraction(aIteration, 1);
    }
    int my_rank = 0;
    MPI_Comm_rank(mPlatoApp->getComm(), &my_rank);
    if(my_rank == 0)
    {
        FILE *fp = fopen("last_time_step.txt", "w");
        if(fp)
        {
            fprintf(fp, "%s\n", tIterationNumberString.c_str());
            fclose(fp);
            Plato::system("ls Iteration*.exo >> last_time_step.txt");
        }
    }
#endif
}

void PlatoMainOutput::operator()()
{
    // time operation
    if(mPlatoApp->getTimersTree())
    {
        mPlatoApp->getTimersTree()->begin_partition(Plato::timer_partition_t::timer_partition_t::file_input_output);
    }

    LightMP* tLightMP = mPlatoApp->getLightMP();
    double tTime = tLightMP->getCurrentTime();
    tTime += 1.0;
    tLightMP->setCurrentTime(tTime);
    int tIntegerTime = (int)tTime;
    tLightMP->WriteOutput();
    int tMyRank = 0;
    MPI_Comm_rank(mPlatoApp->getComm(), &tMyRank);
    if(mOutputFrequency > 0 && tIntegerTime % mOutputFrequency == 0)
    {
        if(mDiscretization == "density")
        {
            this->extractIsoSurface(tIntegerTime);

            // Write restart file
            if((tMyRank == 0) && mWriteRestart)
            {
                std::ostringstream tTheCommand;
                std::string tInputFilename = "platomain.exo.1.0";
                int tNumProcs = 0;
                MPI_Comm_size(mPlatoApp->getComm(), &tNumProcs);
                if(tNumProcs > 1)
                {
                    tTheCommand << "epu -auto platomain.exo." << tNumProcs << ".0 > epu.txt;";
                    tInputFilename = "platomain.exo";
                }
                tTheCommand << "echo times " << tIntegerTime << " > commands.txt;";
                tTheCommand << "echo save " << mRestartFieldName << " >> commands.txt;";
                tTheCommand << "echo end >> commands.txt;";
                tTheCommand << "algebra " << tInputFilename << " restart_" << tIntegerTime << ".exo < commands.txt > algebra.txt";
                std::cout << "\nExecuting system call: " << tTheCommand.str() << "\n";
                Plato::system(tTheCommand.str().c_str());
            }
        }
        else if(mDiscretization == "levelset")
        {
            if((tMyRank == 0) && mWriteRestart)
            {
                std::string tListCommand = "ls -t IterationHistory* > junk.txt";
                Plato::system(tListCommand.c_str());
                FILE *tFile = fopen("junk.txt", "r");
                if(tFile)
                {
                    char tLastHistFileName[200] = " ";
                    fscanf(tFile, "%s", tLastHistFileName);
                    fclose(tFile);
                    std::string tNewFilename = "Iteration";
                    std::string tIterationString = "";
                    buildIterationNumberString(tIntegerTime, tIterationString);
                    tNewFilename += tIterationString;
                    tNewFilename += ".exo";
                    std::string tCopyCommand = "cp ";
                    tCopyCommand += tLastHistFileName;
                    tCopyCommand += " ";
                    tCopyCommand += tNewFilename;
                    Plato::system(tCopyCommand.c_str());
                    Plato::system("rm -f IterationHistory*");
                    tFile = fopen("last_time_step.txt", "w");
                    if(tFile)
                    {
                        fprintf(tFile, "%s\n", tIterationString.c_str());
                        fclose(tFile);
                        Plato::system("ls Iteration*.exo >> last_time_step.txt");
                    }
                }
            }
        }
    }

    // end I/O timer
    if(mPlatoApp->getTimersTree())
    {
        mPlatoApp->getTimersTree()->end_partition();
    }
}

}
