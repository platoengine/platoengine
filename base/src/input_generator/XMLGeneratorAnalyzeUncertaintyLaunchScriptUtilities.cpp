#include "pugixml.hpp"

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorLaunchScriptUtilities.hpp"
#include "XMLGeneratorAnalyzeUncertaintyLaunchScriptUtilities.hpp"
#include <iostream>
#include <fstream>

namespace XMLGen
{
inline bool is_robust_optimization_problem
(const XMLGen::InputData& aMetaData)
{
    if(aMetaData.mRandomMetaData.empty())
    {
        return false;
    }
    else
    {
        return true;
    }
}

inline void append_plato_analyze_code_path
(const XMLGen::InputData& aInputData,
 FILE*& aFile,
 const std::string& aServiceID,
 const std::string& aDeviceID)
{
    // Build id string based on the performer id.
    std::string tIDString;
    if(aServiceID != "")
    {
        tIDString = "_" + aServiceID + "_";
    }
    else
    {
        tIDString = "_";
    }

    // Add executable name.
    if(aInputData.codepaths.plato_analyze_path.length() != 0)
    {
        fprintf(aFile, "%s ", aInputData.codepaths.plato_analyze_path.c_str());
    }
    else
    {
        fprintf(aFile, "analyze_MPMD ");
    }

    // Add kokkos-device setting if requested.
    if(aDeviceID != "")
    {
        fprintf(aFile, "--kokkos-device=%s ", aDeviceID.c_str());
    }

    // Add the input deck syntax.
    fprintf(aFile, "--input-config=plato_analyze%sinput_deck.xml \\\n", tIDString.c_str());
}

inline void append_analyze_mpirun_commands
(const XMLGen::InputData& aInputData,
 FILE*& aFile)
{
    std::string tEnvString, tSeparationString, tLaunchString, tNumProcsString;
    XMLGen::determine_mpi_env_and_separation_strings(tEnvString, tSeparationString);
    XMLGen::determine_mpi_launch_strings(aInputData, tLaunchString, tNumProcsString);

    int tServiceIndex = 1;
    for(auto &tService : aInputData.mPerformerServices)
    {
        if(tService.code() == "plato_analyze")
        {
            fprintf(aFile,
                ": %s %s %s PLATO_PERFORMER_ID%s%d \\\n",
                tNumProcsString.c_str(),
                tService.numberProcessors().c_str(),
                tEnvString.c_str(),
                tSeparationString.c_str(),
                tServiceIndex);

            fprintf(aFile, "%s PLATO_INTERFACE_FILE%sinterface.xml \\\n", tEnvString.c_str(), tSeparationString.c_str());
            fprintf(aFile, "%s PLATO_APP_FILE%splato_analyze_%s_operations.xml \\\n", tEnvString.c_str(), tSeparationString.c_str(), tService.id().c_str());
            XMLGen::append_plato_analyze_code_path(aInputData, aFile, tService.id(), "");
        }
        tServiceIndex++;
    }
}

void append_analyze_mpirun_commands_robust_optimization_problems
(const XMLGen::InputData& aInputData,
 FILE*& aFile)
{
    std::string tEnvString, tSeparationString, tLaunchString, tNumProcsString;
    XMLGen::determine_mpi_env_and_separation_strings(tEnvString, tSeparationString);
    XMLGen::determine_mpi_launch_strings(aInputData, tLaunchString, tNumProcsString);

    if(aInputData.m_UncertaintyMetaData.numPerformers <= 0)
    {
        THROWERR("Number of performers for uncertainty workflow must be greater than zero\n")
    }

    XMLGen::Service tService = aInputData.mPerformerServices[0];
    std::vector<std::string> tDeviceIDs = tService.deviceIDs();

    // If no device ids were specified just put all of the performers in one executable 
    // statement in the mpirun.source file
    if(tDeviceIDs.size() == 0)
    {
        fprintf(aFile,
            ": %s %s %s PLATO_PERFORMER_ID%s1 \\\n",
            tNumProcsString.c_str(),
            Plato::to_string(aInputData.m_UncertaintyMetaData.numPerformers).c_str(),
            tEnvString.c_str(),
            tSeparationString.c_str());
        if(aInputData.mPerformerServices.size() == 0)
        {
            THROWERR("Number of services must be greater than zero\n")
        }

        fprintf(aFile, "%s PLATO_INTERFACE_FILE%sinterface.xml \\\n", tEnvString.c_str(), tSeparationString.c_str());
        fprintf(aFile, "%s PLATO_APP_FILE%splato_analyze_%s_operations.xml \\\n", tEnvString.c_str(), tSeparationString.c_str(), tService.id().c_str());
        XMLGen::append_plato_analyze_code_path(aInputData, aFile, tService.id(), "");
    }
    // Id device ids were specified spread the performers out on the devices by putting them in 
    // separate executable statements and specifying the kokkos-device id.
    else
    {
        size_t tMinNumPerformersPerDevice = aInputData.m_UncertaintyMetaData.numPerformers/tDeviceIDs.size();
        size_t tRemainder = aInputData.m_UncertaintyMetaData.numPerformers%tDeviceIDs.size();
        for(size_t i=0; i<tDeviceIDs.size(); ++i)
        {
            int tNumPerformersOnThisDevice = tMinNumPerformersPerDevice;
            // On the last device add the remainder performers
            if(i == (tDeviceIDs.size()-1))
            {
                tNumPerformersOnThisDevice += tRemainder;
            }
            fprintf(aFile,
                ": %s %d %s PLATO_PERFORMER_ID%s1 \\\n",
                tNumProcsString.c_str(),
                tNumPerformersOnThisDevice,
                tEnvString.c_str(),
                tSeparationString.c_str());
            fprintf(aFile, "%s PLATO_INTERFACE_FILE%sinterface.xml \\\n", tEnvString.c_str(), tSeparationString.c_str());
            fprintf(aFile, "%s PLATO_APP_FILE%splato_analyze_%s_operations.xml \\\n", tEnvString.c_str(), tSeparationString.c_str(), tService.id().c_str());
            XMLGen::append_plato_analyze_code_path(aInputData, aFile, tService.id(), tDeviceIDs[i]);
        }
    }
}

void generate_launch_script(const XMLGen::InputData& aInputData)
{
    if(aInputData.m_Arch == XMLGen::Arch::SUMMIT)
        XMLGen::generate_summit_launch_scripts(aInputData);
    else
        XMLGen::generate_mpirun_launch_script(aInputData);
}

void generate_mpirun_launch_script(const XMLGen::InputData& aInputData)
{
    FILE *fp = fopen("mpirun.source", "w");

    XMLGen::append_prune_and_refine_lines_to_mpirun_launch_script(aInputData, fp);
    XMLGen::append_decomp_lines_to_mpirun_launch_script(aInputData, fp);
    XMLGen::append_engine_mpirun_lines(aInputData, fp);
    XMLGen::append_analyze_mpirun_lines(aInputData, fp);

    fclose(fp);
}

void generate_summit_launch_scripts(const XMLGen::InputData& aInputData)
{
    for(auto &tServiceID : aInputData.objective.serviceIDs)
    {
        XMLGen::Service tService = aInputData.service(tServiceID);
        if(tService.code() != "plato_analyze")
            THROWERR("ERROR: Summit output is only supported for Plato Analyze performers")
    }

    size_t tNumPerformers = aInputData.m_UncertaintyMetaData.numPerformers;

    XMLGen::generate_engine_bash_script();
    generate_analyze_bash_script();
    XMLGen::generate_jsrun_script(tNumPerformers, "analyze");
    XMLGen::generate_batch_script(tNumPerformers);
}

void generate_analyze_bash_script()
{
    std::ofstream analyzeBash;
    std::string filename = "analyze.sh";
    analyzeBash.open(filename);
    analyzeBash << "export PLATO_PERFORMER_ID=1\n";
    analyzeBash << "export PLATO_INTERFACE_FILE=interface.xml\n";
    analyzeBash << "export PLATO_APP_FILE=plato_analyze_operations.xml\n";
    analyzeBash << "\n";
    analyzeBash << "analyze_MPMD --input-config=plato_analyze_input_deck.xml\n";

    analyzeBash.close();
}

void append_analyze_mpirun_lines(const XMLGen::InputData& aInputData, FILE*& aFile)
{
    if(XMLGen::is_robust_optimization_problem(aInputData))
    {
        XMLGen::append_analyze_mpirun_commands_robust_optimization_problems(aInputData, aFile);
    }
    else
    {
        XMLGen::append_analyze_mpirun_commands(aInputData, aFile);
    }
}

}
// namespace XMLGen
