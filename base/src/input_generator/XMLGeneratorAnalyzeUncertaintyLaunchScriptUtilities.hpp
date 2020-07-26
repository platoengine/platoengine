#pragma once

#include "pugixml.hpp"

#include "XMLGeneratorDataStruct.hpp"

namespace XMLGen
{
    void generate_launch_script(const XMLGen::InputData& aInputData);
    void generate_summit_launch_scripts(const XMLGen::InputData& aInputData);
    void generate_analyze_bash_script();
    void append_analyze_mpirun_lines(const XMLGen::InputData& aInputData, FILE*& fp);
    void generate_mpirun_launch_script(const XMLGen::InputData& aInputData);

    void append_analyze_mpirun_commands_robust_optimization_problems
    (const XMLGen::InputData& aInputData,
     FILE*& aFile);
}
