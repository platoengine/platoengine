#include <iostream>
#include "InputXMLChecker.hpp"

/******************************************************************************/
int main(int argc, char *argv[])
/******************************************************************************/
{
    int num_physics_apps = argc-3;
    if(num_physics_apps > 0)
    {
        std::string interface_file = argv[1];
        std::string plato_operations_file = argv[2];
        std::vector<std::string> physics_app_files;
        for(int i=0; i<num_physics_apps; ++i)
        {
            physics_app_files.push_back(argv[3+i]);
        }
        std::cout << "Running checker on the following input files:" << std::endl;
        std::cout << "    Interface file: " << interface_file << std::endl;
        std::cout << "    PlatoMain operations file: " << plato_operations_file << std::endl;
        for(int i=0; i<num_physics_apps; ++i)
        {
            std::cout << "    Physics app operations file: " << physics_app_files[i] << std::endl;
        }
        InputXMLChecker checker(interface_file, plato_operations_file, physics_app_files);
        checker.check();
    }
    else
    {
        std::cout << "Not enough input files were entered." << std::endl;
    }
}



