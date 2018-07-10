#include "PlatoEngine_FilterFactory.hpp"

#include "PlatoEngine_AbstractFilter.hpp"
#include "PlatoEngine_IdentityFilter.hpp"
#include "PlatoEngine_KernelFilter.hpp"
#include "Plato_Interface.hpp"
#include "data_mesh.hpp"
#include "Plato_Parser.hpp"

#include <memory>
#include <string>
#include <cstdlib>
#include <iostream>

namespace Plato
{

Plato::AbstractFilter* build_filter(InputData aInputData, MPI_Comm& aLocalComm, DataMesh* aMesh)
{
    Plato::AbstractFilter* tResult = nullptr;

    if( aInputData.size<Plato::InputData>("Filter") == 0 )
    {
        // no filter
        tResult = new Plato::IdentityFilter();
    }
    else
    {
        auto tFilterNode = aInputData.get<Plato::InputData>("Filter");
        std::string tNameString = Plato::Get::String(tFilterNode, "Name");
        if(tNameString == "Kernel")
        {
            tResult = new Plato::KernelFilter();
        }
        else if(tNameString == "Identity")
        {
            tResult = new Plato::IdentityFilter();
        }
        else
        {
            std::cout << "build_filter could not build '" << tNameString << "'" << std::endl;
            std::abort();
        }
    }
    tResult->build(aInputData, aLocalComm, aMesh);
    return tResult;
}

}
