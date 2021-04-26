/*
 * XMLGeneratorAnalyzeNaturalBCTagFunctionInterface.cpp
 *
 *  Created on: Jun 15, 2020
 */

#include "XMLGeneratorPlatoAnalyzeInputFileUtilities.hpp"
#include "XMLGeneratorAnalyzeNaturalBCTagFunctionInterface.hpp"

namespace XMLGen
{

namespace Private
{

std::string return_traction_load_name
(const XMLGen::NaturalBoundaryCondition& aLoad)
{
    std::string tOutput;
    if(aLoad.is_random())
    {
        tOutput = std::string("Random Traction Vector Boundary Condition with ID ") + aLoad.id();
    }
    else
    {
        tOutput = std::string("Traction Vector Boundary Condition with ID ") + aLoad.id();
    }
    return tOutput;
}
// function return_traction_load_name

std::string return_pressure_load_name
(const XMLGen::NaturalBoundaryCondition& aLoad)
{
    std::string tOutput;
    if(aLoad.is_random())
    {
        tOutput = std::string("Random Uniform Pressure Boundary Condition with ID ") + aLoad.id();
    }
    else
    {
        tOutput = std::string("Uniform Pressure Boundary Condition with ID ") + aLoad.id();
    }
    return tOutput;
}
// function return_pressure_load_name

std::string return_surface_potential_load_name
(const XMLGen::NaturalBoundaryCondition& aLoad)
{
    std::string tOutput;
    if(aLoad.is_random())
    {
        tOutput = std::string("Random Uniform Surface Potential Boundary Condition with ID ") + aLoad.id();
    }
    else
    {
        tOutput = std::string("Uniform Surface Potential Boundary Condition with ID ") + aLoad.id();
    }
    return tOutput;
}
// function return_surface_potential_load_name

std::string return_surface_flux_load_name
(const XMLGen::NaturalBoundaryCondition& aLoad)
{
    std::string tOutput;
    if(aLoad.is_random())
    {
        tOutput = std::string("Random Uniform Surface Flux Boundary Condition with ID ") + aLoad.id();
    }
    else
    {
        tOutput = std::string("Uniform Surface Flux Boundary Condition with ID ") + aLoad.id();
    }
    return tOutput;
}
// function return_surface_flux_load_name

}
// namespace Private

}
// namespace XMLGen

namespace XMLGen
{

void NaturalBoundaryConditionTag::insert()
{
    // traction load
    auto tFuncIndex = std::type_index(typeid(XMLGen::Private::return_traction_load_name));
    mMap.insert(std::make_pair("traction",
      std::make_pair((XMLGen::Analyze::NaturalBCTagFunc)XMLGen::Private::return_traction_load_name, tFuncIndex)));

    // uniform pressure load
    tFuncIndex = std::type_index(typeid(XMLGen::Private::return_pressure_load_name));
    mMap.insert(std::make_pair("pressure",
      std::make_pair((XMLGen::Analyze::NaturalBCTagFunc)XMLGen::Private::return_pressure_load_name, tFuncIndex)));

    // uniform surface potential
    tFuncIndex = std::type_index(typeid(XMLGen::Private::return_surface_potential_load_name));
    mMap.insert(std::make_pair("uniform_surface_potential",
      std::make_pair((XMLGen::Analyze::NaturalBCTagFunc)XMLGen::Private::return_surface_potential_load_name, tFuncIndex)));

    // uniform surface flux
    tFuncIndex = std::type_index(typeid(XMLGen::Private::return_surface_flux_load_name));
    mMap.insert(std::make_pair("uniform_surface_flux",
      std::make_pair((XMLGen::Analyze::NaturalBCTagFunc)XMLGen::Private::return_surface_flux_load_name, tFuncIndex)));
}

std::string NaturalBoundaryConditionTag::call(const XMLGen::NaturalBoundaryCondition& aLoad) const
{
    auto tCategory = Plato::tolower(aLoad.type());
    auto tMapItr = mMap.find(tCategory);
    if(tMapItr == mMap.end())
    {
        THROWERR(std::string("Natural Boundary Condition Name Function Interface: Did not find natural boundary condition function with tag '")
            + tCategory + "' in list.")
    }
    auto tTypeCastedFunc = reinterpret_cast<std::string(*)(const XMLGen::NaturalBoundaryCondition&)>(tMapItr->second.first);
    if(tMapItr->second.second == std::type_index(typeid(tTypeCastedFunc)))
    {
        THROWERR(std::string("Natural Boundary Condition Name Function Interface: Reinterpret cast for natural boundary condition function with tag '")
            + tCategory + "' failed.")
    }
    auto tName = tTypeCastedFunc(aLoad);
    return tName;
}

}
// namespace XMLGen
