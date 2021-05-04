/*
 * XMLGeneratorAnalyzeEssentialBCTagFunctionInterface.cpp
 *
 *  Created on: Jun 15, 2020
 */

#include "XMLGeneratorAnalyzeEssentialBCTagFunctionInterface.hpp"

namespace XMLGen
{

namespace Private
{

std::string return_displacement_bc_name
(const XMLGen::EssentialBoundaryCondition& aBC)
{
    std::string tOutput;
    if(aBC.value("is_random") == "true")
    {
        tOutput = std::string("Random Displacement Boundary Condition with ID ") + aBC.id();
    }
    else
    {
        tOutput = std::string("Displacement Boundary Condition with ID ") + aBC.id();
    }
    return tOutput;
}
// function return_displacement_bc_name

std::string return_potential_bc_name
(const XMLGen::EssentialBoundaryCondition& aBC)
{
    std::string tOutput;
    if(aBC.value("is_random") == "true")
    {
        tOutput = std::string("Random Potential Boundary Condition with ID ") + aBC.id();
    }
    else
    {
        tOutput = std::string("Potential Boundary Condition with ID ") + aBC.id();
    }
    return tOutput;
}
// function return_potential_bc_name

std::string return_velocity_bc_name
(const XMLGen::EssentialBoundaryCondition& aBC)
{
    std::string tOutput;
    if(aBC.value("is_random") == "true")
    {
        tOutput = std::string("Random Velocity Boundary Condition with ID ") + aBC.id();
    }
    else
    {
        tOutput = std::string("Velocity Boundary Condition with ID ") + aBC.id();
    }
    return tOutput;
}
// function return_velocity_bc_name

std::string return_pressure_bc_name
(const XMLGen::EssentialBoundaryCondition& aBC)
{
    std::string tOutput;
    if(aBC.value("is_random") == "true")
    {
        tOutput = std::string("Random Pressure Boundary Condition with ID ") + aBC.id();
    }
    else
    {
        tOutput = std::string("Pressure Boundary Condition with ID ") + aBC.id();
    }
    return tOutput;
}
// function return_pressure_bc_name

std::string return_temperature_bc_name
(const XMLGen::EssentialBoundaryCondition& aBC)
{
    std::string tOutput;
    if(aBC.value("is_random") == "true")
    {
        tOutput = std::string("Random Temperature Boundary Condition with ID ") + aBC.id();
    }
    else
    {
        tOutput = std::string("Temperature Boundary Condition with ID ") + aBC.id();
    }
    return tOutput;
}
// function return_temperature_bc_name

}
// namespace Private

}
// namespace XMLGen

namespace XMLGen
{

void EssentialBoundaryConditionTag::insert()
{
    // temperature
    auto tFuncIndex = std::type_index(typeid(XMLGen::Private::return_temperature_bc_name));
    mMap.insert(std::make_pair("temperature",
      std::make_pair((XMLGen::Analyze::EssentialBCTagFunc)XMLGen::Private::return_temperature_bc_name, tFuncIndex)));
    tFuncIndex = std::type_index(typeid(XMLGen::Private::return_temperature_bc_name));
    mMap.insert(std::make_pair("temp",
      std::make_pair((XMLGen::Analyze::EssentialBCTagFunc)XMLGen::Private::return_temperature_bc_name, tFuncIndex)));

    // pressure
    tFuncIndex = std::type_index(typeid(XMLGen::Private::return_pressure_bc_name));
    mMap.insert(std::make_pair("pressure",
      std::make_pair((XMLGen::Analyze::EssentialBCTagFunc)XMLGen::Private::return_pressure_bc_name, tFuncIndex)));
    mMap.insert(std::make_pair("press",
      std::make_pair((XMLGen::Analyze::EssentialBCTagFunc)XMLGen::Private::return_pressure_bc_name, tFuncIndex)));

    // velocity
    tFuncIndex = std::type_index(typeid(XMLGen::Private::return_velocity_bc_name));
    mMap.insert(std::make_pair("velocity",
      std::make_pair((XMLGen::Analyze::EssentialBCTagFunc)XMLGen::Private::return_velocity_bc_name, tFuncIndex)));
    mMap.insert(std::make_pair("velx",
      std::make_pair((XMLGen::Analyze::EssentialBCTagFunc)XMLGen::Private::return_velocity_bc_name, tFuncIndex)));
    mMap.insert(std::make_pair("vely",
      std::make_pair((XMLGen::Analyze::EssentialBCTagFunc)XMLGen::Private::return_velocity_bc_name, tFuncIndex)));
    mMap.insert(std::make_pair("velz",
      std::make_pair((XMLGen::Analyze::EssentialBCTagFunc)XMLGen::Private::return_velocity_bc_name, tFuncIndex)));

    // potential
    tFuncIndex = std::type_index(typeid(XMLGen::Private::return_potential_bc_name));
    mMap.insert(std::make_pair("potential",
      std::make_pair((XMLGen::Analyze::EssentialBCTagFunc)XMLGen::Private::return_potential_bc_name, tFuncIndex)));

    // displacement
    tFuncIndex = std::type_index(typeid(XMLGen::Private::return_displacement_bc_name));
    mMap.insert(std::make_pair("displacement",
      std::make_pair((XMLGen::Analyze::EssentialBCTagFunc)XMLGen::Private::return_displacement_bc_name, tFuncIndex)));
    tFuncIndex = std::type_index(typeid(XMLGen::Private::return_displacement_bc_name));
    mMap.insert(std::make_pair("dispx",
      std::make_pair((XMLGen::Analyze::EssentialBCTagFunc)XMLGen::Private::return_displacement_bc_name, tFuncIndex)));
    tFuncIndex = std::type_index(typeid(XMLGen::Private::return_displacement_bc_name));
    mMap.insert(std::make_pair("dispy",
      std::make_pair((XMLGen::Analyze::EssentialBCTagFunc)XMLGen::Private::return_displacement_bc_name, tFuncIndex)));
    tFuncIndex = std::type_index(typeid(XMLGen::Private::return_displacement_bc_name));
    mMap.insert(std::make_pair("dispz",
      std::make_pair((XMLGen::Analyze::EssentialBCTagFunc)XMLGen::Private::return_displacement_bc_name, tFuncIndex)));
}

std::string EssentialBoundaryConditionTag::call(const XMLGen::EssentialBoundaryCondition& aBC) const
{
    auto tTag = Plato::tolower(aBC.value("degree_of_freedom"));
    // This was originally implemented to key off of the tags "displacment", "temperature", etc. 
    // but since we no longer need these specified because of how the dofs are encoded with the type 
    // I am just using the dof value instead.
    //auto tTag = Plato::tolower(aBC.value("type"));
    auto tMapItr = mMap.find(tTag);
    if(tMapItr == mMap.end())
    {
        THROWERR(std::string("Essential Boundary Condition Tag Function Interface: Did not find essential ")
            + "boundary condition function with tag '" + tTag + "' in essential boundary condition function list.")
    }
    auto tTypeCastedFunc = reinterpret_cast<std::string(*)(const XMLGen::EssentialBoundaryCondition&)>(tMapItr->second.first);
    if(tMapItr->second.second == std::type_index(typeid(tTypeCastedFunc)))
    {
        THROWERR(std::string("Essential Boundary Condition Tag Function Interface: Reinterpret cast for essential ")
            + "boundary condition function with tag '" + tTag + "' failed.")
    }
    auto tName = tTypeCastedFunc(aBC);
    return tName;
}

}
// namespace XMLGen
