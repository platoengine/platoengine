/*
 * XMLGeneratorAnalyzeLoadFunctionInterface.cpp
 *
 *  Created on: Jun 15, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeInputFileUtilities.hpp"
#include "XMLGeneratorAnalyzeLoadFunctionInterface.hpp"

namespace XMLGen
{

namespace Private
{

std::string return_load_vector_values
(const XMLGen::Load& aLoad)
{
    std::vector<std::string> tTokens;
    for(auto& tToken : aLoad.load_values())
    {
        tTokens.push_back(tToken);
    }
    auto tOutput = XMLGen::transform_tokens_for_plato_analyze_input_deck(tTokens);
    return tOutput;
}
// function return_load_vector_values

void append_uniform_vector_valued_load_to_plato_problem
(const std::string& aName,
 const XMLGen::Load& aLoad,
 pugi::xml_node &aParentNode)
{
    auto tVecValuedLoad = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {aName}, tVecValuedLoad);
    std::vector<std::string> tKeys = {"name", "type","value"};
    std::vector<std::string> tValues = {"Type", "string", "Uniform"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tVecValuedLoad);

    auto tVecValues = XMLGen::Private::return_load_vector_values(aLoad);
    if (aLoad.is_random())
        tValues = {"Values", "Array(double)", tVecValues};
    else
        tValues = {"Values", "Array(string)", tVecValues};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tVecValuedLoad);
    tValues = {"Sides", "string", aLoad.location_name()};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tVecValuedLoad);
}
// function append_uniform_vector_valued_load_to_plato_problem

void append_uniform_single_valued_load_to_plato_problem
(const std::string& aName,
 const XMLGen::Load& aLoad,
 pugi::xml_node& aParentNode)
{
    auto tUniformSingleValuedLoad = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {aName}, tUniformSingleValuedLoad);
    std::vector<std::string> tKeys = {"name", "type","value"};
    std::vector<std::string> tValues = {"Type", "string", "Uniform"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tUniformSingleValuedLoad);

    if(aLoad.load_values().size() > 1u)
    {
        THROWERR(std::string("Single valued load functions are expected to have one value. However, uniform load with name '" )
            + aName + "' has '" + std::to_string(aLoad.load_values().size()) + "' load values instead of one value. "
            + "Check load values for uniform singled valued load with id '" + aLoad.id() + "' applied at location with name '"
            + aLoad.location_name() +"'.")
    }
    if (aLoad.is_random())
        tValues = {"Value", "double", aLoad.load_values()[0]};
    else
        tValues = {"Value", "string", aLoad.load_values()[0]};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tUniformSingleValuedLoad);
    tValues = {"Sides", "string", aLoad.location_name()};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tUniformSingleValuedLoad);
}
// function append_uniform_single_valued_load_to_plato_problem

void append_uniform_source_load_to_plato_problem
(const std::string& aName,
 const XMLGen::Load& aLoad,
 pugi::xml_node& aParentNode)
{
    auto tUniformSource = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {aName}, tUniformSource);
    std::vector<std::string> tKeys = {"name", "type","value"};
    std::vector<std::string> tValues = {"Type", "string", "Uniform"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tUniformSource);
    if(aLoad.load_values().size() > 1u)
    {
        THROWERR(std::string("Uniform Source Loads are expected to have one value. However, uniform load with name '")
            + aName + "' has '" + std::to_string(aLoad.load_values().size()) + "' load values instead of one value. "
            + "Check load values for uniform source load with id '" + aLoad.id() + "' applied at location with name '"
            + aLoad.location_name() +"'.")
    }

    if (aLoad.is_random())
        { tValues = {"Value", "double", aLoad.load_values()[0]}; }
    else
        { tValues = {"Value", "string", aLoad.load_values()[0]}; }
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tUniformSource);

    tValues = {"Element Block", "string", aLoad.location_name()};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tUniformSource);
}
// function append_uniform_source_load_to_plato_problem

}
// namespace Private

}
// namespace XMLGen

namespace XMLGen
{

void AppendLoad::insert()
{
    // traction load
    auto tFuncIndex = std::type_index(typeid(XMLGen::Private::append_uniform_vector_valued_load_to_plato_problem));
    mMap.insert(std::make_pair("traction",
      std::make_pair((XMLGen::Analyze::LoadFunc)XMLGen::Private::append_uniform_vector_valued_load_to_plato_problem, tFuncIndex)));

    // uniform pressure load
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_uniform_single_valued_load_to_plato_problem));
    mMap.insert(std::make_pair("pressure",
      std::make_pair((XMLGen::Analyze::LoadFunc)XMLGen::Private::append_uniform_single_valued_load_to_plato_problem, tFuncIndex)));

    // uniform surface potential
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_uniform_single_valued_load_to_plato_problem));
    mMap.insert(std::make_pair("uniform_surface_potential",
      std::make_pair((XMLGen::Analyze::LoadFunc)XMLGen::Private::append_uniform_single_valued_load_to_plato_problem, tFuncIndex)));

    // uniform surface flux
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_uniform_single_valued_load_to_plato_problem));
    mMap.insert(std::make_pair("uniform_surface_flux",
      std::make_pair((XMLGen::Analyze::LoadFunc)XMLGen::Private::append_uniform_single_valued_load_to_plato_problem, tFuncIndex)));

    // uniform source
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_uniform_source_load_to_plato_problem));
    mMap.insert(std::make_pair("uniform_source",
      std::make_pair((XMLGen::Analyze::LoadFunc)XMLGen::Private::append_uniform_source_load_to_plato_problem, tFuncIndex)));
}

void AppendLoad::call
(const std::string& aName,
 const XMLGen::Load& aLoad,
 pugi::xml_node& aParentNode) const
{
    auto tCategory = Plato::tolower(aLoad.type());
    auto tMapItr = mMap.find(tCategory);
    if(tMapItr == mMap.end())
    {
        THROWERR(std::string("Load Function Interface: Did not find load function with tag '") + tCategory + "' in load function list.")
    }
    auto tTypeCastedFunc = reinterpret_cast<void(*)(const std::string&, const XMLGen::Load&, pugi::xml_node&)>(tMapItr->second.first);
    if(tMapItr->second.second == std::type_index(typeid(tTypeCastedFunc)))
    {
        THROWERR(std::string("Load Interface: Reinterpret cast for load function with tag '") + tCategory + "' failed.")
    }
    tTypeCastedFunc(aName, aLoad, aParentNode);
}

}
// namespace XMLGen
