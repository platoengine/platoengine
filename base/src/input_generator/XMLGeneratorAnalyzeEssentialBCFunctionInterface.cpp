/*
 * XMLGeneratorAnalyzeEssentialBCFunctionInterface.cpp
 *
 *  Created on: Jun 15, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorAnalyzeEssentialBCFunctionInterface.hpp"

namespace XMLGen
{

namespace Private
{

std::string check_essential_boundary_condition_application_name_keyword
(const XMLGen::EssentialBoundaryCondition& aBC)
{
    if(aBC.value("location_name").empty())
    {
        THROWERR(std::string("Check Essential Boundary Condition Application Set Name: ")
            + "Application set name, e.g. sideset or nodeset, for Essential Boundary Condition "
            + "with identification number '" + aBC.id() + "' is empty.")
    }
    return aBC.value("location_name");
}
// function check_essential_boundary_condition_application_name_keyword

void check_essential_boundary_condition_value_keyword
(const XMLGen::EssentialBoundaryCondition& aBC)
{
    if(aBC.value("value").empty())
    {
        THROWERR(std::string("Check Essential Boundary Condition Value: Value parameter for Essential Boundary Condition ")
            + "with identification number '" + aBC.id() + "' is empty.")
    }
}
// function check_essential_boundary_condition_value_keyword

void append_rigid_essential_boundary_condition_to_plato_problem
(const std::string& aName,
 const std::string& aPhysics,
 const XMLGen::EssentialBoundaryCondition& aBC,
 pugi::xml_node &aParentNode)
{
    XMLGen::ValidDofsKeys tValidDofs;
    auto tDofNames = tValidDofs.names(aPhysics);
    std::vector<std::string> tKeys = {"name", "type", "value"};
    auto tSetName = XMLGen::Private::check_essential_boundary_condition_application_name_keyword(aBC);
    for(auto& tDofName : tDofNames)
    {
        auto tBCName = aName + " applied to Dof with tag " + Plato::toupper(tDofName);
        auto tEssentialBoundaryCondParentNode = aParentNode.append_child("ParameterList");
        XMLGen::append_attributes({"name"}, {tBCName}, tEssentialBoundaryCondParentNode);
        std::vector<std::string> tValues = {"Type", "string", "Zero Value"};
        XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);
        auto tDof = tValidDofs.dof(aPhysics, tDofName);
        tValues = {"Index", "int", tDof};
        XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);
        tValues = {"Sides", "string", tSetName};
        XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);
    }
}
// function append_rigid_essential_boundary_condition_to_plato_problem

void append_zero_value_essential_boundary_condition_to_plato_problem
(const std::string& aName,
 const std::string& aPhysics,
 const XMLGen::EssentialBoundaryCondition& aBC,
 pugi::xml_node &aParentNode)
{
    auto tBCName = aName + " applied to Dof with tag " + Plato::toupper(aBC.value("degree_of_freedom"));
    auto tEssentialBoundaryCondParentNode = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {tBCName}, tEssentialBoundaryCondParentNode);

    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Type", "string", "Zero Value"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);

    XMLGen::ValidDofsKeys tValidDofs;
    auto tDofInteger = tValidDofs.dof(aPhysics, aBC.value("degree_of_freedom"));
    tValues = {"Index", "int", tDofInteger};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);

    auto tSetName = XMLGen::Private::check_essential_boundary_condition_application_name_keyword(aBC);
    tValues = {"Sides", "string", tSetName};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);
}
// function append_zero_value_essential_boundary_condition_to_plato_problem

void append_fixed_value_essential_boundary_condition_to_plato_problem
(const std::string& aName,
 const std::string& aPhysics,
 const XMLGen::EssentialBoundaryCondition& aBC,
 pugi::xml_node &aParentNode)
{
    auto tBCName = aName + " applied to Dof with tag " + Plato::toupper(aBC.value("degree_of_freedom"));
    auto tEssentialBoundaryCondParentNode = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {tBCName}, tEssentialBoundaryCondParentNode);

    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Type", "string", "Fixed Value"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);

    XMLGen::ValidDofsKeys tValidDofs;
    auto tDofInteger = tValidDofs.dof(aPhysics, aBC.value("degree_of_freedom"));
    tValues = {"Index", "int", tDofInteger};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);

    auto tSetName = XMLGen::Private::check_essential_boundary_condition_application_name_keyword(aBC);
    tValues = {"Sides", "string", tSetName};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);

    XMLGen::Private::check_essential_boundary_condition_value_keyword(aBC);
    tValues = {"Value", "double", aBC.value("value")};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);
}
// function append_fixed_value_essential_boundary_condition_to_plato_problem

void append_time_function_essential_boundary_condition_to_plato_problem
(const std::string& aName,
 const std::string& aPhysics,
 const XMLGen::EssentialBoundaryCondition& aBC,
 pugi::xml_node &aParentNode)
{
    auto tBCName = aName + " applied to Dof with tag " + Plato::toupper(aBC.value("degree_of_freedom"));
    auto tEssentialBoundaryCondParentNode = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {tBCName}, tEssentialBoundaryCondParentNode);

    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Type", "string", "Time Dependent"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);

    XMLGen::ValidDofsKeys tValidDofs;
    auto tDofInteger = tValidDofs.dof(aPhysics, aBC.value("degree_of_freedom"));
    tValues = {"Index", "int", tDofInteger};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);

    auto tSetName = XMLGen::Private::check_essential_boundary_condition_application_name_keyword(aBC);
    tValues = {"Sides", "string", tSetName};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);

    XMLGen::Private::check_essential_boundary_condition_value_keyword(aBC);
    tValues = {"Function", "string", aBC.value("value")};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);
}
// function append_fixed_value_essential_boundary_condition_to_plato_problem

}
// namespace Private

}
// namespace XMLGen

namespace XMLGen
{

void AppendEssentialBoundaryCondition::insert()
{
    // rigid
    auto tFuncIndex = std::type_index(typeid(XMLGen::Private::append_rigid_essential_boundary_condition_to_plato_problem));
    mMap.insert(std::make_pair("rigid",
      std::make_pair((XMLGen::Analyze::EssentialBCFunc)XMLGen::Private::append_rigid_essential_boundary_condition_to_plato_problem, tFuncIndex)));

    // fixed = rigid
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_rigid_essential_boundary_condition_to_plato_problem));
    mMap.insert(std::make_pair("fixed",
      std::make_pair((XMLGen::Analyze::EssentialBCFunc)XMLGen::Private::append_rigid_essential_boundary_condition_to_plato_problem, tFuncIndex)));
    // zero value
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_zero_value_essential_boundary_condition_to_plato_problem));
    mMap.insert(std::make_pair("zero_value",
      std::make_pair((XMLGen::Analyze::EssentialBCFunc)XMLGen::Private::append_zero_value_essential_boundary_condition_to_plato_problem, tFuncIndex)));

    // fixed_value
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_fixed_value_essential_boundary_condition_to_plato_problem));
    mMap.insert(std::make_pair("fixed_value",
      std::make_pair((XMLGen::Analyze::EssentialBCFunc)XMLGen::Private::append_fixed_value_essential_boundary_condition_to_plato_problem, tFuncIndex)));

    // insulated
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_zero_value_essential_boundary_condition_to_plato_problem));
    mMap.insert(std::make_pair("insulated",
      std::make_pair((XMLGen::Analyze::EssentialBCFunc)XMLGen::Private::append_zero_value_essential_boundary_condition_to_plato_problem, tFuncIndex)));
    
    // time_function
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_time_function_essential_boundary_condition_to_plato_problem));
    mMap.insert(std::make_pair("time_function",
      std::make_pair((XMLGen::Analyze::EssentialBCFunc)XMLGen::Private::append_time_function_essential_boundary_condition_to_plato_problem, tFuncIndex)));
}

void AppendEssentialBoundaryCondition::call
(const std::string& aName,
 const std::string& aPhysics,
 const XMLGen::EssentialBoundaryCondition& aBC,
 pugi::xml_node& aParentNode) const
{
    auto tCategory = Plato::tolower(aBC.value("type"));
    auto tMapItr = mMap.find(tCategory);
    if(tMapItr == mMap.end())
    {
        THROWERR(std::string("Essential Boundary Condition Function Interface: Did not find essential boundary condition function with tag '")
            + tCategory + "' in list. Essential boundary condition '" + tCategory + "' is not supported in Plato Analyze.")
    }
    auto tTypeCastedFunc = reinterpret_cast<void(*)(const std::string&, const std::string&, const XMLGen::EssentialBoundaryCondition&, pugi::xml_node&)>(tMapItr->second.first);
    if(tMapItr->second.second == std::type_index(typeid(tTypeCastedFunc)))
    {
        THROWERR(std::string("Essential Boundary Condition Interface: Reinterpret cast for essential boundary condition function with tag '")
            + tCategory + "' failed.")
    }
    tTypeCastedFunc(aName, aPhysics, aBC, aParentNode);
}

}
// namespace XMLGen
