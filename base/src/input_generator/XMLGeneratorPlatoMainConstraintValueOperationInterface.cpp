/*
 * XMLGeneratorPlatoMainConstraintValueOperationInterface.cpp
 *
 *  Created on: Jun 28, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorPlatoMainConstraintValueOperationInterface.hpp"

namespace XMLGen
{

namespace Private
{

void append_compute_constraint_value_operation_platomain
(const XMLGen::Constraint& aConstraint,
 pugi::xml_node& aParentNode)
{
    auto tPerformer = aConstraint.performer();
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Compute Constraint Value", tPerformer}, tOperationNode);

    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Topology", "Topology"}, tInputNode);

    auto tSharedDataName = std::string("Constraint Value ID-") + aConstraint.name();
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Volume", tSharedDataName}, tOutputNode);

    tSharedDataName = std::string("Constraint Gradient ID-") + aConstraint.name();
    tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Volume Gradient", tSharedDataName}, tOutputNode);
}

void append_compute_constraint_value_operation_platoanalyze
(const XMLGen::Constraint& aConstraint,
 pugi::xml_node& aParentNode)
{
    auto tPerformer = aConstraint.performer();
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Compute Constraint Value", tPerformer}, tOperationNode);

    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Topology", "Topology"}, tInputNode);

    auto tSharedDataName = std::string("Constraint Value ID-") + aConstraint.name();
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Constraint Value", tSharedDataName}, tOutputNode);
}

}
// namespace Private

void ConstraintValueOperation::insert()
{
    // code = platomain
    auto tFuncIndex = std::type_index(typeid(XMLGen::Private::append_compute_constraint_value_operation_platomain));
    mMap.insert(std::make_pair("platomain",
      std::make_pair((XMLGen::PlatoMain::ConstraintValueOp)XMLGen::Private::append_compute_constraint_value_operation_platomain, tFuncIndex)));

    // code = plato_analyze
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_compute_constraint_value_operation_platoanalyze));
    mMap.insert(std::make_pair("plato_analyze",
      std::make_pair((XMLGen::PlatoMain::ConstraintValueOp)XMLGen::Private::append_compute_constraint_value_operation_platoanalyze, tFuncIndex)));
}

void ConstraintValueOperation::call
(const XMLGen::Constraint& aConstraint,
 pugi::xml_node& aParentNode) const
{
    auto tLowerCode = Plato::tolower(aConstraint.code());
    auto tMapItr = mMap.find(tLowerCode);
    if(tMapItr == mMap.end())
    {
        THROWERR(std::string("Constraint Value Operation Interface: Did not find 'code' keyword with tag '")
            + tLowerCode + "', 'code' keyword '" + tLowerCode + "' is not supported.")
    }
    auto tTypeCastedFunc = reinterpret_cast<void(*)(const XMLGen::Constraint&, pugi::xml_node&)>(tMapItr->second.first);
    if(tMapItr->second.second == std::type_index(typeid(tTypeCastedFunc)))
    {
        THROWERR(std::string("Constraint Value Operation Interface: Reinterpret cast of constraint value operation ")
            + "for code with tag '" + tLowerCode + "' failed.")
    }
    tTypeCastedFunc(aConstraint, aParentNode);
}

}
// namespace XMLGen