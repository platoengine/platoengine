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
 const std::string &aPerformer,
 const std::string &aDesignVariableName,
 pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Compute Constraint Value", aPerformer}, tOperationNode);

    if(aDesignVariableName != "")
    {
        auto tInputNode = tOperationNode.append_child("Input");
        XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Topology", aDesignVariableName}, tInputNode);
    }

    std::string tCriterionID = aConstraint.criterion();
    std::string tServiceID = aConstraint.service();
    std::string tScenarioID = aConstraint.scenario();
    ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
    auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);

    auto tSharedDataName = std::string("Criterion Value - ") + tIdentifierString;
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Volume", tSharedDataName}, tOutputNode);

    tSharedDataName = std::string("Criterion Gradient - ") + tIdentifierString;
    tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Volume Gradient", tSharedDataName}, tOutputNode);
}

void append_compute_constraint_value_operation_platoanalyze
(const XMLGen::Constraint& aConstraint,
 const std::string &aPerformer,
 const std::string &aDesignVariableName,
 pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Compute Constraint Value", aPerformer}, tOperationNode);

    if(aDesignVariableName != "")
    {
        auto tInputNode = tOperationNode.append_child("Input");
        XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Topology", aDesignVariableName}, tInputNode);
    }

    std::string tCriterionID = aConstraint.criterion();
    std::string tServiceID = aConstraint.service();
    std::string tScenarioID = aConstraint.scenario();
    ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
    auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);

    auto tSharedDataName = std::string("Criterion Value - ") + tIdentifierString;
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Constraint Value", tSharedDataName}, tOutputNode);
}

void append_compute_constraint_value_operation_sierra_sd
(const XMLGen::Constraint& aConstraint,
 const std::string &aPerformer,
 const std::string &aDesignVariableName,
 pugi::xml_node& aParentNode)
{
/*
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Compute Constraint Value", aPerformer}, tOperationNode);

    if(aDesignVariableName != "")
    {
        auto tInputNode = tOperationNode.append_child("Input");
        XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Topology", aDesignVariableName}, tInputNode);
    }

    std::string tCriterionID = aConstraint.criterion();
    std::string tServiceID = aConstraint.service();
    std::string tScenarioID = aConstraint.scenario();
    ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
    auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);

    auto tSharedDataName = std::string("Criterion Value - ") + tIdentifierString;
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Constraint Value", tSharedDataName}, tOutputNode);
*/
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

    // code = sierra_sd
    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_compute_constraint_value_operation_sierra_sd));
    mMap.insert(std::make_pair("sierra_sd",
      std::make_pair((XMLGen::PlatoMain::ConstraintValueOp)XMLGen::Private::append_compute_constraint_value_operation_sierra_sd, tFuncIndex)));
}

void ConstraintValueOperation::call
(const XMLGen::Constraint& aConstraint,
 const std::string &aPerformer,
 const std::string &aDesignVariableName,
 const std::string &aCode,
 pugi::xml_node& aParentNode) const
{
    auto tLowerCode = Plato::tolower(aCode);
    auto tMapItr = mMap.find(tLowerCode);
    if(tMapItr == mMap.end())
    {
        THROWERR(std::string("Constraint Value Operation Interface: Did not find 'code' keyword with tag '")
            + tLowerCode + "', 'code' keyword '" + tLowerCode + "' is not supported.")
    }
    auto tTypeCastedFunc = reinterpret_cast<void(*)(const XMLGen::Constraint&, const std::string &aPerformer, const std::string &aDesignVariableName, pugi::xml_node&)>(tMapItr->second.first);
    if(tMapItr->second.second == std::type_index(typeid(tTypeCastedFunc)))
    {
        THROWERR(std::string("Constraint Value Operation Interface: Reinterpret cast of constraint value operation ")
            + "for code with tag '" + tLowerCode + "' failed.")
    }
    tTypeCastedFunc(aConstraint, aPerformer, aDesignVariableName, aParentNode);
}

}
// namespace XMLGen
