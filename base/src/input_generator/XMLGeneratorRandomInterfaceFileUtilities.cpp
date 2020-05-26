/*
 * XMLGeneratorRandomInterfaceFileUtilities.cpp
 *
 *  Created on: May 25, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorInterfaceFileUtilities.hpp"
#include "XMLGeneratorRandomInterfaceFileUtilities.hpp"

namespace XMLGen
{

/******************************************************************************/
void append_nondeterministic_shared_data
(const std::vector<std::string>& aKeys,
 const std::vector<std::string>& aValues,
 pugi::xml_document& aDocument)
{
    auto tForNode = aDocument.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tForNode);
    tForNode = tForNode.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tForNode);
    auto tSharedDataNode = tForNode.append_child("SharedData");
    XMLGen::append_childs(aKeys, aValues, tSharedDataNode);
}
// function append_nondeterministic_shared_data
/******************************************************************************/

/******************************************************************************/
void append_criterion_shared_data_for_nondeterministic_usecase
(const std::string& aCriterion,
 const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.objectives.empty())
    {
        THROWERR(std::string("Append Criterion Shared Data For Nondeterministic Use Case: ")
            + "Objective function list is empty.")
    }

    // shared data - nondeterministic criterion value
    auto tOwnerName = aXMLMetaData.objectives[0].performer_name + " {PerformerIndex}";
    auto tTag = aCriterion + " Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {tTag, "Scalar", "Global", "1", tOwnerName, "PlatoMain"};
    XMLGen::append_nondeterministic_shared_data(tKeys, tValues, aDocument);

    // shared data - nondeterministic criterion gradient
    tTag = aCriterion + " Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
    tValues = {tTag, "Scalar", "Nodal Field", "IGNORE", tOwnerName, "PlatoMain"};
    XMLGen::append_nondeterministic_shared_data(tKeys, tValues, aDocument);
}
//function append_criterion_shared_data_for_nondeterministic_usecase
/******************************************************************************/

/******************************************************************************/
void append_qoi_shared_data_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.objectives.empty())
    {
        THROWERR(std::string("Append QOI Shared Data For Nondeterministic Use Case: ")
            + "Objective function list is empty.")
    }
    // shared data - QOI statistics
    // 1 loop over QOI
    // 2 define tag/name and set "Name" key
    auto tOwnerName = aXMLMetaData.objectives[0].performer_name + " {PerformerIndex}";
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Von Mises {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}", "Scalar",
                                        "Element Field", "IGNORE", tOwnerName, "PlatoMain"};
    XMLGen::append_nondeterministic_shared_data(tKeys, tValues, aDocument);
}
//function append_qoi_shared_data_for_nondeterministic_usecase
/******************************************************************************/

/******************************************************************************/
void append_topology_shared_data_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.objectives.empty())
    {
        THROWERR(std::string("Append QOI Shared Data For Nondeterministic Use Case: ")
            + "Objective function list is empty.")
    }

    auto tSharedData = aDocument.append_child("SharedData");
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Topology", "Scalar", "Nodal Field", "IGNORE", "PlatoMain", "PlatoMain"};
    XMLGen::append_childs(tKeys, tValues, tSharedData);
    auto tForNode = tSharedData.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tForNode);
    auto tUserName = aXMLMetaData.objectives[0].performer_name + " {PerformerIndex}";
    XMLGen::append_childs({"UserName"}, {tUserName}, tForNode);
}
//function append_topology_shared_data_for_nondeterministic_usecase
/******************************************************************************/

/******************************************************************************/
void append_physics_performers_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.objectives.empty())
    {
        THROWERR(std::string("Append QOI Shared Data For Nondeterministic Use Case: ")
            + "Objective function list is empty.")
    }

    for(auto& tObjective : aXMLMetaData.objectives)
    {
        const int tID = (&tObjective - &aXMLMetaData.objectives[0]) + 1;
        auto tPerformerNode = aDocument.append_child("Performer");
        XMLGen::append_childs( { "PerformerID" }, { std::to_string(tID) }, tPerformerNode);
        auto tForNode = tPerformerNode.append_child("For");
        XMLGen::append_attributes( { "var", "in" }, { "PerformerIndex", "Performers" }, tForNode);
        auto tPerformerName = tObjective.performer_name + " {PerformerIndex}";
        XMLGen::append_childs( { "Name", "Code" }, { tPerformerName, tObjective.code_name }, tForNode);
    }
}
// function append_physics_performers_for_nondeterministic_usecase
/******************************************************************************/

/******************************************************************************/
void append_shared_data_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    XMLGen::append_control_shared_data(aDocument);
    XMLGen::append_lower_bounds_shared_data(aDocument);
    XMLGen::append_upper_bounds_shared_data(aDocument);
    XMLGen::append_design_volume_shared_data(aDocument);

    auto tOwnerName = std::string("PlatoMain");
    XMLGen::append_criterion_shared_data("Objective", aXMLMetaData, aDocument, tOwnerName);
    tOwnerName = aXMLMetaData.objectives[0].performer_name + "_0";
    XMLGen::append_criterion_shared_data("Constraint", aXMLMetaData, aDocument, tOwnerName);

    XMLGen::append_qoi_shared_data_for_nondeterministic_usecase(aXMLMetaData, aDocument);
    XMLGen::append_topology_shared_data_for_nondeterministic_usecase(aXMLMetaData, aDocument);
    XMLGen::append_criterion_shared_data_for_nondeterministic_usecase("Objective", aXMLMetaData, aDocument);
}
// function append_shared_data_for_nondeterministic_usecase
/******************************************************************************/

}
// namespace XMLGen
