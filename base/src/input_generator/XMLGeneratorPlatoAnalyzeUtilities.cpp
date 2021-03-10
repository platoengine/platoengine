/*
 * XMLGeneratorPlatoAnalyzeUtilities.cpp
 *
 *  Created on: Jun 8, 2020
 */

#include "XMLGeneratorPlatoAnalyzeUtilities.hpp"

namespace XMLGen
{

/******************************************************************************/
bool is_plato_analyze_code(const std::string& aCode)
{
    if(aCode.empty())
    {
        THROWERR("Is Plato Analyze Code: Input argument is empty.")
    }
    auto tLowerKey = Plato::tolower(aCode);
    auto tIsPlatoAnalyze = tLowerKey.compare("plato_analyze") == 0;
    return (tIsPlatoAnalyze);
}
/******************************************************************************/

/******************************************************************************/
bool is_topology_optimization_problem(const std::string& aProblemType)
{
    if(aProblemType.empty())
    {
        THROWERR("Is Topology Optimization Problem?: Input argument is empty.")
    }
    auto tLowerKey = Plato::tolower(aProblemType);
    auto tIsTopologyOptimization = tLowerKey.compare("topology") == 0;
    return (tIsTopologyOptimization);
}
/******************************************************************************/

/******************************************************************************/
bool is_any_service_provided_by_plato_analyze
(const XMLGen::InputData& aXMLMetaData)
{
    auto tAtLeastOnePerformerIsPlatoAnalyze = false;
    for(auto& tService : aXMLMetaData.services())
    {
        if(XMLGen::is_plato_analyze_code(tService.code()))
        {
            tAtLeastOnePerformerIsPlatoAnalyze = true;
            break;
        }
    }
    return (tAtLeastOnePerformerIsPlatoAnalyze);
}
/******************************************************************************/

/******************************************************************************/
bool is_any_constraint_computed_by_plato_analyze
(const XMLGen::InputData& aXMLMetaData)
{
    auto tAtLeastOnePerformerIsPlatoAnalyze = false;
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        auto tService = aXMLMetaData.service(tConstraint.service());
        if(XMLGen::is_plato_analyze_code(tService.code()))
        {
            tAtLeastOnePerformerIsPlatoAnalyze = true;
            break;
        }
    }
    return (tAtLeastOnePerformerIsPlatoAnalyze);
}
/******************************************************************************/

/******************************************************************************/
bool is_any_objective_computed_by_plato_analyze
(const XMLGen::InputData& aXMLMetaData)
{
    auto tAtLeastOnePerformerIsPlatoAnalyze = false;
    for(size_t i=0; i<aXMLMetaData.objective.serviceIDs.size(); i++)
    {
        auto tService = aXMLMetaData.service(aXMLMetaData.objective.serviceIDs[i]);
        if(XMLGen::is_plato_analyze_code(tService.code()))
        {
            tAtLeastOnePerformerIsPlatoAnalyze = true;
            break;
        }
    }
    return (tAtLeastOnePerformerIsPlatoAnalyze);
}
/******************************************************************************/

/******************************************************************************/
std::vector<std::string>
return_constraints_computed_by_plato_analyze
(const XMLGen::InputData& aXMLMetaData)
{
    std::vector<std::string> tCategories;
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        auto tService = aXMLMetaData.service(tConstraint.service());
        if(tService.code().compare("plato_analyze") == 0)
        {
            auto tCriterion = aXMLMetaData.criterion(tConstraint.criterion());
            tCategories.push_back(tCriterion.type());
        }
    }
    return tCategories;
}
/******************************************************************************/

/******************************************************************************/
std::vector<std::string>
return_objectives_computed_by_plato_analyze
(const XMLGen::InputData& aXMLMetaData)
{
    std::vector<std::string> tCategories;
    for(size_t i=0; i<aXMLMetaData.objective.serviceIDs.size(); ++i)
    {
        auto &tService = aXMLMetaData.service(aXMLMetaData.objective.serviceIDs[i]);
        if(tService.code().compare("plato_analyze") == 0)
        {
            auto &tCriterion = aXMLMetaData.criterion(aXMLMetaData.objective.criteriaIDs[i]);
            tCategories.push_back(tCriterion.type());
        }
    }
    return tCategories;
}
/******************************************************************************/

}
// namespace XMLGen
