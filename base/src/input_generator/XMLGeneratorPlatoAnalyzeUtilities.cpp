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
        if(XMLGen::is_plato_analyze_code(tConstraint.code()))
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
        if(tConstraint.code().compare("plato_analyze") == 0)
        {
            tCategories.push_back(tConstraint.category());
        }
    }
    return tCategories;
}
/******************************************************************************/

/******************************************************************************/
// std::vector<std::string>
// return_objectives_computed_by_plato_analyze
// (const XMLGen::InputData& aXMLMetaData)
// {
//     std::vector<std::string> tCategories;
//     for(auto& tObjective : aXMLMetaData.objectives)
//     {
//         if(tObjective.code().compare("plato_analyze") == 0)
//         {
//             tCategories.push_back(tObjective.category());
//         }
//     }
//     return tCategories;
// }
/******************************************************************************/

}
// namespace XMLGen
