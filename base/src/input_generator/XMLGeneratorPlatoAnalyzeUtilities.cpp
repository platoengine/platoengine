/*
 * XMLGeneratorPlatoAnalyzeUtilities.cpp
 *
 *  Created on: Jun 8, 2020
 */

#include "XMLGeneratorPlatoAnalyzeUtilities.hpp"

namespace XMLGen
{

/******************************************************************************/
bool is_plato_analyze_performer(const std::string& aPerformer)
{
    if(aPerformer.empty())
    {
        THROWERR("Is Plato Analyze Performer?: Input argument is empty.")
    }
    auto tLowerKey = Plato::tolower(aPerformer);
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
bool is_any_objective_computed_by_plato_analyze
(const XMLGen::InputData& aXMLMetaData)
{
    auto tAtLeastOnePerformerIsPlatoAnalyze = false;
    for(auto& tObjective : aXMLMetaData.objectives)
    {
        if(XMLGen::is_plato_analyze_performer(tObjective.mPerformerName))
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
        if(XMLGen::is_plato_analyze_performer(tConstraint.performer()))
        {
            tAtLeastOnePerformerIsPlatoAnalyze = true;
            break;
        }
    }
    return (tAtLeastOnePerformerIsPlatoAnalyze);
}
/******************************************************************************/

}
// namespace XMLGen
