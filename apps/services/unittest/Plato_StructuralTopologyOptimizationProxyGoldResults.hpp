/*
 * Plato_StructuralTopologyOptimizationProxyGoldResults.hpp
 *
 *  Created on: Dec 14, 2017
 */

#ifndef PLATO_STRUCTURALTOPOLOGYOPTIMIZATIONPROXYGOLDRESULTS_HPP_
#define PLATO_STRUCTURALTOPOLOGYOPTIMIZATIONPROXYGOLDRESULTS_HPP_

#include <vector>

namespace TopoProxy
{

std::vector<double> getGoldControlRolTest();
std::vector<double> getGoldControlMmaTest();
std::vector<double> getGoldControlGcmmaTest();
std::vector<double> getGoldControlTrustRegionTest();
std::vector<double> getGoldControlTrustRegionBoundTest();
std::vector<double> getGoldControlOptimalityCriteriaTest();

std::vector<double> getGoldStateData();
std::vector<double> getGoldGradientData();
std::vector<double> getGoldFilteredGradient();
std::vector<double> getGoldElemStiffnessMatrix();
std::vector<double> getGoldElemStiffnessMatrix();
std::vector<double> getGoldHessianTimesVectorData();
std::vector<double> getGoldNormalizedFilteredGradient();
std::vector<double> getGoldNormalizedHessianTimesVector();
std::vector<double> getGoldNormalizedCompoundFilteredGradient();
std::vector<double> getGoldNormalizedCompoundHessianTimesVector();

} // namespace TopoProxy

#endif /* PLATO_STRUCTURALTOPOLOGYOPTIMIZATIONPROXYGOLDRESULTS_HPP_ */
