/*
 //@HEADER
 // *************************************************************************
 //   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
 //                    Solutions of Sandia, LLC (NTESS).
 //
 // Under the terms of Contract DE-NA0003525 with NTESS,
 // the U.S. Government retains certain rights in this software.
 //
 // Redistribution and use in source and binary forms, with or without
 // modification, are permitted provided that the following conditions are
 // met:
 //
 // 1. Redistributions of source code must retain the above copyright
 // notice, this list of conditions and the following disclaimer.
 //
 // 2. Redistributions in binary form must reproduce the above copyright
 // notice, this list of conditions and the following disclaimer in the
 // documentation and/or other materials provided with the distribution.
 //
 // 3. Neither the name of the Sandia Corporation nor the names of the
 // contributors may be used to endorse or promote products derived from
 // this software without specific prior written permission.
 //
 // THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
 // EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 // IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 // PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
 // CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 // EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 // PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 // PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 // LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 // NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 // SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 //
 // Questions? Contact the Plato team (plato3D-help@sandia.gov)
 //
 // *************************************************************************
 //@HEADER
 */

/*
 * Plato_StatisticsOperationsUtilities.cpp
 *
 *  Created on: Jul 6, 2019
 */

#include <cmath>

#include "Plato_OperationsUtilities.hpp"
#include "Plato_StatisticsOperationsUtilities.hpp"

namespace Plato
{

void compute_scalar_value_mean(const std::map<std::string, double>& aSampleProbMap,
                               const std::map<std::string, std::string>& aStatsArgumentNameMap,
                               PlatoApp* aPlatoApp)
{
    double tLocalValue = 0;
    // tIterator->first = Argument name & tIterator->second = Probability
    for(auto tIterator = aSampleProbMap.begin(); tIterator != aSampleProbMap.end(); ++ tIterator)
    {
        std::vector<double>* tInputValue = aPlatoApp->getValue(tIterator->first);
        tLocalValue += tIterator->second * (*tInputValue)[0];
    }

    double tGlobalValue = 0.;
    aPlatoApp->reduceScalarValue(tLocalValue, tGlobalValue);
    const std::string& tOutputArgumentName = aStatsArgumentNameMap.find("MEAN")->second;
    std::vector<double>* tOutputValue = aPlatoApp->getValue(tOutputArgumentName);
    (*tOutputValue)[0] = tGlobalValue;
}
// function compute_scalar_value_mean

void compute_node_field_mean(const std::map<std::string, double>& aSampleProbMap,
                             const std::map<std::string, std::string>& aStatsArgumentNameMap,
                             PlatoApp* aPlatoApp)
{
    const std::string& tOutputArgumentName = aStatsArgumentNameMap.find("MEAN")->second;
    const size_t tLength = aPlatoApp->getNodeFieldLength(tOutputArgumentName);
    double* tOutputData = aPlatoApp->getNodeFieldData(tOutputArgumentName);
    Plato::zero(tLength, tOutputData);

    // tIterator->first = Argument name & tIterator->second = Probability
    for(auto tIterator = aSampleProbMap.begin(); tIterator != aSampleProbMap.end(); ++ tIterator)
    {
        double* tInputData = aPlatoApp->getNodeFieldData(tIterator->first);
        for(size_t tIndex = 0; tIndex < tLength; tIndex++)
        {
            tOutputData[tIndex] += tIterator->second * tInputData[tIndex];
        }
    }

    aPlatoApp->compressAndUpdateNodeField(tOutputArgumentName);
}
// function compute_node_field_mean

void compute_element_field_mean(const std::map<std::string, double>& aSampleProbMap,
                                const std::map<std::string, std::string>& aStatsArgumentNameMap,
                                PlatoApp* aPlatoApp)
{
    const size_t tLocalNumElements = aPlatoApp->getLocalNumElements();
    const std::string& tOutputArgumentName = aStatsArgumentNameMap.find("MEAN")->second;
    double* tOutputData = aPlatoApp->getElementFieldData(tOutputArgumentName);
    Plato::zero(tLocalNumElements, tOutputData);

    // tIterator->first = Argument name & tIterator->second = Probability
    for(auto tIterator = aSampleProbMap.begin(); tIterator != aSampleProbMap.end(); ++ tIterator)
    {
        double* tInputDataView = aPlatoApp->getElementFieldData(tIterator->first);
        for(size_t tIndex = 0; tIndex < tLocalNumElements; tIndex++)
        {
            tOutputData[tIndex] += tIterator->second * tInputDataView[tIndex];
        }
    }
}
// function compute_element_field_mean

void compute_scalar_value_standard_deviation(const std::map<std::string, double>& aSampleProbMap,
                                             const std::map<std::string, std::string>& aStatsArgumentNameMap,
                                             PlatoApp* aPlatoApp)
{
    // STD_DEV(f(x)) = sqrt( E(f(x)^2) - E(f(x))^2 )
    double tLocalValue = 0;
    // tIterator->first = Argument name & tIterator->second = Probability
    for(auto tIterator = aSampleProbMap.begin(); tIterator != aSampleProbMap.end(); ++ tIterator)
    {
        std::vector<double>* tMySampleData = aPlatoApp->getValue(tIterator->first);
        tLocalValue += tIterator->second * std::pow((*tMySampleData)[0], 2.0);
    }

    // Compute E(f(x)^2)
    double tGlobalValue = 0.;
    aPlatoApp->reduceScalarValue(tLocalValue, tGlobalValue);

    // Compute standard deviation, i.e. sqrt( E(f(x)^2) - E(f(x))^2 )
    const std::string& tArgumentMean = aStatsArgumentNameMap.find("MEAN")->second;
    std::vector<double>* tMean = aPlatoApp->getValue(tArgumentMean);
    const double tValue = tGlobalValue - std::pow((*tMean)[0], 2.0);
    const std::string& tOutputArgumentStdDev = aStatsArgumentNameMap.find("STDDEV")->second;
    std::vector<double>* tOutputSigmaData = aPlatoApp->getValue(tOutputArgumentStdDev);
    (*tOutputSigmaData)[0] = std::sqrt(tValue);
}
// function compute_scalar_value_standard_deviation

void compute_node_field_standard_deviation(const std::map<std::string, double>& aSampleProbMap,
                                           const std::map<std::string, std::string>& aStatsArgumentNameMap,
                                           PlatoApp* aPlatoApp)
{
    // STD_DEV(f(x)) = sqrt( E(f(x)^2) - E(f(x))^2 )
    const std::string& tArgumentStdDev = aStatsArgumentNameMap.find("STDDEV")->second;
    double* tStandardDeviation = aPlatoApp->getNodeFieldData(tArgumentStdDev);

    // 1. Compute E(f(x)^2)
    const size_t tLocalLength = aPlatoApp->getNodeFieldLength(tArgumentStdDev);
    Plato::zero(tLocalLength, tStandardDeviation);
    // tIterator->first = Argument name & tIterator->second = Probability
    for(auto tIterator = aSampleProbMap.begin(); tIterator != aSampleProbMap.end(); ++ tIterator)
    {
        double* tMySample = aPlatoApp->getNodeFieldData(tIterator->first);
        for(size_t tIndex = 0; tIndex < tLocalLength; tIndex++)
        {
            tStandardDeviation[tIndex] += tIterator->second * std::pow(tMySample[tIndex], 2.0);
        }
    }

    // 2. Compute standard deviation, i.e. sqrt( E(f(x)^2) - E(f(x))^2 )
    const std::string& tArgumentMean = aStatsArgumentNameMap.find("MEAN")->second;
    double* tMean = aPlatoApp->getNodeFieldData(tArgumentMean);
    for(size_t tIndex = 0; tIndex < tLocalLength; tIndex++)
    {
        double tValue = tStandardDeviation[tIndex] - std::pow(tMean[tIndex], 2.0);
        tStandardDeviation[tIndex] = std::sqrt(tValue);
    }

    // 3. Resolve shared data across processes
    aPlatoApp->compressAndUpdateNodeField(tArgumentStdDev);
}
// function compute_node_field_standard_deviation

void compute_element_field_standard_deviation(const std::map<std::string, double>& aSampleProbMap,
                                              const std::map<std::string, std::string>& aStatsArgumentNameMap,
                                              PlatoApp* aPlatoApp)
{
    // STD_DEV(f(x)) = sqrt( E(f(x)^2) - E(f(x))^2 )
    const std::string& tArgumentStdDev = aStatsArgumentNameMap.find("STDDEV")->second;
    double* tStandardDeviation = aPlatoApp->getElementFieldData(tArgumentStdDev);

    // 1. Compute E(f(x)^2)
    const size_t tLocalLength = aPlatoApp->getLocalNumElements();
    Plato::zero(tLocalLength, tStandardDeviation);
    // tIterator->first = Argument name & tIterator->second = Probability
    for(auto tIterator = aSampleProbMap.begin(); tIterator != aSampleProbMap.end(); ++ tIterator)
    {
        double* tMySample = aPlatoApp->getElementFieldData(tIterator->first);
        for(size_t tIndex = 0; tIndex < tLocalLength; tIndex++)
        {
            tStandardDeviation[tIndex] += tIterator->second * std::pow(tMySample[tIndex], 2.0);
        }
    }

    // 2. Compute standard deviation, i.e. sqrt( E(f(x)^2) - E(f(x))^2 )
    const std::string& tArgumentMean = aStatsArgumentNameMap.find("MEAN")->second;
    double* tMean = aPlatoApp->getElementFieldData(tArgumentMean);
    for(size_t tIndex = 0; tIndex < tLocalLength; tIndex++)
    {
        double tValue = tStandardDeviation[tIndex] - std::pow(tMean[tIndex], 2.0);
        tStandardDeviation[tIndex] = std::sqrt(tValue);
    }
}
// function compute_element_field_standard_deviation

}
// namespace Plato
