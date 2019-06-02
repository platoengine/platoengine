/*
 * Plato_Test_UncertainLoadGeneratorXML.cpp
 *
 *  Created on: Jun 1, 2019
 */

#include <gtest/gtest.h>

namespace Plato
{

inline bool register_all_load_ids(const std::vector<LoadCase> & aLoadCases, Plato::UniqueCounter & aUniqueLoadCounter)
{
    if(aLoadCases.size() <= 0)
    {
        std::cout<< "\nFILE: " __FILE__
                 << "\nFUNCTION: " __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: INPUT SET OF LOAD CASES IS EMPTY.\n";
        return (false);
    }

    const int tNumLoadCases = aLoadCases.size();
    for(int tPrivateLoadIndex = 0; tPrivateLoadIndex < tNumLoadCases; tPrivateLoadIndex++)
    {
        // register load case
        const int tMyLoadCaseId = std::atoi(aLoadCases[tPrivateLoadIndex].id.c_str());
        aUniqueLoadCounter.mark(tMyLoadCaseId);

        const std::vector<Load> & tMyLoadCaseLoads = aLoadCases[tPrivateLoadIndex].loads;
        const int tMyLoadCaseNumLoads = tMyLoadCaseLoads.size();
        for(int tLoadIndex = 0; tLoadIndex < tMyLoadCaseNumLoads; tLoadIndex++)
        {
            // register load id
            const int tMyLoadID = std::atoi(tMyLoadCaseLoads[tLoadIndex].load_id.c_str());
            aUniqueLoadCounter.mark(tMyLoadID);
        }
    }

    return (true);
}

inline bool register_all_uncertain_load_ids(const std::vector<Uncertainty> & aUncertainLoadCases,
                                            Plato::UniqueCounter & aUniqueLoadCounter)
{
    if(aUncertainLoadCases.size() <= 0)
    {
        std::cout<< "\nFILE: " __FILE__
                 << "\nFUNCTION: " __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: INPUT SET OF UNCERTAINTY CASES IS EMPTY.\n";
        return (false);
    }

    const int tNumLoadCases = aUncertainLoadCases.size();
    for(int tPrivateLoadIndex = 0; tPrivateLoadIndex < tNumLoadCases; tPrivateLoadIndex++)
    {
        // register load case
        const int tMyLoadCaseId = std::atoi(aUncertainLoadCases[tPrivateLoadIndex].id.c_str());
        aUniqueLoadCounter.mark(tMyLoadCaseId);
    }

    return (true);
}

inline bool define_distribution(const Uncertainty & aMyRandomVar, Plato::UncertaintyInputStruct<double> & aInput)
{
    if(aMyRandomVar.distribution == "normal")
    {
        aInput.mDistribution = Plato::DistrubtionName::type_t::normal;
    }
    else if(aMyRandomVar.distribution == "uniform")
    {
        aInput.mDistribution = Plato::DistrubtionName::type_t::uniform;
    }
    else if(aMyRandomVar.distribution == "beta")
    {
        aInput.mDistribution = Plato::DistrubtionName::type_t::beta;
    }
    else
    {
        std::cout<< "\nFILE: " __FILE__
                 << "\nFUNCTION: " __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: UNDEFINE DISTRIBUTION. OPTIONS ARE NORMAL, UNIFORM, AND BETA.\n";
        return (false);
    }

    return (true);
}

inline bool compute_sample_probability_pairs(const std::vector<Uncertainty> & aUncertainLoadCases)
{
    const size_t tNumRandomVars = aUncertainLoadCases.size();
    for(size_t tRandomVarIndex = 0; tRandomVarIndex < tNumRandomVars; tRandomVarIndex++)
    {
        // pose uncertainty
        Plato::UncertaintyInputStruct<double> tInput;
        const Uncertainty & tMyRandomVar = aUncertainLoadCases[tRandomVarIndex];
        if(Plato::define_distribution(tMyRandomVar, tInput))
        {
            return (false);
        }

        tInput.mMean = std::atof(tMyRandomVar.mean.c_str());
        tInput.mLowerBound = std::atof(tMyRandomVar.lower.c_str());
        tInput.mUpperBound = std::atof(tMyRandomVar.upper.c_str());
        const double tStdDev = std::atof(tMyRandomVar.standard_deviation.c_str());
        tInput.mVariance = tStdDev * tStdDev;
        const size_t tNumSamples = std::atoi(tMyRandomVar.num_samples.c_str());
        tInput.mNumSamples = tNumSamples;

        // solve uncertainty sub-problem
        const bool tEnableOutput = true;
        Plato::AlgorithmInputsKSAL<double> tAlgoInputs;
        Plato::SromProblemDiagnosticsStruct<double> tSromDiagnostics;
        std::vector<Plato::UncertaintyOutputStruct<double>> tSromOutput;
        Plato::solve_uncertainty(tInput, tAlgoInputs, tSromDiagnostics, tSromOutput, tEnableOutput);
    }

    return (true);
}

}
