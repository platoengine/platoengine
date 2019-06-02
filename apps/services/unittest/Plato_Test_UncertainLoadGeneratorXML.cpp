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
        const int tMyLoadCaseId = std::atoi(aUncertainLoadCases[tPrivateLoadIndex].load_id.c_str());
        aUniqueLoadCounter.mark(tMyLoadCaseId);
    }

    return (true);
}

}
