#include "PSL_LinearCongruentialGenerator.hpp"

#include <cassert>

namespace PlatoSubproblemLibrary
{

LinearCongruentialGenerator::LinearCongruentialGenerator() :
        m_modulus(0),
        m_multiplier(0),
        m_increment(0),
        m_current_state(0)
{
}
LinearCongruentialGenerator::~LinearCongruentialGenerator()
{
}
void LinearCongruentialGenerator::build(const unsigned long int& modulus,
                                        const unsigned long int& multiplier,
                                        const unsigned long int& increment,
                                        const unsigned long int& seed)
{
    m_modulus = modulus;
    m_multiplier = multiplier;
    m_increment = increment;
    m_current_state = seed;
}
unsigned long int LinearCongruentialGenerator::get_modulus()
{
    return m_modulus;
}

unsigned long int LinearCongruentialGenerator::generate()
{
    assert(0 < m_modulus);
    m_current_state = (m_increment + ((m_multiplier * m_current_state) % m_modulus)) % m_modulus;
    return m_current_state;
}

LinearCongruentialGenerator psl_global_lcg;
void globalLcg_deterministic_seed()
{
    const unsigned long int modulus = 2147483647;
    const unsigned long int multiplier = 48271;
    const unsigned long int increment = 0;
    const unsigned long int seed = 42;
    psl_global_lcg.build(modulus, multiplier, increment, seed);
}
unsigned long int globalLcg_getRandInt()
{
    return psl_global_lcg.generate();
}
double globalLcg_getRandDouble()
{
    const long double numerator = psl_global_lcg.generate();
    const long double denominator = psl_global_lcg.get_modulus();
    return numerator / denominator;
}

}
