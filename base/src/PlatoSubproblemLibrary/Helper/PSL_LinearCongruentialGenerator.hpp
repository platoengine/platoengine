#pragma once

namespace PlatoSubproblemLibrary
{

class LinearCongruentialGenerator
{
public:
    LinearCongruentialGenerator();
    ~LinearCongruentialGenerator();

    void build(const unsigned long int& modulus,
               const unsigned long int& multiplier,
               const unsigned long int& increment,
               const unsigned long int& seed);
    unsigned long int get_modulus();

    unsigned long int generate();

private:
    unsigned long int m_modulus;
    unsigned long int m_multiplier;
    unsigned long int m_increment;
    unsigned long int m_current_state;

};

void globalLcg_deterministic_seed();
unsigned long int globalLcg_getRandInt();
double globalLcg_getRandDouble();

}
