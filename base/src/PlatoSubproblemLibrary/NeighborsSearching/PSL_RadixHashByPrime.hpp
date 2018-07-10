#pragma once

#include <vector>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;

class RadixHashByPrime
{
public:
    RadixHashByPrime(AbstractAuthority* authority);
    ~RadixHashByPrime();

    void initialize(const int& maximum_hash_size, const std::vector<int>& radix_sizes);
    int get_hash_size();
    int hash(const std::vector<int>& this_radix);

protected:
    AbstractAuthority* m_authority;
    std::vector<int> m_radix_multipliers;
    int m_hash_size;

};

}
