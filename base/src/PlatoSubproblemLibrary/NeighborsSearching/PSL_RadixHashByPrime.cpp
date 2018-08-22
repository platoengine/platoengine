#include "PSL_RadixHashByPrime.hpp"

#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <cstddef>
#include <cassert>

namespace PlatoSubproblemLibrary
{

RadixHashByPrime::RadixHashByPrime(AbstractAuthority* authority) :
        m_authority(authority),
        m_radix_multipliers(),
        m_hash_size()
{
}
RadixHashByPrime::~RadixHashByPrime()
{
}

void RadixHashByPrime::initialize(const int& maximum_hash_size, const std::vector<int>& radix_sizes)
{
    const int length = radix_sizes.size();

    m_hash_size = maximum_hash_size;
    while(!is_prime(m_hash_size))
    {
        m_hash_size--;
    }

    // compute multipliers
    m_radix_multipliers.assign(length, 1);
    for(int i = 0; i + 1 < length; i++)
    {
        m_radix_multipliers[i + 1] = (m_radix_multipliers[i] * radix_sizes[i]) % maximum_hash_size;
    }
}

int RadixHashByPrime::get_hash_size()
{
    return m_hash_size;
}

int RadixHashByPrime::hash(const std::vector<int>& this_radix)
{
    const int length = m_radix_multipliers.size();
    assert(length == int(this_radix.size()));

    // hash it up
    int result = 0;
    for(int i = 0; i < length; i++)
    {
        result = (result + this_radix[i] * m_radix_multipliers[i]) % m_hash_size;
    }
    return result;
}

}
