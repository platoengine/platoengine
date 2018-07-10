/*
 * Plato_Distribution.hpp
 *
 *  Created on: Jan 31, 2018
 */

#ifndef PLATO_DISTRIBUTION_HPP_
#define PLATO_DISTRIBUTION_HPP_

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class Distribution
{
public:
    virtual ~Distribution()
    {
    }

    virtual ScalarType pdf(const ScalarType & tInput) = 0;
    virtual ScalarType cdf(const ScalarType & tInput) = 0;
    virtual ScalarType moment(const OrdinalType & tInput) = 0;
};

} // namespace Plato

#endif /* PLATO_DISTRIBUTION_HPP_ */
