#pragma once

#include "PSL_ParameterDataEnums.hpp"

namespace PlatoSubproblemLibrary
{

class InvertibleUnitRange
{
public:
    InvertibleUnitRange(invertible_unit_range_t::invertible_unit_range_t type,
                        const double& negative_scale,
                        const double& positive_scale);
    virtual ~InvertibleUnitRange();

    invertible_unit_range_t::invertible_unit_range_t get_type();

    void set_scale(const double& negative_scale, const double& positive_scale);

    virtual double evaluate(const double& x) = 0;
    virtual double invert(const double& y) = 0;

protected:
    invertible_unit_range_t::invertible_unit_range_t m_type;
    double m_negative_scale;
    double m_positive_scale;

};

}

