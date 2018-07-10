#include "PSL_InvertibleUnitRange.hpp"

#include "PSL_ParameterDataEnums.hpp"

#include <cassert>

namespace PlatoSubproblemLibrary
{

InvertibleUnitRange::InvertibleUnitRange(invertible_unit_range_t::invertible_unit_range_t type,
                                         const double& negative_scale,
                                         const double& positive_scale) :
        m_type(type),
        m_negative_scale(0.),
        m_positive_scale(0.)
{
    set_scale(negative_scale, positive_scale);
}
InvertibleUnitRange::~InvertibleUnitRange()
{
}

invertible_unit_range_t::invertible_unit_range_t InvertibleUnitRange::get_type()
{
    return m_type;
}

void InvertibleUnitRange::set_scale(const double& negative_scale, const double& positive_scale)
{
    m_negative_scale = negative_scale;
    m_positive_scale = positive_scale;
    assert(m_negative_scale > 0.);
    assert(m_positive_scale > 0.);
}

}


