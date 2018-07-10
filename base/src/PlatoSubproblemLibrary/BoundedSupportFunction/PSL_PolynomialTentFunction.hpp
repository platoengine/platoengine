// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_BoundedSupportFunction.hpp"

namespace PlatoSubproblemLibrary
{
class Point;
class ParameterData;

class PolynomialTentFunction : public Abstract_BoundedSupportFunction
{
public:
    PolynomialTentFunction();
    virtual ~PolynomialTentFunction();

    virtual void build(double support, ParameterData* input_data);
    virtual double evaluate(Point* center, Point* other);
    virtual double get_support();

protected:
    double m_radius;
    double m_radius_squared;
    double m_weighting_penalty;

};

}
