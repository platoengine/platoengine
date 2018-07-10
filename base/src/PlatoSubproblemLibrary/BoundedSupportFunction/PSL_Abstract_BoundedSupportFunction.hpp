// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#pragma once

namespace PlatoSubproblemLibrary
{
class Point;
class ParameterData;

class Abstract_BoundedSupportFunction
{
public:
    Abstract_BoundedSupportFunction();
    virtual ~Abstract_BoundedSupportFunction();

    virtual void build(double support, ParameterData* input_data) = 0;
    virtual double evaluate(Point* center, Point* other) = 0;
    virtual double get_support() = 0;

protected:

};

}
