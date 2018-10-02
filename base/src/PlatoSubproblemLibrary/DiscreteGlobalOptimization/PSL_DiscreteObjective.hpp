#pragma once

/* Abstract discrete objective.
 * Main function is evaluate(...)
 */

#include <vector>

namespace PlatoSubproblemLibrary
{

class DiscreteObjective
{
public:
    DiscreteObjective();
    virtual ~DiscreteObjective();

    virtual void get_domain(std::vector<double>& aInclusiveLower,
                            std::vector<double>& aInclusiveUpper,
                            std::vector<int>& aNumValues) = 0;
    virtual double evaluate(const std::vector<double>& aParameters) = 0;

protected:

};

}
