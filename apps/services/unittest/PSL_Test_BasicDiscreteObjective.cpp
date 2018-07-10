#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Interface_MpiWrapper.hpp"
#include "PSL_DiscreteObjective.hpp"
#include "PSL_IterativeSelection.hpp"
#include "PSL_RandomSearch.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_Random.hpp"

#include <vector>
#include <string>
#include <mpi.h>
#include <cmath>
#include <math.h>
#include <cassert>
#include <iostream>

namespace PlatoSubproblemLibrary
{
namespace TestingBasicDiscreteObjective
{

class BasicDiscreteObjective : public DiscreteObjective
{
public:
    BasicDiscreteObjective() :
        DiscreteObjective()
    {
    }

    virtual ~BasicDiscreteObjective()
    {
    }

    virtual void get_domain(std::vector<double>& inclusive_lower,
                            std::vector<double>& inclusive_upper,
                            std::vector<int>& num_values)
    {
        inclusive_lower = {0., 0., 1., 3., -5., -6., 0., -3.1};
        inclusive_upper = {4., 7., 4., 8.,  5., -2., 5.,  2.6};
        num_values =      {9,  8,  7,  9,   21,  13, 6,   5  }; // domain size = 37,149,840
    }
    virtual double evaluate(const std::vector<double>& parameters)
    {
        const int length = 8;
        assert(parameters.size() == size_t(length));
        double result = 11.;

        // compute result
        for(int i=0;i<length-1;i++)
        {
            result += log(1.+fabs(parameters[i]-2.));
        }
        result += pow(parameters[0]-2.,2.) * pow(parameters[1]-4.5,2.);
        result += exp(pow(parameters[2]-2.,2.)) * pow(parameters[3]-7.,2.);
        result += pow(parameters[4],3.) * exp(parameters[5]);
        result += pow(parameters[1]-1.,2.) * pow(parameters[4]-1.5,2.) * pow(parameters[6]-4.,2.);

        return result;
    }

protected:

};

PSL_TEST(BasicDiscreteObjective, testObjective)
{
    set_rand_seed();
    BasicDiscreteObjective obj;

    // test point 0
    const std::vector<double> x0 = {1.5, 2., 2.5, 6.5, 1.5, -3., 1., 2.71828};
    const double actual_obj0 = obj.evaluate(x0);
    const double expected_obj0 = 18.457587776264393;
    EXPECT_FLOAT_EQ(actual_obj0, expected_obj0);

    // test point 1
    const std::vector<double> x1 = {2., 1., 2., 7., -5., -2., 2., 3.14};
    const double actual_obj1 = obj.evaluate(x1);
    const double expected_obj1 = .256875699325349;
    EXPECT_FLOAT_EQ(actual_obj1, expected_obj1);
}

bool passed_BasicDiscreteObjective_ISandRandom()
{
    AbstractAuthority authority;

    // define objective
    BasicDiscreteObjective obj;

    // define searcher
    IterativeSelection IS_searcher(&authority);
    IS_searcher.set_objective(&obj);

    // find a minimum
    std::vector<double> IS_bestParameters;
    const double IS_foundMin = IS_searcher.find_min(IS_bestParameters);

    // equal by determinism of objective
    EXPECT_FLOAT_EQ(IS_foundMin, obj.evaluate(IS_bestParameters));

    // define another searcher
    RandomSearch R_searcher(&authority);
    R_searcher.set_objective(&obj);
    R_searcher.set_num_trials(IS_searcher.get_number_of_evaluations());

    // find a minimum
    std::vector<double> R_bestParameters;
    const double R_foundMin = R_searcher.find_min(R_bestParameters);

    // equal by determinism of objective
    EXPECT_FLOAT_EQ(R_foundMin, obj.evaluate(R_bestParameters));

    // expect random to be worse
    return (R_foundMin >= IS_foundMin);
}

PSL_TEST(BasicDiscreteObjective, testSearcherPass)
{
    set_rand_seed();
    stocastic_test(passed_BasicDiscreteObjective_ISandRandom, 50, 150, .80);
}

}
}
