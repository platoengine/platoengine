#include "PSL_GradientCheck.hpp"

#include <algorithm>
#include <cmath>
#include <math.h>
#include <iostream>

namespace PlatoSubproblemLibrary
{

GradientCheck::GradientCheck(const double& dot_tol) :
        m_dot_tol(dot_tol)
{
}

GradientCheck::~GradientCheck()
{
}

bool GradientCheck::check_pass()
{
    initialize();
    const double left = 2. * gradient_dot_step();
    const double right = objective_of_plus_step() - objective_of_minus_step();
    const double denom = std::max(std::max(fabs(left), fabs(right)), 1e-10);
    const double actual_dot_error = fabs(left - right) / denom;
    const bool pass = actual_dot_error < m_dot_tol;

    return pass;
}

}
