#pragma once

namespace PlatoSubproblemLibrary
{

class GradientCheck
{
public:
    GradientCheck(const double& dot_tol);
    virtual ~GradientCheck();

    virtual void initialize() = 0;
    virtual double gradient_dot_step() = 0;
    virtual double objective_of_minus_step() = 0;
    virtual double objective_of_plus_step() = 0;

    bool check_pass();

protected:
    double m_dot_tol;

};

}
