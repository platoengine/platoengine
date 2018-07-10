/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
*/

#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Interface_BasicDenseVectorOperations.hpp"
#include "PSL_GradientCheck.hpp"
#include "PSL_Random.hpp"

#include <vector>
#include <cstddef>
#include <cassert>

namespace PlatoSubproblemLibrary
{
namespace TestingGradientCheck
{

class simpleAccurateGradientCheck : public GradientCheck
{
public:
    simpleAccurateGradientCheck() :
            GradientCheck(1e-2),
            m_operations(),
            m_grad_constant(2.),
            m_weights(),
            m_center(),
            m_step()
    {
    }
    virtual ~simpleAccurateGradientCheck()
    {
    }

    virtual void initialize()
    {
        // set weights
        m_weights = { 1., 3., -4.2, .1};
        const size_t num_components = m_weights.size();

        // set center
        m_center.resize(num_components);
        uniform_rand_double(.5, 2.5, m_center);

        // set step
        m_step.resize(num_components);
        uniform_rand_double(-1e-3, 1e-3, m_step);
    }
    virtual double gradient_dot_step()
    {
        // compute gradient
        std::vector<double> gradient;
        evaluate(m_center, gradient);

        // return dot
        return m_operations.dot(m_step, gradient);
    }
    virtual double objective_of_minus_step()
    {
        // compute shift
        std::vector<double> center_minus_step = m_center;
        m_operations.axpy(-1., m_step, center_minus_step);

        // return objective
        std::vector<double> gradient;
        return evaluate(center_minus_step, gradient);
    }
    virtual double objective_of_plus_step()
    {
        // compute shift
        std::vector<double> center_plus_step = m_center;
        m_operations.axpy(1., m_step, center_plus_step);

        // return objective
        std::vector<double> gradient;
        return evaluate(center_plus_step, gradient);
    }

    double evaluate(const std::vector<double>& x, std::vector<double>& gradient)
    {
        // allocate
        const size_t num_components = m_weights.size();
        assert(num_components == x.size());
        gradient.resize(num_components);
        double result = 0.;

        // calculate
        for(size_t c=0u; c<num_components; c++)
        {
            result += m_weights[c] * x[c] * x[c];
            gradient[c] = m_grad_constant * m_weights[c] * x[c];
        }

        return result;
    }

protected:
    example::Interface_BasicDenseVectorOperations m_operations;
    double m_grad_constant;
    std::vector<double> m_weights;
    std::vector<double> m_center;
    std::vector<double> m_step;

};

class simpleInaccurateGradientCheck : public simpleAccurateGradientCheck
{
public:
    simpleInaccurateGradientCheck(double grad_constant) :
            simpleAccurateGradientCheck()
    {
        m_grad_constant = grad_constant;
    }
    virtual ~simpleInaccurateGradientCheck()
    {
    }
};

PSL_TEST(GradientCheck, simpleAccurate)
{
    set_rand_seed();
    simpleAccurateGradientCheck checker;
    EXPECT_EQ(checker.check_pass(), true);
}

PSL_TEST(GradientCheck, simpleInaccurate)
{
    set_rand_seed();
    simpleInaccurateGradientCheck checker0(1.85);
    EXPECT_EQ(checker0.check_pass(), false);

    simpleInaccurateGradientCheck checker1(-2.);
    EXPECT_EQ(checker1.check_pass(), false);
}

}
}
