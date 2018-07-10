/*
 * Plato_Types.hpp
 *
 *  Created on: Oct 21, 2017
 */

#ifndef Plato_TYPES_HPP_
#define Plato_TYPES_HPP_

#include <sstream>

namespace Plato
{

struct algorithm
{
    enum stop_t
    {
        NaN_NORM_TRIAL_STEP = 1,
        NaN_NORM_GRADIENT = 2,
        NORM_GRADIENT = 3,
        NORM_STEP = 4,
        OBJECTIVE_STAGNATION = 5,
        MAX_NUMBER_ITERATIONS = 6,
        OPTIMALITY_AND_FEASIBILITY = 7,
        ACTUAL_REDUCTION_TOLERANCE = 8,
        CONTROL_STAGNATION = 9,
        STATIONARITY_MEASURE = 10,
        SMALL_TRUST_REGION_RADIUS = 11,
        SMALL_PENALTY_PARAMETER = 12,
        NaN_OBJECTIVE_GRADIENT = 13,
        NaN_OBJECTIVE_STAGNATION = 14,
        NaN_CONTROL_STAGNATION = 15,
        NaN_STATIONARITY_MEASURE = 16,
        NaN_ACTUAL_REDUCTION = 17,
        NOT_CONVERGED = 18
    };
};

inline void getStopCriterion(const Plato::algorithm::stop_t & aStopCriterion, std::ostringstream & aOutput)
{
    switch(aStopCriterion)
    {
        case Plato::algorithm::stop_t::STATIONARITY_MEASURE:
        {
            aOutput << "****** Optimization stopping due to stationary measure being met. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::ACTUAL_REDUCTION_TOLERANCE:
        {
            aOutput << "****** Optimization stopping due to actual reduction tolerance being met. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::OBJECTIVE_STAGNATION:
        {
            aOutput << "****** Optimization stopping due to objective stagnation. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::SMALL_TRUST_REGION_RADIUS:
        {
            aOutput << "****** Optimization stopping due to small trust region radius. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::CONTROL_STAGNATION:
        {
            aOutput << "****** Optimization stopping due to control (i.e. design variable) stagnation. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::MAX_NUMBER_ITERATIONS:
        {
            aOutput << "****** Optimization stopping due to exceeding maximum number of iterations. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NaN_NORM_TRIAL_STEP:
        {
            aOutput << "****** Optimization stopping due to NaN norm of trial step vector. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NaN_NORM_GRADIENT:
        {
            aOutput << "****** Optimization stopping due to NaN norm of gradient vector. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NORM_GRADIENT:
        {
            aOutput << "****** Optimization stopping due to norm of gradient tolerance being met. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NORM_STEP:
        {
            aOutput << "****** Optimization stopping due to norm of trial step tolerance being met. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::OPTIMALITY_AND_FEASIBILITY:
        {
            aOutput << "****** Optimization stopping due to optimality and feasibility tolerance being met. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::SMALL_PENALTY_PARAMETER:
        {
            aOutput << "****** Optimization stopping due to small penalty parameter in augmented Lagrangian being met. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NaN_OBJECTIVE_GRADIENT:
        {
            aOutput << "****** Optimization stopping due to NaN norm of objective gradient vector. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NaN_OBJECTIVE_STAGNATION:
        {
            aOutput << "****** Optimization stopping due to NaN objective stagnation. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NaN_CONTROL_STAGNATION:
        {
            aOutput << "****** Optimization stopping due to NaN control stagnation. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NaN_STATIONARITY_MEASURE:
        {
            aOutput << "****** Optimization stopping due to NaN stationary measure. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NaN_ACTUAL_REDUCTION:
        {
            aOutput << "****** Optimization stopping due to NaN actual reduction. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NOT_CONVERGED:
        {
            aOutput << "****** Optimization algorithm did not converge. ******\n\n";
            break;
        }
    }
}

} // namespace Plato

#endif /* Plato_TYPES_HPP_ */
