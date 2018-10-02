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

/*
 * Plato_GradFreeRocketObjective.hpp
 *
 *  Created on: Aug 30, 2018
 */

#pragma once

#include <vector>
#include <memory>
#include <utility>
#include <cstddef>

#include "PSL_DiscreteObjective.hpp"
#include "Plato_AlgebraicRocketModel.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Gradient-free objective interface for quasi-steady rocket model.
 **********************************************************************************/
class GradFreeRocketObjective : public PlatoSubproblemLibrary::DiscreteObjective
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aRocketInputs struct with rocket model inputs
     * @param [in] aChamberGeom chamber's geometry model
     **********************************************************************************/
    GradFreeRocketObjective(const Plato::AlgebraicRocketInputs<double>& aRocketInputs,
                            const std::shared_ptr<Plato::GeometryModel<double>>& aChamberGeom);

    /******************************************************************************//**
     * @brief Destructor
     **********************************************************************************/
    virtual ~GradFreeRocketObjective();

    /******************************************************************************//**
     * @brief Set input parameters needed to pose optimization problem
     * @param [in] aNumEvaluationsPerDim number of function evaluations per design variable dimension
     * @param [in] aBounds lower and upper bounds for design variables
     **********************************************************************************/
    void setOptimizationInputs(const std::vector<int>& aNumEvaluationsPerDim,
                               const std::pair<std::vector<double>, std::vector<double>>& aBounds /* <lower,upper> */);

    /******************************************************************************//**
     * @brief Return thrust profile
     * @return thrust profile
     **********************************************************************************/
    std::vector<double> getThrustProfile() const;

    /******************************************************************************//**
     * @brief Get algorithm inputs needed to pose optimization problem from objective interface
     * @param [out] aLowerBounds design variables lower bounds
     * @param [out] aUpperBounds design variables upper bounds
     * @param [out] aNumEvaluationsPerDim number of function evaluations per design variable dimension
     **********************************************************************************/
    void get_domain(std::vector<double>& aLowerBounds,
                    std::vector<double>& aUpperBounds,
                    std::vector<int>& aNumEvaluationsPerDim);

    /******************************************************************************//**
     * @brief Evaluate objective function
     * @param [in] aControls design variables
     **********************************************************************************/
    double evaluate(const std::vector<double>& aControls);

private:
    /******************************************************************************//**
     * @brief Set target data and disable console output.
     **********************************************************************************/
    void initialize();

    /******************************************************************************//**
     * @brief update parameters (e.g. design variables) for simulation.
     * @param [in] aControls design variables
     **********************************************************************************/
    void update(const std::vector<double>& aControls);

private:
    double mNormTargetValues;

    std::vector<double> mTargetThrustProfile;
    std::vector<int> mNumEvaluationsPerDim;

    Plato::AlgebraicRocketModel<double> mRocketModel;
    std::pair<std::vector<double>, std::vector<double>> mBounds; /* <lower,upper> */
};
// class SimpleRocketObjectiveGradFree

}// namespace Plato
