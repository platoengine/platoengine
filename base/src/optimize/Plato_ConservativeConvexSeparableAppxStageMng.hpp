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
 * Plato_ConservativeConvexSeparableAppxStageMng.hpp
 *
 *  Created on: Nov 4, 2017
 */

#pragma once

namespace Plato
{

template<typename ScalarType, typename OrdinalType>
class MultiVector;
template<typename ScalarType, typename OrdinalType>
class MultiVectorList;
template<typename ScalarType, typename OrdinalType>
class ConservativeConvexSeparableAppxDataMng;

/******************************************************************************//**
 * @brief Manages evaluations of criteria associated with the optimization problem
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class ConservativeConvexSeparableAppxStageMng
{
public:
    /******************************************************************************//**
     * @brief Destructor
     **********************************************************************************/
    virtual ~ConservativeConvexSeparableAppxStageMng()
    {
    }

    /******************************************************************************//**
     * @brief Sends directive to application to inform that it is save to cache
     *        app-based data.
     **********************************************************************************/
    virtual void cacheData() = 0;

    /******************************************************************************//**
     * @brief Allows application to perform continuation on app-based parameters.
     * @param [in] aControl design variables
     **********************************************************************************/
    virtual void updateProblem(const Plato::MultiVector<ScalarType, OrdinalType> & aControl) = 0;

    /******************************************************************************//**
     * @brief Update optimization problem's data, e.g. objective and constraint(s)
     *        values and gradients.
     * @param [in] aDataMng holds data associated with the optimization problem
     **********************************************************************************/
    virtual void update(Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng) = 0;

    /******************************************************************************//**
     * @brief Evaluate objective function
     * @param [in] aControl design variables
     * @return objective function evaluation
     **********************************************************************************/
    virtual ScalarType evaluateObjective(const Plato::MultiVector<ScalarType, OrdinalType> & aControl) = 0;

    /******************************************************************************//**
     * @brief Compute objective gradient
     * @param [in] aControl design variables
     * @param [out] aOutput objective function gradient
     **********************************************************************************/
    virtual void computeGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput) = 0;

    /******************************************************************************//**
     * @brief Evaluate constraint(s)
     * @param [in] aControl design variables
     * @param [out] aOutput constraint value(s)
     **********************************************************************************/
    virtual void evaluateConstraints(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                     Plato::MultiVector<ScalarType, OrdinalType> & aOutput) = 0;

    /******************************************************************************//**
     * @brief Compute constraint gradient(s)
     * @param [in] aControl design variables
     * @param [out] aOutput constraint gradient(s)
     **********************************************************************************/
    virtual void computeConstraintGradients(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                            Plato::MultiVectorList<ScalarType, OrdinalType> & aOutput) = 0;
};
// class ConservativeConvexSeparableAppxStageMng

}
// namespace Plato
