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
 * Plato_GradBasedRocketObjective.hpp
 *
 *  Created on: Aug 31, 2018
 */

#pragma once

#include <map>
#include <string>
#include <vector>
#include <memory>
#include <sstream>
#include <utility>
#include <cassert>
#include <cstddef>
#include <numeric>
#include <stdexcept>

#include "Plato_AlgebraicRocketModel.hpp"

#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class GradBasedRocketObjective : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aRocketInputs struct with rocket model inputs
     * @param [in] aChamberGeom chamber's geometry model
     **********************************************************************************/
    GradBasedRocketObjective(const Plato::AlgebraicRocketInputs<ScalarType>& aRocketInputs,
                             const std::shared_ptr<Plato::GeometryModel<ScalarType>>& aChamberGeom) :
            mNumFuncEvaluations(0),
            mEpsilon(1e-4),
            mNormTargetValues(0),
            mRocketModel(aRocketInputs, aChamberGeom),
            mTargetThrustProfile(),
            mCachedThrustProfile(),
            mNormalizationConstants()
    {
        this->initialize();
    }

    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aTargetThrustProfile target thrust profile
     * @param [in] aRocketInputs struct with rocket model inputs
     * @param [in] aChamberGeom chamber's geometry model
     **********************************************************************************/
    GradBasedRocketObjective(const std::vector<ScalarType> & aTargetThrustProfile,
                             const Plato::AlgebraicRocketInputs<ScalarType>& aRocketInputs,
                             const std::shared_ptr<Plato::GeometryModel<ScalarType>>& aChamberGeom) :
            mNumFuncEvaluations(0),
            mEpsilon(1e-4),
            mNormTargetValues(std::inner_product(aTargetThrustProfile.begin(), aTargetThrustProfile.end(), aTargetThrustProfile.begin(), static_cast<ScalarType>(0.0))),
            mRocketModel(aRocketInputs, aChamberGeom),
            mTargetThrustProfile(aTargetThrustProfile),
            mCachedThrustProfile(),
            mNormalizationConstants()
    {
        this->initialize();
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~GradBasedRocketObjective()
    {
    }

    /******************************************************************************//**
     * @brief Set normalization constant for each design variable.
     * @param [in] aNormalizationConstants normalization constant for each design variable
     **********************************************************************************/
    void setNormalizationConstants(const std::vector<ScalarType>& aNormalizationConstants)
    {
        mNormalizationConstants = aNormalizationConstants;
    }

    /******************************************************************************//**
     * @brief Set perturbation parameter used for finite difference gradient approximation.
     * @param [in] aInput perturbation parameter
     **********************************************************************************/
    void setPerturbationParameter(const ScalarType& aInput)
    {
        mEpsilon = aInput;
    }

    /******************************************************************************//**
     * @brief Set target thrust profile
     * @param [in] aInput target thrust profile
     **********************************************************************************/
    void setTargetThrustProfile(const std::vector<ScalarType> & aInput)
    {
        mTargetThrustProfile = aInput;
        const ScalarType tBaseValue = 0;
        mNormTargetValues = std::inner_product(mTargetThrustProfile.begin(), mTargetThrustProfile.end(), mTargetThrustProfile.begin(), tBaseValue);
    }

    /******************************************************************************//**
     * @brief Return thrust profile
     * @return thrust profile
     **********************************************************************************/
    std::vector<ScalarType> getThrustProfile() const
    {
        return (mRocketModel.getThrustProfile());
    }

    /******************************************************************************//**
     * @brief Return number of objective function evaluations
     * @return number of objective function evaluations
     **********************************************************************************/
    OrdinalType getNumFunctionEvaluations() const
    {
        return (mNumFuncEvaluations);
    }

    /******************************************************************************//**
     * @brief Cache current thrust profile solution
     **********************************************************************************/
    void cacheData()
    {
        mCachedThrustProfile = mRocketModel.getThrustProfile();
    }

    /******************************************************************************//**
     * @brief Return criterion evaluation
     * @param [in] aControl controls (i.e. design variables)
     * @return criterion evaluation
     **********************************************************************************/
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        mNumFuncEvaluations++;

        this->update(aControl);

        mRocketModel.solve();
        std::vector<ScalarType> tSimulationThrustProfile = mRocketModel.getThrustProfile();
        assert(tSimulationThrustProfile.size() == mTargetThrustProfile.size());

        ScalarType tObjectiveValue = 0;
        for(OrdinalType tIndex = 0; tIndex < mTargetThrustProfile.size(); tIndex++)
        {
            ScalarType tDeltaThrust = tSimulationThrustProfile[tIndex] - mTargetThrustProfile[tIndex];
            tObjectiveValue += tDeltaThrust * tDeltaThrust;
        }
        const ScalarType tNumElements = mTargetThrustProfile.size();
        const ScalarType tDenominator = static_cast<ScalarType>(2.0) * tNumElements * mNormTargetValues;
        tObjectiveValue = (static_cast<ScalarType>(1.0) / tDenominator) * tObjectiveValue;

        return tObjectiveValue;
    }

    /******************************************************************************//**
     * @brief Return gradient of criterion with respect to the design variables
     * @param [in] aControl controls (i.e. design variables)
     * @param [in,out] aOutput gradient
     **********************************************************************************/
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        const OrdinalType tVECTOR_INDEX = 0;
        const OrdinalType tNumControls = aControl[tVECTOR_INDEX].size();
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tControlCopy = aControl.create();
        Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *tControlCopy);

        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            // modify base value
            (*tControlCopy)(tVECTOR_INDEX, tIndex) = aControl(tVECTOR_INDEX, tIndex)
                    +  (aControl(tVECTOR_INDEX, tIndex) * mEpsilon);
            // evaluate criterion with modified value
            ScalarType tForwardCriterionValue = this->value(*tControlCopy);
            // reset base value
            (*tControlCopy)(tVECTOR_INDEX, tIndex) = aControl(tVECTOR_INDEX, tIndex);

            // modify base value
            (*tControlCopy)(tVECTOR_INDEX, tIndex) = aControl(tVECTOR_INDEX, tIndex)
                    - (aControl(tVECTOR_INDEX, tIndex) * mEpsilon);
            // evaluate criterion with modified value
            ScalarType tBackwardCriterionValue = this->value(*tControlCopy);
            // reset base value
            (*tControlCopy)(tVECTOR_INDEX, tIndex) = aControl(tVECTOR_INDEX, tIndex);

            // central difference gradient approximation
            aOutput(tVECTOR_INDEX, tIndex) = (tForwardCriterionValue - tBackwardCriterionValue)
                    / (static_cast<ScalarType>(2) * mEpsilon);
        }
    }

    /******************************************************************************//**
     * @brief Compute application of vector to Hessian operator
     * @param [in] aControl controls (i.e. design variables)
     * @param [in] aVector descent direction
     * @param [in,out] application of vector to Hessian operator (Identity used for this example)
     **********************************************************************************/
    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                 const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        Plato::update(static_cast<ScalarType>(1), aVector, static_cast<ScalarType>(0), aOutput);
    }

private:
    /******************************************************************************//**
     * @brief Set target data and disable console output.
     **********************************************************************************/
    void initialize()
    {
        mRocketModel.disableOutput();
    }

    /******************************************************************************//**
     * @brief update parameters (e.g. design variables) for simulation.
     * @param [in] aControl design variables
     **********************************************************************************/
    void update(const Plato::MultiVector<ScalarType, OrdinalType>& aControl)
    {
        this->checkSize();
        this->checkTargetThrustProfile();

        const OrdinalType tVECTOR_INDEX = 0;
        std::map<std::string, ScalarType> tChamberGeomParam;
        const ScalarType tRadius = aControl(tVECTOR_INDEX, 0) * mNormalizationConstants[0];
        tChamberGeomParam.insert(std::pair<std::string, ScalarType>("Radius", tRadius));
        tChamberGeomParam.insert(std::pair<std::string, ScalarType>("Configuration", Plato::Configuration::INITIAL));
        mRocketModel.updateInitialChamberGeometry(tChamberGeomParam);

        const ScalarType tRefBurnRate = aControl(tVECTOR_INDEX, 1) * mNormalizationConstants[1];
        std::map<std::string, ScalarType> tSimParam;
        tSimParam.insert(std::pair<std::string, ScalarType>("RefBurnRate", tRefBurnRate));
        mRocketModel.updateSimulation(tSimParam);

    }

    /******************************************************************************//**
     * @brief check if normalization constants were allocated.
     **********************************************************************************/
    void checkSize()
    {
        try
        {
            if(mNormalizationConstants.empty() == true)
            {
                std::ostringstream tErrorMsg;
                tErrorMsg << "\n\n ******* PLATO: ERROR IN FILE: " << __FILE__ << ", FUNCTION: " << __PRETTY_FUNCTION__ << ", LINE: "
                          << __LINE__ << ", MESSAGE: VECTOR WITH NORMALIZATION CONSTANTS WAS NOT SET. ABORT! ******\n\n";
                throw std::invalid_argument(tErrorMsg.str().c_str());
            }
        }
        catch(const std::invalid_argument& tErrorMsg)
        {
            std::cout << tErrorMsg.what() << std::flush;
            throw tErrorMsg;
        }
    }

    /******************************************************************************//**
     * @brief check if target thrust profile was initialized.
     **********************************************************************************/
    void checkTargetThrustProfile()
    {
        try
        {
            if(mNormalizationConstants.empty() == true)
            {
                std::ostringstream tErrorMsg;
                tErrorMsg << "\n\n ******* PLATO: ERROR IN FILE: " << __FILE__ << ", FUNCTION: " << __PRETTY_FUNCTION__ << ", LINE: "
                          << __LINE__ << ", MESSAGE: VECTOR WITH NORMALIZATION CONSTANTS WAS NOT SET. ABORT! ******\n\n";
                throw std::invalid_argument(tErrorMsg.str().c_str());
            }
        }
        catch(const std::invalid_argument& tErrorMsg)
        {
            std::cout << tErrorMsg.what() << std::flush;
            throw tErrorMsg;
        }
    }

private:
    OrdinalType mNumFuncEvaluations;

    ScalarType mEpsilon;
    ScalarType mNormTargetValues;
    Plato::AlgebraicRocketModel<ScalarType> mRocketModel;

    std::vector<ScalarType> mTargetThrustProfile;
    std::vector<ScalarType> mCachedThrustProfile;
    std::vector<ScalarType> mNormalizationConstants;
};
// class GradBasedRocketObjective

} // namespace Plato
