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
 * Plato_SimpleRocketObjective.hpp
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
class SimpleRocketObjective : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aRocketInputs struct with rocket model inputs
     * @param [in] aChamberGeom chamber's geometry model
     **********************************************************************************/
    SimpleRocketObjective(const Plato::AlgebraicRocketInputs<ScalarType>& aRocketInputs,
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
     * @brief Destructor
    **********************************************************************************/
    virtual ~SimpleRocketObjective()
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
     * @brief Return thrust profile
     * @return thrust profile
     **********************************************************************************/
    std::vector<ScalarType> getThrustProfile() const
    {
        return (mRocketModel.getThrustProfile());
    }

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
        mTargetThrustProfile =
        {   0, 1656714.377766964, 1684717.520617273, 1713123.001583093, 1741935.586049868,
            1771160.083875437, 1800801.349693849, 1830864.28322051, 1861353.829558637,
            1892274.979507048, 1923632.769869272, 1955432.283763989, 1987678.650936801,
            2020377.048073344, 2053532.699113719, 2087150.875568287, 2121236.896834771,
            2155796.130516737, 2190833.992743404, 2226355.948490792, 2262367.511904243,
            2298874.246622283, 2335881.766101836, 2373395.733944806, 2411421.864226017,
            2449965.921822503, 2489033.722744186, 2528631.134465915, 2568764.076260844,
            2609438.519535244, 2650660.488164633, 2692436.058831303, 2734771.361363255,
            2777672.579074459, 2821145.949106557, 2865197.762771913, 2909834.365898075,
            2955062.159173611, 3000887.598495364, 3047317.195317072, 3094357.516999425,
            3142015.18716148, 3190296.886033527, 3239209.350811319, 3288759.376011737,
            3338953.813829865, 3389799.574497465, 3441303.626642879, 3493472.997652346,
            3546314.774032734, 3599836.101775718, 3654044.186723352, 3708946.294935087,
            3764549.753056224, 3820861.948687783, 3877890.330757833, 3935642.409894215,
            3994125.758798767, 4053348.012622938, 4113316.869344868, 4174040.090147917,
            4235525.499800648, 4297780.987038235, 4360814.504945371, 4424634.071340578,
            4489247.76916203, 4554663.746854796, 4620890.218759571, 4687935.465502855,
            4755807.834388626, 4824515.739791448, 4894067.663551098, 4964472.155368621,
            5035737.83320389, 5107873.383674653, 5180887.562457044, 5254789.194687578,
            5329587.175366664, 5405290.469763565, 5481908.11382287, 5559449.214572486,
            5637922.950533082, 5717338.572129052, 5797705.402100981, 5879032.835919643,
            5961330.342201422, 6044607.46312535, 6128873.814851565, 6214139.087941348,
            6300413.047778608, 6387705.534992979, 6476026.465884338, 6565385.832848894,
            6655793.704806847, 6747260.227631442, 6839795.624579719, 6933410.196724654,
            7028114.32338894, 7123918.462580209, 7220833.151427887};

        const ScalarType tBaseValue = 0;
        mNormTargetValues = std::inner_product(mTargetThrustProfile.begin(), mTargetThrustProfile.end(), mTargetThrustProfile.begin(), tBaseValue);
        mRocketModel.disableOutput();
    }

    /******************************************************************************//**
     * @brief update parameters (e.g. design variables) for simulation.
     * @param [in] aControl design variables
     **********************************************************************************/
    void update(const Plato::MultiVector<ScalarType, OrdinalType>& aControl)
    {
        this->checkSize();

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

private:
    OrdinalType mNumFuncEvaluations;

    ScalarType mEpsilon;
    ScalarType mNormTargetValues;
    Plato::AlgebraicRocketModel<ScalarType> mRocketModel;

    std::vector<ScalarType> mTargetThrustProfile;
    std::vector<ScalarType> mCachedThrustProfile;
    std::vector<ScalarType> mNormalizationConstants;
};
// class SimpleRocketObjective

} // namespace Plato
