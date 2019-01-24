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
 * Plato_GradFreeRocketObjFunc.hpp
 *
 *  Created on: Jan 23, 2019
 */

#pragma once

#include "Plato_Vector.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_GradFreeCriterion.hpp"
#include "Plato_AlgebraicRocketModel.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Interface for gradient free algebraic rocket criterion
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class GradFreeRocketObjFunc : public Plato::GradFreeCriterion<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aUpperBounds upper bounds on optimization variables
     * @param [in] aRocketInputs algebraic rocket model's input data structure
     * @param [in] aChamberGeom geometry interface
    **********************************************************************************/
    GradFreeRocketObjFunc(const Plato::Vector<ScalarType, OrdinalType>& aUpperBounds,
                          const Plato::AlgebraicRocketInputs<ScalarType>& aRocketInputs,
                          const std::shared_ptr<Plato::GeometryModel<ScalarType>>& aChamberGeom) :
            mTargetThrustProfileSet(false),
            mNormTargetThrustProfile(0),
            mAlgebraicRocketModel(aRocketInputs, aChamberGeom),
            mTargetThrustProfile(),
            mNormalizationConstants(aUpperBounds.create())
    {
        this->initialize(aUpperBounds);
    }

    /******************************************************************************//**
     * @brief Desturtor
    **********************************************************************************/
    virtual ~GradFreeRocketObjFunc()
    {
    }

    /******************************************************************************//**
     * @brief Returns the norm of the target thrust profile
     * @return norm of the target thrust profile
    **********************************************************************************/
    ScalarType getNormTargetThrustProfile() const
    {
        return (mNormTargetThrustProfile);
    }

    /******************************************************************************//**
     * @brief Returns the normalization constants used to normalized the optimization variables
     * @return normalization constants
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getNormalizationConstants() const
    {
        return (*mNormalizationConstants);
    }

    /******************************************************************************//**
     * @brief Disables output to console from algebraic rocket model.
    **********************************************************************************/
    void disableOutput()
    {
        mAlgebraicRocketModel.disableOutput();
    }

    /******************************************************************************//**
     * @brief Set target thrust profile
     * @param [in] aInput target thrust profile
    **********************************************************************************/
    void setTargetThrustProfile(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        if(mTargetThrustProfile.get() == nullptr)
        {
            mTargetThrustProfile = aInput.create();
        }
        assert(mTargetThrustProfile->size() == aInput.size());
        mTargetThrustProfile->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
        mNormTargetThrustProfile = mTargetThrustProfile->dot(*mTargetThrustProfile);
        //mNormTargetValues = std::pow(mNormTargetValues, 0.5);
        mTargetThrustProfileSet = true;
    }

    /******************************************************************************//**
     * @brief Evaluates criterion at all particle positions
     * @param [in] aControls particle positions
     * @param [in] aOutput criterion values
    **********************************************************************************/
    void value(const Plato::MultiVector<ScalarType, OrdinalType> & aControls,
               Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {
        try
        {
            this->isTargetThrustProfileSet();
        }
        catch(const std::invalid_argument& tErrorMsg)
        {

            std::ostringstream tMessage;
            tMessage << "\n\n ******** ERROR IN FILE: " << __FILE__ << ", FUNCTION: " << __PRETTY_FUNCTION__
            << ", LINE: " << __LINE__ << " ******** \n\n";
            tMessage << tErrorMsg.what();
            throw std::invalid_argument(tMessage.str().c_str());
        }

        const OrdinalType tNumParticles = aControls.getNumVectors();
        assert(tNumParticles > static_cast<OrdinalType>(0));
        assert(aOutput.size() == tNumParticles);
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            aOutput[tIndex] = this->evaluate(aControls[tIndex]);
        }
    }

    /******************************************************************************//**
     * @brief Solve quasi-static algebraic rocket simulation and evaluates criterion
     * @param [in] aControls particle positions
     * @param [in] aOutput criterion values
    **********************************************************************************/
    void solve(const Plato::Vector<ScalarType, OrdinalType> & aControls,
               Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {
        this->update(aControls);
        mAlgebraicRocketModel.solve();
        std::vector<ScalarType> tTrialThrustProfile = mAlgebraicRocketModel.getThrustProfile();
        assert(aOutput.size() == tTrialThrustProfile.size());
        for(OrdinalType tIndex = 0; tIndex < aOutput.size(); tIndex++)
        {
            aOutput[tIndex] = tTrialThrustProfile[tIndex];
        }
    }

private:
    /******************************************************************************//**
     * @brief Initialize class members (i.e. native data structure)
     * @param [in] aUpperBounds particles' upper bounds
    **********************************************************************************/
    void initialize(const Plato::Vector<ScalarType, OrdinalType> & aUpperBounds)
    {
        assert(aUpperBounds.size() > static_cast<OrdinalType>(0));
        std::vector<ScalarType> tNormalizationConstants(aUpperBounds.size());
        for(OrdinalType tIndex = 0; tIndex < aUpperBounds.size(); tIndex++)
        {
            (*mNormalizationConstants)[tIndex] = aUpperBounds[tIndex];
        }
    }

    /******************************************************************************//**
     * @brief Updates geometry model and material properties
     * @param [in] aControls particle positions
    **********************************************************************************/
    void update(const Plato::Vector<ScalarType, OrdinalType> & aControls)
    {
        std::map<std::string, ScalarType> tChamberGeomParam;
        const ScalarType tRadius = aControls[0] * (*mNormalizationConstants)[0];
        tChamberGeomParam.insert(std::pair<std::string, ScalarType>("Radius", tRadius));
        tChamberGeomParam.insert(std::pair<std::string, ScalarType>("Configuration", Plato::Configuration::INITIAL));
        mAlgebraicRocketModel.updateInitialChamberGeometry(tChamberGeomParam);

        const ScalarType tRefBurnRate = aControls[1] * (*mNormalizationConstants)[1];
        std::map<std::string, ScalarType> tSimParam;
        tSimParam.insert(std::pair<std::string, ScalarType>("RefBurnRate", tRefBurnRate));
        mAlgebraicRocketModel.updateSimulation(tSimParam);
    }

    /******************************************************************************//**
     * @brief Checks that target profile has been initialized
    **********************************************************************************/
    void isTargetThrustProfileSet()
    {
        try
        {
            if(mTargetThrustProfileSet == false)
            {
                throw std::invalid_argument("\n\n ******** MESSAGE: TARGET THRUST PROFILE IS NOT SET. ABORT! ******** \n\n");
            }
        }
        catch(const std::invalid_argument & tError)
        {
            throw tError;
        }
    }

    /******************************************************************************//**
     * @brief Evaluates algebraic rocket criterion
     * @param [in] aControls particle positions
    **********************************************************************************/
    ScalarType evaluate(const Plato::Vector<ScalarType, OrdinalType> & aControls)
    {
        this->update(aControls);

        mAlgebraicRocketModel.solve();
        std::vector<ScalarType> tTrialThrustProfile = mAlgebraicRocketModel.getThrustProfile();
        assert(tTrialThrustProfile.size() == mTargetThrustProfile->size());

        ScalarType tObjFuncValue = 0;
        const OrdinalType tNumThrustEvalPoints = mTargetThrustProfile->size();
        for(OrdinalType tIndex = 0; tIndex < tNumThrustEvalPoints; tIndex++)
        {
            ScalarType tMisfit = tTrialThrustProfile[tIndex] - (*mTargetThrustProfile)[tIndex];
            tObjFuncValue += tMisfit * tMisfit;
        }
        tObjFuncValue = static_cast<ScalarType>(1.0 / (2.0 * tNumThrustEvalPoints * mNormTargetThrustProfile)) * tObjFuncValue;

        return tObjFuncValue;
    }

private:
    bool mTargetThrustProfileSet; /*!< flag : true or false */
    ScalarType mNormTargetThrustProfile; /*!< norm of target thrust profile */
    Plato::AlgebraicRocketModel<ScalarType> mAlgebraicRocketModel; /*!< algebraic rocket simulator */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mTargetThrustProfile; /*!< target thrust profile */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mNormalizationConstants; /*!< normalization constants for optimization variables */

private:
    GradFreeRocketObjFunc(const Plato::GradFreeRocketObjFunc<ScalarType, OrdinalType> & aRhs);
    Plato::GradFreeRocketObjFunc<ScalarType, OrdinalType> & operator=(const Plato::GradFreeRocketObjFunc<ScalarType, OrdinalType> & aRhs);
};
// class GradFreeRocketObjFunc

} // namespace Plato
