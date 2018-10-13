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
 * Plato_AlgebraicRocketModel.hpp
 *
 *  Created on: Aug 29, 2018
 */

#pragma once

#define _USE_MATH_DEFINES

#include <map>
#include <cmath>
#include <math.h>
#include <cstdio>
#include <memory>
#include <cassert>
#include <cstddef>

#include "Plato_Cylinder.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Data structure for rocket problem input parameters.
**********************************************************************************/
template<typename ScalarType = double>
struct AlgebraicRocketInputs
{
    size_t mMaxNumNewtonItr;

    ScalarType mAlpha;
    ScalarType mDeltaTime;                // seconds
    ScalarType mRefBurnRate;              // meters/seconds
    ScalarType mRefPressure;              // Pascal
    ScalarType mTotalBurnTime;            // seconds
    ScalarType mChamberRadius;            // meters
    ScalarType mChamberLength;            // meters
    ScalarType mThroatDiameter;           // meters
    ScalarType mNewtonTolerance;          // Pascal
    ScalarType mAmbientPressure;          // Pascal
    ScalarType mPropellantDensity;        // kilogram/meter^3
    ScalarType mCharacteristicVelocity;   // meters/seconds

    /******************************************************************************//**
     * @brief Default constructor
    **********************************************************************************/
    AlgebraicRocketInputs() :
            mMaxNumNewtonItr(1000),
            mAlpha(0.38),
            mDeltaTime(0.1),
            mRefBurnRate(0.005),
            mRefPressure(3.5e6),
            mTotalBurnTime(10),
            mChamberRadius(0.075),
            mChamberLength(0.65),
            mThroatDiameter(0.04),
            mNewtonTolerance(1.e-8),
            mAmbientPressure(101.325),
            mPropellantDensity(1744),
            mCharacteristicVelocity(1554.5)
    {
    }
};
// struct AlgebraicRocketInputs

/******************************************************************************//**
 * @brief Design the rocket chamber to achieve desired QoI profile.
 *
 * Nomenclature:
 * QoI - quantity of interest
 * m - meters
 * sec - seconds
 * Pa - Pascal
 * kg - kilograms
 **********************************************************************************/
template<typename ScalarType = double>
class AlgebraicRocketModel
{
public:
    /******************************************************************************//**
     * @brief Default constructor
    **********************************************************************************/
    AlgebraicRocketModel() :
            mPrint(true),
            mNumNewtonItr(0),
            mMaxNumNewtonItr(1000),
            mChamberLength(0.65), // m
            mChamberRadius(0.075), // m
            mRefBurnRate(0.005), // m/sec
            mRefPressure(3.5e6), // Pa
            mAlpha(0.38),
            mThroatDiameter(0.04), // m
            mCharacteristicVelocity(1554.5), // m/sec
            mPropellantDensity(1744), // kg/m^3
            mAmbientPressure(101.325), // Pa
            mDeltaTime(0.1), // sec
            mTotalBurnTime(10), // sec
            mNewtonTolerance(1.e-8), // Pa
            mInvPrefAlpha(),
            mTimes(),
            mThrustProfile(),
            mPressureProfile(),
            mChamberGeomModel(std::make_shared<Plato::Cylinder<ScalarType>>(mChamberRadius, mChamberLength))
    {
    }

    /******************************************************************************//**
     * @brief Constructor
     * @param aInputs input parameters for simulation
     * @param aGeomModel geometry model used for the rocket chamber
     **********************************************************************************/
    explicit AlgebraicRocketModel(const Plato::AlgebraicRocketInputs<ScalarType>& aInputs,
                                  const std::shared_ptr<Plato::GeometryModel<ScalarType>>& aChamberGeomModel) :
            mPrint(true),
            mNumNewtonItr(0),
            mMaxNumNewtonItr(aInputs.mMaxNumNewtonItr),
            mChamberLength(aInputs.mChamberLength), // m
            mChamberRadius(aInputs.mChamberRadius), // m
            mRefBurnRate(aInputs.mRefBurnRate), // m/sec
            mRefPressure(aInputs.mRefPressure), // Pa
            mAlpha(aInputs.mAlpha),
            mThroatDiameter(aInputs.mThroatDiameter), // m
            mCharacteristicVelocity(aInputs.mCharacteristicVelocity), // m/sec
            mPropellantDensity(aInputs.mPropellantDensity), // kg/m^3
            mAmbientPressure(aInputs.mAmbientPressure), // Pa
            mDeltaTime(aInputs.mDeltaTime), // sec
            mTotalBurnTime(aInputs.mTotalBurnTime), // sec
            mNewtonTolerance(aInputs.mNewtonTolerance), // Pa
            mInvPrefAlpha(),
            mTimes(),
            mThrustProfile(),
            mPressureProfile(),
            mChamberGeomModel(aChamberGeomModel)
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~AlgebraicRocketModel()
    {
    }

    /******************************************************************************//**
     * @brief Disables output to console.
    **********************************************************************************/
    void disableOutput()
    {
        mPrint = false;
    }

    /******************************************************************************//**
     * @brief set maximum number of iterations for chamber's total pressure calculation.
     * @param aInput maximum number of iterations
     **********************************************************************************/
    void setMaxNumIterations(const size_t& aInput)
    {
        mMaxNumNewtonItr = aInput;
    }

    /******************************************************************************//**
     * @brief set chamber's length.
     * @param aInput chamber's length
     **********************************************************************************/
    void setChamberLength(const ScalarType& aInput)
    {
        mChamberLength = aInput;
    }

    /******************************************************************************//**
     * @brief set propellant's reference burn rate at a reference pressure.
     * @param aInput propellant's reference burn rate
     **********************************************************************************/
    void setRefBurnRate(const ScalarType& aInput)
    {
        mRefBurnRate = aInput;
    }

    /******************************************************************************//**
     * @brief set propellant's reference pressure.
     * @param aInput propellant's reference pressure
     **********************************************************************************/
    void setRefPressure(const ScalarType& aInput)
    {
        mRefPressure = aInput;
    }

    /******************************************************************************//**
     * @brief set exponent in burn rate equation.
     * @param aInput burn rate exponent
     **********************************************************************************/
    void setBurnRateExponent(const ScalarType& aInput)
    {
        mAlpha = aInput;
    }

    /******************************************************************************//**
     * @brief set throat diameter.
     * @param aInput throat diameter
     **********************************************************************************/
    void setThroatDiameter(const ScalarType& aInput)
    {
        mThroatDiameter = aInput;
    }

    /******************************************************************************//**
     * @brief set characteristic velocity.
     * @param aInput characteristic velocity
     **********************************************************************************/
    void setCharacteristicVelocity(const ScalarType& aInput)
    {
        mCharacteristicVelocity = aInput;
    }

    /******************************************************************************//**
     * @brief set propellant's density.
     * @param aInput propellant's density
     **********************************************************************************/
    void setPropellantDensity(const ScalarType& aInput)
    {
        mPropellantDensity = aInput;
    }

    /******************************************************************************//**
     * @brief set ambient pressure.
     * @param aInput ambient pressure
     **********************************************************************************/
    void setAmbientPressure(const ScalarType& aInput)
    {
        mAmbientPressure = aInput;
    }

    /******************************************************************************//**
     * @brief set burn time step.
     * @param aInput time step
     **********************************************************************************/
    void setBurnTimeStep(const ScalarType& aInput)
    {
        mDeltaTime = aInput;
    }

    /******************************************************************************//**
     * @brief set total burn time.
     * @param aInput total burn time
     **********************************************************************************/
    void setTotalBurnTime(const ScalarType& aInput)
    {
        mTotalBurnTime = aInput;
    }

    /******************************************************************************//**
     * @brief returns time steps.
     **********************************************************************************/
    std::vector<ScalarType> getTimeProfile() const
    {
        return (mTimes);
    }

    /******************************************************************************//**
     * @brief returns thrust values for each time snapshot.
     **********************************************************************************/
    std::vector<ScalarType> getThrustProfile() const
    {
        return (mThrustProfile);
    }

    /******************************************************************************//**
     * @brief returns pressure values for each time snapshot.
     **********************************************************************************/
    std::vector<ScalarType> getPressuresProfile() const
    {
        return (mPressureProfile);
    }

    /******************************************************************************//**
     * @brief update parameters associated with the simulation.
     * @param aParam simulation parameters
     **********************************************************************************/
    void updateSimulation(const std::map<std::string, ScalarType>& aParam)
    {
        // set simulation-specific data
        mRefBurnRate = aParam.find("RefBurnRate")->second;
    }

    /******************************************************************************//**
     * @brief update chambers geometry.
     * @param aParam parameters associated with the chamber's geometry
     **********************************************************************************/
    void updateInitialChamberGeometry(const std::map<std::string, ScalarType>& aParam)
    {
        mChamberGeomModel->update(aParam);
    }

    /******************************************************************************//**
     * @brief compute thrust and pressure profiles given a simple algebraic model for a rocket.
     **********************************************************************************/
    void solve()
    {
        mTimes.clear();
        assert(mTimes.empty() == true);
        mThrustProfile.clear();
        assert(mThrustProfile.empty() == true);
        mPressureProfile.clear();
        assert(mPressureProfile.empty() == true);

        // circular chamber parameterization
        mInvPrefAlpha = static_cast<ScalarType>(1.0) / std::pow(mRefPressure, mAlpha);
        ScalarType tThroatArea = M_PI * mThroatDiameter * mThroatDiameter / static_cast<ScalarType>(4.0);

        // initialize variables
        ScalarType tTime = 0.0;
        ScalarType tThrust = 0.0;
        ScalarType tTotalPressure = mRefPressure; // initial guess

        // initialize geometry map
        std::map<std::string, ScalarType> tChamberGeom;
        tChamberGeom.insert(std::pair<std::string, ScalarType>("BurnRate", 0.0));
        tChamberGeom.insert(std::pair<std::string, ScalarType>("DeltaTime", mDeltaTime));
        tChamberGeom.insert(std::pair<std::string, ScalarType>("Configuration", Plato::Configuration::DYNAMIC));

        bool tBurning = true;
        while(tBurning == true)
        {
            if(mPrint == true)
            {
                printf("Time = %f,\t Thrust = %f,\t Pressure = %f\n", tTime, tThrust, tTotalPressure);
            }

            mTimes.push_back(tTime);
            mThrustProfile.push_back(tThrust);
            mPressureProfile.push_back(tTotalPressure);
            ScalarType tChamberArea = mChamberGeomModel->area();

            tTotalPressure = this->newton(tChamberArea, tTotalPressure, tThroatArea);

            tThrust = static_cast<ScalarType>(269.0) * static_cast<ScalarType>(9.8)
                    * tChamberArea * (tTotalPressure - mAmbientPressure)
                    / mCharacteristicVelocity;

            ScalarType tRdot = mRefBurnRate * std::pow(tTotalPressure, mAlpha) * mInvPrefAlpha;
            tChamberGeom.find("BurnRate")->second = tRdot;
            mChamberGeomModel->update(tChamberGeom);
            tTime += mDeltaTime;

            tBurning = tTime + mNewtonTolerance < mTotalBurnTime;
        }
    }

private:
    /******************************************************************************//**
     * @brief Newton solver.
     * @param aChamberArea current chamber area
     * @param aTotalPressure total pressure at current time step
     * @param aThroatArea current throat area
     **********************************************************************************/
    ScalarType newton(const ScalarType& aChamberArea, const ScalarType& aTotalPressure, const ScalarType& aThroatArea)
    {
        bool tDone = false;
        ScalarType tNewTotalPressure = aTotalPressure;

        mNumNewtonItr = 0;
        while(tDone == false)
        {
            ScalarType tMyResidualEvaluation = this->residual(aChamberArea, tNewTotalPressure, aThroatArea);
            ScalarType tMyJacobianEvaluation = this->jacobian(aChamberArea, tNewTotalPressure, aThroatArea);
            ScalarType tDeltaPressure = static_cast<ScalarType>(-1.0) * tMyResidualEvaluation / tMyJacobianEvaluation;
            tNewTotalPressure += tDeltaPressure;

            mNumNewtonItr += static_cast<size_t>(1);
            tDone = std::abs(tDeltaPressure) < mNewtonTolerance || mNumNewtonItr > mMaxNumNewtonItr;
        }

        return (tNewTotalPressure);
    }

    /******************************************************************************//**
     * @brief Jacobian evaluation.
     * @param aChamberArea current chamber area
     * @param aTotalPressure total pressure at current time step
     * @param aThroatArea current throat area
     **********************************************************************************/
    ScalarType jacobian(const ScalarType& aChamberArea, const ScalarType& aTotalPressure, const ScalarType& aThroatArea)
    {
        ScalarType tPower = mAlpha - static_cast<ScalarType>(1);
        ScalarType tValue = mPropellantDensity * aChamberArea * mRefBurnRate * mAlpha * mInvPrefAlpha
                            * std::pow(aTotalPressure, tPower)
                            - aThroatArea / mCharacteristicVelocity;
        return tValue;
    }

    /******************************************************************************//**
     * @brief Residual evaluation.
     * @param aChamberArea current chamber area
     * @param aTotalPressure total pressure at current time step
     * @param aThroatArea current throat area
     **********************************************************************************/
    ScalarType residual(const ScalarType& aChamberArea, const ScalarType& aTotalPressure, const ScalarType& aThroatArea)
    {
        ScalarType tValue = mPropellantDensity * aChamberArea * mRefBurnRate * mInvPrefAlpha * std::pow(aTotalPressure, mAlpha)
                - aThroatArea * aTotalPressure / mCharacteristicVelocity;
        return tValue;
    }

private:
    bool mPrint;
    size_t mNumNewtonItr;
    size_t mMaxNumNewtonItr;

    ScalarType mChamberLength; // m
    ScalarType mChamberRadius; // m
    ScalarType mRefBurnRate; // m/sec
    ScalarType mRefPressure; // Pa
    ScalarType mAlpha;
    ScalarType mThroatDiameter; // m
    ScalarType mCharacteristicVelocity; // m/sec
    ScalarType mPropellantDensity; // kg/m^3
    ScalarType mAmbientPressure; // Pa
    ScalarType mDeltaTime; // sec
    ScalarType mTotalBurnTime; // sec
    ScalarType mNewtonTolerance; // Pa

    ScalarType mInvPrefAlpha;

    std::vector<ScalarType> mTimes;
    std::vector<ScalarType> mThrustProfile;
    std::vector<ScalarType> mPressureProfile;

    std::shared_ptr<Plato::GeometryModel<ScalarType>> mChamberGeomModel;
};
// class AlgebraicRocketModel

} //namespace Plato
