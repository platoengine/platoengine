/*
 * Plato_SimpleRocketTest.cpp
 *
 *  Created on: Aug 27, 2018
 */

#include "gtest/gtest.h"

#define _USE_MATH_DEFINES

#include <cmath>
#include <math.h>
#include <cassert>

namespace Plato
{

/******************************************************************************//**
 * @brief Design the rocket chamber to achieve desired QoI profile.
 * Design the rocket chamber using the algebraic rockect model provided by NASA.
 *
 * Nomenclature:
 * QoI - quantity of interest
 * m - meters
 * sec - seconds
 * Pa - Pascal
 * kg - kilograms
 **********************************************************************************/
template<typename ScalarType = double>
class SimpleRocket
{
public:
    SimpleRocket() :
            mMaxItr(1000),
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
            mTolerance(1.e-8), // Pa
            mInvPrefAlpha(),
            mTimes(),
            mThrustProfile(),
            mPressureProfile()
    {
    }

    ~SimpleRocket()
    {
    }

    /******************************************************************************//**
     * @brief set maximum number of iterations for chamber's total pressure calculation.
     * @param aInput maximum number of iterations
     **********************************************************************************/
    void setMaxNumIterations(const size_t& aInput)
    {
        mMaxItr = aInput;
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
     * @brief return time steps.
     **********************************************************************************/
    std::vector<ScalarType> getTimeProfile() const
    {
        return (mTimes);
    }

    /******************************************************************************//**
     * @brief return thrust values for each time snapshot.
     **********************************************************************************/
    std::vector<ScalarType> getThrustProfile() const
    {
        return (mThrustProfile);
    }

    /******************************************************************************//**
     * @brief return pressure values for each time snapshot.
     **********************************************************************************/
    std::vector<ScalarType> getPressuresProfile() const
    {
        return (mPressureProfile);
    }

    /******************************************************************************//**
     * @brief compute thrust and pressure profiles using an algebraic model for the solid fuel rocket.
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
        ScalarType tChamberArea = static_cast<ScalarType>(2.0) * M_PI * mChamberRadius * mChamberRadius;

        bool tBurning = true;
        while(tBurning == true)
        {
            printf("time = %f,\t Thrust = %f,\t Pressure = %f\n", tTime, tThrust, tTotalPressure);

            mTimes.push_back(tTime);
            mThrustProfile.push_back(tThrust);
            mPressureProfile.push_back(tTotalPressure);
            tChamberArea = static_cast<ScalarType>(2.0) * M_PI * mChamberRadius * mChamberLength;

            tTotalPressure = this->computeTotalPressure(tChamberArea, tTotalPressure, tThroatArea);

            ScalarType tRdot = mRefBurnRate * std::pow(tTotalPressure, mAlpha) * mInvPrefAlpha;
            tThrust = static_cast<ScalarType>(269.0) * static_cast<ScalarType>(9.8) * tChamberArea * (tTotalPressure - mAmbientPressure)
                    / mCharacteristicVelocity;

            mChamberRadius += tRdot * mDeltaTime;
            tTime += mDeltaTime;

            tBurning = tTime + mTolerance < mTotalBurnTime;
        }
    }

private:
    ScalarType computeTotalPressure(const ScalarType& aChamberArea, const ScalarType& aTotalPressure, const ScalarType& aThroatArea)
    {
        bool tDone = false;
        size_t tIteration = 0;
        ScalarType tNewTotalPressure = aTotalPressure;

        while(tDone == false)
        {
            ScalarType tMyThrust = this->f(aChamberArea, tNewTotalPressure, aThroatArea);
            ScalarType tMyThrustChange = this->dfdpTotal(aChamberArea, tNewTotalPressure, aThroatArea);
            ScalarType tDeltaPressure = static_cast<ScalarType>(-1.0) * tMyThrust / tMyThrustChange;
            tNewTotalPressure += tDeltaPressure;

            tIteration += static_cast<size_t>(1);
            tDone = std::abs(tDeltaPressure) < mTolerance || tIteration > mMaxItr;
        }

        return (tNewTotalPressure);
    }

    ScalarType dfdpTotal(const ScalarType& aChamberArea, const ScalarType& aTotalPressure, const ScalarType& aThroatArea)
    {
        ScalarType tPower = mAlpha - static_cast<ScalarType>(1);
        ScalarType tValue = mPropellantDensity * aChamberArea * mRefBurnRate * mAlpha * mInvPrefAlpha
                            * std::pow(aTotalPressure, tPower)
                            - aThroatArea / mCharacteristicVelocity;
        return tValue;
    }

    ScalarType f(const ScalarType& aChamberArea, const ScalarType& aTotalPressure, const ScalarType& aThroatArea)
    {
        ScalarType tValue = mPropellantDensity * aChamberArea * mRefBurnRate * mInvPrefAlpha * std::pow(aTotalPressure, mAlpha)
                - aThroatArea * aTotalPressure / mCharacteristicVelocity;
        return tValue;
    }

private:
    size_t mMaxItr;

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
    ScalarType mTolerance; // Pa

    ScalarType mInvPrefAlpha;

    std::vector<ScalarType> mTimes;
    std::vector<ScalarType> mThrustProfile;
    std::vector<ScalarType> mPressureProfile;
};
// class SimpleRocket

}// namespace Plato

namespace PlatoTest
{

TEST(PlatoTest, SimpleRocket)
{
    Plato::SimpleRocket<double> tDriver;

    tDriver.solve();

    std::vector<double> tTimes = tDriver.getTimeProfile();
    std::vector<double> tThrustProfile = tDriver.getThrustProfile();
    std::vector<double> tPressureProfile = tDriver.getPressuresProfile();

    const double tTolerance = 1e-4;
    std::vector<double> tGoldTimes = {0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8,
                                      1.9, 2, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7,
                                      3.8, 3.9, 4, 4.1, 4.2, 4.3, 4.4, 4.5, 4.6, 4.7, 4.8, 4.9, 5, 5.1, 5.2, 5.3, 5.4, 5.5, 5.6,
                                      5.7, 5.8, 5.9, 6, 6.1, 6.2, 6.3, 6.4, 6.5, 6.6, 6.7, 6.8, 6.9, 7, 7.1, 7.2, 7.3, 7.4, 7.5,
                                      7.6, 7.7, 7.8, 7.9, 8, 8.1, 8.2, 8.3, 8.4, 8.5, 8.6, 8.7, 8.8, 8.9, 9, 9.1, 9.2, 9.3, 9.4,
                                      9.5, 9.6, 9.7, 9.8, 9.9};

    std::vector<double> tGoldThrust = {0, 1656714.377766964, 1684717.520617273, 1713123.001583093, 1741935.586049868,
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

    std::vector<double> tGoldPressure = {3.5e+06, 3189475.86890643, 3222646.886584704, 3256079.602944708, 3289775.428409753,
                                         3323735.777492982, 3357962.068799077, 3392455.72502597, 3427218.172966537,
                                         3462250.84351031, 3497555.171645161, 3533132.596459002, 3568984.561141468,
                                         3605112.512985606, 3641517.903389558, 3678202.18785824, 3715166.826005007,
                                         3752413.281553337, 3789943.022338498, 3827757.520309201, 3865858.251529277,
                                         3904246.696179328, 3942924.338558375, 3981892.667085526, 4021153.174301613,
                                         4060707.356870828, 4100556.715582388, 4140702.755352164, 4181146.985224297,
                                         4221890.918372861, 4262936.072103474, 4304283.967854919, 4345936.131200789,
                                         4387894.091851073, 4430159.383653805, 4472733.544596657, 4515618.116808556,
                                         4558814.646561285, 4602324.684271098, 4646149.784500302, 4690291.505958878,
                                         4734751.411506055, 4779531.068151912, 4824632.04705896, 4870055.923543734,
                                         4915804.277078374, 4961878.691292197, 5008280.753973287, 5055012.057070049,
                                         5102074.196692795, 5149468.773115309, 5197197.390776409, 5245261.658281502,
                                         5293663.188404156, 5342403.598087644, 5391484.508446511, 5440907.544768098,
                                         5490674.336514118, 5540786.517322188, 5591245.725007365, 5642053.601563688,
                                         5693211.793165727, 5744721.950170086, 5796585.72711697, 5848804.782731681,
                                         5901380.779926158, 5954315.385800499, 6007610.271644478, 6061267.112939054,
                                         6115287.589357896, 6169673.384768886, 6224426.187235633, 6279547.689018974,
                                         6335039.586578473, 6390903.580573932, 6447141.375866881, 6503754.681522069,
                                         6560745.210808973, 6618114.681203265, 6675864.814388307, 6733997.336256648,
                                         6792513.976911491, 6851416.470668174, 6910706.556055644, 6970385.975817952,
                                         7030456.476915687, 7090919.810527483, 7151777.73205145, 7213032.001106677,
                                         7274684.381534644, 7336736.641400733, 7399190.552995646, 7462047.892836868,
                                         7525310.441670142, 7588979.984470865, 7653058.310445594, 7717547.213033441,
                                         7782448.48990756, 7847763.942976533, 7913495.378385854};

    for(size_t tIndex = 0; tIndex < tTimes.size(); tIndex++)
    {
        EXPECT_NEAR(tTimes[tIndex], tGoldTimes[tIndex], tTolerance);
        EXPECT_NEAR(tThrustProfile[tIndex], tGoldThrust[tIndex], tTolerance);
        EXPECT_NEAR(tPressureProfile[tIndex], tGoldPressure[tIndex], tTolerance);
    }
}

} // namespace PlatoTest
