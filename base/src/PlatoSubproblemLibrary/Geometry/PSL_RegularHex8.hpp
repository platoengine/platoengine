#pragma once

#include <PSL_Vector.hpp>

namespace PlatoSubproblemLibrary
{

class RegularHex8
{
public:
    RegularHex8(const Vector& aMinCoords,
                const Vector& aMaxCoords) :
                mMinCoords(aMinCoords),
                mMaxCoords(aMaxCoords)
            {
                for(size_t i = 0; i < 3; ++i)
                {
                    if(mMaxCoords(i) < mMinCoords(i))
                        throw(std::domain_error("RegularHex8: Minimum provided coordinate greater than maximum provided coordinate"));
                }

                double tLength0 =  mMaxCoords(0) - mMinCoords(0);
                double tLength1 =  mMaxCoords(1) - mMinCoords(1);
                double tLength2 =  mMaxCoords(2) - mMinCoords(2);

                mVolume = tLength0*tLength1*tLength2;
            }

    ~RegularHex8(){}

    double interpolateScalar(const Vector& aPoint, const std::vector<double>& aScalars) const;
    // interpolateScalar associates scalars to Hex nodes via the following ordering
    // (i  ,j  ,k  )
    // (i+1,j  ,k  )
    // (i  ,j+1,k  )
    // (i+1,j+1,k  )
    // (i  ,j  ,k+1)
    // (i+1,j  ,k+1)
    // (i  ,j+1,k+1)
    // (i+1,j+1,k+1)

private:

    double a0(const std::vector<double>& aScalars) const;
    double a1(const std::vector<double>& aScalars) const;
    double a2(const std::vector<double>& aScalars) const;
    double a3(const std::vector<double>& aScalars) const;
    double a4(const std::vector<double>& aScalars) const;
    double a5(const std::vector<double>& aScalars) const;
    double a6(const std::vector<double>& aScalars) const;
    double a7(const std::vector<double>& aScalars) const;

    const Vector& mMinCoords;
    const Vector& mMaxCoords;

    double mVolume;
};

}
