#include <PSL_RegularHex8.hpp>

namespace PlatoSubproblemLibrary
{

double RegularHex8::interpolateScalar(const Vector& aPoint, const std::vector<double>& aScalars) const
{
    if(aScalars.size() != 8u)
        throw(std::domain_error("RegularHex8::interpolateScalar: Incorrect number of scalar values provided"));

    double tA0 = a0(aPoint,aScalars);
    double tA1 = a1(aPoint,aScalars);
    double tA2 = a2(aPoint,aScalars);
    double tA3 = a3(aPoint,aScalars);
    double tA4 = a4(aPoint,aScalars);
    double tA5 = a5(aPoint,aScalars);
    double tA6 = a6(aPoint,aScalars);
    double tA7 = a7(aPoint,aScalars);

    return tA0 + tA1*aPoint.X() + tA2*aPoint.Y() + tA3*aPoint.Z() 
         + tA4*aPoint.X()*aPoint.Y() + tA5*aPoint.X()*aPoint.Z() + tA6*aPoint.Y()*aPoint.Z()
         + tA7*aPoint.X()*aPoint.Y()*aPoint.Z();
}

double RegularHex8::a0(const Vector& aPoint, const std::vector<double>& aScalars) const
{
    std::vector<double> tTerms;
    tTerms.push_back(-aScalars[0] * mMaxCoords(0) * mMaxCoords(1) * mMaxCoords(2));
    tTerms.push_back(aScalars[4]  * mMaxCoords(0) * mMaxCoords(1) * mMinCoords(2));
    tTerms.push_back(aScalars[2]  * mMaxCoords(0) * mMinCoords(1) * mMaxCoords(2));
    tTerms.push_back(-aScalars[6] * mMaxCoords(0) * mMinCoords(1) * mMinCoords(2));
    tTerms.push_back(aScalars[1]  * mMinCoords(0) * mMaxCoords(1) * mMaxCoords(2));
    tTerms.push_back(-aScalars[5] * mMinCoords(0) * mMaxCoords(1) * mMinCoords(2));
    tTerms.push_back(-aScalars[3] * mMinCoords(0) * mMinCoords(1) * mMaxCoords(2));
    tTerms.push_back(aScalars[7]  * mMinCoords(0) * mMinCoords(1) * mMinCoords(2));

    double tNumerator = 0;

    for(auto tTerm : tTerms)
        tNumerator += tTerm;
        
    return tNumerator/-mVolume;
}

double RegularHex8::a1(const Vector& aPoint, const std::vector<double>& aScalars) const
{
    std::vector<double> tTerms;
    tTerms.push_back(aScalars[0]  * 1.0           * mMaxCoords(1) * mMaxCoords(2));
    tTerms.push_back(-aScalars[4] * 1.0           * mMaxCoords(1) * mMinCoords(2));
    tTerms.push_back(-aScalars[2] * 1.0           * mMinCoords(1) * mMaxCoords(2));
    tTerms.push_back(aScalars[6]  * 1.0           * mMinCoords(1) * mMinCoords(2));
    tTerms.push_back(-aScalars[1] * 1.0           * mMaxCoords(1) * mMaxCoords(2));
    tTerms.push_back(aScalars[5]  * 1.0           * mMaxCoords(1) * mMinCoords(2));
    tTerms.push_back(aScalars[3]  * 1.0           * mMinCoords(1) * mMaxCoords(2));
    tTerms.push_back(-aScalars[7] * 1.0           * mMinCoords(1) * mMinCoords(2));

    double tNumerator = 0;

    for(auto tTerm : tTerms)
        tNumerator += tTerm;
        
    return tNumerator/-mVolume;
}

double RegularHex8::a2(const Vector& aPoint, const std::vector<double>& aScalars) const
{
    std::vector<double> tTerms;
    tTerms.push_back(aScalars[0]  * mMaxCoords(0) * 1.0           * mMaxCoords(2));
    tTerms.push_back(-aScalars[4] * mMaxCoords(0) * 1.0           * mMinCoords(2));
    tTerms.push_back(-aScalars[2] * mMaxCoords(0) * 1.0           * mMaxCoords(2));
    tTerms.push_back(aScalars[6]  * mMaxCoords(0) * 1.0           * mMinCoords(2));
    tTerms.push_back(-aScalars[1] * mMinCoords(0) * 1.0           * mMaxCoords(2));
    tTerms.push_back(aScalars[5]  * mMinCoords(0) * 1.0           * mMinCoords(2));
    tTerms.push_back(aScalars[3]  * mMinCoords(0) * 1.0           * mMaxCoords(2));
    tTerms.push_back(-aScalars[7] * mMinCoords(0) * 1.0           * mMinCoords(2));

    double tNumerator = 0;

    for(auto tTerm : tTerms)
        tNumerator += tTerm;
        
    return tNumerator/-mVolume;
}

double RegularHex8::a3(const Vector& aPoint, const std::vector<double>& aScalars) const
{
    std::vector<double> tTerms;
    tTerms.push_back(aScalars[0]  * mMaxCoords(0) * mMaxCoords(1) * 1.0);
    tTerms.push_back(-aScalars[4] * mMaxCoords(0) * mMaxCoords(1) * 1.0);
    tTerms.push_back(-aScalars[2] * mMaxCoords(0) * mMinCoords(1) * 1.0);
    tTerms.push_back(aScalars[6]  * mMaxCoords(0) * mMinCoords(1) * 1.0);
    tTerms.push_back(-aScalars[1] * mMinCoords(0) * mMaxCoords(1) * 1.0);
    tTerms.push_back(aScalars[5]  * mMinCoords(0) * mMaxCoords(1) * 1.0);
    tTerms.push_back(aScalars[3]  * mMinCoords(0) * mMinCoords(1) * 1.0);
    tTerms.push_back(-aScalars[7] * mMinCoords(0) * mMinCoords(1) * 1.0);

    double tNumerator = 0;

    for(auto tTerm : tTerms)
        tNumerator += tTerm;
        
    return tNumerator/-mVolume;
}

double RegularHex8::a4(const Vector& aPoint, const std::vector<double>& aScalars) const
{
    std::vector<double> tTerms;
    tTerms.push_back(-aScalars[0] * mMaxCoords(2));
    tTerms.push_back(aScalars[4]  * mMinCoords(2));
    tTerms.push_back(aScalars[2]  * mMaxCoords(2));
    tTerms.push_back(-aScalars[6] * mMinCoords(2));
    tTerms.push_back(aScalars[1]  * mMaxCoords(2));
    tTerms.push_back(-aScalars[5] * mMinCoords(2));
    tTerms.push_back(-aScalars[3] * mMaxCoords(2));
    tTerms.push_back(aScalars[7]  * mMinCoords(2));

    double tNumerator = 0;

    for(auto tTerm : tTerms)
        tNumerator += tTerm;
        
    return tNumerator/-mVolume;
}

double RegularHex8::a5(const Vector& aPoint, const std::vector<double>& aScalars) const
{
    std::vector<double> tTerms;
    tTerms.push_back(-aScalars[0] * mMaxCoords(1));
    tTerms.push_back(aScalars[4]  * mMinCoords(1));
    tTerms.push_back(aScalars[2]  * mMaxCoords(1));
    tTerms.push_back(-aScalars[6] * mMinCoords(1));
    tTerms.push_back(aScalars[1]  * mMaxCoords(1));
    tTerms.push_back(-aScalars[5] * mMinCoords(1));
    tTerms.push_back(-aScalars[3] * mMaxCoords(1));
    tTerms.push_back(aScalars[7]  * mMinCoords(1));

    double tNumerator = 0;

    for(auto tTerm : tTerms)
        tNumerator += tTerm;
        
    return tNumerator/-mVolume;
}

double RegularHex8::a6(const Vector& aPoint, const std::vector<double>& aScalars) const
{
    std::vector<double> tTerms;
    tTerms.push_back(-aScalars[0] * mMaxCoords(0));
    tTerms.push_back(aScalars[4]  * mMinCoords(0));
    tTerms.push_back(aScalars[2]  * mMaxCoords(0));
    tTerms.push_back(-aScalars[6] * mMinCoords(0));
    tTerms.push_back(aScalars[1]  * mMaxCoords(0));
    tTerms.push_back(-aScalars[5] * mMinCoords(0));
    tTerms.push_back(-aScalars[3] * mMaxCoords(0));
    tTerms.push_back(aScalars[7]  * mMinCoords(0));

    double tNumerator = 0;

    for(auto tTerm : tTerms)
        tNumerator += tTerm;
        
    return tNumerator/-mVolume;
}

double RegularHex8::a7(const Vector& aPoint, const std::vector<double>& aScalars) const
{
    std::vector<double> tTerms;
    tTerms.push_back(aScalars[0]);
    tTerms.push_back(-aScalars[4]);
    tTerms.push_back(-aScalars[2]);
    tTerms.push_back(aScalars[6]);
    tTerms.push_back(-aScalars[1]);
    tTerms.push_back(aScalars[5]);
    tTerms.push_back(aScalars[3]);
    tTerms.push_back(-aScalars[7]);

    double tNumerator = 0;

    for(auto tTerm : tTerms)
        tNumerator += tTerm;
        
    return tNumerator/-mVolume;
}

}