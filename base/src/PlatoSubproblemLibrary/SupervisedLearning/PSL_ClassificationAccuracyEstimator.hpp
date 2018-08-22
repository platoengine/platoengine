#pragma once

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class ParameterData;
class Classifier;
class ClassificationAssessor;

class ClassificationAccuracyEstimator
{
public:
    ClassificationAccuracyEstimator(AbstractAuthority* authority);
    virtual ~ClassificationAccuracyEstimator();

    virtual void estimate_accuracy(ParameterData* parameter_data, ClassificationAssessor* assessor) = 0;
protected:
    AbstractAuthority* m_authority;

};

}
