#pragma once

#include "PSL_ClassificationAccuracyEstimator.hpp"

#include <string>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class ParameterData;
class ClassificationAssessor;

class SplitTrainTestEstimator : public ClassificationAccuracyEstimator
{
public:
    SplitTrainTestEstimator(AbstractAuthority* authority);
    virtual ~SplitTrainTestEstimator();

    void initialize(const std::string& archive_filename, const bool& delete_archive, const double& testing_holdout);
    virtual void estimate_accuracy(ParameterData* parameter_data, ClassificationAssessor* assessor);
private:
    std::string m_archive_filename;
    bool m_delete_archive;
    double m_testing_holdout;

};

}

