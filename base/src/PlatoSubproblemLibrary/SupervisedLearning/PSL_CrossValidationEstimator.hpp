#pragma once

#include "PSL_ClassificationAccuracyEstimator.hpp"

#include <string>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class ParameterData;
class Classifier;
class ClassificationAssessor;

class CrossValidationEstimator : public ClassificationAccuracyEstimator
{
public:
    CrossValidationEstimator(AbstractAuthority* authority);
    virtual ~CrossValidationEstimator();

    void initialize(const std::string& archive_filename, const bool& delete_archive, const int& num_folds);
    virtual void estimate_accuracy(ParameterData* parameter_data, ClassificationAssessor* assessor);
private:
    std::string m_archive_filename;
    bool m_delete_archive;
    int m_num_folds;

};

}

