#pragma once

#include <string>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class SupervisedGenerator;
class ParameterData;
class ClassificationArchive;

class SupervisedGeneratorTester
{
public:
    SupervisedGeneratorTester(AbstractAuthority* authority);
    virtual ~SupervisedGeneratorTester();

    double get_accuracy(SupervisedGenerator* generator, const int& num_training, ParameterData* parameters, const int& num_testing);

protected:

    void build_archive(SupervisedGenerator* generator, const int& num_training, ParameterData* parameters);
    void train_on_archive(ParameterData* parameters, const std::string& classifier_filename);
    double test_classifier(SupervisedGenerator* generator,
                           ParameterData* parameters,
                           const int& num_testing,
                           const std::string& classifier_filename);
    AbstractAuthority* m_authority;

};

}
