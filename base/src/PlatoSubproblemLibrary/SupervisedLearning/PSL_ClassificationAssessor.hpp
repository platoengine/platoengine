#pragma once

#include <vector>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class DenseMatrix;
}
class AbstractAuthority;
class ParameterData;
class Classifier;
class ClassificationArchive;

class ClassificationAssessor
{
public:
    ClassificationAssessor(AbstractAuthority* authority);
    ~ClassificationAssessor();

    void initialize(ParameterData* parameters, int output_enum_size);
    void clear();

    void assess(Classifier* classifier, ClassificationArchive* archive, const std::vector<int>& rows);
    void assess(Classifier* classifier, ClassificationArchive* archive);

    void add_to_assessment(const int& actual_output, const int& predicted_output);

    AbstractInterface::DenseMatrix* get_confusion_matrix();
    double get_classification_accuracy();

protected:

    void internal_assess(Classifier* classifier,
                         ClassificationArchive* archive,
                         const bool& all_rows,
                         const std::vector<int>& rows);

    AbstractAuthority* m_authority;
    ParameterData* m_parameters;
    int m_output_enum_size;
    AbstractInterface::DenseMatrix* m_confusion_matrix;
};

}
