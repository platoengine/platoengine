#pragma once

#include "PSL_Classifier.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class DenseMatrix;
}
class AbstractAuthority;
class ParameterData;
class ClassificationArchive;
class VectorKernel;

class SupportVectorMachine : public Classifier
{
public:
    SupportVectorMachine(AbstractAuthority* authority);
    virtual ~SupportVectorMachine();

    virtual void unpack_data();

    virtual void initialize(ParameterData* parameters, ClassificationArchive* archive);
    virtual void train(ParameterData* parameters, ClassificationArchive* archive);
    virtual int classify(const std::vector<double>& input_scalars, const std::vector<int>& input_enums);

protected:

    void clear_built_data();
    void allocate_classifier_data(ParameterData* parameters, ClassificationArchive* archive);
    void initialize_vectors(ParameterData* parameters, const int& input_dimension);
    void train_1v1(ParameterData* parameters,
                   AbstractInterface::DenseMatrix* dataset,
                   const std::vector<int>& negative_class_rows,
                   const std::vector<int>& positive_class_rows,
                   std::vector<std::vector<double>*>& support_vector_snapshots);
    void train_an_epoch(const double& structural_risk_parameter,
                        AbstractInterface::DenseMatrix* dataset,
                        const std::vector<int>& negative_class_rows,
                        const std::vector<int>& positive_class_rows,
                        std::vector<double>* support_vector,
                        double& experience);
    void check_parameters(ParameterData* parameters);

    // DataSequence data
    int* m_output_enum_size;
    int* m_kernel_function_enum;
    int* m_kernel_function_parameter;
    int* m_num_snapshots;
    std::vector<int>* m_input_enum_sizes;
    std::vector<std::vector<double>*> m_support_vectors;

    // built from DataSequence
    VectorKernel* m_kernel_function;

};

}
