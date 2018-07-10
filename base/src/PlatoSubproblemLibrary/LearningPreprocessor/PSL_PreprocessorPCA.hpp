#pragma once

#include "PSL_Preprocessor.hpp"
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

class PreprocessorPCA : public Preprocessor
{
public:
    PreprocessorPCA(AbstractAuthority* authority);
    virtual ~PreprocessorPCA();

    // for data flow
    virtual void unpack_data();
    virtual void allocate_data();

    // for preprocessor
    virtual void preprocess(std::vector<double>& input_all);
    virtual void internal_initialize(ParameterData* parameters,
                                     ClassificationArchive* archive,
                                     AbstractInterface::DenseMatrix* input,
                                     const int& considered_columns,
                                     int& processed_length);

protected:
    std::vector<double>* m_input_column_means;
    std::vector<double>* m_basis_column_stds;
    AbstractInterface::DenseMatrix** m_compressed_basis;

};

}
