#pragma once

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_DataFlow.hpp"
#include <vector>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class DenseMatrix;
}
class AbstractAuthority;
class ClassificationArchive;
class ParameterData;

class Preprocessor : public DataFlow
{
public:
    Preprocessor(AbstractAuthority* authority);
    virtual ~Preprocessor();

    void set_preprocessor_type(const preprocessor_t::preprocessor_t& type);
    preprocessor_t::preprocessor_t get_preprocessor_type();

    // for data flow
    virtual void unpack_data();
    virtual void allocate_data();

    void begin_initialize(ParameterData* parameters, ClassificationArchive* archive, int& processed_length);

    // for preprocessor
    virtual void preprocess(std::vector<double>& input_all);
    virtual void internal_initialize(ParameterData* parameters,
                                     ClassificationArchive* archive,
                                     AbstractInterface::DenseMatrix* input,
                                     const int& considered_columns,
                                     int& processed_length);

protected:
    preprocessor_t::preprocessor_t m_preprocessor_type;

};

}
