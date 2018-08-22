#pragma once

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_Preprocessor.hpp"
#include <vector>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class ParameterData;
class ClassificationArchive;

class MultistagePreprocessor : public Preprocessor
{
public:
    MultistagePreprocessor(AbstractAuthority* authority,
                           const std::vector<preprocessor_t::preprocessor_t>& preprocess_stages,
                           DataSequence* data_sequence);
    virtual ~MultistagePreprocessor();

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
    std::vector<Preprocessor*> m_staged_preprocessors;

};

}
