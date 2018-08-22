#pragma once

#include "PSL_Preprocessor.hpp"
#include <vector>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class ParameterData;
class ClassificationArchive;

class BipolarNormalization : public Preprocessor
{
public:
    BipolarNormalization(AbstractAuthority* authority);
    virtual ~BipolarNormalization();

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
    std::vector<double>* m_mins;
    std::vector<double>* m_maxs;

};

}
