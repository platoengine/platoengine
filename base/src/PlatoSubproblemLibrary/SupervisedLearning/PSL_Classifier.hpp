#pragma once

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_DataFlow.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class GlobalUtilities;
class MpiWrapper;
}
class AbstractAuthority;
class ParameterData;
class ClassificationArchive;
class Preprocessor;

class Classifier : public DataFlow
{
public:
    Classifier(AbstractAuthority* authority);
    virtual ~Classifier();

    virtual void initialize(ParameterData* parameters, ClassificationArchive* archive) = 0;
    virtual void train(ParameterData* parameters, ClassificationArchive* archive) = 0;
    // TODO: write classify with encode/decoded and calling one another, child chooses desired format
    virtual int classify(const std::vector<double>& input_scalars, const std::vector<int>& input_enums) = 0;

protected:

    void allocate_preprocessor(ParameterData* parameters);
    void unpack_preprocessor(const std::vector<preprocessor_t::preprocessor_t>& types);
    std::vector<int>* m_preprocessor_enum;
    Preprocessor* m_preprocessor;

};

}
