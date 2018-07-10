#pragma once

#include "PSL_DataSequence.hpp"

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;

class MultiLayoutVector : public DataSequence
{
public:
    MultiLayoutVector(AbstractAuthority* authority);
    virtual ~MultiLayoutVector();

    void zeros(MultiLayoutVector* other);
    void scale(const double& scale);
    void axpy(const double& scale, MultiLayoutVector* other);
    double dot(MultiLayoutVector* other);
    void random_double(const double& lower, const double& upper);

protected:
    AbstractAuthority* m_authority;

};

}
