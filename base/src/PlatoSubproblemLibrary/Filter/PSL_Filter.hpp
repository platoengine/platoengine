// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class ParallelVector;
}

class Filter
{
public:
    Filter();
    virtual ~Filter();

    virtual void build() = 0;
    virtual void apply(AbstractInterface::ParallelVector* field) = 0;
    virtual void apply(AbstractInterface::ParallelVector* base_field, AbstractInterface::ParallelVector* gradient) = 0;
    virtual void advance_continuation();

protected:

};

}
