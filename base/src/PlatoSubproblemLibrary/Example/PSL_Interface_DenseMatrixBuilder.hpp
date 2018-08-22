// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include <cstddef>
#include <vector>

#include "PSL_Abstract_DenseMatrixBuilder.hpp"

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class GlobalUtilities;
class DenseMatrix;
}

namespace example
{

class Interface_DenseMatrixBuilder : public AbstractInterface::DenseMatrixBuilder
{
public:
    Interface_DenseMatrixBuilder(AbstractInterface::GlobalUtilities* utilities);
    virtual ~Interface_DenseMatrixBuilder();

    virtual AbstractInterface::DenseMatrix* build_by_row_major(size_t num_rows, size_t num_columns, const std::vector<double>& in);
protected:
};

}
}
