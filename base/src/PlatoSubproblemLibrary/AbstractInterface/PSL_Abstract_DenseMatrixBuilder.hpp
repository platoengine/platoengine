// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include <cstddef>
#include <vector>

namespace PlatoSubproblemLibrary
{

namespace AbstractInterface
{
class GlobalUtilities;
class DenseMatrix;

class DenseMatrixBuilder
{
public:
    DenseMatrixBuilder(GlobalUtilities* utilities);
    virtual ~DenseMatrixBuilder();

    virtual GlobalUtilities* get_utilities();

    virtual DenseMatrix* build_by_row_major(size_t num_rows, size_t num_columns, const std::vector<double>& in) = 0;

    virtual DenseMatrix* build_by_fill(size_t num_rows, size_t num_columns, double fill_value);
    virtual DenseMatrix* build_diagonal(const std::vector<double>& diagonal);
    virtual DenseMatrix* build_submatrix(size_t begin_row_inclusive,
                                         size_t end_row_exclusive,
                                         size_t begin_column_inclusive,
                                         size_t end_column_exclusive,
                                         DenseMatrix* matrix);
    virtual DenseMatrix* build_by_outerProduct(const std::vector<double>& x,const std::vector<double>& y);

protected:
    GlobalUtilities* m_utilities;
};

}

}
