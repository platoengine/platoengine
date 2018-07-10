// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class GlobalUtilities;
class DenseMatrixBuilder;

class DenseMatrix
{
public:
    DenseMatrix(GlobalUtilities* utilities, DenseMatrixBuilder* builder);
    virtual ~DenseMatrix();

    virtual DenseMatrixBuilder* get_builder();

    virtual size_t get_num_rows() = 0;
    virtual size_t get_num_columns() = 0;
    virtual double get_value(size_t row, size_t column) = 0;
    virtual void set_value(size_t row, size_t column, double value) = 0;

    virtual void matvec(const std::vector<double>& in, std::vector<double>& out, bool transpose) = 0;
    virtual void fill(double alpha) = 0;
    virtual void fill_by_row_major(const std::vector<double>& in) = 0;
    virtual void set_to_identity() = 0;
    virtual void copy(DenseMatrix* source) = 0;
    virtual void aXpY(double alpha, DenseMatrix* X) = 0;
    virtual void matrix_matrix_product(double alpha, DenseMatrix* X, bool X_transpose, DenseMatrix* Y, bool Y_transpose) = 0;
    virtual void scale(double alpha) = 0;
    virtual double dot(DenseMatrix* other) = 0;
    virtual void get_row(const int& row_index, std::vector<double>& row) = 0;
    virtual void get_column(const int& column_index, std::vector<double>& column) = 0;
    virtual void get_diagonal(std::vector<double>& diagonal) = 0;
    virtual void permute_columns(const std::vector<int>& permutation) = 0;
    virtual void scale_column(const int& column_index, const double& scale) = 0;

    virtual void get_row_major(std::vector<std::vector<double> >& matrix);

protected:
    GlobalUtilities* m_utilities;
    DenseMatrixBuilder* m_builder;

};

}
}
