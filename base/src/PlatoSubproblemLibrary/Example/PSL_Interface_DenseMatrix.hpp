// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include <vector>
#include <cstddef>

#include "PSL_Abstract_DenseMatrix.hpp"

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class GlobalUtilities;
class DenseMatrixBuilder;
}

namespace example
{

class Interface_DenseMatrix : public AbstractInterface::DenseMatrix
{
public:
    Interface_DenseMatrix(AbstractInterface::GlobalUtilities* global,
                          AbstractInterface::DenseMatrixBuilder* builder);
    virtual ~Interface_DenseMatrix();

    void receive(double** data, size_t num_rows, size_t num_columns);

    virtual size_t get_num_rows();
    virtual size_t get_num_columns();
    virtual double get_value(size_t row, size_t column);
    virtual void set_value(size_t row, size_t column, double value);

    virtual void matvec(const std::vector<double>& in, std::vector<double>& out, bool transpose);
    virtual void fill(double alpha);
    virtual void fill_by_row_major(const std::vector<double>& in);
    virtual void set_to_identity();
    virtual void copy(AbstractInterface::DenseMatrix* source);
    virtual void aXpY(double alpha, AbstractInterface::DenseMatrix* X);
    virtual void matrix_matrix_product(double alpha,
                                       AbstractInterface::DenseMatrix* X, bool X_transpose,
                                       AbstractInterface::DenseMatrix* Y, bool Y_transpose);
    virtual void scale(double alpha);
    virtual double dot(AbstractInterface::DenseMatrix* other);
    virtual void get_row(const int& row_index, std::vector<double>& row);
    virtual void get_column(const int& column_index, std::vector<double>& column);
    virtual void get_diagonal(std::vector<double>& diagonal);
    virtual void permute_columns(const std::vector<int>& permutation);
    virtual void scale_column(const int& column_index, const double& scale);

    double** m_data;

protected:

    size_t m_num_rows;
    size_t m_num_columns;
};

}
}
