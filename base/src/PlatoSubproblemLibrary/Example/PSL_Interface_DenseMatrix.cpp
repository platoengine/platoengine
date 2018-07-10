// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Interface_DenseMatrix.hpp"

#include <vector>
#include <cstddef>
#include <cassert>
#include <algorithm>

#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_Implementation_DenseMatrix.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"

namespace PlatoSubproblemLibrary
{
namespace example
{

Interface_DenseMatrix::Interface_DenseMatrix(AbstractInterface::GlobalUtilities* utilities,
                                             AbstractInterface::DenseMatrixBuilder* builder) :
        AbstractInterface::DenseMatrix(utilities, builder),
        m_data(NULL),
        m_num_rows(0u),
        m_num_columns(0u)
{
}

Interface_DenseMatrix::~Interface_DenseMatrix()
{
    dense_matrix::destroy(m_data, m_num_rows, m_num_columns);
}

void Interface_DenseMatrix::receive(double** data, size_t num_rows, size_t num_columns)
{
    dense_matrix::destroy(m_data, m_num_rows, m_num_columns);
    m_data = data;
    m_num_rows = num_rows;
    m_num_columns = num_columns;
}

size_t Interface_DenseMatrix::get_num_rows()
{
    return m_num_rows;
}

size_t Interface_DenseMatrix::get_num_columns()
{
    return m_num_columns;
}

double Interface_DenseMatrix::get_value(size_t row, size_t column)
{
    assert(row < m_num_rows);
    assert(column < m_num_columns);
    return m_data[row][column];
}

void Interface_DenseMatrix::set_value(size_t row, size_t column, double value)
{
    assert(row < m_num_rows);
    assert(column < m_num_columns);
    m_data[row][column] = value;
}

void Interface_DenseMatrix::matvec(const std::vector<double>& in, std::vector<double>& out, bool transpose)
{
    if(transpose)
    {
        out.resize(m_num_columns);
    }
    else
    {
        out.resize(m_num_rows);
    }
    dense_matrix::matvec(m_num_rows, m_num_columns, m_data, in.data(), out.data(), transpose);
}

void Interface_DenseMatrix::fill(double alpha)
{
    dense_matrix::fill(alpha, m_data, m_num_rows, m_num_columns);
}

void Interface_DenseMatrix::fill_by_row_major(const std::vector<double>& in)
{
    dense_matrix::fill_by_row_major(in.data(), m_data, m_num_rows, m_num_columns);
}

void Interface_DenseMatrix::set_to_identity()
{
    dense_matrix::identity(m_data, m_num_rows, m_num_columns);
}

void Interface_DenseMatrix::copy(AbstractInterface::DenseMatrix* source)
{
    dense_matrix::destroy(m_data, m_num_rows, m_num_columns);
    m_num_rows = source->get_num_rows();
    m_num_columns = source->get_num_columns();
    m_data = dense_matrix::create(m_num_rows, m_num_columns);
    for(size_t row = 0u; row < m_num_rows; row++)
    {
        for(size_t column = 0u; column < m_num_columns; column++)
        {
            m_data[row][column] = source->get_value(row, column);
        }
    }
}

void Interface_DenseMatrix::aXpY(double alpha, AbstractInterface::DenseMatrix* X)
{
    Interface_DenseMatrix* X_casted = dynamic_cast<Interface_DenseMatrix*>(X);
    assert(m_utilities);
    if(!X_casted)
    {
        m_utilities->fatal_error("Interface_DenseMatrix: failed to dynamic cast pointer. Aborting.\n\n");
    }
    dense_matrix::aXpY(alpha, X_casted->m_data, m_data, m_num_rows, m_num_columns);
}

void Interface_DenseMatrix::matrix_matrix_product(double alpha,
                                                  AbstractInterface::DenseMatrix* X,
                                                  bool X_transpose,
                                                  AbstractInterface::DenseMatrix* Y,
                                                  bool Y_transpose)
{
    Interface_DenseMatrix* X_casted = dynamic_cast<Interface_DenseMatrix*>(X);
    Interface_DenseMatrix* Y_casted = dynamic_cast<Interface_DenseMatrix*>(Y);
    assert(m_utilities);
    if(!X_casted || !Y_casted)
    {
        m_utilities->fatal_error("Interface_DenseMatrix: failed to dynamic cast pointer. Aborting.\n\n");
    }
    dense_matrix::matrix_matrix_product(alpha,
                                        X_casted->m_data,
                                        X->get_num_rows(),
                                        X->get_num_columns(),
                                        X_transpose,
                                        Y_casted->m_data,
                                        Y->get_num_rows(),
                                        Y->get_num_columns(),
                                        Y_transpose,
                                        m_data);
}

void Interface_DenseMatrix::scale(double alpha)
{
    dense_matrix::scale(alpha, m_data, m_num_rows, m_num_columns);
}

double Interface_DenseMatrix::dot(AbstractInterface::DenseMatrix* other)
{
    double result = 0.;

    // element-wise dot product as if matrix was interpreted as vector
    assert(other->get_num_rows() == m_num_rows);
    assert(other->get_num_columns() == m_num_columns);
    for(size_t row = 0u; row < m_num_rows; row++)
    {
        for(size_t column = 0u; column < m_num_columns; column++)
        {
            result += (m_data[row][column]) * other->get_value(row, column);
        }
    }

    return result;
}

void Interface_DenseMatrix::get_row(const int& row_index, std::vector<double>& row)
{
    row.assign(&m_data[row_index][0], &m_data[row_index][m_num_columns]);
}

void Interface_DenseMatrix::get_column(const int& column_index, std::vector<double>& column)
{
    column.resize(m_num_rows);
    for(size_t row = 0u; row < m_num_rows; row++)
    {
        column[row] = m_data[row][column_index];
    }
}
void Interface_DenseMatrix::get_diagonal(std::vector<double>& diagonal)
{
    const size_t min_size = std::min(m_num_rows, m_num_columns);
    diagonal.resize(min_size);
    for(size_t i = 0u; i < min_size; i++)
    {
        diagonal[i] = m_data[i][i];
    }
}
void Interface_DenseMatrix::permute_columns(const std::vector<int>& permutation)
{
    assert(permutation.size() == m_num_columns);
    std::vector<double> this_row;
    for(size_t row = 0u; row < m_num_rows; row++)
    {
        get_row(row, this_row);
        for(size_t column = 0u; column < m_num_columns; column++)
        {
            assert(0 <= permutation[column]);
            assert(permutation[column] < int(m_num_columns));
            m_data[row][column] = this_row[permutation[column]];
        }
    }
}
void Interface_DenseMatrix::scale_column(const int& column_index, const double& scale)
{
    for(size_t row = 0u; row < m_num_rows; row++)
    {
        m_data[row][column_index] *= scale;
    }
}

}
}
