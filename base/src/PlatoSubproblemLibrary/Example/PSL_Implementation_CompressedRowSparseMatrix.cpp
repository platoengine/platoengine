// PlatoSubproblemLibraryVersion(6): a stand-alone library for the kernel filter for plato.
#include "PSL_Implementation_CompressedRowSparseMatrix.hpp"

#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_FreeHelpers.hpp"

#include <vector>
#include <cassert>
#include <algorithm>
#include <map>
#include <cstddef>
#include <mpi.h>

namespace PlatoSubproblemLibrary
{

namespace example
{

CompressedRowSparseMatrix::CompressedRowSparseMatrix(const size_t num_rows,
                                                     const size_t num_columns,
                                                     const std::vector<size_t>& integer_row_bounds,
                                                     const std::vector<size_t>& columns,
                                                     const std::vector<double>& data) :
        AbstractInterface::SparseMatrix(),
        m_matrix_row_bounds(integer_row_bounds),
        m_matrix_columns(columns),
        m_matrix_data(data),
        m_num_rows(num_rows),
        m_num_columns(num_columns),
        m_built_nonzero_sorted_rows_and_columns(false),
        m_nonzero_sorted_rows(),
        m_nonzero_sorted_columns()
{
    assert(m_num_rows + 1u == m_matrix_row_bounds.size());
}

CompressedRowSparseMatrix::~CompressedRowSparseMatrix()
{
}

size_t CompressedRowSparseMatrix::getNumRows()
{
    return m_num_rows;
}
size_t CompressedRowSparseMatrix::getNumColumns()
{
    return m_num_columns;
}

void CompressedRowSparseMatrix::matVec(const std::vector<double>& input, std::vector<double>& output, bool transpose)
{
    if(!transpose)
    {
        // output = M * input

        assert(m_num_columns == input.size());
        output.assign(m_num_rows, 0.);
        for(size_t row = 0; row < m_num_rows; row++)
        {
            const size_t nz_begin = m_matrix_row_bounds[row];
            const size_t nz_end = m_matrix_row_bounds[row + 1u];

            for(size_t nz = nz_begin; nz < nz_end; nz++)
            {
                output[row] += m_matrix_data[nz] * input[m_matrix_columns[nz]];
            }
        }
    }
    else
    {
        // output = M' * input

        assert(transpose);
        assert(m_num_rows == input.size());
        output.assign(m_num_columns, 0.);
        for(size_t row = 0; row < m_num_rows; row++)
        {
            const size_t nz_begin = m_matrix_row_bounds[row];
            const size_t nz_end = m_matrix_row_bounds[row + 1u];

            for(size_t nz = nz_begin; nz < nz_end; nz++)
            {
                output[m_matrix_columns[nz]] += m_matrix_data[nz] * input[row];
            }
        }
    }
}

void CompressedRowSparseMatrix::matVecToReduced(const std::vector<double>& input, std::vector<double>& output, bool transpose)
{
    // if the matrix is very sparse, it may be helpful to work with effective sparse vectors.
    // the output vector from this function corresponds to the rows/columns (depending on transpose)
    // that are completely nonzero.

    // so, the below code makes sense:
    // std::vector<int> nzsr = CSRM->getNonZeroSortedRows();
    // CSRM->matVecToReduce(input,output,false);
    // for(size_t i=0u;i<output.size();i++){
    //     std::cout<<"index:"<<nzsr[i]<<",val:"<<output[i]<<std::endl;
    // }

    this->internal_build_nonzero_sorted_rows_and_columns();
    assert(m_built_nonzero_sorted_rows_and_columns);

    if(!transpose)
    {
        // output = M * input

        assert(m_num_columns == input.size());
        output.assign(m_nonzero_sorted_rows.size(), 0.);
        size_t reduced_row = 0;
        for(size_t row = 0; row < m_num_rows; row++)
        {
            const size_t nz_begin = m_matrix_row_bounds[row];
            const size_t nz_end = m_matrix_row_bounds[row + 1];
            if(nz_begin != nz_end)
            {
                for(size_t nz = nz_begin; nz < nz_end; nz++)
                {
                    output[reduced_row] += m_matrix_data[nz] * input[m_matrix_columns[nz]];
                }
                reduced_row++;
            }
        }
    }
    else
    {
        // output = M' * input

        assert(transpose);
        assert(m_num_rows == input.size());
        size_t num_nonzero_columns = m_nonzero_sorted_columns.size();
        output.assign(num_nonzero_columns, 0.);
        for(size_t row = 0; row < m_num_rows; row++)
        {
            const size_t nz_begin = m_matrix_row_bounds[row];
            const size_t nz_end = m_matrix_row_bounds[row + 1];

            for(size_t nz = nz_begin; nz < nz_end; nz++)
            {
                const size_t reduced_column = m_full_column_to_reduced_column[m_matrix_columns[nz]];
                output[reduced_column] += m_matrix_data[nz] * input[row];
            }
        }
    }
}

void CompressedRowSparseMatrix::rowNormalize(const std::vector<double>& rowNormalizationFactors)
{
    // multiply each row by its normalization factor

    assert(m_num_rows == rowNormalizationFactors.size());
    for(size_t row = 0; row < m_num_rows; row++)
    {
        const size_t nz_begin = m_matrix_row_bounds[row];
        const size_t nz_end = m_matrix_row_bounds[row + 1];
        for(size_t nz = nz_begin; nz < nz_end; nz++)
        {
            m_matrix_data[nz] *= rowNormalizationFactors[row];
        }
    }
}

void CompressedRowSparseMatrix::columnNormalize(const std::vector<double>& columnNormalizationFactors)
{
    // multiply each row by its normalization factor

    assert(m_num_columns == columnNormalizationFactors.size());
    size_t nnz = m_matrix_data.size();
    for(size_t nz = 0; nz < nnz; nz++)
    {
        const size_t column = m_matrix_columns[nz];
        m_matrix_data[nz] *= columnNormalizationFactors[column];
    }
}

void CompressedRowSparseMatrix::getNonZeroSortedRows(std::vector<size_t>& nonZeroRows)
{
    this->internal_build_nonzero_sorted_rows_and_columns();
    assert(m_built_nonzero_sorted_rows_and_columns);
    nonZeroRows.assign(m_nonzero_sorted_rows.begin(), m_nonzero_sorted_rows.end());
}

size_t CompressedRowSparseMatrix::getNumNonZeroSortedRows()
{
    this->internal_build_nonzero_sorted_rows_and_columns();
    assert(m_built_nonzero_sorted_rows_and_columns);
    return m_nonzero_sorted_rows.size();
}

void CompressedRowSparseMatrix::getNonZeroSortedColumns(std::vector<size_t>& nonZeroColumns)
{
    this->internal_build_nonzero_sorted_rows_and_columns();
    assert(m_built_nonzero_sorted_rows_and_columns);
    nonZeroColumns.assign(m_nonzero_sorted_columns.begin(), m_nonzero_sorted_columns.end());
}

size_t CompressedRowSparseMatrix::getNumNonZeroSortedColumns()
{
    this->internal_build_nonzero_sorted_rows_and_columns();
    assert(m_built_nonzero_sorted_rows_and_columns);
    return m_nonzero_sorted_columns.size();
}

void CompressedRowSparseMatrix::getRow(size_t row, std::vector<double>& data, std::vector<size_t>& columns)
{
    const size_t nz_begin = m_matrix_row_bounds[row];
    const size_t nz_end = m_matrix_row_bounds[row + 1];
    data.assign(&m_matrix_data[nz_begin], &m_matrix_data[nz_end]);
    columns.assign(&m_matrix_columns[nz_begin], &m_matrix_columns[nz_end]);
}

void CompressedRowSparseMatrix::setRow(size_t row, const std::vector<double>& data)
{
    const size_t nz_begin = m_matrix_row_bounds[row];
    std::copy(data.begin(), data.end(), &m_matrix_data[nz_begin]);
}

void CompressedRowSparseMatrix::internal_build_nonzero_sorted_rows_and_columns()
{
    if(m_built_nonzero_sorted_rows_and_columns)
    {
        return;
    }
    m_built_nonzero_sorted_rows_and_columns = true;

    // build nonzero sorted rows by intentional push backs
    m_nonzero_sorted_rows.clear();
    for(size_t row = 0; row < m_num_rows; row++)
    {
        if(m_matrix_row_bounds[row] != m_matrix_row_bounds[row + 1])
        {
            m_nonzero_sorted_rows.push_back(row);
        }
    }

    // prepare all columns
    std::vector<size_t> columns(m_matrix_columns);
    std::sort(columns.begin(), columns.end());

    // build nonzero sorted columns by intentional push backs
    m_nonzero_sorted_columns.clear();
    const size_t num_all_columns = columns.size();
    if(num_all_columns > 0)
    {
        m_nonzero_sorted_columns.push_back(columns[0]);
        for(size_t index = 1; index < num_all_columns; index++)
        {
            if(columns[index - 1] != columns[index])
            {
                m_nonzero_sorted_columns.push_back(columns[index]);
            }
        }
    }

    // invert the nonzero sorted columns
    size_t num_nonzero_columns = m_nonzero_sorted_columns.size();
    for(size_t nz_columns = 0; nz_columns < num_nonzero_columns; nz_columns++)
    {
        m_full_column_to_reduced_column[m_nonzero_sorted_columns[nz_columns]] = nz_columns;
    }
}

CompressedRowSparseMatrix* transposeCompressedRowSparseMatrix(CompressedRowSparseMatrix* input)
{
    // input sizes
    const size_t input_num_nonzeros = input->m_matrix_data.size();
    const size_t input_num_rows = input->getNumRows();
    const size_t input_num_columns = input->getNumColumns();

    // output allocation
    const size_t output_num_nonzeros = input_num_nonzeros;
    const size_t output_num_rows = input_num_columns;
    const size_t output_num_columns = input_num_rows;
    std::vector<size_t> output_row_bounds(output_num_rows + 1);
    std::vector<size_t> output_columns(output_num_nonzeros);
    std::vector<double> output_data(output_num_nonzeros);

    // build row bounds
    std::vector<size_t> tmp_ouput_row_bounds(output_num_rows + 1, 0);
    for(size_t nz = 0; nz < input_num_nonzeros; nz++)
    {
        tmp_ouput_row_bounds[1 + input->m_matrix_columns[nz]]++;
    }
    cumulative_sum(tmp_ouput_row_bounds, output_row_bounds);

    // build columns and data
    std::fill(tmp_ouput_row_bounds.begin(), tmp_ouput_row_bounds.end(), 0);
    for(size_t input_row = 0; input_row < input_num_rows; input_row++)
    {
        const size_t nz_begin = input->m_matrix_row_bounds[input_row];
        const size_t nz_end = input->m_matrix_row_bounds[input_row + 1];

        for(size_t nz = nz_begin; nz < nz_end; nz++)
        {
            const size_t input_column = input->m_matrix_columns[nz];
            const double input_data = input->m_matrix_data[nz];

            const size_t nonzero_index = output_row_bounds[input_column] + tmp_ouput_row_bounds[input_column]++;

            output_columns[nonzero_index] = input_row;
            output_data[nonzero_index] = input_data;
        }
    }

    CompressedRowSparseMatrix* output = new CompressedRowSparseMatrix(output_num_rows,
                                                                      output_num_columns,
                                                                      output_row_bounds,
                                                                      output_columns,
                                                                      output_data);
    return output;
}

void sendCompressedRowSparseMatrix(AbstractInterface::MpiWrapper* mpi_wrapper, size_t send_rank, CompressedRowSparseMatrix* input)
{
    // send sizes
    const size_t num_sizes_need_to_send = 3;
    std::vector<int> matrix_sizes(num_sizes_need_to_send, 0);
    if(input == NULL)
    {
        // handle no matrix to be send
        matrix_sizes[0] = 0u;
        mpi_wrapper->send(send_rank, matrix_sizes);
        return;
    }
    const size_t num_rows = input->getNumRows();
    matrix_sizes[0] = num_rows;
    const size_t num_columns = input->getNumColumns();
    matrix_sizes[1] = num_columns;
    const size_t num_nonzeros = input->m_matrix_data.size();
    matrix_sizes[2] = num_nonzeros;
    mpi_wrapper->send(send_rank, matrix_sizes);

    // send data
    std::vector<int> row_bounds(input->m_matrix_row_bounds.begin(), input->m_matrix_row_bounds.end());
    mpi_wrapper->send(send_rank, row_bounds);
    std::vector<int> columns(input->m_matrix_columns.begin(), input->m_matrix_columns.end());
    mpi_wrapper->send(send_rank, columns);
    mpi_wrapper->send(send_rank, input->m_matrix_data);
}

CompressedRowSparseMatrix* receiveCompressedRowSparseMatrix(AbstractInterface::MpiWrapper* mpi_wrapper, size_t recv_rank)
{
    // receive sizes
    const size_t num_sizes_need_to_recv = 3;
    std::vector<int> matrix_sizes(num_sizes_need_to_recv);
    mpi_wrapper->receive(recv_rank, matrix_sizes);
    const size_t num_rows = matrix_sizes[0];
    const size_t num_columns = matrix_sizes[1];
    const size_t num_nonzeros = matrix_sizes[2];

    if(num_rows == 0u)
    {
        // handle no matrix received
        return NULL;
    }

    // allocate for data
    std::vector<int> row_bounds(num_rows + 1);
    std::vector<int> columns(num_nonzeros);
    std::vector<double> data(num_nonzeros);

    // receive data
    mpi_wrapper->receive(recv_rank, row_bounds);
    mpi_wrapper->receive(recv_rank, columns);
    mpi_wrapper->receive(recv_rank, data);

    // build
    std::vector<size_t> sizet_row_bounds(row_bounds.begin(), row_bounds.end());
    std::vector<size_t> sizet_columns(columns.begin(), columns.end());
    CompressedRowSparseMatrix* result = new CompressedRowSparseMatrix(num_rows,
                                                                      num_columns,
                                                                      sizet_row_bounds,
                                                                      sizet_columns,
                                                                      data);
    return result;
}

}

}
