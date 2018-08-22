// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include <cstddef>
#include <vector>

#include "PSL_Abstract_SparseMatrixBuilder.hpp"

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class SparseMatrix;
}

namespace example
{

class SparseMatrixBuilder : public AbstractInterface::SparseMatrixBuilder
{
public:
    SparseMatrixBuilder(AbstractInterface::MpiWrapper* mpi_wrapper);
    virtual ~SparseMatrixBuilder();

    virtual size_t get_number_of_passes_over_all_nonzero_entries();
    virtual void begin_build(size_t num_rows, size_t num_columns);

    virtual bool needs_value_this_pass();
    virtual void specify_nonzero(size_t row, size_t column);
    virtual void specify_nonzero(size_t row, size_t column, double value);
    virtual void advance_pass();

    virtual AbstractInterface::SparseMatrix* end_build();

    virtual void send_matrix(size_t send_rank, AbstractInterface::SparseMatrix* matrix);
    virtual AbstractInterface::SparseMatrix* receive_matrix(size_t recv_rank);

    virtual AbstractInterface::SparseMatrix* transpose(AbstractInterface::SparseMatrix* input);

protected:
    virtual void reset();

    size_t m_pass;
    size_t m_num_rows;
    size_t m_num_columns;
    std::vector<size_t> m_temporary_for_integer_row_bounds;
    std::vector<size_t> m_integer_row_bounds;
    std::vector<size_t> m_columns;
    std::vector<double> m_data;

};

}
}
