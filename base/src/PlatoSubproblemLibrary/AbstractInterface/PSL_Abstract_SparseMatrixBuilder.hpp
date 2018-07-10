// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include <cstddef>

namespace PlatoSubproblemLibrary
{

namespace AbstractInterface
{
class SparseMatrix;
class MpiWrapper;
class GlobalUtilities;

class SparseMatrixBuilder
{
public:
    SparseMatrixBuilder(MpiWrapper* mpi_wrapper);
    virtual ~SparseMatrixBuilder();

    virtual MpiWrapper* get_mpi_wrapper();
    virtual GlobalUtilities* get_utilities();

    virtual size_t get_number_of_passes_over_all_nonzero_entries() = 0;
    virtual void begin_build(size_t num_rows, size_t num_columns) = 0;

    virtual bool needs_value_this_pass() = 0;
    virtual void specify_nonzero(size_t row, size_t column) = 0;
    virtual void specify_nonzero(size_t row, size_t column, double value) = 0;
    virtual void advance_pass() = 0;

    virtual SparseMatrix* end_build() = 0;

    virtual void send_matrix(size_t rank, SparseMatrix* matrix) = 0;
    virtual SparseMatrix* receive_matrix(size_t rank) = 0;

    virtual SparseMatrix* transpose(SparseMatrix* input) = 0;
protected:
    MpiWrapper* m_mpi_wrapper;
    GlobalUtilities* m_utilities;

};

}

}
