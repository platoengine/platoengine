// PlatoSubproblemLibraryVersion(5): a stand-alone library for the kernel filter for plato.
#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Implementation_CompressedRowSparseMatrix.hpp"
#include "PSL_Implementation_MpiWrapper.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Interface_MpiWrapper.hpp"
#include "PSL_Random.hpp"

#include <iostream> // for std::cout
#include <vector>
#include <mpi.h>
#include <cstddef> // for size_t

namespace PlatoSubproblemLibrary
{
namespace TestingCompressedRowSparseMatrixImplementation
{

PSL_TEST(CompressedRowSparseMatrixImplementation,constructor)
{
    set_rand_seed();
    // test that the class constructor properly allocates data

    /* A = [ 0 5 4 7
             0 0 0 0
             1 0 0 0 ] */
    const size_t num_rows = 3;
    const size_t num_columns = 4;
    std::vector<size_t> row_bounds = {0, 3, 3, 4};
    std::vector<size_t> columns = {1, 2, 3, 0};
    std::vector<double> data = {5., 4., 7., 1.};
    example::CompressedRowSparseMatrix sparseMatrix(num_rows, num_columns, row_bounds, columns, data);

    EXPECT_EQ(sparseMatrix.getNumRows(), num_rows);
    EXPECT_EQ(sparseMatrix.getNumColumns(), num_columns);
    expect_equal_vectors(row_bounds, sparseMatrix.m_matrix_row_bounds);
    expect_equal_vectors(columns, sparseMatrix.m_matrix_columns);
    expect_equal_float_vectors(data, sparseMatrix.m_matrix_data);
}

PSL_TEST(CompressedRowSparseMatrixImplementation,getNonZeros)
{
    set_rand_seed();
    // test the getNonZeroSortedRows() and getNonZeroSortedColumns()

    /* A = [ 0 0 0 3
             0 1 0 2
             0 0 0 0
             0 0 0 5
             0 0 0 0 ] */
    const size_t num_rows = 5;
    const size_t num_columns = 4;
    std::vector<size_t> row_bounds = {0, 1, 3, 3, 4, 4};
    std::vector<size_t> columns = {3, 1, 3, 3};
    std::vector<double> data = {3., 1., 2., 5.};
    example::CompressedRowSparseMatrix sparseMatrix(num_rows, num_columns, row_bounds, columns, data);

    std::vector<size_t> gold_nonZeroSortedRows = {0, 1, 3};
    std::vector<size_t> nonZeroSortedRows;
    sparseMatrix.getNonZeroSortedRows(nonZeroSortedRows);
    expect_equal_vectors(gold_nonZeroSortedRows, nonZeroSortedRows);

    std::vector<size_t> gold_nonZeroSortedColumns = {1, 3};
    std::vector<size_t> nonZeroSortedColumns;
    sparseMatrix.getNonZeroSortedColumns(nonZeroSortedColumns);
    expect_equal_vectors(gold_nonZeroSortedColumns, nonZeroSortedColumns);
}

PSL_TEST(CompressedRowSparseMatrixImplementation,matVec)
{
    set_rand_seed();
    // test the matVec(...)

    /* A = [ 0 0 0 3
             7 1 0 2
             0 0 0 0
             5 0 0 0
             4 0 0 6 ] */
    const size_t num_rows = 5;
    const size_t num_columns = 4;
    std::vector<size_t> row_bounds = {0, 1, 4, 4, 5, 7};
    std::vector<size_t> columns = {3, 0, 1, 3, 0, 0, 3};
    std::vector<double> data = {3., 7., 1., 2., 5., 4., 6.};
    example::CompressedRowSparseMatrix sparseMatrix(num_rows, num_columns, row_bounds, columns, data);

    std::vector<double> x = {1., 2., 3., 4.};
    std::vector<double> b;
    sparseMatrix.matVec(x, b);

    std::vector<double> b_gold = {12., 17., 0., 5., 28.};
    expect_equal_float_vectors(b_gold, b);
}

PSL_TEST(CompressedRowSparseMatrixImplementation,matVecTranspose)
{
    set_rand_seed();
    // test the matVec(...) with the transpose option

    /* A = [ 0 0 0 3
             7 1 0 2
             0 0 0 0
             5 0 0 0
             4 0 0 6 ] */
    const size_t num_rows = 5;
    const size_t num_columns = 4;
    std::vector<size_t> row_bounds = {0, 1, 4, 4, 5, 7};
    std::vector<size_t> columns = {3, 0, 1, 3, 0, 0, 3};
    std::vector<double> data = {3., 7., 1., 2., 5., 4., 6.};
    example::CompressedRowSparseMatrix sparseMatrix(num_rows, num_columns, row_bounds, columns, data);

    std::vector<double> x = {1., 2., 3., 4., 5.};
    std::vector<double> b;
    sparseMatrix.matVec(x, b, true);

    std::vector<double> b_gold = {54., 2., 0., 37.};
    expect_equal_float_vectors(b_gold, b);
}

PSL_TEST(CompressedRowSparseMatrixImplementation,matVecToReduced)
{
    set_rand_seed();
    // test the matVecToReduced(...)

    /* A = [ 0 0 0 3
             7 1 0 2
             0 0 0 0
             5 0 0 0
             4 0 0 6 ] */
    const size_t num_rows = 5;
    const size_t num_columns = 4;
    std::vector<size_t> row_bounds = {0, 1, 4, 4, 5, 7};
    std::vector<size_t> columns = {3, 0, 1, 3, 0, 0, 3};
    std::vector<double> data = {3., 7., 1., 2., 5., 4., 6.};
    example::CompressedRowSparseMatrix sparseMatrix(num_rows, num_columns, row_bounds, columns, data);

    std::vector<double> x = {1., 2., 3., 4.};
    std::vector<double> b;
    sparseMatrix.matVecToReduced(x, b);

    std::vector<double> b_gold = {12., 17., 5., 28.};
    expect_equal_float_vectors(b_gold, b);
}

PSL_TEST(CompressedRowSparseMatrixImplementation,matVecToReducedTranspose)
{
    set_rand_seed();
    // test the matVecToReduced(...) with the transpose option

    /* A = [ 0 0 0 3
             7 1 0 2
             0 0 0 0
             5 0 0 0
             4 0 0 6 ] */
    const size_t num_rows = 5;
    const size_t num_columns = 4;
    std::vector<size_t> row_bounds = {0, 1, 4, 4, 5, 7};
    std::vector<size_t> columns = {3, 0, 1, 3, 0, 0, 3};
    std::vector<double> data = {3., 7., 1., 2., 5., 4., 6.};
    example::CompressedRowSparseMatrix sparseMatrix(num_rows, num_columns, row_bounds, columns, data);

    std::vector<double> x = {1., 2., 3., 4., 5.};
    std::vector<double> b;
    sparseMatrix.matVecToReduced(x, b, true);

    std::vector<double> b_gold = {54., 2., 37.};
    expect_equal_float_vectors(b_gold, b);
}

PSL_TEST(CompressedRowSparseMatrixImplementation,sendAndRecv)
{
    set_rand_seed();
    // test the sendCompressedRowSparseMatrix(...) and receiveCompressedRowSparseMatrix(...)

    MPI_Comm comm = MPI_COMM_WORLD;
    example::Interface_BasicGlobalUtilities utilities;
    example::Interface_MpiWrapper mpi_interface(&utilities, &comm);

    const size_t rank = mpi_interface.get_rank();
    const size_t size = mpi_interface.get_size();

    if(size > 1u)
    {
        /* A = [ 0 7 0 rank
                 0 0 0 0
                 1 0 0 0 ] */
        const size_t num_rows = 3;
        const size_t num_columns = 4;
        std::vector<size_t> row_bounds = {0, 2, 2, 3};
        std::vector<size_t> columns = {1, 3, 0};
        std::vector<double> data = {7., (double)rank, 1.};

        if(rank == 0)
        {
            example::CompressedRowSparseMatrix* sparseMatrix = NULL;

            // receive from other processors, test against gold
            for(size_t other_rank = 1; other_rank < size; other_rank++)
            {
                sparseMatrix = example::receiveCompressedRowSparseMatrix(&mpi_interface, other_rank);

                EXPECT_EQ(sparseMatrix->getNumRows(), num_rows);
                EXPECT_EQ(sparseMatrix->getNumColumns(), num_columns);
                expect_equal_vectors(row_bounds, sparseMatrix->m_matrix_row_bounds);
                expect_equal_vectors(columns, sparseMatrix->m_matrix_columns);
                data[1] = other_rank;
                expect_equal_float_vectors(data, sparseMatrix->m_matrix_data);

                delete sparseMatrix;
            }
        }
        else
        {
            example::CompressedRowSparseMatrix sparseMatrix(num_rows, num_columns, row_bounds, columns, data);
            example::sendCompressedRowSparseMatrix(&mpi_interface, 0, &sparseMatrix);
        }
    }
}

PSL_TEST(CompressedRowSparseMatrixImplementation,transpose)
{
    set_rand_seed();
    // test transposeCompressedRowSparseMatrix(...)

    /* A = [ 0 5 4 7
             0 0 0 0
             1 0 3 0 ] */
    const size_t num_rows = 3;
    const size_t num_columns = 4;
    std::vector<size_t> row_bounds = {0, 3, 3, 5};
    std::vector<size_t> columns = {1, 2, 3, 0, 2};
    std::vector<double> data = {5., 4., 7., 1., 3.};
    example::CompressedRowSparseMatrix sparseMatrix(num_rows, num_columns, row_bounds, columns, data);

    example::CompressedRowSparseMatrix* transposeSparseMatrix = example::transposeCompressedRowSparseMatrix(&sparseMatrix);

    /* A = [ 0 0 1
             5 0 0
             4 0 3
             7 0 0 ] */
    const size_t transpose_num_rows = 4;
    const size_t transpose_num_columns = 3;
    std::vector<size_t> transpose_row_bounds = {0, 1, 2, 4, 5};
    std::vector<size_t> transpose_columns = {2, 0, 0, 2, 0};
    std::vector<double> transpose_data = {1., 5., 4., 3., 7.};

    EXPECT_EQ(transposeSparseMatrix->getNumRows(), transpose_num_rows);
    EXPECT_EQ(transposeSparseMatrix->getNumColumns(), transpose_num_columns);
    expect_equal_vectors(transpose_row_bounds, transposeSparseMatrix->m_matrix_row_bounds);
    expect_equal_vectors(transpose_columns, transposeSparseMatrix->m_matrix_columns);
    expect_equal_float_vectors(transpose_data, transposeSparseMatrix->m_matrix_data);

    delete transposeSparseMatrix;
}

}

}
