/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
*/

// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Interface_SparseMatrixBuilder.hpp"
#include "PSL_Implementation_MpiWrapper.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Interface_MpiWrapper.hpp"
#include "PSL_Abstract_SparseMatrix.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Random.hpp"

#include <iostream> // for std::cout
#include <vector>
#include <mpi.h>
#include <cstddef> // for size_t
#include <string>

namespace PlatoSubproblemLibrary
{

#define CompressedRowSparseMatrixBuilderTest_AllocateUtilities \
    MPI_Comm comm = MPI_COMM_WORLD; \
    example::Interface_BasicGlobalUtilities utilities; \
    example::Interface_MpiWrapper mpi_interface(&utilities, &comm); \
    example::SparseMatrixBuilder sparse_matrix_builder(&mpi_interface); \
    AbstractInterface::SparseMatrixBuilder* builder = &sparse_matrix_builder;

namespace TestingCompressedRowSparseMatrixBuilder
{

void checkResults(const std::vector<double> & gold_, const std::vector<double> & results_)
{
    const size_t gold_size = gold_.size();
    ASSERT_EQ(gold_size, results_.size());
    for(size_t i = 0; i < gold_size; ++i)
    {
        EXPECT_FLOAT_EQ(gold_[i], results_[i]);
    }
}

void checkResults(const std::vector<size_t> & gold_, const std::vector<size_t> & results_)
{
    const size_t gold_size = gold_.size();
    ASSERT_EQ(gold_size, results_.size());
    for(size_t i = 0; i < gold_size; ++i)
    {
        EXPECT_EQ(gold_[i], results_[i]);
    }
}

PSL_TEST(CompressedRowSparseMatrixBuilder,constructor)
{
    set_rand_seed();
    // test that the class constructor properly allocates data
    CompressedRowSparseMatrixBuilderTest_AllocateUtilities

    /* A = [ 0 5 4 7
             0 0 0 0
             1 0 0 0 ] */

    // build abstract
    const size_t num_rows = 3u;
    const size_t num_columns = 4u;
    builder->begin_build(num_rows, num_columns);
    for(size_t repeat = 0; repeat < 2u; repeat++)
    {
        builder->specify_nonzero(0, 1, 5.);
        builder->specify_nonzero(0, 2, 4.);
        builder->specify_nonzero(0, 3, 7.);
        builder->specify_nonzero(2, 0, 1.);
        builder->advance_pass();
    }
    AbstractInterface::SparseMatrix* built_matrix = builder->end_build();

    // check dimensions
    ASSERT_EQ(built_matrix->getNumRows(), num_rows);
    ASSERT_EQ(built_matrix->getNumColumns(), num_columns);

    // check matrix by matvecs
    std::vector<double> output;

    // e1
    std::vector<double> e1 = {1., 0., 0., 0.};
    built_matrix->matVec(e1,output,false);
    std::vector<double> expected_output_e1 = {0., 0., 1.};
    checkResults(expected_output_e1, output);

    // e2
    std::vector<double> e2 = {0., 1., 0., 0.};
    built_matrix->matVec(e2, output, false);
    std::vector<double> expected_output_e2 = {5., 0., 0.};
    checkResults(expected_output_e2, output);

    // e3
    std::vector<double> e3 = {0., 0., 1., 0.};
    built_matrix->matVec(e3, output, false);
    std::vector<double> expected_output_e3 = {4., 0., 0.};
    checkResults(expected_output_e3, output);

    // e4
    std::vector<double> e4 = {0., 0., 0., 1.};
    built_matrix->matVec(e4, output, false);
    std::vector<double> expected_output_e4 = {7., 0., 0.};
    checkResults(expected_output_e4, output);

    // clean up
    delete built_matrix;
}

PSL_TEST(CompressedRowSparseMatrixBuilder,getNonZeros)
{
    set_rand_seed();
    // test the getNonZeroSortedRows() and getNonZeroSortedColumns()
    CompressedRowSparseMatrixBuilderTest_AllocateUtilities

    /* A = [ 0 0 0 3
             0 1 0 2
             0 0 0 0
             0 0 0 5
             0 0 0 0 ] */

    const size_t num_rows = 5u;
    const size_t num_columns = 4u;
    builder->begin_build(num_rows, num_columns);
    for(size_t repeat = 0; repeat < 2u; repeat++)
    {
        builder->specify_nonzero(0, 3, 3.);
        builder->specify_nonzero(1, 1, 1.);
        builder->specify_nonzero(1, 3, 2.);
        builder->specify_nonzero(3, 3, 5.);
        builder->advance_pass();
    }
    AbstractInterface::SparseMatrix* built_matrix = builder->end_build();

    // check nonzeros
    std::vector<size_t> gold_nonZeroSortedRows = {0u, 1u, 3u};
    std::vector<size_t> nonZeroSortedRows;
    built_matrix->getNonZeroSortedRows(nonZeroSortedRows);
    checkResults(gold_nonZeroSortedRows, nonZeroSortedRows);
    std::vector<size_t> gold_nonZeroSortedColumns = {1u, 3u};
    std::vector<size_t> nonZeroSortedColumns;
    built_matrix->getNonZeroSortedColumns(nonZeroSortedColumns);
    checkResults(gold_nonZeroSortedColumns, nonZeroSortedColumns);

    // check dimensions
    ASSERT_EQ(built_matrix->getNumRows(), num_rows);
    ASSERT_EQ(built_matrix->getNumColumns(), num_columns);

    // check matrix by matvecs
    std::vector<double> output;

    // e1
    std::vector<double> e1 = {1., 0., 0., 0.};
    built_matrix->matVec(e1, output, false);
    std::vector<double> expected_output_e1 = {0., 0., 0., 0., 0.};
    checkResults(expected_output_e1, output);

    // e2
    std::vector<double> e2 = {0., 1., 0., 0.};
    built_matrix->matVec(e2, output, false);
    std::vector<double> expected_output_e2 = {0., 1., 0., 0., 0.};
    checkResults(expected_output_e2, output);

    // e3
    std::vector<double> e3 = {0., 0., 1., 0.};
    built_matrix->matVec(e3, output, false);
    std::vector<double> expected_output_e3 = {0., 0., 0., 0., 0.};
    checkResults(expected_output_e3, output);

    // e4
    std::vector<double> e4 = {0., 0., 0., 1.};
    built_matrix->matVec(e4, output, false);
    std::vector<double> expected_output_e4 = {3., 2., 0., 5., 0.};
    checkResults(expected_output_e4, output);

    // clean up
    delete built_matrix;
}

PSL_TEST(CompressedRowSparseMatrixBuilder,matVec)
{
    set_rand_seed();
    // test the matVec(...)
    CompressedRowSparseMatrixBuilderTest_AllocateUtilities

    /* A = [ 0 0 0 3
             7 1 0 2
             0 0 0 0
             5 0 0 0
             4 0 0 6 ] */

    // with builder
    const size_t num_rows = 5u;
    const size_t num_columns = 4u;
    builder->begin_build(num_rows, num_columns);
    for(size_t repeat = 0; repeat < 2u; repeat++)
    {
        builder->specify_nonzero(0, 3, 3.);
        builder->specify_nonzero(1, 0, 7.);
        builder->specify_nonzero(1, 1, 1.);
        builder->specify_nonzero(1, 3, 2.);
        builder->specify_nonzero(3, 0, 5.);
        builder->specify_nonzero(4, 0, 4.);
        builder->specify_nonzero(4, 3, 6.);
        builder->advance_pass();
    }
    AbstractInterface::SparseMatrix* built_matrix = builder->end_build();

    // matrix vector product
    std::vector<double> x = {1., 2., 3., 4.};
    std::vector<double> b;
    built_matrix->matVec(x, b, false);

    // check matrix vector product
    std::vector<double> b_gold = {12., 17., 0., 5., 28.};
    checkResults(b_gold, b);

    // check dimensions
    ASSERT_EQ(built_matrix->getNumRows(), num_rows);
    ASSERT_EQ(built_matrix->getNumColumns(), num_columns);

    // check matrix by matvecs
    std::vector<double> output;

    // e1
    std::vector<double> e1 = {1., 0., 0., 0.};
    built_matrix->matVec(e1, output, false);
    std::vector<double> expected_output_e1 = {0., 7., 0., 5., 4.};
    checkResults(expected_output_e1, output);

    // e2
    std::vector<double> e2 = {0., 1., 0., 0.};
    built_matrix->matVec(e2, output, false);
    std::vector<double> expected_output_e2 = {0., 1., 0., 0., 0.};
    checkResults(expected_output_e2, output);

    // e3
    std::vector<double> e3 = {0., 0., 1., 0.};
    built_matrix->matVec(e3, output, false);
    std::vector<double> expected_output_e3 = {0., 0., 0., 0., 0.};
    checkResults(expected_output_e3, output);

    // e4
    std::vector<double> e4 = {0., 0., 0., 1.};
    built_matrix->matVec(e4, output, false);
    std::vector<double> expected_output_e4 = {3., 2., 0., 0., 6.};
    checkResults(expected_output_e4, output);

    // clean up
    delete built_matrix;
}

PSL_TEST(CompressedRowSparseMatrixBuilder,matVecTranspose)
{
    set_rand_seed();
    // test the matVec(...) with the transpose option
    CompressedRowSparseMatrixBuilderTest_AllocateUtilities

    /* A = [ 0 0 0 3
             7 1 0 2
             0 0 0 0
             5 0 0 0
             4 0 0 6 ] */

    // with builder
    const size_t num_rows = 5u;
    const size_t num_columns = 4u;
    builder->begin_build(num_rows, num_columns);
    for(size_t repeat = 0; repeat < 2u; repeat++)
    {
        builder->specify_nonzero(0, 3, 3.);
        builder->specify_nonzero(1, 0, 7.);
        builder->specify_nonzero(1, 1, 1.);
        builder->specify_nonzero(1, 3, 2.);
        builder->specify_nonzero(3, 0, 5.);
        builder->specify_nonzero(4, 0, 4.);
        builder->specify_nonzero(4, 3, 6.);
        builder->advance_pass();
    }
    AbstractInterface::SparseMatrix* built_matrix = builder->end_build();

    std::vector<double> x = {1., 2., 3., 4., 5.};
    std::vector<double> b;
    built_matrix->matVec(x, b, true);

    std::vector<double> b_gold = {54., 2., 0., 37.};
    checkResults(b_gold, b);

    // clean up
    delete built_matrix;
}

PSL_TEST(CompressedRowSparseMatrixBuilder,matVecToReduced)
{
    set_rand_seed();
    // test the matVecToReduced(...)
    CompressedRowSparseMatrixBuilderTest_AllocateUtilities

    /* A = [ 0 0 0 3
             7 1 0 2
             0 0 0 0
             5 0 0 0
             4 0 0 6 ] */

    // with builder
    const size_t num_rows = 5u;
    const size_t num_columns = 4u;
    builder->begin_build(num_rows, num_columns);
    for(size_t repeat = 0; repeat < 2u; repeat++)
    {
        builder->specify_nonzero(0, 3, 3.);
        builder->specify_nonzero(1, 0, 7.);
        builder->specify_nonzero(1, 1, 1.);
        builder->specify_nonzero(1, 3, 2.);
        builder->specify_nonzero(3, 0, 5.);
        builder->specify_nonzero(4, 0, 4.);
        builder->specify_nonzero(4, 3, 6.);
        builder->advance_pass();
    }
    AbstractInterface::SparseMatrix* built_matrix = builder->end_build();

    std::vector<double> x = {1., 2., 3., 4.};
    std::vector<double> b;
    built_matrix->matVecToReduced(x, b, false);

    std::vector<double> b_gold = {12., 17., 5., 28.};
    checkResults(b_gold, b);

    // clean up
    delete built_matrix;
}

PSL_TEST(CompressedRowSparseMatrixBuilder,matVecToReducedTranspose)
{
    set_rand_seed();
    // test the matVecToReduced(...) with the transpose option
    CompressedRowSparseMatrixBuilderTest_AllocateUtilities

    /* A = [ 0 0 0 3
             7 1 0 2
             0 0 0 0
             5 0 0 0
             4 0 0 6 ] */

    // with builder
    const size_t num_rows = 5u;
    const size_t num_columns = 4u;
    builder->begin_build(num_rows, num_columns);
    for(size_t repeat = 0; repeat < 2u; repeat++)
    {
        builder->specify_nonzero(0, 3, 3.);
        builder->specify_nonzero(1, 0, 7.);
        builder->specify_nonzero(1, 1, 1.);
        builder->specify_nonzero(1, 3, 2.);
        builder->specify_nonzero(3, 0, 5.);
        builder->specify_nonzero(4, 0, 4.);
        builder->specify_nonzero(4, 3, 6.);
        builder->advance_pass();
    }
    AbstractInterface::SparseMatrix* built_matrix = builder->end_build();

    std::vector<double> x = {1., 2., 3., 4., 5.};
    std::vector<double> b;
    built_matrix->matVecToReduced(x, b, true);

    std::vector<double> b_gold = {54., 2., 37.};
    checkResults(b_gold, b);

    // clean up
    delete built_matrix;
}

PSL_TEST(CompressedRowSparseMatrixBuilder,sendAndRecv)
{
    set_rand_seed();
    // test the sendCompressedRowSparseMatrix(...) and receiveCompressedRowSparseMatrix(...)
    CompressedRowSparseMatrixBuilderTest_AllocateUtilities

    AbstractInterface::MpiWrapper* mpi_wrapper = builder->get_mpi_wrapper();

    const size_t rank = mpi_wrapper->get_rank();
    const size_t size = mpi_wrapper->get_size();

    if(size > 1u)
    {
        /* A = [ 0 7 0 rank
                 0 0 0 0
                 1 0 0 0 ] */
        const size_t num_rows = 3u;
        const size_t num_columns = 4u;

        builder->begin_build(num_rows, num_columns);
        for(size_t repeat = 0; repeat < 2u; repeat++)
        {
            builder->specify_nonzero(0, 1, 7.);
            builder->specify_nonzero(0, 3, double(rank));
            builder->specify_nonzero(2, 0, 1.);
            builder->advance_pass();
        }
        AbstractInterface::SparseMatrix* built_matrix = builder->end_build();

        if(rank == 0u)
        {
            AbstractInterface::SparseMatrix* recv_matrix;

            // receive from other processors, test against gold
            for(size_t other_rank = 1u; other_rank < size; other_rank++)
            {
                recv_matrix = builder->receive_matrix(other_rank);

                // check dimensions
                EXPECT_EQ(recv_matrix->getNumRows(), num_rows);
                EXPECT_EQ(recv_matrix->getNumColumns(), num_columns);

                // check matrix by matvecs
                std::vector<double> output;

                // e1
                std::vector<double> e1 = {1., 0., 0., 0.};
                recv_matrix->matVec(e1, output, false);
                std::vector<double> expected_output_e1 = {0., 0., 1.};
                checkResults(expected_output_e1, output);

                // e2
                std::vector<double> e2 = {0., 1., 0., 0.};
                recv_matrix->matVec(e2, output, false);
                std::vector<double> expected_output_e2 = {7., 0., 0.};
                checkResults(expected_output_e2, output);

                // e3
                std::vector<double> e3 = {0., 0., 1., 0.};
                recv_matrix->matVec(e3, output, false);
                std::vector<double> expected_output_e3 = {0., 0., 0.};
                checkResults(expected_output_e3, output);

                // e4
                std::vector<double> e4 = {0., 0., 0., 1.};
                recv_matrix->matVec(e4, output, false);
                std::vector<double> expected_output_e4 = {double(other_rank), 0., 0.};
                checkResults(expected_output_e4, output);

                delete recv_matrix;
            }
        }
        else
        {
            builder->send_matrix(0u, built_matrix);
        }

        delete built_matrix;
    }
}

PSL_TEST(CompressedRowSparseMatrixBuilder,transpose)
{
    set_rand_seed();
    // test transposeCompressedRowSparseMatrix(...)
    CompressedRowSparseMatrixBuilderTest_AllocateUtilities

    /* A = [ 0 5 4 7
             0 0 0 0
             1 0 3 0 ] */

    // build abstract
    const size_t num_rows = 3u;
    const size_t num_columns = 4u;
    builder->begin_build(num_rows, num_columns);
    for(size_t repeat = 0; repeat < 2u; repeat++)
    {
        builder->specify_nonzero(0, 1, 5.);
        builder->specify_nonzero(0, 2, 4.);
        builder->specify_nonzero(0, 3, 7.);
        builder->specify_nonzero(2, 0, 1.);
        builder->specify_nonzero(2, 2, 3.);
        builder->advance_pass();
    }
    AbstractInterface::SparseMatrix* built_matrix = builder->end_build();

    // transpose
    AbstractInterface::SparseMatrix* transposed_built_matrix = builder->transpose(built_matrix);

    /* A = [ 0 0 1
             5 0 0
             4 0 3
             7 0 0 ] */
    const size_t transpose_num_rows = num_columns;
    const size_t transpose_num_columns = num_rows;

    // check dimensions
    ASSERT_EQ(transposed_built_matrix->getNumRows(), transpose_num_rows);
    ASSERT_EQ(transposed_built_matrix->getNumColumns(), transpose_num_columns);

    // check matrix by matvecs
    std::vector<double> output;

    // e1
    std::vector<double> e1 = {1., 0., 0.};
    transposed_built_matrix->matVec(e1, output, false);
    std::vector<double> expected_output_e1 = {0., 5., 4., 7.};
    checkResults(expected_output_e1, output);

    // e2
    std::vector<double> e2 = {0., 1., 0.};
    transposed_built_matrix->matVec(e2, output, false);
    std::vector<double> expected_output_e2 = {0., 0., 0., 0.};
    checkResults(expected_output_e2, output);

    // e3
    std::vector<double> e3 = {0., 0., 1.};
    transposed_built_matrix->matVec(e3, output, false);
    std::vector<double> expected_output_e3 = {1., 0., 3., 0.};
    checkResults(expected_output_e3, output);

    delete built_matrix;
    delete transposed_built_matrix;
}

PSL_TEST(CompressedRowSparseMatrixBuilder,OutOfRowOrderContruction)
{
    set_rand_seed();
    // test specification out of row order
    CompressedRowSparseMatrixBuilderTest_AllocateUtilities

    /* A = [ 0 0 0 3
             7 1 0 2
             0 0 0 0
             5 0 0 0
             4 0 0 6 ] */

    // with builder
    const size_t num_rows = 5u;
    const size_t num_columns = 4u;
    builder->begin_build(num_rows, num_columns);
    for(size_t repeat = 0; repeat < 2u; repeat++)
    {
        builder->specify_nonzero(1, 1, 1.);
        builder->specify_nonzero(3, 0, 5.);
        builder->specify_nonzero(1, 0, 7.);
        builder->specify_nonzero(0, 3, 3.);
        builder->specify_nonzero(4, 3, 6.);
        builder->specify_nonzero(4, 0, 4.);
        builder->specify_nonzero(1, 3, 2.);
        builder->advance_pass();
    }
    AbstractInterface::SparseMatrix* built_matrix = builder->end_build();

    // check dimensions
    ASSERT_EQ(built_matrix->getNumRows(), num_rows);
    ASSERT_EQ(built_matrix->getNumColumns(), num_columns);

    // check matrix by matvecs
    std::vector<double> output;

    // e1
    std::vector<double> e1 = {1., 0., 0., 0.};
    built_matrix->matVec(e1, output, false);
    std::vector<double> expected_output_e1 = {0., 7., 0., 5., 4.};
    checkResults(expected_output_e1, output);

    // e2
    std::vector<double> e2 = {0., 1., 0., 0.};
    built_matrix->matVec(e2, output, false);
    std::vector<double> expected_output_e2 = {0., 1., 0., 0., 0.};
    checkResults(expected_output_e2, output);

    // e3
    std::vector<double> e3 = {0., 0., 1., 0.};
    built_matrix->matVec(e3, output, false);
    std::vector<double> expected_output_e3 = {0., 0., 0., 0., 0.};
    checkResults(expected_output_e3, output);

    // e4
    std::vector<double> e4 = {0., 0., 0., 1.};
    built_matrix->matVec(e4, output, false);
    std::vector<double> expected_output_e4 = {3., 2., 0., 0., 6.};
    checkResults(expected_output_e4, output);

    // clean up
    delete built_matrix;
}

}

}
