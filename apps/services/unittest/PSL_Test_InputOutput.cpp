#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Interface_DenseMatrix.hpp"
#include "PSL_InputOutputManager.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Random.hpp"

#include <string>
#include <mpi.h>
#include <algorithm>
#include <fstream>
#include <vector>

namespace PlatoSubproblemLibrary
{
namespace TestingInputOutput
{

PSL_TEST(InputOutput, basicReadWrite)
{
    set_rand_seed();
    AbstractAuthority authority;
    InputOutputManager io_manager(&authority);

    // build scalar
    double scalar_value = 1.92;

    // make empty file
    std::string example_filename = "_basic_example.psl";
    io_manager.set_filename(example_filename);

    // expect empty at start
    const bool is_empty_at_start = io_manager.is_empty();
    EXPECT_EQ(true, is_empty_at_start);

    // output
    io_manager.open_file();
    io_manager.write_scalar(scalar_value);
    io_manager.close_file();

    // input
    double scalar_read = -1;
    io_manager.open_file();
    io_manager.read_scalar(scalar_read);
    io_manager.close_file();

    EXPECT_FLOAT_EQ(scalar_read, scalar_value);

    // expect non-empty at end
    const bool is_empty_at_end = io_manager.is_empty();
    EXPECT_EQ(is_empty_at_end,false);

    // clean up file
    io_manager.delete_file();
}

PSL_TEST(InputOutput, multipleReadWrite)
{
    set_rand_seed();
    AbstractAuthority authority;
    InputOutputManager io_manager(&authority);

    // build scalar
    double scalar_value = 42.197;
    int int_value = 421972;

    // build matrix
    int num_rows = 2;
    int num_columns = 3;
    std::vector<double> matrix_values = {1., 0., .5, 2., 4., .25};
    AbstractInterface::DenseMatrix* matrix = authority.dense_builder->build_by_row_major(num_rows, num_columns, matrix_values);
    example::Interface_DenseMatrix* e_matrix = dynamic_cast<example::Interface_DenseMatrix*>(matrix);

    // build vector
    std::vector<double> vector_values = {-3., 4.2, 7.15, 6.};
    const size_t vec_length = vector_values.size();

    // make empty file
    std::string example_filename = "_multiple_example.psl";
    io_manager.set_filename(example_filename);

    // begin output
    io_manager.open_file();

    // write output
    io_manager.write_scalar(scalar_value);
    io_manager.write_scalar(int_value);
    io_manager.write_matrix(e_matrix->m_data, num_rows, num_columns);
    io_manager.write_vector(vector_values);

    // end output
    io_manager.close_file();

    // begin input
    io_manager.open_file();

    // read matrix
    double scalar_read = -1;
    int int_read = -1;
    double** matrix_read = NULL;
    int read_num_rows = -1;
    int read_num_cols = -1;
    std::vector<double> vector_read;
    io_manager.read_scalar(scalar_read);
    io_manager.read_scalar(int_read);
    io_manager.read_matrix(matrix_read, read_num_rows, read_num_cols);
    io_manager.read_vector(vector_read);

    // end input
    io_manager.close_file();

    // expect read size to be same
    ASSERT_EQ(read_num_rows, num_rows);
    ASSERT_EQ(read_num_cols, num_columns);
    ASSERT_EQ(vec_length, vector_read.size());

    EXPECT_FLOAT_EQ(scalar_read, scalar_value);
    EXPECT_EQ(int_read, int_value);

    // build read matrix
    example::Interface_DenseMatrix e_read_matrix(authority.utilities, authority.dense_builder);
    e_read_matrix.receive(matrix_read, num_rows, num_columns);

    // expect matrix equal
    expect_equal_matrix(&e_read_matrix, e_matrix);

    // expect vector equal
    expect_equal_float_vectors(vector_values, vector_read);

    // clean up file
    io_manager.delete_file();

    delete matrix;
}

PSL_TEST(InputOutput, scalarSkips)
{
    set_rand_seed();
    AbstractAuthority authority;
    InputOutputManager io_manager(&authority);

    // build scalar
    double scalar_value_0 = 1.92;
    double scalar_value_1 = -4.2;

    // make empty file
    std::string example_filename = "_skips_example.psl";
    io_manager.set_filename(example_filename);

    // output
    io_manager.open_file();
    io_manager.write_scalar(scalar_value_0);
    io_manager.write_scalar(scalar_value_1);
    io_manager.close_file();

    // input
    double scalar_read = -1;

    // skip, then read
    io_manager.open_file();
    io_manager.skip_scalar<double>();
    io_manager.read_scalar(scalar_read);
    io_manager.close_file();
    EXPECT_FLOAT_EQ(scalar_read, scalar_value_1);

    // read, then skip
    io_manager.open_file();
    io_manager.read_scalar(scalar_read);
    io_manager.skip_scalar<double>();
    io_manager.close_file();
    EXPECT_FLOAT_EQ(scalar_read, scalar_value_0);

    // new values
    double new_scalar_value_0 = 11.27;
    double new_scalar_value_1 = 5.19;
    EXPECT_NE(scalar_value_0, new_scalar_value_0);
    EXPECT_NE(scalar_value_1, new_scalar_value_1);

    // skip, then write
    io_manager.open_file();
    io_manager.skip_scalar<double>();
    io_manager.write_scalar(new_scalar_value_1);
    io_manager.close_file();

    // write, then skip
    io_manager.open_file();
    io_manager.write_scalar(new_scalar_value_0);
    io_manager.skip_scalar<double>();
    io_manager.close_file();

    // read finals
    double final_scalar_value_0 = -1;
    double final_scalar_value_1 = -1;
    io_manager.open_file();
    io_manager.read_scalar(final_scalar_value_0);
    io_manager.read_scalar(final_scalar_value_1);
    io_manager.close_file();

    EXPECT_FLOAT_EQ(final_scalar_value_0, new_scalar_value_0);
    EXPECT_FLOAT_EQ(final_scalar_value_1, new_scalar_value_1);

    // clean up file
    io_manager.delete_file();
}

void test_inputOutput_vectorSkips(const bool& prefer_binary)
{
    AbstractAuthority authority;
    InputOutputManager io_manager(&authority);

    // build vectors
    std::vector<double> vector_value_0 = {5.2, -5.4, 3.1};
    std::vector<int> vector_value_1 = {42, 31, 17, -19};

    // make empty file
    std::string example_filename = "_vector_skips_example.psl";
    io_manager.set_filename(example_filename);
    io_manager.set_prefer_binary(prefer_binary);

    // output
    io_manager.open_file();
    io_manager.write_vector(vector_value_0);
    io_manager.write_vector(vector_value_1);
    io_manager.close_file();

    // read buffers
    std::vector<double> read_vector_0;
    std::vector<int> read_vector_1;

    // skip, then read
    io_manager.open_file();
    io_manager.skip_vector<double>();
    io_manager.read_vector(read_vector_1);
    io_manager.close_file();
    expect_equal_vectors(read_vector_1, vector_value_1);

    // read, then skip
    io_manager.open_file();
    io_manager.read_vector(read_vector_0);
    io_manager.skip_vector<int>();
    io_manager.close_file();
    expect_equal_float_vectors(read_vector_0, vector_value_0);

    // build new vectors
    std::vector<double> new_vector_0 = {2.5, -4.5, 1.3};
    std::vector<int> new_vector_1 = {24, 13, 71, -91, 152};

    // skip, then write
    io_manager.open_file();
    io_manager.skip_vector<double>();
    io_manager.write_vector(new_vector_1);
    io_manager.close_file();

    if(prefer_binary)
    {
        // write, then skip
        io_manager.open_file();
        io_manager.write_vector(new_vector_0);
        io_manager.skip_vector<int>();
        io_manager.close_file();
    }

    // read finals
    std::vector<double> final_vector_0;
    std::vector<int> final_vector_1;
    io_manager.open_file();
    io_manager.read_vector(final_vector_0);
    io_manager.read_vector(final_vector_1);
    io_manager.close_file();

    if(prefer_binary)
    {
        expect_equal_float_vectors(final_vector_0, new_vector_0);
    }
    else
    {
        expect_equal_float_vectors(final_vector_0, vector_value_0);
    }
    expect_equal_vectors(final_vector_1, new_vector_1);

    // compare file to expectations
    EXPECT_EQ(io_manager.is_empty(), false);
    EXPECT_EQ(prefer_binary, io_manager.is_binary());

    io_manager.delete_file();
}
PSL_TEST(InputOutput, vectorSkips_binary)
{
    set_rand_seed();
    test_inputOutput_vectorSkips(true);
}
PSL_TEST(InputOutput, vectorSkips_plainText)
{
    set_rand_seed();
    test_inputOutput_vectorSkips(false);
}

PSL_TEST(InputOutput, matrixSkips)
{
    set_rand_seed();
    AbstractAuthority authority;
    InputOutputManager io_manager(&authority);

    // build some matrices
    int num_rows_0 = 2;
    int num_columns_0 = 5;
    std::vector<double> matrix_values_0 = {1., 0., .5, 2., 4., .25, -1.2, 7., .1, 4.2};
    AbstractInterface::DenseMatrix* matrix_0 = authority.dense_builder->build_by_row_major(num_rows_0,
                                                                                           num_columns_0,
                                                                                           matrix_values_0);
    example::Interface_DenseMatrix* e_matrix_0 = dynamic_cast<example::Interface_DenseMatrix*>(matrix_0);
    int num_rows_1 = 3;
    int num_columns_1 = 4;
    std::vector<double> matrix_values_1 = {3., -4., 7., .12, .15, .72, .31, .59, -.9, -.2, -.5, .2};
    AbstractInterface::DenseMatrix* matrix_1 = authority.dense_builder->build_by_row_major(num_rows_1,
                                                                                           num_columns_1,
                                                                                           matrix_values_1);
    example::Interface_DenseMatrix* e_matrix_1 = dynamic_cast<example::Interface_DenseMatrix*>(matrix_1);

    // make empty file
    std::string example_filename = "_matrix_skips_example.psl";
    io_manager.set_filename(example_filename);

    // output
    io_manager.open_file();
    io_manager.write_matrix(e_matrix_0->m_data, num_rows_0, num_columns_0);
    io_manager.write_matrix(e_matrix_1->m_data, num_rows_1, num_columns_1);
    io_manager.close_file();

    // read buffers
    int read_row_size_0 = -1;
    int read_column_size_0 = -1;
    double** read_matrix_0 = NULL;
    int read_row_size_1 = -1;
    int read_column_size_1 = -1;
    double** read_matrix_1 = NULL;

    // skip, then read
    io_manager.open_file();
    io_manager.skip_matrix<double>();
    io_manager.read_matrix(read_matrix_1, read_row_size_1, read_column_size_1);
    io_manager.close_file();

    // check matrix
    example::Interface_DenseMatrix e_read_matrix_1(authority.utilities, authority.dense_builder);
    e_read_matrix_1.receive(read_matrix_1, read_row_size_1, read_column_size_1);
    expect_equal_matrix(e_matrix_1, &e_read_matrix_1);

    // read, then skip
    io_manager.open_file();
    io_manager.read_matrix(read_matrix_0, read_row_size_0, read_column_size_0);
    io_manager.skip_matrix<double>();
    io_manager.close_file();

    // check matrix
    example::Interface_DenseMatrix e_read_matrix_0(authority.utilities, authority.dense_builder);
    e_read_matrix_0.receive(read_matrix_0, read_row_size_0, read_column_size_0);
    expect_equal_matrix(e_matrix_0, &e_read_matrix_0);

    // build new vectors
    std::vector<double> new_matrix_values_0 = {5.7, 1.3, 1., 0., .5, 2., 4., .25, -1.2, 7.};
    AbstractInterface::DenseMatrix* new_matrix_0 = authority.dense_builder->build_by_row_major(num_rows_0, num_columns_0, new_matrix_values_0);
    example::Interface_DenseMatrix* e_new_matrix_0 = dynamic_cast<example::Interface_DenseMatrix*>(new_matrix_0);
    std::vector<double> new_matrix_values_1 = {6., 3., -4., 2.1, 5.1, 7., .12, .15, .72, .31, .59, -.9};
    AbstractInterface::DenseMatrix* new_matrix_1 = authority.dense_builder->build_by_row_major(num_rows_1, num_columns_1, new_matrix_values_1);
    example::Interface_DenseMatrix* e_new_matrix_1 = dynamic_cast<example::Interface_DenseMatrix*>(new_matrix_1);

    // skip, then write
    io_manager.open_file();
    io_manager.skip_matrix<double>();
    io_manager.write_matrix(e_new_matrix_1->m_data, num_rows_1, num_columns_1);
    io_manager.close_file();

    // write, then skip
    io_manager.open_file();
    io_manager.write_matrix(e_new_matrix_0->m_data, num_rows_0, num_columns_0);
    io_manager.skip_vector<int>();
    io_manager.close_file();

    // allocate finals
    int final_row_size_0 = -1;
    int final_column_size_0 = -1;
    double** final_matrix_0 = NULL;
    int final_row_size_1 = -1;
    int final_column_size_1 = -1;
    double** final_matrix_1 = NULL;

    // read finals
    io_manager.open_file();
    io_manager.read_matrix(final_matrix_0, final_row_size_0, final_column_size_0);
    io_manager.read_matrix(final_matrix_1, final_row_size_1, final_column_size_1);
    io_manager.close_file();

    // transfer finals
    example::Interface_DenseMatrix e_final_matrix_0(authority.utilities, authority.dense_builder);
    e_final_matrix_0.receive(final_matrix_0, final_row_size_0, final_column_size_0);
    example::Interface_DenseMatrix e_final_matrix_1(authority.utilities, authority.dense_builder);
    e_final_matrix_1.receive(final_matrix_1, final_row_size_1, final_column_size_1);

    // check finals
    expect_equal_matrix(e_new_matrix_0, &e_final_matrix_0);
    expect_equal_matrix(e_new_matrix_1, &e_final_matrix_1);

    io_manager.delete_file();

    delete matrix_0;
    delete matrix_1;
    delete new_matrix_0;
    delete new_matrix_1;
}

PSL_TEST(InputOutput, vectorAppend)
{
    set_rand_seed();
    AbstractAuthority authority;
    InputOutputManager io_manager(&authority);

    // build data
    int value_0 = 142;
    double value_1 = 24.13;
    std::vector<double> vector_values = {5.2, -5.4, 3.1};

    // make empty file
    std::string example_filename = "_vector_append_example.psl";
    io_manager.set_filename(example_filename);

    // output
    io_manager.open_file();
    io_manager.write_scalar(value_0);
    io_manager.write_scalar(value_1);
    io_manager.write_vector(vector_values);
    io_manager.close_file();

    // append data
    double append_value = -42.42;

    // skip, skip, append
    io_manager.open_file();
    io_manager.skip_scalar<int>();
    io_manager.skip_scalar<double>();
    io_manager.append_vector(append_value);
    io_manager.close_file();

    // read buffers
    int read_value_0 = -1;
    double read_value_1 = -1.;
    std::vector<double> read_vector;

    // skip, skip, read
    io_manager.open_file();
    io_manager.skip_scalar<int>();
    io_manager.skip_scalar<double>();
    io_manager.read_vector(read_vector);
    io_manager.close_file();

    // skip, read, skip
    io_manager.open_file();
    io_manager.skip_scalar<int>();
    io_manager.read_scalar(read_value_1);
    io_manager.skip_vector<double>();
    io_manager.close_file();

    // read, skip, skip
    io_manager.open_file();
    io_manager.read_scalar(read_value_0);
    io_manager.skip_scalar<double>();
    io_manager.skip_vector<double>();
    io_manager.close_file();

    EXPECT_EQ(value_0, read_value_0);
    EXPECT_FLOAT_EQ(value_1, read_value_1);
    vector_values.push_back(append_value);
    expect_equal_float_vectors(vector_values, read_vector);

    io_manager.delete_file();
}

PSL_TEST(InputOutput, matrixAppend)
{
    set_rand_seed();
    AbstractAuthority authority;
    InputOutputManager io_manager(&authority);

    // build data
    int scalar = 142;
    std::vector<double> vector_values = {5.2, -5.4, 3.1, 17.42};
    int num_rows = 2;
    int num_columns = 3;
    std::vector<double> matrix_values = {1., 0., .5, 2., 4., .25};
    AbstractInterface::DenseMatrix* matrix = authority.dense_builder->build_by_row_major(num_rows, num_columns, matrix_values);
    example::Interface_DenseMatrix* e_matrix = dynamic_cast<example::Interface_DenseMatrix*>(matrix);

    // make empty file
    std::string example_filename = "_matrix_append_example.psl";
    io_manager.set_filename(example_filename);

    // output
    io_manager.open_file();
    io_manager.write_scalar(scalar);
    io_manager.write_vector(vector_values);
    io_manager.write_matrix(e_matrix->m_data, num_rows, num_columns);
    io_manager.close_file();

    // append row
    std::vector<std::vector<double> > append_values = { {2., -3.1, 4.27}, {1.7, 0., -.1}};

    // skip, skip, append
    io_manager.open_file();
    io_manager.skip_scalar<int>();
    io_manager.skip_vector<double>();
    io_manager.append_rows_to_matrix(append_values);
    io_manager.close_file();

    // build full matrix
    std::vector<double> full_values = matrix_values;
    full_values.insert(full_values.end(), append_values[0].begin(), append_values[0].end());
    full_values.insert(full_values.end(), append_values[1].begin(), append_values[1].end());
    AbstractInterface::DenseMatrix* full_matrix = authority.dense_builder->build_by_row_major(num_rows + 2,
                                                                                              num_columns,
                                                                                              full_values);

    // read buffers
    int read_value = -1;
    std::vector<double> read_vector;
    int read_num_rows = -1;
    int read_num_columns = -1;
    double** read_matrix = NULL;

    // skip, skip, read
    io_manager.open_file();
    io_manager.skip_scalar<int>();
    io_manager.skip_vector<double>();
    io_manager.read_matrix(read_matrix, read_num_rows, read_num_columns);
    io_manager.close_file();

    // skip, read, skip
    io_manager.open_file();
    io_manager.skip_scalar<int>();
    io_manager.read_vector(read_vector);
    io_manager.skip_matrix<double>();
    io_manager.close_file();

    // read, skip, skip
    io_manager.open_file();
    io_manager.read_scalar(read_value);
    io_manager.skip_vector<double>();
    io_manager.skip_matrix<double>();
    io_manager.close_file();

    // convert read matrix
    example::Interface_DenseMatrix e_read_matrix(authority.utilities, authority.dense_builder);
    e_read_matrix.receive(read_matrix, read_num_rows, read_num_columns);

    EXPECT_EQ(scalar, read_value);
    expect_equal_float_vectors(vector_values, read_vector);
    expect_equal_matrix(full_matrix, &e_read_matrix);

    EXPECT_EQ(io_manager.is_empty(), false);
    EXPECT_EQ(true, io_manager.is_binary());
    io_manager.delete_file();

    delete matrix;
    delete full_matrix;
}

}
}
