#include "PSL_UnitTestingHelper.hpp"

#include "PSL_ClassificationArchive.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_VectorCoding.hpp"
#include "PSL_Interface_DenseMatrixBuilder.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_InputOutputManager.hpp"
#include "PSL_Random.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"

namespace PlatoSubproblemLibrary
{

namespace TestingClassificationArchive
{

PSL_TEST(ClassificationArchive, simple)
{
    set_rand_seed();
    AbstractAuthority authority;
    ClassificationArchive archive(&authority);
    ParameterData parameter_data;

    // initialize archive
    parameter_data.set_archive_filename("_simple_archive.psl");
    archive.initialize(&parameter_data);

    // enum sizes
    std::vector<int> gold_input_enum_sizes = {3, 2};
    int gold_output_enum_size = 4;
    archive.set_enum_size(gold_input_enum_sizes, gold_output_enum_size);

    // add row0
    std::vector<double> scalar_data_row0 = {0.1, 0.3, -0.5};
    std::vector<int> input_enum_data_row0 = {0, 1};
    int output_enum_row0 = 3;
    archive.add_row(scalar_data_row0, input_enum_data_row0, output_enum_row0);
    archive.finalize_archive();

    // check archive enum sizes
    std::vector<int> computed_input_enum_sizes;
    int computed_output_enum_size = -1;
    archive.get_enum_size(computed_input_enum_sizes, computed_output_enum_size);
    expect_equal_vectors(computed_input_enum_sizes, gold_input_enum_sizes);
    EXPECT_EQ(computed_output_enum_size, gold_output_enum_size);

    // get archived data
    AbstractInterface::DenseMatrix* computed_first_matrix = archive.get_all_rows_onehot_encoded();

    // build gold matrix
    std::vector<double> gold_row0;
    onehot_encode(scalar_data_row0,
                  input_enum_data_row0,
                  gold_input_enum_sizes,
                  output_enum_row0,
                  gold_output_enum_size,
                  gold_row0);
    const size_t num_matrix_columns = gold_row0.size();
    AbstractInterface::DenseMatrix* gold_first_matrix = authority.dense_builder->build_by_row_major(1,
                                                                                                    num_matrix_columns,
                                                                                                    gold_row0);

    // check archived data
    expect_equal_matrix(gold_first_matrix, computed_first_matrix);
    delete computed_first_matrix;
    delete gold_first_matrix;

    // add row1
    std::vector<double> scalar_data_row1 = {4.2, -5.1, -2.4};
    std::vector<int> input_enum_data_row1 = {2, 0};
    int output_enum_row1 = 0;
    archive.add_row(scalar_data_row1, input_enum_data_row1, output_enum_row1);

    // add row2
    std::vector<double> scalar_data_row2 = {4.2, -5.1, -2.4};
    std::vector<int> input_enum_data_row2 = {2, 0};
    int output_enum_row2 = 0;
    archive.add_row(scalar_data_row2, input_enum_data_row2, output_enum_row2);
    archive.finalize_archive();

    // get archived data
    AbstractInterface::DenseMatrix* computed_second_matrix = archive.get_all_rows_onehot_encoded();

    // build gold matrix
    std::vector<double> second_matrix_by_row = gold_row0;
    std::vector<double> gold_row1;
    onehot_encode(scalar_data_row1,
                  input_enum_data_row1,
                  gold_input_enum_sizes,
                  output_enum_row1,
                  gold_output_enum_size,
                  gold_row1);
    second_matrix_by_row.insert(second_matrix_by_row.end(), gold_row1.begin(), gold_row1.end());
    std::vector<double> gold_row2;
    onehot_encode(scalar_data_row2,
                  input_enum_data_row2,
                  gold_input_enum_sizes,
                  output_enum_row2,
                  gold_output_enum_size,
                  gold_row2);
    second_matrix_by_row.insert(second_matrix_by_row.end(), gold_row2.begin(), gold_row2.end());
    AbstractInterface::DenseMatrix* gold_second_matrix = authority.dense_builder->build_by_row_major(3,
                                                                                                     num_matrix_columns,
                                                                                                     second_matrix_by_row);

    // check archived data
    expect_equal_matrix(gold_second_matrix, computed_second_matrix);
    delete gold_second_matrix;

    // check archive enum sizes
    computed_input_enum_sizes.assign(3, -1);
    computed_output_enum_size = -72;
    archive.get_enum_size(computed_input_enum_sizes, computed_output_enum_size);
    expect_equal_vectors(computed_input_enum_sizes, gold_input_enum_sizes);
    EXPECT_EQ(computed_output_enum_size, gold_output_enum_size);

    // get from fresh archive
    ClassificationArchive fresh_archive(&authority);
    fresh_archive.initialize(&parameter_data);

    // check fresh archive enum sizes
    std::vector<int> fresh_input_enum_sizes = {4, 7, 9, 11};
    int fresh_output_enum_size = 1;
    fresh_archive.get_enum_size(fresh_input_enum_sizes, fresh_output_enum_size);
    expect_equal_vectors(fresh_input_enum_sizes, gold_input_enum_sizes);
    EXPECT_EQ(fresh_output_enum_size, gold_output_enum_size);

    // get fresh archived data
    AbstractInterface::DenseMatrix* fresh_second_matrix = fresh_archive.get_all_rows_onehot_encoded();

    // check archived data
    expect_equal_matrix(fresh_second_matrix, computed_second_matrix);
    delete fresh_second_matrix;
    delete computed_second_matrix;

    // clean-up archive file
    fresh_archive.get_io_manager()->delete_file();
}

void mark_found_rows(AbstractInterface::DenseMatrix* global_matrix,
                     AbstractInterface::DenseMatrix* subset_matrix,
                     std::vector<bool>& found_rows)
{
    // for each row
    const int subset_num_rows = subset_matrix->get_num_rows();
    for(int r = 0; r < subset_num_rows; r++)
    {
        std::vector<double> division_this_row;
        subset_matrix->get_row(r, division_this_row);

        // first column contains global row index
        const int global_row = division_this_row[0];

        // first time found
        EXPECT_EQ(found_rows[global_row], false);
        found_rows[global_row] = true;

        // get row from global
        std::vector<double> global_this_row;
        global_matrix->get_row(global_row, global_this_row);

        // expect match
        expect_equal_float_vectors(global_this_row, division_this_row);
    }
}

PSL_TEST(ClassificationArchive, splitDataset)
{
    set_rand_seed();
    AbstractAuthority authority;
    ClassificationArchive archive(&authority);
    ParameterData parameter_data;

    // initialize archive
    const std::string original_filename = "_simple_archive.psl";
    parameter_data.set_archive_filename(original_filename);
    archive.initialize(&parameter_data);

    // enum sizes
    std::vector<int> gold_input_enum_sizes = {2, 3};
    int gold_output_enum_size = 4;
    archive.set_enum_size(gold_input_enum_sizes, gold_output_enum_size);

    // pose data set
    std::vector<std::vector<double> > scalar_data = { {0., .1, .2, -.5},
                                                      {1., 2.1, .8, -4.5},
                                                      {2., 3.2, .3, 5.},
                                                      {3., 1.2, .2, -.5},
                                                      {4., -5.1, .1, 3.},
                                                      {5., 4.9, 4., 2.},
                                                      {6., 7.2, -7., 6.},
                                                      {7., 5.7, -3., .1}};
    std::vector<std::vector<int> > input_enum_data = { {0, 1},
                                                       {1, 2},
                                                       {0, 0},
                                                       {1, 0},
                                                       {1, 2},
                                                       {0, 1},
                                                       {0, 2},
                                                       {1, 2}};
    std::vector<int> output_enum_data = {0, 3, 2, 1, 2, 1, 0, 3};

    // insert data set
    const int total_num_insert_rows = output_enum_data.size();
    for(int i = 0; i < total_num_insert_rows; i++)
    {
        archive.add_row(scalar_data[i], input_enum_data[i], output_enum_data[i]);
    }
    archive.finalize_archive();

    // check archive enum sizes
    std::vector<int> computed_input_enum_sizes;
    int computed_output_enum_size = -1;
    archive.get_enum_size(computed_input_enum_sizes, computed_output_enum_size);
    expect_equal_vectors(computed_input_enum_sizes, gold_input_enum_sizes);
    EXPECT_EQ(computed_output_enum_size, gold_output_enum_size);

    // get archived data
    AbstractInterface::DenseMatrix* global_matrix = archive.get_all_rows_onehot_encoded();
    const int total_num_rows = global_matrix->get_num_rows();
    EXPECT_EQ(total_num_rows, total_num_insert_rows);

    // split it
    std::vector<double> percents = {.125, .25, .625};
    std::vector<std::string> split_names;
    archive.split_dataset(percents, split_names);
    const int num_divisions = split_names.size();

    // unite for each possible omission
    for(int omit_unite = 0; omit_unite < num_divisions; omit_unite++)
    {
        // get names
        std::vector<std::string> unite_input_names;
        for(int div = 0; div < num_divisions; div++)
        {
            if(div != omit_unite)
            {
                unite_input_names.push_back(split_names[div]);
            }
        }

        // unite
        ClassificationArchive omit_archive(&authority);
        std::string united_filename;
        omit_archive.unite_dataset(unite_input_names, united_filename);

        // set parameter data
        ParameterData united_parameter_data;
        united_parameter_data.set_archive_filename(united_filename);
        omit_archive.initialize(&united_parameter_data);

        // expect sizes
        computed_input_enum_sizes.clear();
        computed_output_enum_size = -1;
        omit_archive.get_enum_size(computed_input_enum_sizes, computed_output_enum_size);
        expect_equal_vectors(computed_input_enum_sizes, gold_input_enum_sizes);
        EXPECT_EQ(computed_output_enum_size, gold_output_enum_size);

        // find each row
        std::vector<bool> found_for_unites(total_num_rows, false);

        // get matrix
        AbstractInterface::DenseMatrix* unite_matrix = omit_archive.get_all_rows_onehot_encoded();

        mark_found_rows(global_matrix, unite_matrix, found_for_unites);

        // expect proper subset
        const int num_found = count(found_for_unites);
        EXPECT_GT(num_found, 0);
        EXPECT_GT(total_num_rows, num_found);

        // clean up this archive
        omit_archive.get_io_manager()->delete_file();
        delete unite_matrix;
    }

    // find each row
    std::vector<bool> found_for_divisions(total_num_rows, false);

    // for each division archive
    for(int div = 0; div < num_divisions; div++)
    {
        // set parameter data
        ParameterData division_parameter_data;
        division_parameter_data.set_archive_filename(split_names[div]);

        // get archive
        ClassificationArchive division_archive(&authority);
        division_archive.initialize(&division_parameter_data);

        // expect sizes
        computed_input_enum_sizes.clear();
        computed_output_enum_size = -1;
        archive.get_enum_size(computed_input_enum_sizes, computed_output_enum_size);
        expect_equal_vectors(computed_input_enum_sizes, gold_input_enum_sizes);
        EXPECT_EQ(computed_output_enum_size, gold_output_enum_size);

        // get matrix
        AbstractInterface::DenseMatrix* division_matrix = division_archive.get_all_rows_onehot_encoded();

        mark_found_rows(global_matrix, division_matrix, found_for_divisions);

        // clean up this archive
        division_archive.get_io_manager()->delete_file();
        delete division_matrix;
    }

    // expect all found
    std::vector<bool> all_found(total_num_rows, true);
    expect_equal_vectors(found_for_divisions, all_found);

    // clean up original archive
    archive.get_io_manager()->delete_file();
    delete global_matrix;
}

}
}
