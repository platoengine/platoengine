#include "PSL_ClassificationArchive.hpp"

#include "PSL_ParameterData.hpp"
#include "PSL_InputOutputManager.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_VectorCoding.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_Interface_DenseMatrix.hpp"
#include "PSL_Random.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Implementation_DenseMatrix.hpp"

#include <vector>
#include <string>
#include <cassert>

namespace PlatoSubproblemLibrary
{

ClassificationArchive::ClassificationArchive(AbstractAuthority* authority) :
        m_authority(authority),
        m_parameters(NULL),
        m_input_enum_sizes(),
        m_output_enum_size(-1),
        m_io_manager(NULL),
        m_stored_rows()
{
}

ClassificationArchive::~ClassificationArchive()
{
    m_authority = NULL;
    m_parameters = NULL;
    safe_free(m_io_manager);
}

void ClassificationArchive::initialize(ParameterData* parameters)
{
    m_parameters = parameters;

    // prepare io
    m_io_manager = new InputOutputManager(m_authority);
    std::string filename = m_parameters->get_archive_filename();
    m_io_manager->set_filename(filename);
    if(parameters->didUserInput_archive_filetype_binary_preference())
    {
        m_io_manager->set_prefer_binary(parameters->get_archive_filetype_binary_preference());
    }
}

// supporting information about enum sizes
void ClassificationArchive::set_enum_size(const std::vector<int>& input_enum_sizes, const int& output_enum_size)
{
    m_input_enum_sizes = input_enum_sizes;
    m_output_enum_size = output_enum_size;
}
void ClassificationArchive::get_enum_size(std::vector<int>& input_enum_sizes, int& output_enum_size)
{
    // ensure file non-empty
    if(m_io_manager->is_empty())
    {
        m_authority->utilities->fatal_error(std::string(__func__) + std::string(": cannot get sizes from empty file"));
        return;
    }

    // read sizes
    m_io_manager->open_file();
    input_enum_sizes.clear();
    m_io_manager->read_vector(input_enum_sizes);
    m_io_manager->read_scalar(output_enum_size);
    m_io_manager->close_file();

    m_input_enum_sizes = input_enum_sizes;
    m_output_enum_size = output_enum_size;
}

// add a row to the archive
void ClassificationArchive::add_row(const std::vector<double>& input_scalar,
                                    const std::vector<int>& input_enum,
                                    const int& output_enum)
{
    // encode row
    m_stored_rows.push_back(std::vector<double>());
    sidebyside_encode(input_scalar, input_enum, output_enum, m_stored_rows.back());
}

void ClassificationArchive::add_sidebyside_encoded_rows(const std::vector<std::vector<double> >& rows)
{
    m_stored_rows.insert(m_stored_rows.end(), rows.begin(), rows.end());
}

void ClassificationArchive::finalize_archive()
{
    if(m_io_manager->is_empty())
    {
        // file is empty; needs sizes header

        m_io_manager->open_file();

        // write enum sizes
        m_io_manager->write_vector(m_input_enum_sizes);
        m_io_manager->write_scalar(m_output_enum_size);

        // write archive matrix
        m_io_manager->write_matrix(m_stored_rows);

        m_io_manager->close_file();
    }
    else
    {
        // file is not empty; contains sizes header and an archive matrix

        m_io_manager->open_file();

        // skip enum sizes
        m_io_manager->skip_vector<int>();
        m_io_manager->skip_scalar<int>();

        // add row to matrix
        m_io_manager->append_rows_to_matrix(m_stored_rows);

        m_io_manager->close_file();
    }
    m_stored_rows.clear();
}

// get entire archive
AbstractInterface::DenseMatrix* ClassificationArchive::get_all_rows_sidebyside_encoded()
{
    double** matrix_read = NULL;
    int num_rows = -1;
    int num_columns = -1;
    internal_get_all_rows(matrix_read, num_rows, num_columns);

    // build dense matrix
    // TODO: technically a violation of abstraction
    example::Interface_DenseMatrix* e_read_matrix = new example::Interface_DenseMatrix(m_authority->utilities, NULL);
    e_read_matrix->receive(matrix_read, num_rows, num_columns);
    return e_read_matrix;
}

AbstractInterface::DenseMatrix* ClassificationArchive::get_all_rows_onehot_encoded()
{
    double** matrix_read = NULL;
    int num_rows = -1;
    int num_columns = -1;
    internal_get_all_rows(matrix_read, num_rows, num_columns);

    // build dense matrix
    example::Interface_DenseMatrix e_read_matrix(m_authority->utilities, NULL);
    e_read_matrix.receive(matrix_read, num_rows, num_columns);

    // get sizes
    std::vector<int> input_enum_sizes;
    int output_enum_size = -1;
    get_enum_size(input_enum_sizes, output_enum_size);
    const int num_input_enums = input_enum_sizes.size();

    // prepare output matrix
    double** output_matrix = NULL;
    int output_num_columns = -1;

    // convert each row
    std::vector<double> this_row_original;
    std::vector<double> this_row_scalars;
    std::vector<int> this_row_input_enums;
    int this_row_output_enum;
    std::vector<double> this_row_onehot;
    for(int row = 0; row < num_rows; row++)
    {
        // get sidebyside
        e_read_matrix.get_row(row, this_row_original);

        // decode sidebyside
        sidebyside_decode(this_row_original, num_input_enums, this_row_scalars, this_row_input_enums, this_row_output_enum);

        // get onehot
        onehot_encode(this_row_scalars,
                      this_row_input_enums,
                      input_enum_sizes,
                      this_row_output_enum,
                      output_enum_size,
                      this_row_onehot);

        // if first row, allocate output
        if(row <= 0)
        {
            output_num_columns = this_row_onehot.size();
            output_matrix = example::dense_matrix::create(num_rows, output_num_columns);
        }

        // transfer to output matrix
        std::copy(this_row_onehot.begin(), this_row_onehot.end(), &output_matrix[row][0]);
    }

    // build output
    example::Interface_DenseMatrix* output = new example::Interface_DenseMatrix(m_authority->utilities, NULL);
    output->receive(output_matrix, num_rows, output_num_columns);

    return output;
}
void ClassificationArchive::get_all_rows(std::vector<std::vector<double> >& input_scalars,
                                         std::vector<std::vector<int> >& input_enums,
                                         std::vector<int>& output_enums)
{
    double** matrix_read = NULL;
    int num_rows = -1;
    int num_columns = -1;
    internal_get_all_rows(matrix_read, num_rows, num_columns);

    // allocate results
    input_scalars.resize(num_rows);
    input_enums.resize(num_rows);
    output_enums.resize(num_rows);

    // build dense matrix
    example::Interface_DenseMatrix e_read_matrix(m_authority->utilities, NULL);
    e_read_matrix.receive(matrix_read, num_rows, num_columns);

    // get sizes
    std::vector<int> input_enum_sizes;
    int output_enum_size = -1;
    get_enum_size(input_enum_sizes, output_enum_size);
    const int num_input_enums = input_enum_sizes.size();

    // convert each row
    std::vector<double> this_row_original;
    for(int row = 0; row < num_rows; row++)
    {
        // get sidebyside
        e_read_matrix.get_row(row, this_row_original);

        // decode sidebyside
        sidebyside_decode(this_row_original, num_input_enums, input_scalars[row], input_enums[row], output_enums[row]);
    }
}

void ClassificationArchive::split_dataset(const double& percent, std::string& first_name, std::string& second_name)
{
    // determine percents
    std::vector<double> percents = {percent, 1. - percent};
    assert((0. <= percent) && (percent <= 1.));

    // split
    std::vector<std::string> names;
    split_dataset(percents, names);

    // transfer names
    assert(names.size() == 2u);
    first_name = names[0];
    second_name = names[1];
}
void ClassificationArchive::split_dataset(const std::vector<double>& percents, std::vector<std::string>& names)
{
    const std::string original_filename = m_io_manager->get_filename();
    const std::string noPrefix_filename = remove_all_filename_prefix(original_filename);

    // get sizes
    std::vector<int> input_enum_sizes;
    int output_enum_size = -1;
    get_enum_size(input_enum_sizes, output_enum_size);

    // get data set
    AbstractInterface::DenseMatrix* dataset = get_all_rows_sidebyside_encoded();
    const int total_num_rows = dataset->get_num_rows();

    // get divisions
    std::vector<std::vector<int> > divisions;
    random_division(total_num_rows, percents, divisions);
    const int num_divisions = divisions.size();
    names.resize(num_divisions);

    // write divisions
    for(int div = 0; div < num_divisions; div++)
    {
        const int division_num_rows = divisions[div].size();
        std::vector<std::vector<double> > division_dataset(division_num_rows);

        // get rows
        for(int r = 0; r < division_num_rows; r++)
        {
            dataset->get_row(divisions[div][r], division_dataset[r]);
        }

        // set division name
        names[div] = std::string("_") + std::to_string(div) + std::string("of") + std::to_string(num_divisions) + std::string("_")
                     + noPrefix_filename;

        ClassificationArchive division_archive(m_authority);

        // parameter data for division
        ParameterData division_parameters;
        division_parameters.set_archive_filename(names[div]);

        // initialize
        division_archive.initialize(&division_parameters);
        division_archive.set_enum_size(input_enum_sizes, output_enum_size);

        // write rows
        division_archive.add_sidebyside_encoded_rows(division_dataset);
        division_archive.finalize_archive();
    }

    // cleanup memory
    safe_free(dataset);
}

void ClassificationArchive::unite_dataset(const std::vector<std::string>& input_names, std::string& output_name)
{
    // allocate data
    std::vector<std::vector<double> > all_datasets;
    size_t all_datasets_counter = 0u;

    // allocate sizes
    std::vector<int> input_enum_sizes;
    int output_enum_size = -1;

    // for each input
    const int num_input = input_names.size();
    for(int i = 0; i < num_input; i++)
    {
        ClassificationArchive division_archive(m_authority);

        // parameter data for division
        ParameterData division_parameters;
        division_parameters.set_archive_filename(input_names[i]);

        division_archive.initialize(&division_parameters);

        // get sizes
        if(i == 0)
        {
            division_archive.get_enum_size(input_enum_sizes, output_enum_size);
        }

        // get data
        AbstractInterface::DenseMatrix* dataset = division_archive.get_all_rows_sidebyside_encoded();

        // transfer to all_datasets
        const size_t num_rows = dataset->get_num_rows();
        all_datasets.resize(all_datasets_counter + num_rows);
        for(size_t r = 0u; r < num_rows; r++)
        {
            dataset->get_row(r, all_datasets[all_datasets_counter + r]);
        }
        all_datasets_counter += num_rows;

        safe_free(dataset);
    }

    // set united name
    output_name = std::string("_united_") + std::to_string(num_input) + std::string("_with_first_") + input_names[0];

    // parameter data for unite
    ParameterData united_parameters;
    united_parameters.set_archive_filename(output_name);

    // initialize
    ClassificationArchive united_archive(m_authority);
    united_archive.initialize(&united_parameters);
    united_archive.set_enum_size(input_enum_sizes, output_enum_size);

    // write rows
    united_archive.add_sidebyside_encoded_rows(all_datasets);
    united_archive.finalize_archive();
}

InputOutputManager* ClassificationArchive::get_io_manager()
{
    return m_io_manager;
}

void ClassificationArchive::internal_get_all_rows(double**& matrix_read, int& num_rows, int& num_columns)
{
    if(m_io_manager->is_empty())
    {
        m_authority->utilities->fatal_error(std::string(__func__) + std::string(": cannot get from empty file"));
    }

    m_io_manager->open_file();

    // skip enum sizes
    m_io_manager->skip_vector<int>();
    m_io_manager->skip_scalar<int>();

    // read matrix
    m_io_manager->read_matrix(matrix_read, num_rows, num_columns);

    m_io_manager->close_file();
}

}
