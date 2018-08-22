#include "PSL_VectorCoding.hpp"

#include "PSL_FreeHelpers.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"

#include <cstddef>
#include <vector>
#include <cassert>

namespace PlatoSubproblemLibrary
{

void onehot_encode(const std::vector<double>& input_scalar,
                   const std::vector<int>& input_enum,
                   const std::vector<int>& input_enum_sizes,
                   std::vector<double>& onehot_input_row)
{
    // begin with scalar elements
    onehot_input_row.assign(input_scalar.begin(), input_scalar.end());

    // allocate for enums
    int starting_index = onehot_input_row.size();
    onehot_input_row.resize(starting_index + sum(input_enum_sizes), 0.);

    // for each input enum
    const size_t num_input_enum = input_enum.size();
    assert(num_input_enum == input_enum_sizes.size());
    for(size_t input_enum_index = 0u; input_enum_index < num_input_enum; input_enum_index++)
    {
        const int this_enum_value = input_enum[input_enum_index];
        const int this_enum_size = input_enum_sizes[input_enum_index];
        assert((0 <= this_enum_value) && (this_enum_value < this_enum_size));

        // encode 1
        onehot_input_row[starting_index + this_enum_value] = 1.;
        starting_index += this_enum_size;
    }
}

void onehot_encode(const std::vector<double>& input_scalar,
                   const std::vector<int>& input_enum,
                   const std::vector<int>& input_enum_sizes,
                   const int& output_enum,
                   const int& output_enum_size,
                   std::vector<double>& onehot_row)
{
    onehot_encode(input_scalar, input_enum, input_enum_sizes, onehot_row);
    const size_t input_size = onehot_row.size();

    // allocate for output enum
    onehot_row.resize(input_size + output_enum_size, 0.);

    // encode output
    assert((0 <= output_enum) && (output_enum < output_enum_size));
    onehot_row[input_size + output_enum] = 1.;
}

void onehot_decode(const std::vector<double>& onehot_row,
                   const std::vector<int>& input_enum_sizes,
                   const int& output_enum_size,
                   std::vector<double>& input_scalar,
                   std::vector<int>& input_enum,
                   int& output_enum)
{
    // separate output_enum
    onehot_decode(onehot_row, output_enum_size, input_scalar);
    onehot_decode(onehot_row, output_enum_size, output_enum);

    // allocate input enums
    const int num_input_enum = input_enum_sizes.size();
    input_enum.assign(num_input_enum, -1);

    // fill input enums
    int ending_index = input_scalar.size();
    for(int which_input_enum = num_input_enum - 1; 0 <= which_input_enum; which_input_enum--)
    {
        const int this_input_enum_size = input_enum_sizes[which_input_enum];
        ending_index -= this_input_enum_size;

        // find first one, iterating forwards within this input enum
        for(int input_enum_index = 0; input_enum_index < this_input_enum_size; input_enum_index++)
        {
            if(input_scalar[ending_index + input_enum_index] == 1.)
            {
                input_enum[which_input_enum] = input_enum_index;
                break;
            }
        }

        assert(0 <= input_enum[which_input_enum]);
    }

    // truncate to only include scalars, not enums
    input_scalar.resize(ending_index);
}

void onehot_decode(const std::vector<double>& onehot_row, const int& output_enum_size, std::vector<double>& all_input)
{
    // set all input
    const int num_onehot_row = onehot_row.size();
    all_input.assign(onehot_row.data(), &onehot_row[num_onehot_row - output_enum_size]);
}

void onehot_decode(const std::vector<double>& onehot_row,
                   const int& output_enum_size,
                   int& output_enum)
{
    const int num_onehot_row = onehot_row.size();

    // set output enum
    output_enum = -1;
    for(int output_enum_index = 0; output_enum_index < output_enum_size; output_enum_index++)
    {
        if(onehot_row[num_onehot_row - output_enum_size + output_enum_index] == 1.)
        {
            output_enum = output_enum_index;
            break;
        }
    }
    assert(0 <= output_enum);
}

void onehot_decode_all_rows(AbstractInterface::DenseMatrix* dataset,
                            const int& output_enum_size,
                            std::vector<std::vector<int> >& rows_of_each_output_enum)
{
    // allocate rows
    rows_of_each_output_enum.assign(output_enum_size, std::vector<int>());

    // for each row
    const int num_rows = dataset->get_num_rows();
    for(int r = 0; r < num_rows; r++)
    {
        // get row
        std::vector<double> this_row;
        dataset->get_row(r, this_row);

        // get output enum value
        int this_output_value = -1;
        onehot_decode(this_row, output_enum_size, this_output_value);
        assert(0 <= this_output_value);

        // add to rows of that value
        rows_of_each_output_enum[this_output_value].push_back(r);
    }
}

void onehot_split(const std::vector<double>& onehot_row,
                  const int& output_enum_size,
                  std::vector<double>& all_input,
                  std::vector<double>& all_output)
{
    const int num_onehot_row = onehot_row.size();
    const int input_size = num_onehot_row - output_enum_size;
    const int output_size = output_enum_size;

    // set input
    all_input.resize(input_size);
    std::copy(&onehot_row[0], &onehot_row[num_onehot_row - output_enum_size], all_input.data());

    // set output
    all_output.resize(output_size);
    std::copy(onehot_row.begin() + (num_onehot_row - output_enum_size), onehot_row.end(), all_output.begin());
}

void sidebyside_encode(const std::vector<double>& input_scalar,
                       const std::vector<int>& input_enum,
                       const int& output_enum,
                       std::vector<double>& sidebyside_row)
{
    // count
    const size_t num_is = input_scalar.size();
    const size_t num_ie = input_enum.size();

    // allocate
    sidebyside_row.resize(num_is + num_ie + 1u);

    // fill
    for(size_t i = 0u; i < num_is; i++)
    {
        sidebyside_row[i] = input_scalar[i];
    }
    for(size_t i = 0u; i < num_ie; i++)
    {
        sidebyside_row[num_is + i] = input_enum[i];
    }
    sidebyside_row[num_is + num_ie] = output_enum;
}
void sidebyside_decode(const std::vector<double>& sidebyside_row,
                       const int& num_input_enums,
                       std::vector<double>& input_scalar,
                       std::vector<int>& input_enum,
                       int& output_enum)
{
    assert(num_input_enums >= 0);

    // count
    const int num_all = sidebyside_row.size();
    assert(num_input_enums < num_all);
    const int num_scalar = num_all - num_input_enums - 1;

    // allocate
    input_scalar.resize(num_scalar);
    input_enum.resize(num_input_enums);

    // fill
    for(int i = 0; i < num_scalar; i++)
    {
        input_scalar[i] = sidebyside_row[i];
    }
    for(int i = 0; i < num_input_enums; i++)
    {
        input_enum[i] = sidebyside_row[num_scalar + i];
    }
    output_enum = sidebyside_row[num_scalar + num_input_enums];
}

}
