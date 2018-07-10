#pragma once

#include <vector>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class DenseMatrix;
}

void onehot_encode(const std::vector<double>& input_scalar,
                   const std::vector<int>& input_enum,
                   const std::vector<int>& input_enum_sizes,
                   std::vector<double>& onehot_input_row);

void onehot_encode(const std::vector<double>& input_scalar,
                   const std::vector<int>& input_enum,
                   const std::vector<int>& input_enum_sizes,
                   const int& output_enum,
                   const int& output_enum_size,
                   std::vector<double>& onehot_row);

void onehot_decode(const std::vector<double>& onehot_row,
                   const std::vector<int>& input_enum_sizes,
                   const int& output_enum_size,
                   std::vector<double>& input_scalar,
                   std::vector<int>& input_enum,
                   int& output_enum);

void onehot_decode(const std::vector<double>& onehot_row, const int& output_enum_size, std::vector<double>& all_input);

void onehot_decode(const std::vector<double>& onehot_row, const int& output_enum_size, int& output_enum);

void onehot_decode_all_rows(AbstractInterface::DenseMatrix* dataset,
                            const int& output_enum_size,
                            std::vector<std::vector<int> >& rows_of_each_output_enum);

void onehot_split(const std::vector<double>& onehot_row,
                  const int& output_enum_size,
                  std::vector<double>& all_input,
                  std::vector<double>& all_output);

void sidebyside_encode(const std::vector<double>& input_scalar,
                       const std::vector<int>& input_enum,
                       const int& output_enum,
                       std::vector<double>& sidebyside_row);
void sidebyside_decode(const std::vector<double>& sidebyside_row,
                       const int& num_input_enums,
                       std::vector<double>& input_scalar,
                       std::vector<int>& input_enum,
                       int& output_enum);


}
