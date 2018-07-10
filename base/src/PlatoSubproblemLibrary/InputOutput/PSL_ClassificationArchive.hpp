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

#pragma once

#include <vector>
#include <string>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class DenseMatrix;
}
class AbstractAuthority;
class ParameterData;
class InputOutputManager;

class ClassificationArchive
{
public:
    ClassificationArchive(AbstractAuthority* authority);
    ~ClassificationArchive();

    void initialize(ParameterData* parameters);

    // supporting information about enum sizes
    void set_enum_size(const std::vector<int>& input_enum_sizes, const int& output_enum_size);
    void get_enum_size(std::vector<int>& input_enum_sizes, int& output_enum_size);

    // add a row to the archive
    void add_row(const std::vector<double>& input_scalar, const std::vector<int>& input_enum, const int& output_enum);
    void add_sidebyside_encoded_rows(const std::vector<std::vector<double> >& rows);
    void finalize_archive();

    // get entire archive
    AbstractInterface::DenseMatrix* get_all_rows_sidebyside_encoded();
    AbstractInterface::DenseMatrix* get_all_rows_onehot_encoded();
    void get_all_rows(std::vector<std::vector<double> >& input_scalars,
                      std::vector<std::vector<int> >& input_enums,
                      std::vector<int>& output_enums);

    // split data set
    void split_dataset(const double& percent, std::string& first_name, std::string& second_name);
    void split_dataset(const std::vector<double>& percents, std::vector<std::string>& names);

    void unite_dataset(const std::vector<std::string>& input_names, std::string& output_name);

    void delete_archive_file();
    InputOutputManager* get_io_manager();

protected:

    void internal_get_all_rows(double**& matrix_read, int& num_rows, int& num_columns);

    AbstractAuthority* m_authority;
    ParameterData* m_parameters;
    std::vector<int> m_input_enum_sizes;
    int m_output_enum_size;
    InputOutputManager* m_io_manager;
    std::vector<std::vector<double>> m_stored_rows;
};

}
