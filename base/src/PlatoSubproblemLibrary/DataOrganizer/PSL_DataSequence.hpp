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

#include "PSL_ParameterDataEnums.hpp"

#include <vector>
#include <string>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class DenseMatrix;
}
class AbstractAuthority;

class DataSequence
{
public:
    DataSequence(AbstractAuthority* authority,
                 data_flow_t::data_flow_t type);
    virtual ~DataSequence();

    void set_data_flow_type(const data_flow_t::data_flow_t& type);
    data_flow_t::data_flow_t get_data_flow_type();

    void save_to_file(const std::string& filename,
                      const bool& prefer_binary);
    void load_from_file(const std::string& filename);
    void write_as_code_file();

    // integer scalars
    void resize_ints(const int& size);
    int get_ints_size();
    int get_int(const int& index);
    int* get_int_ptr(const int& index);
    void set_int(const int& index, const int& value);
    void get_all_ints(std::vector<int>& output);
    void expect_atleast_ints(const int& minimum_expectation);
    void enqueue_int(const int& value);
    int dequeue_int(const int& multiplicity = 1);

    // double scalars
    void resize_doubles(const int& size);
    int get_doubles_size();
    double get_double(const int& index);
    double* get_double_ptr(const int& index);
    void set_double(const int& index, const double& value);
    void get_all_doubles(std::vector<double>& output);
    void expect_atleast_doubles(const int& minimum_expectation);
    void enqueue_double(const double& value);
    int dequeue_double(const int& multiplicity = 1);

    // integer vectors
    void resize_int_vectors(const int& size);
    int get_int_vectors_size();
    std::vector<int>* get_int_vector(const int& index);
    void set_int_vector(const int& index, std::vector<int>* value);
    void transfer_int_vector(const int& index, const std::vector<int>& value);
    void expect_atleast_int_vector(const int& minimum_expectation);
    int enqueue_int_vector(const int& multiplicity = 1);
    int dequeue_int_vector(const int& multiplicity = 1);

    // double vectors
    void resize_double_vectors(const int& size);
    int get_double_vectors_size();
    std::vector<double>* get_double_vector(const int& index);
    void set_double_vector(const int& index, std::vector<double>* value);
    void transfer_double_vector(const int& index, const std::vector<double>& value);
    void expect_atleast_double_vector(const int& minimum_expectation);
    int enqueue_double_vector(const int& multiplicity = 1);
    int dequeue_double_vector(const int& multiplicity = 1);

    // matrices
    void resize_matrices(const int& size);
    int get_matrices_size();
    AbstractInterface::DenseMatrix* get_matrix(const int& index);
    AbstractInterface::DenseMatrix** get_matrix_ptr(const int& index);
    void set_matrix(const int& index, AbstractInterface::DenseMatrix* value);
    void transfer_matrix(const int& index, AbstractInterface::DenseMatrix* value);
    void expect_atleast_matrix(const int& minimum_expectation);
    int enqueue_matrix(const int& multiplicity = 1);
    int dequeue_matrix(const int& multiplicity = 1);

protected:

    void generic_expect_atleast(const int& minimum_expectation,
                                const int& current_size,
                                const std::string& type_name);
    void clear_owned_matrices(const int& inclusive_begin, const int& exclusive_end);

    AbstractAuthority* m_authority;
    data_flow_t::data_flow_t m_data_flow_type;
    std::vector<int> m_int_scalar;
    std::vector<double> m_double_scalar;
    // ownership is the responsibility to deallocate memory
    std::vector<std::vector<int> > m_owned_int_vector;
    std::vector<std::vector<int>* > m_not_owned_int_vector;
    std::vector<std::vector<double> > m_owned_double_vector;
    std::vector<std::vector<double>* > m_not_owned_double_vector;
    std::vector<AbstractInterface::DenseMatrix* > m_owned_matrix;
    std::vector<AbstractInterface::DenseMatrix* > m_not_owned_matrix;
    int m_dequeue_index_int_scalar;
    int m_dequeue_index_double_scalar;
    int m_dequeue_index_int_vector;
    int m_dequeue_index_double_vector;
    int m_dequeue_index_matrix;

};

}
