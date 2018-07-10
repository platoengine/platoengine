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

#include "PSL_DataSequence.hpp"

#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Interface_DenseMatrix.hpp"
#include "PSL_InputOutputManager.hpp"

#include <vector>
#include <string>
#include <cstddef>
#include <cassert>

namespace PlatoSubproblemLibrary
{

DataSequence::DataSequence(AbstractAuthority* authority, data_flow_t::data_flow_t type) :
        m_authority(authority),
        m_data_flow_type(type),
        m_int_scalar(),
        m_double_scalar(),
        m_owned_int_vector(),
        m_not_owned_int_vector(),
        m_owned_double_vector(),
        m_not_owned_double_vector(),
        m_owned_matrix(),
        m_not_owned_matrix(),
        m_dequeue_index_int_scalar(-1),
        m_dequeue_index_double_scalar(-1),
        m_dequeue_index_int_vector(-1),
        m_dequeue_index_double_vector(-1),
        m_dequeue_index_matrix(-1)
{
}

DataSequence::~DataSequence()
{
    // clean up matrices
    clear_owned_matrices(0, m_owned_matrix.size());
    m_owned_matrix.clear();
    m_not_owned_matrix.clear();
}

void DataSequence::set_data_flow_type(const data_flow_t::data_flow_t& type)
{
    m_data_flow_type = type;
}
data_flow_t::data_flow_t DataSequence::get_data_flow_type()
{
    return m_data_flow_type;
}

void DataSequence::save_to_file(const std::string& filename,
                                const bool& prefer_binary)
{
    InputOutputManager io_manager(m_authority);
    io_manager.set_filename(filename);
    io_manager.set_prefer_binary(prefer_binary);

    // save
    io_manager.open_file();

    // write type
    const int int_type = m_data_flow_type;
    io_manager.write_scalar(int_type);

    // write sizes
    const int num_ints = get_ints_size();
    io_manager.write_scalar(num_ints);
    const int num_doubles = get_doubles_size();
    io_manager.write_scalar(num_doubles);
    const int num_int_vectors = get_int_vectors_size();
    io_manager.write_scalar(num_int_vectors);
    const int num_double_vectors = get_double_vectors_size();
    io_manager.write_scalar(num_double_vectors);
    const int num_matrices = get_matrices_size();
    io_manager.write_scalar(num_matrices);

    // write integer scalars
    for(int i = 0; i < num_ints; i++)
    {
        io_manager.write_scalar(get_int(i));
    }

    // write double scalars
    for(int d = 0; d < num_doubles; d++)
    {
        io_manager.write_scalar(get_double(d));
    }

    // write integer vectors
    for(int i = 0; i < num_int_vectors; i++)
    {
        io_manager.write_vector(*get_int_vector(i));
    }

    // write double vectors
    for(int d = 0; d < num_double_vectors; d++)
    {
        io_manager.write_vector(*get_double_vector(d));
    }

    // write matrices
    for(int m = 0; m < num_matrices; m++)
    {
        AbstractInterface::DenseMatrix* matrix = get_matrix(m);
        std::vector<std::vector<double> > matrix_in_row_major;
        matrix->get_row_major(matrix_in_row_major);
        io_manager.write_matrix(matrix_in_row_major);
    }

    io_manager.close_file();
}
void DataSequence::load_from_file(const std::string& filename)
{
    InputOutputManager io_manager(m_authority);

    io_manager.set_filename(filename);

    // load
    io_manager.open_file();

    // read type
    int flow_type = -1;
    io_manager.read_scalar(flow_type);
    m_data_flow_type = data_flow_t::data_flow_t(flow_type);

    // read sizes
    int num_ints = -1;
    io_manager.read_scalar(num_ints);
    int num_doubles = -1;
    io_manager.read_scalar(num_doubles);
    int num_int_vectors = -1;
    io_manager.read_scalar(num_int_vectors);
    int num_double_vectors = -1;
    io_manager.read_scalar(num_double_vectors);
    int num_matrices = -1;
    io_manager.read_scalar(num_matrices);

    // read integer scalars
    resize_ints(num_ints);
    for(int i = 0; i < num_ints; i++)
    {
        int tmp_int = -1;
        io_manager.read_scalar(tmp_int);
        set_int(i, tmp_int);
    }

    // read double scalars
    resize_doubles(num_doubles);
    for(int d = 0; d < num_doubles; d++)
    {
        double tmp_double = -1.;
        io_manager.read_scalar(tmp_double);
        set_double(d, tmp_double);
    }

    // read integer vectors
    resize_int_vectors(num_int_vectors);
    for(int i = 0; i < num_int_vectors; i++)
    {
        std::vector<int> tmp_ints;
        io_manager.read_vector(tmp_ints);
        transfer_int_vector(i, tmp_ints);
    }

    // read double vectors
    resize_double_vectors(num_double_vectors);
    for(int d = 0; d < num_double_vectors; d++)
    {
        std::vector<double> tmp_doubles;
        io_manager.read_vector(tmp_doubles);
        transfer_double_vector(d, tmp_doubles);
    }

    // read matrices
    resize_matrices(num_matrices);
    for(int m = 0; m < num_matrices; m++)
    {
        double** matrix;
        int num_rows = -1;
        int num_columns = -1;
        io_manager.read_matrix(matrix, num_rows, num_columns);

        // TODO: technically a violation of abstraction
        example::Interface_DenseMatrix* e_read_matrix = new example::Interface_DenseMatrix(m_authority->utilities, NULL);
        e_read_matrix->receive(matrix, num_rows, num_columns);

        transfer_matrix(m, e_read_matrix);
    }

    io_manager.close_file();
}
void DataSequence::write_as_code_file()
{
    // TODO
}

// integer scalars
void DataSequence::resize_ints(const int& size)
{
    m_int_scalar.resize(size);
}
int DataSequence::get_ints_size()
{
    return m_int_scalar.size();
}
int DataSequence::get_int(const int& index)
{
    return m_int_scalar[index];
}
int* DataSequence::get_int_ptr(const int& index)
{
    return &m_int_scalar[index];
}
void DataSequence::set_int(const int& index, const int& value)
{
    m_int_scalar[index] = value;
}
void DataSequence::get_all_ints(std::vector<int>& output)
{
    output.assign(m_int_scalar.begin(), m_int_scalar.end());
}
void DataSequence::expect_atleast_ints(const int& minimum_expectation)
{
    generic_expect_atleast(minimum_expectation, get_ints_size(), "ints");
}
void DataSequence::enqueue_int(const int& value)
{
    m_int_scalar.push_back(value);
}
int DataSequence::dequeue_int(const int& multiplicity)
{
    assert(0 <= multiplicity);
    const int first_dequeue = ++m_dequeue_index_int_scalar;
    m_dequeue_index_int_scalar += multiplicity - 1;
    return first_dequeue;
}

// double scalars
void DataSequence::resize_doubles(const int& size)
{
    m_double_scalar.resize(size);
}
int DataSequence::get_doubles_size()
{
    return m_double_scalar.size();
}
double DataSequence::get_double(const int& index)
{
    return m_double_scalar[index];
}
double* DataSequence::get_double_ptr(const int& index)
{
    return &m_double_scalar[index];
}
void DataSequence::set_double(const int& index, const double& value)
{
    m_double_scalar[index] = value;
}
void DataSequence::get_all_doubles(std::vector<double>& output)
{
    output.assign(m_double_scalar.begin(), m_double_scalar.end());
}
void DataSequence::expect_atleast_doubles(const int& minimum_expectation)
{
    generic_expect_atleast(minimum_expectation, get_doubles_size(), "doubles");
}
void DataSequence::enqueue_double(const double& value)
{
    m_double_scalar.push_back(value);
}
int DataSequence::dequeue_double(const int& multiplicity)
{
    assert(0 <= multiplicity);
    const int first_dequeue = ++m_dequeue_index_double_scalar;
    m_dequeue_index_double_scalar += multiplicity - 1;
    return first_dequeue;
}

// integer vectors
void DataSequence::resize_int_vectors(const int& size)
{
    m_not_owned_int_vector.resize(size, NULL);
    m_owned_int_vector.resize(size);
}
int DataSequence::get_int_vectors_size()
{
    return m_not_owned_int_vector.size();
}
std::vector<int>* DataSequence::get_int_vector(const int& index)
{
    std::vector<int>* not_owned_result = m_not_owned_int_vector[index];
    if(not_owned_result)
    {
        return not_owned_result;
    }

    // must be owned
    return &m_owned_int_vector[index];
}
void DataSequence::set_int_vector(const int& index, std::vector<int>* value)
{
    m_owned_int_vector[index].clear();
    m_not_owned_int_vector[index] = value;
}
void DataSequence::transfer_int_vector(const int& index, const std::vector<int>& value)
{
    m_owned_int_vector[index] = value;
    m_not_owned_int_vector[index] = NULL;
}
void DataSequence::expect_atleast_int_vector(const int& minimum_expectation)
{
    generic_expect_atleast(minimum_expectation, get_int_vectors_size(), "int vectors");
}
int DataSequence::enqueue_int_vector(const int& multiplicity)
{
    assert(0 <= multiplicity);
    const int result = get_int_vectors_size();
    resize_int_vectors(result + multiplicity);
    return result;
}
int DataSequence::dequeue_int_vector(const int& multiplicity)
{
    assert(0 <= multiplicity);
    const int first_dequeue = ++m_dequeue_index_int_vector;
    m_dequeue_index_int_vector += multiplicity - 1;
    return first_dequeue;
}

// double vectors
void DataSequence::resize_double_vectors(const int& size)
{
    m_not_owned_double_vector.resize(size, NULL);
    m_owned_double_vector.resize(size);
}
int DataSequence::get_double_vectors_size()
{
    return m_not_owned_double_vector.size();
}
std::vector<double>* DataSequence::get_double_vector(const int& index)
{
    std::vector<double>* not_owned_result = m_not_owned_double_vector[index];
    if(not_owned_result)
    {
        return not_owned_result;
    }

    // must be owned
    return &m_owned_double_vector[index];
}
void DataSequence::set_double_vector(const int& index, std::vector<double>* value)
{
    m_owned_double_vector[index].clear();
    m_not_owned_double_vector[index] = value;
}
void DataSequence::transfer_double_vector(const int& index, const std::vector<double>& value)
{
    m_owned_double_vector[index] = value;
    m_not_owned_double_vector[index] = NULL;
}
void DataSequence::expect_atleast_double_vector(const int& minimum_expectation)
{
    generic_expect_atleast(minimum_expectation, get_double_vectors_size(), "double vectors");
}
int DataSequence::enqueue_double_vector(const int& multiplicity)
{
    assert(0 <= multiplicity);
    const int result = get_double_vectors_size();
    resize_double_vectors(result + multiplicity);
    return result;
}
int DataSequence::dequeue_double_vector(const int& multiplicity)
{
    assert(0 <= multiplicity);
    const int first_dequeue = ++m_dequeue_index_double_vector;
    m_dequeue_index_double_vector += multiplicity - 1;
    return first_dequeue;
}

// matrices
void DataSequence::resize_matrices(const int& size)
{
    const int initial_size = m_owned_matrix.size();
    clear_owned_matrices(size, initial_size);
    m_owned_matrix.resize(size, NULL);
    m_not_owned_matrix.resize(size, NULL);
}
int DataSequence::get_matrices_size()
{
    return m_owned_matrix.size();
}
AbstractInterface::DenseMatrix* DataSequence::get_matrix(const int& index)
{
    return *get_matrix_ptr(index);
}
AbstractInterface::DenseMatrix** DataSequence::get_matrix_ptr(const int& index)
{
    if(m_not_owned_matrix[index])
    {
        return &m_not_owned_matrix[index];
    }

    // must be owned
    return &m_owned_matrix[index];
}
void DataSequence::set_matrix(const int& index, AbstractInterface::DenseMatrix* value)
{
    clear_owned_matrices(index, index + 1);
    m_not_owned_matrix[index] = value;
}
void DataSequence::transfer_matrix(const int& index, AbstractInterface::DenseMatrix* value)
{
    clear_owned_matrices(index, index + 1);
    m_not_owned_matrix[index] = NULL;
    m_owned_matrix[index] = value;
}
void DataSequence::expect_atleast_matrix(const int& minimum_expectation)
{
    generic_expect_atleast(minimum_expectation, get_matrices_size(), "matrices");
}
int DataSequence::enqueue_matrix(const int& multiplicity)
{
    assert(0 <= multiplicity);
    const int result = get_matrices_size();
    resize_matrices(result + multiplicity);
    return result;
}
int DataSequence::dequeue_matrix(const int& multiplicity)
{
    assert(0 <= multiplicity);
    const int first_dequeue = ++m_dequeue_index_matrix;
    m_dequeue_index_matrix += multiplicity - 1;
    return first_dequeue;
}

void DataSequence::generic_expect_atleast(const int& minimum_expectation, const int& current_size, const std::string& type_name)
{
    if(current_size < minimum_expectation)
    {
        m_authority->utilities->fatal_error(std::string("DataSequence: expected at least ") + std::to_string(minimum_expectation)
                                            + std::string(" ")
                                            + type_name
                                            + std::string(", but found only ")
                                            + std::to_string(current_size)
                                            + std::string("\n\n"));
    }
}
void DataSequence::clear_owned_matrices(const int& inclusive_begin, const int& exclusive_end)
{
    for(int m = inclusive_begin; m < exclusive_end; m++)
    {
        safe_free(m_owned_matrix[m]);
    }
}

}
