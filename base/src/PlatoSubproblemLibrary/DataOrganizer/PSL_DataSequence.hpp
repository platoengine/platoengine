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
