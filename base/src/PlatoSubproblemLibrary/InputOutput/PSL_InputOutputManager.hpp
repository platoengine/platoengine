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

#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <string>
#include <vector>
#include <fstream>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;

class InputOutputManager
{
public:
    InputOutputManager(AbstractAuthority* authority);
    ~InputOutputManager();

    void set_filename(const std::string& filename);
    std::string get_filename();

    void open_file();
    bool is_empty();

    void choose_binary_preference();
    void choose_text_preference();
    bool get_prefer_binary() { return m_prefer_binary; }
    void set_prefer_binary(const bool& prefer) { m_prefer_binary = prefer; }

    bool is_binary();

    // scalar
    template<typename t>
    void write_scalar(t scalar)
    {
        if(!m_authority->mpi_wrapper->is_root())
        {
            return;
        }

        if(m_filestream == NULL)
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": called without a file stream open"));
            return;
        }
        if(m_filestream->fail())
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": error while operating on file"));
            return;
        }

        internal_write(scalar, true);
    }
    template<typename t>
    void read_scalar(t& scalar)
    {
        if(!m_authority->mpi_wrapper->is_root())
        {
            m_authority->mpi_wrapper->broadcast(0u, scalar);
            return;
        }

        if(m_filestream == NULL)
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": called without a file stream open"));
            return;
        }
        if(m_filestream->fail())
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": error while operating on file"));
            return;
        }

        internal_read(scalar);

        m_authority->mpi_wrapper->broadcast(0u, scalar);
    }
    template<typename t>
    void skip_scalar()
    {
        if(!m_authority->mpi_wrapper->is_root())
        {
            return;
        }

        if(m_filestream == NULL)
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": called without a file stream open"));
            return;
        }
        if(m_filestream->fail())
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": error while operating on file"));
            return;
        }

        internal_skip<t>(1);
    }

    // vector
    template<typename t>
    void write_vector(const std::vector<t>& vector)
    {
        if(!m_authority->mpi_wrapper->is_root())
        {
            return;
        }

        if(m_filestream == NULL)
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": called without a file stream open"));
            return;
        }
        if(m_filestream->fail())
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": error while operating on file"));
            return;
        }

        // write size
        int size = vector.size();
        internal_write(size, true);

        // write entries
        for(int i = 0; i < size; i++)
        {
            t entry = vector[i];
            const bool should_endline = (i + 1 >= size);
            internal_write(entry, should_endline);
        }
    }
    template<typename t>
    void read_vector(std::vector<t>& vector)
    {
        // only read from file if rank 0
        int size = -1;
        if(!m_authority->mpi_wrapper->is_root())
        {
            m_authority->mpi_wrapper->broadcast(0u, size);
            vector.assign(size, t(0));
            m_authority->mpi_wrapper->broadcast(0u, vector);
            return;
        }

        if(m_filestream == NULL)
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": called without a file stream open"));
            return;
        }
        if(m_filestream->fail())
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": error while operating on file"));
            return;
        }

        internal_read(size);

        vector.assign(size, t(0));
        int i = 0;
        while(i < size && !m_filestream->eof())
        {
            t read;
            internal_read(read);
            vector[i] = read;

            i++;
        }

        if(i < size)
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": premature end of file while reading"));
            return;
        }

        // broadcast vector from rank 0
        m_authority->mpi_wrapper->broadcast(0u, size);
        m_authority->mpi_wrapper->broadcast(0u, vector);
    }
    template<typename t>
    void skip_vector()
    {
        if(!m_authority->mpi_wrapper->is_root())
        {
            return;
        }

        if(m_filestream == NULL)
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": called without a file stream open"));
            return;
        }
        if(m_filestream->fail())
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": error while operating on file"));
            return;
        }

        int size = -1;
        internal_read(size);
        internal_skip<t>(size);
    }
    template<typename t>
    void append_vector(t value)
    {
        if(!m_authority->mpi_wrapper->is_root())
        {
            return;
        }

        if(m_filestream == NULL)
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": called without a file stream open"));
            return;
        }
        if(m_filestream->fail())
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": error while operating on file"));
            return;
        }

        // read original size
        int original_size = -1;
        internal_read(original_size);

        // new size
        int new_size = original_size + 1;

        // update size
        internal_skip<int>(-1);
        internal_write(new_size, true);

        // append value
        internal_skip<t>(original_size);
        internal_write<t>(value, true);
    }

    // matrix
    template<typename t>
    void write_matrix(const std::vector<std::vector<t> >& matrix)
    {
        const int row_size = matrix.size();
        const int column_size = matrix[0].size();
        this->internal_write_matrix<t>(matrix, row_size, column_size);
    }
    template<typename t>
    void write_matrix(t** matrix, int row_size, int column_size)
    {
        this->internal_write_matrix<t>(matrix, row_size, column_size);
    }
    template<typename t>
    void read_matrix(t**& data, int& row_size, int& column_size)
    {
        // only read data if rank 0
        if(!m_authority->mpi_wrapper->is_root())
        {
            m_authority->mpi_wrapper->broadcast(0u, row_size);
            m_authority->mpi_wrapper->broadcast(0u, column_size);
            data = new t*[row_size];
            for(int row = 0; row < row_size; row++)
            {
                data[row] = new t[column_size];
                std::vector<t> this_row(column_size);
                m_authority->mpi_wrapper->broadcast(0u, this_row);
                std::copy(this_row.begin(), this_row.end(), data[row]);
            }
            return;
        }

        if(m_filestream == NULL)
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": called without a file stream open"));
            return;
        }
        if(m_filestream->fail())
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": error while operating on file"));
            return;
        }

        internal_read(row_size);
        internal_read(column_size);

        data = new t*[row_size];
        int i = 0;
        int j = -1;
        while(i < row_size && !m_filestream->eof())
        {
            data[i] = new t[column_size];

            j = 0;
            while(j < column_size && !m_filestream->eof())
            {
                t read;
                internal_read(read);
                data[i][j] = read;
                j++;
            }
            i++;
        }

        if(i < row_size || j < column_size)
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": premature end of file while reading"));
            return;
        }

        // broadcast read data
        m_authority->mpi_wrapper->broadcast(0u, row_size);
        m_authority->mpi_wrapper->broadcast(0u, column_size);
        for(int row = 0; row < row_size; row++)
        {
            std::vector<t> this_row(data[row], data[row] + column_size);
            m_authority->mpi_wrapper->broadcast(0u, this_row);
        }
    }
    template<typename t>
    void skip_matrix()
    {
        if(!m_authority->mpi_wrapper->is_root())
        {
            return;
        }

        if(m_filestream == NULL)
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": called without a file stream open"));
            return;
        }
        if(m_filestream->fail())
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": error while operating on file"));
            return;
        }

        int row_size = -1;
        int column_size = -1;
        internal_read(row_size);
        internal_read(column_size);
        internal_skip<t>(row_size * column_size);
    }
    template<typename t>
    void append_rows_to_matrix(const std::vector<std::vector<t> >& values)
    {
        if(!m_authority->mpi_wrapper->is_root())
        {
            return;
        }

        if(m_filestream == NULL)
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": called without a file stream open"));
            return;
        }
        if(m_filestream->fail())
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": error while operating on file"));
            return;
        }
        if(!m_prefer_binary)
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": cannot add rows to plain text file"));
            return;
        }

        // read original rows
        int original_row_size = -1;
        internal_read(original_row_size);

        // read num columns
        int column_size = -1;
        internal_read(column_size);

        // new size
        int values_rows = values.size();
        int new_row_size = original_row_size + values_rows;

        // update size
        internal_skip<int>(-2);
        internal_write(new_row_size, true);
        internal_skip<int>(1);

        // seek through matrix
        internal_skip<t>(original_row_size * column_size);

        for(int r = 0; r < values_rows; r++)
        {
            // check sizes
            const int append_size = values[r].size();
            if(column_size != append_size)
            {
                std::string error_message = std::string(__func__) + std::string(": mismatched append size (")
                                            + std::to_string(append_size)
                                            + std::string(") to column size(")
                                            + std::to_string(column_size)
                                            + std::string("). ");
                m_authority->utilities->fatal_error(error_message);
                return;
            }

            // append row
            for(int i = 0; i < append_size; i++)
            {
                const bool should_endline = (i + 1 >= append_size);
                t entry = values[r][i];
                internal_write(entry, should_endline);
            }
        }
    }

    void close_file();

    void delete_file();

protected:

    template<typename t1, typename t2>
    void internal_write_matrix(t2 matrix, int row_size, int column_size)
    {
        if(!m_authority->mpi_wrapper->is_root())
        {
            return;
        }

        if(m_filestream == NULL)
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": called without a file stream open"));
            return;
        }
        if(m_filestream->fail())
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": error while operating on file"));
            return;
        }

        internal_write(row_size, true);
        internal_write(column_size, true);

        for(int i = 0; i < row_size; i++)
        {
            for(int j = 0; j < column_size; j++)
            {
                const bool should_endline = (j + 1 >= column_size);
                t1 entry = matrix[i][j];
                internal_write(entry, should_endline);
            }
        }
    }

    template<typename t>
    void internal_write(t scalar, const bool& should_endline)
    {
        if(m_prefer_binary)
        {
            m_filestream->write(reinterpret_cast<char*>(&scalar), sizeof scalar);
        }
        else
        {
            if(should_endline)
            {
                *m_filestream << scalar << m_endline_character;
            }
            else
            {
                *m_filestream << scalar << m_spacing_character;
            }
        }
    }
    template<typename t>
    void internal_read(t& scalar)
    {
        if(m_prefer_binary)
        {
            m_filestream->read(reinterpret_cast<char*>(&scalar), sizeof scalar);
        }
        else
        {
            *m_filestream >> scalar;
            m_filestream->ignore(1);
        }
    }
    template<typename t>
    void internal_skip(int multiplicity)
    {
        if(m_prefer_binary)
        {
            // binary
            m_filestream->seekp(multiplicity * sizeof(t), std::ios::cur);
        }
        else
        {
            // non-binary
            t tmp_scalar;
            for(int k = 0; k < multiplicity; k++)
            {
                *m_filestream >> tmp_scalar;
                m_filestream->ignore(1);
            }
        }
    }

    void internal_open_reading_file();
    void internal_query_file(bool& is_empty, bool& is_binary);

    AbstractAuthority* m_authority;
    bool m_prefer_binary;
    std::string m_filename;
    std::fstream* m_filestream;
    char m_spacing_character;
    char m_endline_character;
    char m_binary_file_character;
    char m_text_file_character;

};

}
