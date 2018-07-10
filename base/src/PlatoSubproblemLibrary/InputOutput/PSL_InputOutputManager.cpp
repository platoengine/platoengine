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

#include "PSL_InputOutputManager.hpp"

#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_FreeHelpers.hpp"

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstdio>

namespace PlatoSubproblemLibrary
{

InputOutputManager::InputOutputManager(AbstractAuthority* authority) :
        m_authority(authority),
        m_prefer_binary(true),
        m_filename(),
        m_filestream(NULL),
        m_spacing_character(','),
        m_endline_character('\n'),
        m_binary_file_character('b'),
        m_text_file_character('t')
{
}
InputOutputManager::~InputOutputManager()
{
    m_filename.clear();
    close_file();
}

void InputOutputManager::set_filename(const std::string& filename)
{
    m_filename = filename;
}
std::string InputOutputManager::get_filename()
{
    return m_filename;
}

void InputOutputManager::open_file()
{
    if(!m_authority->mpi_wrapper->is_root())
    {
        return;
    }

    // get information about file
    bool file_is_empty = false;
    bool file_is_binary = false;
    internal_query_file(file_is_empty, file_is_binary);

    // if file is already of a type, match that type
    if(!file_is_empty)
    {
        if(file_is_binary)
        {
            choose_binary_preference();
        }
        else
        {
            choose_text_preference();
        }
    }

    std::ios_base::openmode minimal_mode = std::ios::out;
    if(m_prefer_binary)
    {
        minimal_mode |= std::ios::binary;
    }

    safe_free(m_filestream);
    m_filestream = new std::fstream;
    m_filestream->open(m_filename.c_str(), minimal_mode | std::ios::in);
    if(!m_filestream->is_open())
    {
        // no in
        m_filestream->open(m_filename.c_str(), minimal_mode);
        if(!m_filestream->is_open())
        {
            m_authority->utilities->fatal_error(std::string(__func__) + std::string(": error while opening file"));
            return;
        }

        // write filetype mode
        char filetype_character = (m_prefer_binary ? m_binary_file_character : m_text_file_character);
        internal_write(filetype_character, true);
    }
    else
    {
        // skip filetype mode
        internal_skip<char>(1);
    }
}
bool InputOutputManager::is_empty()
{
    int result = -1;
    if(!m_authority->mpi_wrapper->is_root())
    {
        m_authority->mpi_wrapper->broadcast(0u, result);
        return result;
    }

    internal_open_reading_file();
    result = (m_filestream->peek() == std::ifstream::traits_type::eof());
    close_file();

    m_authority->mpi_wrapper->broadcast(0u, result);
    return result;
}

void InputOutputManager::choose_binary_preference()
{
    m_prefer_binary = true;
}
void InputOutputManager::choose_text_preference()
{
    m_prefer_binary = false;
}

bool InputOutputManager::is_binary()
{
    int result = 0;
    if(!m_authority->mpi_wrapper->is_root())
    {
        m_authority->mpi_wrapper->broadcast(0u, result);
        return result;
    }

    // get is_binary result
    bool is_empty = false;
    bool is_binary = false;
    internal_query_file(is_empty, is_binary);
    result = is_binary;

    m_authority->mpi_wrapper->broadcast(0u, result);
    return result;
}

void InputOutputManager::close_file()
{
    if(!m_authority->mpi_wrapper->is_root())
    {
        return;
    }

    if(m_filestream)
    {
        m_filestream->close();
        delete m_filestream;
        m_filestream = NULL;
    }
}

void InputOutputManager::delete_file()
{
    if(!m_authority->mpi_wrapper->is_root())
    {
        return;
    }

    std::remove(m_filename.c_str());
}

void InputOutputManager::internal_open_reading_file()
{
    safe_free(m_filestream);
    m_filestream = new std::fstream;

    // set mode
    std::ios_base::openmode mode = std::ios::in;
    if(m_prefer_binary)
    {
        mode |= std::ios::binary;
    }

    m_filestream->open(m_filename.c_str(), mode);
}

void InputOutputManager::internal_query_file(bool& is_empty, bool& is_binary)
{
    // store binary preference
    bool original_binary_prefer = get_prefer_binary();
    choose_binary_preference();

    // read first char
    is_empty = true;
    internal_open_reading_file();
    char val;
    if(*m_filestream >> val)
    {
        is_empty = false;
        if(val == m_binary_file_character)
        {
            is_binary = true;
        }
        else if(val == m_text_file_character)
        {
            is_binary = false;
        }
        else
        {
            m_authority->utilities->fatal_error("Could not match first char of file. Aborting.\n\n");
        }
    }
    close_file();

    // revert binary preference
    set_prefer_binary(original_binary_prefer);
}

}
