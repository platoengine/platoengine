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

/*
 * Plato_SharedData.hpp
 *
 *  Created on: April 23, 2017
 *
 */

#ifndef SRC_SHAREDDATA_HPP_
#define SRC_SHAREDDATA_HPP_

#include <string>
#include <vector>
#include <cassert>

namespace Plato
{

//!  Inter-program shared data
/*!
 */


/******************************************************************************//**
 * \brief Allowable data format through the Plato Engine architecture.  Only these
 * data formats are supported through the Multiple-Program, Multiple-Data (MPMD)
 * architecture.
**********************************************************************************/
struct data
{
    enum layout_t
    {
        SCALAR = 0, SCALAR_FIELD = 1, VECTOR_FIELD = 2, TENSOR_FIELD = 3, ELEMENT_FIELD = 4, SCALAR_PARAMETER = 5
    };
};

/******************************************************************************//**
 * \brief Return string with data layout description
 * \param [in] aInput 1D data layout description
 * \return string with data layout description
**********************************************************************************/
inline std::string get_data_latout(const Plato::data::layout_t& aInput)
{
    switch (aInput)
    {
        case Plato::data::SCALAR:
        {
            return std::string("GLOBAL SCALAR VALUE");
            break;
        }
        case Plato::data::SCALAR_FIELD:
        {
            return std::string("SCALAR FIELD");
            break;
        }
        case Plato::data::VECTOR_FIELD:
        {
            return std::string("VECTOR FIELD");
            break;
        }
        case Plato::data::TENSOR_FIELD:
        {
            return std::string("TENSOR FIELD");
            break;
        }
        case Plato::data::ELEMENT_FIELD:
        {
            return std::string("ELEMENT FIELD");
            break;
        }
        case Plato::data::SCALAR_PARAMETER:
        {
            return std::string("SCALAR PARAMETER");
            break;
        }
        default:
        {
            throw std::runtime_error(std::string("\n\nFILE: ") + __FILE__ \
            + std::string("\nFUNCTION: ") + __PRETTY_FUNCTION__ \
            + std::string("\nLINE:") + std::to_string(__LINE__) \
            + std::string("\nMESSAGE: ") + "Input 'data layout' is not supported" + "\n\n");
            break;
        }
    }
}
// function get_data_latout

/******************************************************************************//**
 * \brief Shared Data abstract class
**********************************************************************************/
class SharedData
{
public:
    virtual ~SharedData()
    {
    }

    /******************************************************************************//**
     * \brief Return the size of the SharedData container
     * \return container size
    **********************************************************************************/
    virtual int size() const = 0;

    /******************************************************************************//**
     * \brief Return the SharedData name
     * \return data name
    **********************************************************************************/
    virtual std::string myName() const = 0;

    /******************************************************************************//**
     * \brief Return description on the SharedData
     * \return description
    **********************************************************************************/
    virtual std::string myContext() const { return std::string(); }

    /******************************************************************************//**
     * \brief Return data layout/format
     * \return data layout/format
    **********************************************************************************/
    virtual Plato::data::layout_t myLayout() const = 0;

    /******************************************************************************//**
     * \brief Transmit data from owner application to user application
    **********************************************************************************/
    virtual void transmitData() = 0;

    /******************************************************************************//**
     * \brief Set SharedData container values
     * \param [in] aData standard vector
    **********************************************************************************/
    virtual void setData(const std::vector<double> & aData) = 0;

    /******************************************************************************//**
     * \brief Get SharedData container values
     * \param [in] aData standard vector
    **********************************************************************************/
    virtual void getData(std::vector<double> & aData) const = 0;
};
// class SharedData

}
// End namespace Plato


#endif
