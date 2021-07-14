/*
 //\HEADER
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
 // Questions? Contact the Plato team (plato3D-help\sandia.gov)
 //
 // *************************************************************************
 //\HEADER
 */

/*
 * Plato_OperationsUtilities.hpp
 *
 *  Created on: Jun 27, 2019
 */

#pragma once

#include "Plato_SharedData.hpp"

namespace Plato
{

class InputData;

/******************************************************************************//**
 * \brief Return string layout
 * \param [in] aInput enum layout
 * \return string layout
**********************************************************************************/
std::string getLayout(const Plato::data::layout_t & aInput);

/******************************************************************************//**
 * \brief Return shared data layout
 * \param [in] aLayoutStr layout string
 * \return layout
**********************************************************************************/
Plato::data::layout_t getLayout(const std::string & aLayoutStr);

/******************************************************************************//**
 * \brief Return shared data layout. If no 'Layout' keyword is given in the input
 *        xml_node (aNode), aDefaultLayout is returned
 * \param [in] aNode XML input data
 * \param [in,out] aDefaultLayout default layout
 * \return layout
**********************************************************************************/
Plato::data::layout_t getLayout(const Plato::InputData& aNode, Plato::data::layout_t aDefaultLayout);

/******************************************************************************//**
 * \brief Return shared data layout
 * \param [in] aNode XML input data
 * \return layout
**********************************************************************************/
Plato::data::layout_t getLayout(const Plato::InputData& aNode);

/******************************************************************************//**
 * \brief Split collection of strings separated by the '_' delimiter.
 * \param [in] aInput input string
 * \param [in,out] aOutput list of strings
**********************************************************************************/
void split(const std::string & aInput, std::vector<std::string> & aOutput);

/******************************************************************************//**
 * \brief Zero all entries
 * \param [in] aLength container's length
 * \param [in,out] aData container's data
**********************************************************************************/
void zero(const size_t& aLength, double* aData);

}
// namespace Plato
