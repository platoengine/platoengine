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
 * Plato_LocalOperation.hpp
 *
 *  Created on: Jun 27, 2019
 */

#pragma once

#include <vector>

#include "Plato_SharedData.hpp"

class PlatoApp;

namespace Plato
{

/******************************************************************************//**
 * @brief An argument associated with a local operation
**********************************************************************************/
struct LocalArg
{
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aLayout data layout (e.g. scalar, node field, element field, etc.)
     * @param [in] aName argument name
     * @param [in] aLength argument length
     * @param [in] aWrite output data to exodus file
    **********************************************************************************/
    LocalArg(const Plato::data::layout_t & aLayout, const std::string & aName, int aLength = 0, bool aWrite = false) :
            mLayout(aLayout),
            mName(aName),
            mLength(aLength),
            mWrite(aWrite)
    {
    }
    Plato::data::layout_t mLayout; /*!< data layout */
    std::string mName; /*!< argument name */
    int mLength; /*!< argument length */
    bool mWrite; /*!< flag: write data to the exodus output file */
};
// struct LocalArg

/******************************************************************************//**
 * @brief Application-specific operation
**********************************************************************************/
class LocalOp
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aPlatoApp PLATO application
    **********************************************************************************/
    LocalOp(PlatoApp* aPlatoApp) :
            mPlatoApp(aPlatoApp)
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~LocalOp()
    {
    }

    /******************************************************************************//**
     * @brief Perform local operation
    **********************************************************************************/
    virtual void operator()()=0;

    /******************************************************************************//**
     * @brief Return local operation argument list
     * @param [out] aLocalArgs argument list
    **********************************************************************************/
    virtual void getArguments(std::vector<Plato::LocalArg>& aLocalArgs)=0;

protected:
    PlatoApp* mPlatoApp; /*!< PLATO application interface */
};
// class LocalOp

}
// namespace Plato
