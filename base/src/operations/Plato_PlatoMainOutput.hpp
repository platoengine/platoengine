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
 * Plato_PlatoMainOutput.hpp
 *
 *  Created on: Jun 27, 2019
 */

#pragma once

#include "Plato_LocalOperation.hpp"

namespace Plato
{

class InputData;

/******************************************************************************//**
 * @brief Manage PLATO Main output
 **********************************************************************************/
class PlatoMainOutput : public Plato::LocalOp
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aPlatoApp PLATO application
     * @param [in] aNode input XML data
     **********************************************************************************/
    PlatoMainOutput(PlatoApp* aPlatoApp, Plato::InputData& aNode);

    /******************************************************************************//**
     * @brief Destructor
     **********************************************************************************/
    ~PlatoMainOutput();

    /******************************************************************************//**
     * @brief perform local operation - output data
     **********************************************************************************/
    void operator()();

    /******************************************************************************//**
     * @brief Return local operation's argument list
     * @param [out] aLocalArgs argument list
    **********************************************************************************/
    void getArguments(std::vector<Plato::LocalArg> & aLocalArgs);

private:
    /******************************************************************************//**
     * @brief Extract iso-surface
     * @param [in] aIteration current optimization iteration
     **********************************************************************************/
    void extractIsoSurface(int aIteration);

    /******************************************************************************//**
     * @brief build a string based on the current iteration
     * @param [in] aCurIteration current optimization iteration
     * @param [out] aString output iteration string
     **********************************************************************************/
    void buildIterationNumberString(const int &aCurIteration,
                                    std::string &aString);

private:
    std::vector<Plato::LocalArg> mOutputData; /*!< set of output data */
    int mOutputFrequency; /*!< output frequency */
    int mMaxIterations; /*!< max iterations */
    int mOutputMethod; /*!< epu output data - distributed or serial */
    std::string mDiscretization; /*!< topology representation, density or levelset */
    bool mWriteRestart; /*!< flag - write restart file */
    std::string mRestartFieldName; /*!< name of field to put in restart file */
    std::string mBaseName; /*!< output file base name */
    bool mAppendIterationCount; /*!< flag - append optimization iteration count */
    std::vector<std::string> mRequestedFormats; /*!< names of formats to write */
};
// class PlatoMainOutput

}
// namespace Plato
