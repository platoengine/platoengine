/*
 * Plato_ErrorChecks.hpp
 *
 *  Created on: May 6, 2018
 */

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

#ifndef PLATO_ERRORCHECKS_HPP_
#define PLATO_ERRORCHECKS_HPP_

#include <sstream>
#include <fstream>
#include <cassert>
#include <iostream>
#include <stdexcept>

#include "Plato_CommWrapper.hpp"
#include "Plato_MultiVector.hpp"

namespace Plato
{

/******************************************************************************//**
 *
 * @brief Print Plato license statement
 * @param [in,out] aOutputFile output file
 *
**********************************************************************************/
inline void print_plato_license(std::ofstream & aOutputFile)
{
    aOutputFile << "Plato Engine v.1.0: Copyright 2018, National Technology & Engineering Solutions of Sandia, LLC (NTESS).\n\n";
}

namespace error
{

/******************************************************************************//**
 *
 * @brief Check for dimension mismatch.
 * @param [in] aDimOne dimension
 * @param [in] aDimTwo dimension
 *
**********************************************************************************/
template<typename OrdinalType>
void check_dimension(const OrdinalType & aDimOne, const OrdinalType & aDimTwo)
{
    try
    {
        if(aDimOne != aDimTwo)
        {
            std::ostringstream tMessage;
            tMessage << "\n\n ******** MESSAGE: DIMENSION MISMATCH! APP DIM = " << aDimOne << " AND SHARED DATA DIM = "
                    << aDimTwo << ". ABORT! ******** \n\n";
            throw std::invalid_argument(tMessage.str().c_str());
        }
    }
    catch(const std::invalid_argument & tErrorMsg)
    {
        throw tErrorMsg;
    }
}

/******************************************************************************//**
 *
 * @brief Check if distributed memory communicator is null.
 * @param [in] aCommWrapper distributed memory communicator wrapper
 *
**********************************************************************************/
inline void check_null_comm(const Plato::CommWrapper & aCommWrapper)
{
    try
    {
        if(aCommWrapper.isCommInitialized() == false)
        {
            throw std::invalid_argument("\n\n ******** MESSAGE: NULL MPI COMMUNICATOR. ABORT! ******** \n\n");
        }
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        throw tErrorMsg;
    }
}

/******************************************************************************//**
 *
 * @brief Check for dimension mismatch
 * @param [in] aVecOne Plato vector
 * @param [in] aVecTwo Plato vector
 *
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void check_dimension_mismatch(const Plato::Vector<ScalarType, OrdinalType> & aVecOne,
                                     const Plato::Vector<ScalarType, OrdinalType> & aVecTwo)
{
    try
    {
        if(aVecOne.size() != aVecTwo.size())
        {
            std::ostringstream tMessage;
            tMessage << "\n\n ******** MESSAGE: DIMENSION MISMATCH! VEC_1 DIM = " << aVecOne.size() << " AND VEC_2 DIM = "
            << aVecTwo.size() << ". ABORT! ******** \n\n";
            throw std::invalid_argument(tMessage.str().c_str());
        }
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        throw tErrorMsg;
    }
}

/******************************************************************************//**
 *
 * @brief Check for errors associated with the lower and upper bound containers.
 * @param [in] aLowerBounds Plato multivector
 * @param [in] aUpperBounds Plato multivector
 *
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void checkBounds(const Plato::MultiVector<ScalarType, OrdinalType> & aLowerBounds,
                        const Plato::MultiVector<ScalarType, OrdinalType> & aUpperBounds,
                        bool aPrintMessage = false)
{
    try
    {
        OrdinalType tNumLowerBoundVectors = aLowerBounds.getNumVectors();
        OrdinalType tNumUpperBoundVectors = aUpperBounds.getNumVectors();
        if(tNumLowerBoundVectors != tNumUpperBoundVectors)
        {
            std::ostringstream tErrorMessage;
            tErrorMessage << "\n\n ******** ERROR IN FILE: " << __FILE__ << ", FUNCTION: " << __PRETTY_FUNCTION__ << ", LINE: "
            << __LINE__ << ", MESSAGE: DIMENSION MISMATCH BETWEEN UPPER BOUND AND LOWER BOUND MULTIVECTORS."
            << " UPPER BOUND MULTIVECTOR HAS SIZE = " << tNumUpperBoundVectors << " AND LOWER BOUND MULTIVECTOR HAS SIZE = "
            << tNumLowerBoundVectors
            << ". PLEASE MAKE SURE LOWER AND UPPER BOUND MULTIVECTORS DIMENSIONS MATCH. ABORT! ******** \n\n";
            throw std::invalid_argument(tErrorMessage.str().c_str());
        }

        OrdinalType tNumVectors = aLowerBounds.getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            OrdinalType tLowerBoundLength = aLowerBounds[tVectorIndex].size();
            OrdinalType tUpperBoundLength = aUpperBounds[tVectorIndex].size();

            if(tLowerBoundLength != tUpperBoundLength)
            {
                std::ostringstream tErrorMessage;
                tErrorMessage << "\n\n ******** ERROR IN FILE: " << __FILE__ << ", FUNCTION: " << __PRETTY_FUNCTION__
                << ", LINE: " << __LINE__ << ", MESSAGE: DIMENSION MISMATCH BETWEEN UPPER BOUND AND LOWER BOUND VECTORS WITH INDEX = "
                << tVectorIndex << ". UPPER BOUND VECTOR HAS LENGTH = " << tUpperBoundLength << " AND LOWER BOUND VECTOR HAS LENGTH = "
                << tLowerBoundLength << ". PLEASE MAKE SURE LOWER AND UPPER BOUND VECTORS DIMENSIONS MATCH. ABORT! ******** \n\n";
                throw std::invalid_argument(tErrorMessage.str().c_str());
            }

            OrdinalType tNumElements = aLowerBounds[tVectorIndex].size();
            const ScalarType* tLowerBounds = aLowerBounds[tVectorIndex].data();
            const ScalarType* tUpperBounds = aUpperBounds[tVectorIndex].data();
            for(OrdinalType tElemIndex = 0; tElemIndex < tNumElements; tElemIndex++)
            {
                if(tLowerBounds[tElemIndex] > tUpperBounds[tElemIndex])
                {
                    std::ostringstream tErrorMessage;
                    tErrorMessage << "\n\n ******** ERROR IN FILE: " << __FILE__ << ", FUNCTION: " << __PRETTY_FUNCTION__
                    << ", LINE: " << __LINE__ << ", MESSAGE: LOWER BOUND EXCEEDS UPPER BOUND." << " LOWER_BOUND("
                    << tVectorIndex << "," << tElemIndex << ") = " << tLowerBounds[tElemIndex] << " AND UPPER_BOUND("
                    << tVectorIndex << "," << tElemIndex << ") = " << tUpperBounds[tElemIndex]
                    << ". THE FIRST ENTRY DENOTES THE VECTOR INDEX"
                    << " AND THE SECOND ENTRY DENOTES THE ELEMENT INDEX. ABORT! ******** \n\n";
                    throw std::invalid_argument(tErrorMessage.str().c_str());
                }
            }
        }
    }
    catch(const std::invalid_argument & tErrorMsg)
    {
        if(aPrintMessage == true)
        {
            std::cout << tErrorMsg.what() << std::flush;
        }
        throw tErrorMsg;
    }
} // function checkBounds

inline void checkInitialGuessIsSet(const bool & aIsSet, bool aPrintMessage = false)
{
    try
    {
        if(aIsSet == false)
        {
            std::ostringstream tErrorMsg;
            tErrorMsg << "\n\n ******* ERROR IN FILE: " << __FILE__ << ", FUNCTION: " << __PRETTY_FUNCTION__ << ", LINE: "
                      << __LINE__ << ", MESSAGE: USER DID NOT DEFINE INITIAL GUESS. ABORT! ******\n\n";
            throw std::invalid_argument(tErrorMsg.str().c_str());
        }
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        if(aPrintMessage == true)
        {
            std::cout << tErrorMsg.what() << std::flush;
        }
        throw tErrorMsg;
    }
} // function isInitialGuessSet

template<typename ScalarType, typename OrdinalType = size_t>
inline void checkInitialGuess(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                              const Plato::MultiVector<ScalarType, OrdinalType> & aLowerBounds,
                              const Plato::MultiVector<ScalarType, OrdinalType> & aUpperBounds,
                              bool aPrintMessage = false)
{
    try
    {
        Plato::error::checkBounds(aLowerBounds, aUpperBounds);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        if(aPrintMessage == true)
        {
            std::cout << tErrorMsg.what() << std::flush;
        }
        throw tErrorMsg;
    }

    try
    {
        OrdinalType tNumControlVectors = aControl.getNumVectors();
        OrdinalType tNumLowerBoundVectors = aLowerBounds.getNumVectors();
        if(tNumControlVectors != tNumLowerBoundVectors)
        {
            std::ostringstream tErrorMessage;
            tErrorMessage << "\n\n ******** ERROR IN FILE: " << __FILE__ << ", FUNCTION: " << __PRETTY_FUNCTION__ << ", LINE: "
            << __LINE__ << ", MESSAGE: DIMENSION MISMATCH BETWEEN CONTROL AND BOUND MULTIVECTORS."
            << " CONTROL MULTIVECTOR HAS SIZE = " << tNumControlVectors << " AND BOUND CONTAINERS HAVE SIZE = "
            << tNumLowerBoundVectors
            << ". PLEASE MAKE SURE CONTROL AND AND BOUND MULTIVECTORS DIMENSIONS MATCH. ABORT! ******** \n\n";
            throw std::invalid_argument(tErrorMessage.str().c_str());
        }
    }
    catch(const std::invalid_argument & tErrorMsg)
    {
        if(aPrintMessage == true)
        {
            std::cout << tErrorMsg.what() << std::flush;
        }
        throw tErrorMsg;
    }
} // function checkInitialGuess

} //namespace error

} // namespace Plato

#endif /* PLATO_ERRORCHECKS_HPP_ */
