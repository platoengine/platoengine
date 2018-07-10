/*
 * Plato_ErrorChecks.hpp
 *
 *  Created on: May 6, 2018
 */

#ifndef PLATO_ERRORCHECKS_HPP_
#define PLATO_ERRORCHECKS_HPP_

#include <cassert>
#include <iostream>
#include <stdexcept>

#include "Plato_MultiVector.hpp"

namespace Plato
{

namespace error
{

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
