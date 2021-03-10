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
 * Plato_LinearAlgebra.hpp
 *
 *  Created on: Oct 17, 2017
 */


#ifndef PLATO_LINEARALGEBRA_HPP_
#define PLATO_LINEARALGEBRA_HPP_

#include <cmath>
#include <string>
#include <cassert>

#include "Plato_Macros.hpp"
#include "Plato_Vector.hpp"
#include "Plato_MultiVector.hpp"

namespace Plato
{

/******************************************************************************//**
 * \brief Print Plato::MultiVector to terminal.
 * \param [in] aInput input multi-vector
 * \param [in] aName  multi-vector name, default = MultiVector
***********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void print
(const Plato::MultiVector<ScalarType, OrdinalType> & aInput,
 std::string aName = "MultiVector")
{
    auto tNumRows = aInput.getNumVectors();
    for(decltype(tNumRows) tRowIndex = 0; tRowIndex < tNumRows; tRowIndex++)
    {
        const auto& tRow = aInput[tRowIndex];
        auto tNumCols = tRow.size();
        for(decltype(tNumRows) tColIndex = 0; tColIndex < tNumCols; tColIndex++)
        {
            std::cout << aName << "(" << tRowIndex << "," << tColIndex << ") = " << tRow[tColIndex] << "\n";
        }
    }
}

/******************************************************************************//**
 * \brief Print Plato::Vector to terminal.
 * \param [in] aInput input vector
 * \param [in] aName  vector name, default = Vector
***********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void print
(const Plato::Vector<ScalarType, OrdinalType> & aInput,
 std::string aName = "Vector")
{
    auto tNumElems = aInput.size();
    for(decltype(tNumElems) tIndex = 0; tIndex < tNumElems; tIndex++)
    {
        std::cout << aName << "(" << tIndex << ") = " << aInput[tIndex] << "\n";
    }
}

/******************************************************************************//**
 * \brief Copy data from 2D standard vector to Plato multivector.
 * \param [in]  aInput   2D standard vector
 * \param [out] aOutput  Plato multivector
***********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void copy
(const std::vector<std::vector<ScalarType>>& aInput,
 Plato::MultiVector<ScalarType, OrdinalType>& aOutput)
{
    if(aInput.size() != aOutput.getNumVectors())
    {
        THROWERR(std::string("Dimension mismatch: Number of rows do not match. ") + "Input matrix has '"
            + std::to_string(aInput.size()) + "' rows and output matrix has '" + std::to_string(aOutput.getNumVectors())
            + "' rows.")
    }
    if(aInput[0].size() != aOutput[0].size())
    {
        THROWERR(std::string("Dimension mismatch: Number of columns do not match. ") + "Input matrix has '"
            + std::to_string(aInput[0].size()) + "' columns and output matrix has '" + std::to_string(aOutput[0].size())
            + "' columns.")
    }

    for(auto& tRow : aInput)
    {
        auto tRowIndex = &tRow - &aInput[0];
        for(auto& tColValue : tRow)
        {
            auto tColIndex = &tColValue - &tRow[0];
            aOutput(tRowIndex, tColIndex) = tColValue;
        }
    }
}

/******************************************************************************//**
 * \brief Compute inner product: /f$ \alpha = \sum_{i=1}^{N} x_i * y_i /f$
 * \param [in] aVectorOne input multi-vector
 * \param [in] aVectorTwo input multi-vector
 * \return inner product output
***********************************************************************************/
template<typename ScalarType, typename OrdinalType>
ScalarType dot
(const Plato::MultiVector<ScalarType, OrdinalType> & aVectorOne,
 const Plato::MultiVector<ScalarType, OrdinalType> & aVectorTwo)
{
    assert(aVectorOne.getNumVectors() > static_cast<OrdinalType>(0));
    assert(aVectorOne.getNumVectors() == aVectorTwo.getNumVectors());

    ScalarType tCummulativeSum = 0;
    const OrdinalType tNumVectors = aVectorOne.getNumVectors();
    for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
    {
        assert(aVectorOne[tVectorIndex].size() > static_cast<OrdinalType>(0));
        assert(aVectorOne[tVectorIndex].size() == aVectorTwo[tVectorIndex].size());
        const Plato::Vector<ScalarType, OrdinalType> & tVectorOne = aVectorOne[tVectorIndex];
        const Plato::Vector<ScalarType, OrdinalType> & tVectorTwo = aVectorTwo[tVectorIndex];
        tCummulativeSum += tVectorOne.dot(tVectorTwo);
    }
    return(tCummulativeSum);
}

template<typename ScalarType, typename OrdinalType>
void entryWiseProduct
(const Plato::MultiVector<ScalarType, OrdinalType> & aInput,
 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
{
    assert(aInput.getNumVectors() > static_cast<OrdinalType>(0));
    assert(aInput.getNumVectors() == aOutput.getNumVectors());

    const OrdinalType tNumVectors = aInput.getNumVectors();
    for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
    {
        assert(aInput[tVectorIndex].size() > static_cast<OrdinalType>(0));
        assert(aInput[tVectorIndex].size() == aOutput[tVectorIndex].size());
        Plato::Vector<ScalarType, OrdinalType> & tOutput = aOutput[tVectorIndex];
        const Plato::Vector<ScalarType, OrdinalType> & tInput = aInput[tVectorIndex];
        tOutput.entryWiseProduct(tInput);
    }
}

template<typename ScalarType, typename OrdinalType>
void fill
(const ScalarType & aScalar,
 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
{
    assert(aOutput.getNumVectors() > static_cast<OrdinalType>(0));

    const OrdinalType tNumVectors = aOutput.getNumVectors();
    for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
    {
        assert(aOutput[tVectorIndex].size() > static_cast<OrdinalType>(0));
        Plato::Vector<ScalarType, OrdinalType> & tVector = aOutput[tVectorIndex];
        tVector.fill(aScalar);
    }
}

template<typename ScalarType, typename OrdinalType>
void scale
(const ScalarType & aScalar,
 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
{
    assert(aOutput.getNumVectors() > static_cast<OrdinalType>(0));

    const OrdinalType tNumVectors = aOutput.getNumVectors();
    for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
    {
        assert(aOutput[tVectorIndex].size() > static_cast<OrdinalType>(0));
        Plato::Vector<ScalarType, OrdinalType> & tVector = aOutput[tVectorIndex];
        tVector.scale(aScalar);
    }
}

template<typename ScalarType, typename OrdinalType>
ScalarType norm_mean(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
{
    const OrdinalType tVECTOR_INDEX = 0;
    const OrdinalType tNumElem = aInput[tVECTOR_INDEX].size() * aInput.getNumVectors();
    const ScalarType tDotProduct = Plato::dot(aInput, aInput);
    const ScalarType tNorm = tDotProduct / tNumElem;
    return(tNorm);
}

template<typename ScalarType, typename OrdinalType>
ScalarType norm(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
{
    const ScalarType tDotProduct = Plato::dot(aInput, aInput);
    const ScalarType tNorm = std::sqrt(tDotProduct);
    return(tNorm);
}

//! Update vector values with scaled values of A, this = beta*this + alpha*A.
template<typename ScalarType, typename OrdinalType>
void update
(const ScalarType & aAlpha,
 const Plato::MultiVector<ScalarType, OrdinalType> & aInput,
 const ScalarType & aBeta,
 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
{
    assert(aInput.getNumVectors() > static_cast<OrdinalType>(0));
    assert(aInput.getNumVectors() == aOutput.getNumVectors());

    const OrdinalType tNumVectors = aInput.getNumVectors();
    for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
    {
        assert(aInput[tVectorIndex].size() > static_cast<OrdinalType>(0));
        assert(aInput[tVectorIndex].size() == aOutput[tVectorIndex].size());
        Plato::Vector<ScalarType, OrdinalType> & tOutputVector = aOutput[tVectorIndex];
        const Plato::Vector<ScalarType, OrdinalType> & tInputVector = aInput[tVectorIndex];
        tOutputVector.update(aAlpha, tInputVector, aBeta);
    }
}

template<typename ScalarType, typename OrdinalType>
void gemv
(const ScalarType & aAlpha,
 const Plato::MultiVector<ScalarType, OrdinalType> & aMatrix,
 const Plato::Vector<ScalarType, OrdinalType> & aVector,
 const ScalarType & aBeta,
 Plato::Vector<ScalarType, OrdinalType> & aOutput,
 bool aTranspose = false)
{
    const OrdinalType tNumElements = aMatrix[0].size();
    const OrdinalType tNumVectors = aMatrix.getNumVectors();

    if(aTranspose == false)
    {
        assert(tNumVectors == aOutput.size());
        assert(tNumElements == aVector.size());

        aOutput.scale(aBeta);
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            ScalarType tOutputRowValue = aMatrix[tVectorIndex].dot(aVector);
            tOutputRowValue = aAlpha * tOutputRowValue;
            aOutput[tVectorIndex] = aOutput[tVectorIndex] + tOutputRowValue;
        }
    }
    else
    {
        assert(tNumVectors == aVector.size());
        assert(tNumElements == aOutput.size());

        aOutput.scale(aBeta);
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            ScalarType tScaleFactor = aAlpha * aVector[tVectorIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tVector = aMatrix[tVectorIndex];
            aOutput.update(tScaleFactor, tVector, static_cast<ScalarType>(1));
        }
    }
}

}

#endif /* PLATO_LINEARALGEBRA_HPP_ */
