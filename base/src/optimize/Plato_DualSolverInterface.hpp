/*
 * Plato_DualSolverInterface.hpp
 *
 *  Created on: Dec 1, 2017
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

#ifndef PLATO_DUALSOLVERINTERFACE_HPP_
#define PLATO_DUALSOLVERINTERFACE_HPP_

#include "Plato_MultiVector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_DualOptimizer.hpp"
#include "Plato_DualProblemSolver.hpp"
#include "Plato_ConservativeConvexSeparableAppxDataMng.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class DualSolverInterface : public Plato::DualProblemSolver<ScalarType, OrdinalType>
{
public:
    explicit DualSolverInterface(const Plato::DataFactory<ScalarType, OrdinalType> & aFactory) :
            mOptimizer(std::make_shared<Plato::DualOptimizer<ScalarType, OrdinalType>>(aFactory)),
            mInitialGuess(aFactory.dual().create())
    {
    }
    virtual ~DualSolverInterface()
    {
    }

    void solve(Plato::MultiVector<ScalarType, OrdinalType> & aDual, Plato::MultiVector<ScalarType, OrdinalType> & aTrialControl)
    {
        mOptimizer->reset();
        Plato::update(static_cast<ScalarType>(1), *mInitialGuess, static_cast<ScalarType>(0), aDual);
        mOptimizer->solve(aDual, aTrialControl);
        Plato::update(static_cast<ScalarType>(1), aDual, static_cast<ScalarType>(0), *mInitialGuess);
    }
    void update(Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        mOptimizer->update(aDataMng);
    }
    void updateObjectiveCoefficients(Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        mOptimizer->updateObjectiveCoefficients(aDataMng);
    }
    void updateConstraintCoefficients(Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        mOptimizer->updateConstraintCoefficients(aDataMng);
    }
    void initializeAuxiliaryVariables(Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        mOptimizer->initializeAuxiliaryVariables(aDataMng);
    }

private:
    std::shared_ptr<Plato::DualOptimizer<ScalarType, OrdinalType>> mOptimizer;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mInitialGuess;

private:
    DualSolverInterface<ScalarType, OrdinalType>(const Plato::DualSolverInterface<ScalarType, OrdinalType> & aRhs);
    Plato::DualSolverInterface<ScalarType, OrdinalType> & operator=(const Plato::DualSolverInterface<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_DUALSOLVERINTERFACE_HPP_ */
