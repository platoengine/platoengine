/*
 * Plato_DualSolverInterface.hpp
 *
 *  Created on: Dec 1, 2017
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
