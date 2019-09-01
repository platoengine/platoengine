/*
 * Plato_Test_MethodMovingAsymptotesNew.cpp
 *
 *  Created on: Jul 21, 2019
 */

#include <gtest/gtest.h>

#include <stddef.h>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "Plato_Radius.hpp"
#include "Plato_Circle.hpp"
#include "Plato_Rosenbrock.hpp"
#include "Plato_Himmelblau.hpp"
#include "Plato_GoldsteinPrice.hpp"
#include "Plato_ShiftedEllipse.hpp"
#include "Plato_CcsaTestObjective.hpp"
#include "Plato_CcsaTestInequality.hpp"

#include "Plato_ProxyVolume.hpp"
#include "Plato_ProxyCompliance.hpp"
#include "Plato_EpetraSerialDenseVector.hpp"
#include "Plato_EpetraSerialDenseMultiVector.hpp"
#include "Plato_StructuralTopologyOptimization.hpp"

#include "Plato_AugmentedLagrangian.hpp"
#include "Plato_CommWrapper.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_CriterionList.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_Diagnostics.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_MultiVectorList.hpp"
#include "Plato_ReductionOperations.hpp"
#include "Plato_StandardMultiVector.hpp"
#include "Plato_Types.hpp"
#include "Plato_Vector.hpp"
#include "Plato_Macros.hpp"

#include "Plato_UnitTestUtils.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Check inputs for MMA algorithm diagnostics
 * @param [in] aData diagnostic data for MMA algorithm
 * @param [in] aOutputFile output file
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
void check_mma_inputs(const Plato::OutputDataMMA<ScalarType, OrdinalType> &aData,
                      const std::ofstream &aOutputFile)
{
    try
    {
        Plato::error::is_file_open(aOutputFile);
        Plato::error::is_vector_empty(aData.mConstraints);
    }
    catch(const std::invalid_argument& tError)
    {
        throw tError;
    }
}

/******************************************************************************//**
 * @brief Output a brief sentence explaining why the CCSA optimizer stopped.
 * @param [in] aStopCriterion stopping criterion flag
 * @param [in,out] aOutput string with brief description
**********************************************************************************/
inline void print_mma_stop_criterion(const Plato::algorithm::stop_t & aStopCriterion, std::string & aOutput)
{
    aOutput.clear();
    switch(aStopCriterion)
    {
        case Plato::algorithm::stop_t::OBJECTIVE_STAGNATION:
        {
            aOutput = "\n\n****** Optimization stopping due to objective stagnation. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::CONTROL_STAGNATION:
        {
            aOutput = "\n\n****** Optimization stopping due to control (i.e. design variable) stagnation. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::MAX_NUMBER_ITERATIONS:
        {
            aOutput = "\n\n****** Optimization stopping due to exceeding maximum number of iterations. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::OPTIMALITY_AND_FEASIBILITY:
        {
            aOutput = "\n\n****** Optimization stopping due to optimality and feasibility tolerance being met. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NOT_CONVERGED:
        {
            aOutput = "\n\n****** Optimization algorithm did not converge. ******\n\n";
            break;
        }
        default:
        {
            aOutput = "\n\n****** ERROR: Optimization algorithm stopping due to undefined behavior. ******\n\n";
            break;
        }
    }
}


/******************************************************************************//**
 * @brief Print header for MMA diagnostics file
 * @param [in] aData diagnostic data for mma algorithm
 * @param [in,out] aOutputFile output file
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
void print_mma_diagnostics_header(const Plato::OutputDataMMA<ScalarType, OrdinalType> &aData,
                                  std::ofstream &aOutputFile)
{
    try
    {
        Plato::check_mma_inputs(aData, aOutputFile);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        THROWERR(tErrorMsg.what())
    }

    if(aData.mConstraints.size() <= static_cast<OrdinalType>(0))
    {
        THROWERR("CONTAINER WITH CONSTRAINT VALUES IS EMPTY.\n")
    }

    aOutputFile << std::scientific << std::setprecision(6) << std::right << "Iter" << std::setw(10) << "F-count"
            << std::setw(14) << "F(X)" << std::setw(16) << "Norm(F')" << std::setw(10);

    const OrdinalType tNumConstraints = aData.mConstraints.size();
    for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
    {
        if(tIndex != static_cast<OrdinalType>(0))
        {
            aOutputFile << "H" << tIndex + static_cast<OrdinalType>(1) << "(X)" << std::setw(13);
        }
        else
        {
            const OrdinalType tWidth = tNumConstraints > static_cast<OrdinalType>(1) ? 11 : 13;
            aOutputFile << "H" << tIndex + static_cast<OrdinalType>(1) << "(X)" << std::setw(tWidth);
        }
    }

    aOutputFile << std::setw(15) << "abs(dX)" << std::setw(15) << "abs(dF)" << "\n" << std::flush;
}

/******************************************************************************//**
 * @brief Print diagnostics for MMA algorithm
 * @param [in] aData diagnostic data for mma algorithm
 * @param [in,out] aOutputFile output file
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
void print_mma_diagnostics(const Plato::OutputDataMMA<ScalarType, OrdinalType> &aData,
                           std::ofstream &aOutputFile)
{
    try
    {
        Plato::check_mma_inputs(aData, aOutputFile);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        THROWERR(tErrorMsg.what())
    }
    assert(aData.mConstraints.size() > static_cast<OrdinalType>(0));

    aOutputFile << std::scientific << std::setprecision(6) << std::right << aData.mNumIter << std::setw(10)
            << aData.mObjFuncCount << std::setw(20) << aData.mObjFuncValue << std::setw(15) << aData.mNormObjFuncGrad
            << std::setw(15);

    const OrdinalType tNumConstraints = aData.mConstraints.size();
    for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
    {
        aOutputFile << aData.mConstraints[tIndex] << std::setw(15);
    }

    aOutputFile << aData.mControlStagnationMeasure << std::setw(15) << aData.mObjectiveStagnationMeasure << "\n" << std::flush;
}

/******************************************************************************//**
 * @brief Metadata associated with approximation functions, see Svanberg, Krister.
 * "MMA and GCMMA-two methods for nonlinear optimization." vol 1 (2007): 1-15.
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct ApproximationFunctionData
{
    ApproximationFunctionData(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory) :
        mCurrentNormalizedCriterionValue(0),
        mAppxFunctionP(aDataFactory->control().create()),
        mAppxFunctionQ(aDataFactory->control().create()),
        mCurrentControls(aDataFactory->control().create()),
        mLowerAsymptotes(aDataFactory->control().create()),
        mUpperAsymptotes(aDataFactory->control().create())
    {
    }

    ScalarType mCurrentNormalizedCriterionValue;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAppxFunctionP; /*!< approximation function */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAppxFunctionQ; /*!< approximation function */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentControls; /*!< current optimization variables */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mLowerAsymptotes; /*!< current lower asymptotes */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperAsymptotes; /*!< current upper asymptotes */
};
// struct ApproximationFunctionData

/******************************************************************************//**
 * @brief Data manager for Method of Moving Asymptotes (MMA) optimization algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class MethodMovingAsymptotesNewDataMng
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aDataFactory factory used to create internal data
    **********************************************************************************/
    explicit MethodMovingAsymptotesNewDataMng(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory) :
        mFeasibilityMeasure(std::numeric_limits<ScalarType>::max()),
        mNormObjectiveGradient(std::numeric_limits<ScalarType>::max()),
        mCurrentObjectiveValue(std::numeric_limits<ScalarType>::max()),
        mPreviousObjectiveValue(std::numeric_limits<ScalarType>::max()),
        mControlStagnationMeasure(std::numeric_limits<ScalarType>::max()),
        mObjectiveStagnationMeasure(std::numeric_limits<ScalarType>::max()),
        mCurrentNormalizedObjectiveValue(std::numeric_limits<ScalarType>::max()),
        mControlWork(aDataFactory->control(0 /* vector index */).create()),
        mConstraintWork(aDataFactory->dual(0 /* vector index */).create()),
        mConstraintNormalization(aDataFactory->dual(0 /* vector index */).create()),
        mCurrentConstraintValues(aDataFactory->dual(0 /* vector index */).create()),
        mPreviousConstraintValues(aDataFactory->dual(0 /* vector index */).create()),
        mCurrentNormalizedConstraintValue(aDataFactory->dual(0 /* vector index */).create()),
        mCurrentObjectiveGradient(aDataFactory->control().create()),
        mPreviousObjectiveGradient(aDataFactory->control().create()),
        mCurrentConstraintGradients(std::make_shared<MultiVectorList<ScalarType, OrdinalType>>()),
        mPreviousConstraintGradients(std::make_shared<MultiVectorList<ScalarType, OrdinalType>>()),
        mCurrentControls(aDataFactory->control().create()),
        mPreviousControls(aDataFactory->control().create()),
        mAntepenultimateControls(aDataFactory->control().create()),
        mLowerAsymptotes(aDataFactory->control().create()),
        mUpperAsymptotes(aDataFactory->control().create()),
        mControlLowerBounds(aDataFactory->control().create()),
        mControlUpperBounds(aDataFactory->control().create()),
        mSubProblemControlLowerBounds(aDataFactory->control().create()),
        mSubProblemControlUpperBounds(aDataFactory->control().create()),
        mObjFuncAppxFunctionP(aDataFactory->control().create()),
        mObjFuncAppxFunctionQ(aDataFactory->control().create()),
        mConstrAppxFunctionP(std::make_shared<MultiVectorList<ScalarType, OrdinalType>>()),
        mConstrAppxFunctionQ(std::make_shared<MultiVectorList<ScalarType, OrdinalType>>()),
        mComm(aDataFactory->getCommWrapper().create()),
        mDualReductionOps(aDataFactory->getDualReductionOperations().create()),
        mControlReductionOps(aDataFactory->getControlReductionOperations().create())
    {
        this->initialize(*aDataFactory);
    }
    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~MethodMovingAsymptotesNewDataMng()
    {
    }

    /******************************************************************************//**
     * @brief Return a const reference to the distributed memory communication wrapper
     * @return const reference to the distributed memory communication wrapper
    **********************************************************************************/
    const Plato::CommWrapper& getCommWrapper() const
    {
        return (mComm.operator*());
    }

    /******************************************************************************//**
     * @brief Return number of constraints
     * @return number of constraints
    **********************************************************************************/
    OrdinalType getNumConstraints() const
    {
        const OrdinalType tNumConstraints = mCurrentConstraintValues->size();
        return tNumConstraints;
    }

    /******************************************************************************//**
     * @brief Return feasibility measure
     * @return feasibility measure, maximum constraint residual
    **********************************************************************************/
    ScalarType getFeasibilityMeasure() const
    {
        return mFeasibilityMeasure;
    }

    /******************************************************************************//**
     * @brief Return norm of the objective function
     * @return norm of the objective function
    **********************************************************************************/
    ScalarType getNormObjectiveGradient() const
    {
        return mNormObjectiveGradient;
    }

    /******************************************************************************//**
     * @brief Return control stagnation measure
     * @return control stagnation measure
    **********************************************************************************/
    ScalarType getControlStagnationMeasure() const
    {
        return mControlStagnationMeasure;
    }

    /******************************************************************************//**
     * @brief Return objective stagnation measure
     * @return objective stagnation measure
    **********************************************************************************/
    ScalarType getObjectiveStagnationMeasure() const
    {
        return mObjectiveStagnationMeasure;
    }

    /******************************************************************************//**
     * @brief Return current objective function value
     * @return current objective function value
    **********************************************************************************/
    ScalarType getCurrentObjectiveValue() const
    {
        return mCurrentObjectiveValue;
    }

    /******************************************************************************//**
     * @brief Set current objective function value
     * @param [in] aValue current objective function value
    **********************************************************************************/
    void setCurrentObjectiveValue(const ScalarType &aValue)
    {
        mCurrentObjectiveValue = aValue;
    }

    /******************************************************************************//**
     * @brief Return previous objective function value
     * @return previous objective function value
    **********************************************************************************/
    ScalarType getPreviousObjectiveValue() const
    {
        return mPreviousObjectiveValue;
    }

    /******************************************************************************//**
     * @brief Set previous objective function value
     * @param [in] aValue previous objective function value
    **********************************************************************************/
    void setPreviousObjectiveValue(const ScalarType &aValue)
    {
        mPreviousObjectiveValue = aValue;
    }

    /******************************************************************************//**
     * @brief Return constraint normalization value
     * @param [in] aIndex constraint index
     * @return constraint normalization value
    **********************************************************************************/
    ScalarType getConstraintNormalization(const OrdinalType &aIndex) const
    {
        return ((*mConstraintNormalization)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Set constraint normalization parameters
     * @param [in] aInput reference to 1D container of constraint normalization parameters
    **********************************************************************************/
    void setConstraintNormalizationParams(const Plato::Vector<ScalarType, OrdinalType>& aInput)
    {
        mConstraintNormalization->update(static_cast<ScalarType>(1.0), aInput, static_cast<ScalarType>(0.0));
    }

    /******************************************************************************//**
     * @brief Set constraint normalization value
     * @param [in] aIndex constraint index
     * @param [in] aValue constraint normalization value
    **********************************************************************************/
    void setConstraintNormalization(const OrdinalType &aIndex, const ScalarType &aValue)
    {
        (*mConstraintNormalization)[aIndex] = aValue;
    }

    /******************************************************************************//**
     * @brief Return current constraint value
     * @param [in] aIndex constraint index
     * @return current constraint value
    **********************************************************************************/
    ScalarType getCurrentConstraintValue(const OrdinalType &aIndex) const
    {
        return ((*mCurrentConstraintValues)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Set current constraint value
     * @param [in] aIndex constraint index
     * @param [in] aValue current constraint value
    **********************************************************************************/
    void setCurrentConstraintValue(const OrdinalType &aIndex, const ScalarType &aValue)
    {
        (*mCurrentConstraintValues)[aIndex] = aValue;
    }

    /******************************************************************************//**
     * @brief Return previous constraint value
     * @param [in] aIndex constraint index
     * @return previous constraint value
    **********************************************************************************/
    ScalarType getPreviousConstraintValues(const OrdinalType &aIndex) const
    {
        return ((*mPreviousConstraintValues)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Set previous constraint value
     * @param [in] aIndex constraint index
     * @param [in] aValue previous constraint value
    **********************************************************************************/
    void setPreviousConstraintValues(const OrdinalType &aIndex, const ScalarType &aValue)
    {
        (*mPreviousConstraintValues)[aIndex] = aValue;
    }

    /******************************************************************************//**
     * @brief Gather container of constraint values
     * @param [in] aInput reference to 1D container of constraint values
    **********************************************************************************/
    void getCurrentConstraintValues(Plato::Vector<ScalarType, OrdinalType>& aInput) const
    {
        aInput.update(static_cast<ScalarType>(1.0), *mCurrentConstraintValues, static_cast<ScalarType>(0.0));
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of lower asymptotes
     * @return reference to 2D container of lower asymptotes
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getLowerAsymptotes()
    {
        return (*mLowerAsymptotes);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of lower asymptotes
     * @return const reference to 2D container of lower asymptotes
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getLowerAsymptotes() const
    {
        return (*mLowerAsymptotes);
    }

    /******************************************************************************//**
     * @brief Set const reference to 2D container of lower asymptotes
     * @param [in] aInput const reference to 2D container of lower asymptotes
    **********************************************************************************/
    void setLowerAsymptotes(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mLowerAsymptotes);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of upper asymptotes
     * @return reference to 2D container of upper asymptotes
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getUpperAsymptotes()
    {
        return (*mUpperAsymptotes);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of upper asymptotes
     * @return const reference to 2D container of upper asymptotes
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getUpperAsymptotes() const
    {
        return (*mUpperAsymptotes);
    }

    /******************************************************************************//**
     * @brief Set const reference to 2D container of upper asymptotes
     * @param [in] aInput const reference to 2D container of upper asymptotes
    **********************************************************************************/
    void setUpperAsymptotes(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mUpperAsymptotes);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of objective approximation function one (P)
     * @return reference to 2D container of objective approximation function one (P)
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getObjFuncAppxFunctionP()
    {
        return (*mObjFuncAppxFunctionP);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of objective approximation function one (P)
     * @return const reference to 2D container of objective approximation function one (P)
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getObjFuncAppxFunctionP() const
    {
        return (*mObjFuncAppxFunctionP);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of objective approximation function two (Q)
     * @return reference to 2D container of objective approximation function two (Q)
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getObjFuncAppxFunctionQ()
    {
        return (*mObjFuncAppxFunctionQ);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of objective approximation function two (Q)
     * @return const reference to 2D container of objective approximation function two (Q)
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getObjFuncAppxFunctionQ() const
    {
        return (*mObjFuncAppxFunctionQ);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of constraint approximation function one (P)
     * @param [in] aIndex constraint index
     * @return reference to 2D container of constraint approximation function one (P)
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getConstraintAppxFunctionP(const OrdinalType & aIndex)
    {
        return (*mConstrAppxFunctionP)[aIndex];
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of constraint approximation function one (P)
     * @param [in] aIndex constraint index
     * @return const reference to 2D container of constraint approximation function one (P)
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getConstraintAppxFunctionP(const OrdinalType & aIndex) const
    {
        return (*mConstrAppxFunctionP)[aIndex];
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of constraint approximation function two (Q)
     * @param [in] aIndex constraint index
     * @return reference to 2D container of constraint approximation function two (Q)
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getConstraintAppxFunctionQ(const OrdinalType & aIndex)
    {
        return (*mConstrAppxFunctionQ)[aIndex];
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of constraint approximation function two (Q)
     * @param [in] aIndex constraint index
     * @return const reference to 2D container of constraint approximation function two (Q)
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getConstraintAppxFunctionQ(const OrdinalType & aIndex) const
    {
        return (*mConstrAppxFunctionQ)[aIndex];
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of current optimization variables
     * @return reference to 2D container of current optimization variables
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getCurrentControls()
    {
        return (*mCurrentControls);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of current optimization variables
     * @return const reference to 2D container of current optimization variables
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getCurrentControls() const
    {
        return (*mCurrentControls);
    }

    /******************************************************************************//**
     * @brief Set 2D container of current optimization variables
     * @return const reference to 2D container of current optimization variables
    **********************************************************************************/
    void setCurrentControls(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mCurrentControls);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of previous optimization variables
     * @return reference to 2D container of previous optimization variables
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getPreviousControls()
    {
        return (*mPreviousControls);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of previous optimization variables
     * @return const reference to 2D container of previous optimization variables
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getPreviousControls() const
    {
        return (*mPreviousControls);
    }

    /******************************************************************************//**
     * @brief Set 2D container of previous optimization variables
     * @return const reference to 2D container of previous optimization variables
    **********************************************************************************/
    void setPreviousControls(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mPreviousControls);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of antepenultimate optimization variables
     * @return reference to 2D container of antepenultimate optimization variables
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getAntepenultimateControls()
    {
        return (*mAntepenultimateControls);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of antepenultimate optimization variables
     * @return const reference to 2D container of antepenultimate optimization variables
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getAntepenultimateControls() const
    {
        return (*mAntepenultimateControls);
    }

    /******************************************************************************//**
     * @brief Set 2D container of antepenultimate optimization variables
     * @return const reference to 2D container of antepenultimate optimization variables
    **********************************************************************************/
    void setAntepenultimateControls(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mAntepenultimateControls);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of lower bounds on optimization variables
     * @return reference to 2D container of lower bounds on optimization variables
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getControlLowerBounds()
    {
        return (*mControlLowerBounds);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of lower bounds on optimization variables
     * @return const reference to 2D container of lower bounds on optimization variables
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getControlLowerBounds() const
    {
        return (*mControlLowerBounds);
    }

    /******************************************************************************//**
     * @brief Set 2D container of lower bounds on optimization variables
     * @return const reference to 2D container of lower bounds on optimization variables
    **********************************************************************************/
    void setControlLowerBounds(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mControlLowerBounds);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of upper bounds on optimization variables
     * @return reference to 2D container of upper bounds on optimization variables
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getControlUpperBounds()
    {
        return (*mControlUpperBounds);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of upper bounds on optimization variables
     * @return const reference to 2D container of upper bounds on optimization variables
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getControlUpperBounds() const
    {
        return (*mControlUpperBounds);
    }

    /******************************************************************************//**
     * @brief Set 2D container of upper bounds on optimization variables
     * @return const reference to 2D container of upper bounds on optimization variables
    **********************************************************************************/
    void setControlUpperBounds(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mControlUpperBounds);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of lower bounds on optimization variables for MMA subproblem
     * @return reference to 2D container of lower bounds on optimization variables for MMA subproblem
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getSubProblemControlLowerBounds()
    {
        return (*mSubProblemControlLowerBounds);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of lower bounds on optimization variables for MMA subproblem
     * @return const reference to 2D container of lower bounds on optimization variables for MMA subproblem
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getSubProblemControlLowerBounds() const
    {
        return (*mSubProblemControlLowerBounds);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of upper bounds on optimization variables for MMA subproblem
     * @return reference to 2D container of upper bounds on optimization variables for MMA subproblem
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getSubProblemControlUpperBounds()
    {
        return (*mSubProblemControlUpperBounds);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of upper bounds on optimization variables for MMA subproblem
     * @return const reference to 2D container of upper bounds on optimization variables for MMA subproblem
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getSubProblemControlUpperBounds() const
    {
        return (*mSubProblemControlUpperBounds);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of current objective function first-order sensitivities
     * @return reference to 2D container of current objective function first-order sensitivities
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getCurrentObjectiveGradient()
    {
        return (*mCurrentObjectiveGradient);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of current objective function first-order sensitivities
     * @return const reference to 2D container of current objective function first-order sensitivities
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getCurrentObjectiveGradient() const
    {
        return (*mCurrentObjectiveGradient);
    }

    /******************************************************************************//**
     * @brief Set 2D container of current objective function first-order sensitivities
     * @return reference to 2D container ofcurrent  objective function first-order sensitivities
    **********************************************************************************/
    void setCurrentObjectiveGradient(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mCurrentObjectiveGradient);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of previous objective function first-order sensitivities
     * @return reference to 2D container of previous objective function first-order sensitivities
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getPreviousObjectiveGradient()
    {
        return (*mPreviousObjectiveGradient);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of previous objective function first-order sensitivities
     * @return const reference to 2D container of previous objective function first-order sensitivities
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getPreviousObjectiveGradient() const
    {
        return (*mPreviousObjectiveGradient);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of current constraint first-order sensitivities
     * @param [in] aIndex constraint index
     * @return reference to 2D container of current constraint first-order sensitivities
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getCurrentConstraintGradient(const OrdinalType &aIndex)
    {
        return (*mCurrentConstraintGradients)[aIndex];
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of current constraint first-order sensitivities
     * @param [in] aIndex constraint index
     * @return const reference to 2D container of current constraint first-order sensitivities
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getCurrentConstraintGradient(const OrdinalType &aIndex) const
    {
        return (*mCurrentConstraintGradients)[aIndex];
    }

    /******************************************************************************//**
     * @brief Set 2D container of current constraint first-order sensitivities
     * @param [in] aIndex constraint index
     * @param const reference to 2D container of current constraint first-order sensitivities
    **********************************************************************************/
    void setCurrentConstraintGradient(const OrdinalType &aIndex, const Plato::MultiVector<ScalarType, OrdinalType>& aGradient)
    {
        if(aIndex >= mCurrentConstraintGradients->size())
        {
            std::string tMsg = std::string("THE INPUT CONSTRAINT GRADIENT INDEX IS GREATER THAN THE NUMBER OF CONSTRAINTS. ")
                    + "THE NUMBER OF CONSTRAINTS IS SET TO " + std::to_string(aIndex) + " AND THE INPUT INDEX IS "
                    + std::to_string(aIndex) + "\n";
            THROWERR(tMsg)
        }
        Plato::update(static_cast<ScalarType>(1), aGradient, static_cast<ScalarType>(0), (*mCurrentConstraintGradients)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of previous constraint first-order sensitivities
     * @param [in] aIndex constraint index
     * @return reference to 2D container of previous constraint first-order sensitivities
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getPreviousConstraintGradient(const OrdinalType &aIndex)
    {
        return (*mPreviousConstraintGradients)[aIndex];
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of previous constraint first-order sensitivities
     * @param [in] aIndex constraint index
     * @return const reference to 2D container of previous constraint first-order sensitivities
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getPreviousConstraintGradient(const OrdinalType &aIndex) const
    {
        return (*mPreviousConstraintGradients)[aIndex];
    }

    /******************************************************************************//**
     * @brief Cache current state, e.g. controls and objective and constraints values and gradients
    **********************************************************************************/
    void cacheState()
    {
        Plato::update(static_cast<ScalarType>(1), *mPreviousControls, static_cast<ScalarType>(0), *mAntepenultimateControls);
        Plato::update(static_cast<ScalarType>(1), *mCurrentControls, static_cast<ScalarType>(0), *mPreviousControls);

        mPreviousObjectiveValue = mCurrentObjectiveValue;
        Plato::update(static_cast<ScalarType>(1), *mCurrentObjectiveGradient, static_cast<ScalarType>(0), *mPreviousObjectiveGradient);

        mPreviousConstraintValues->update(static_cast<ScalarType>(1), *mCurrentConstraintValues, static_cast<ScalarType>(0));
        const OrdinalType tNumConstraints = mCurrentConstraintValues->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            Plato::update(static_cast<ScalarType>(1),
                          (*mCurrentConstraintGradients)[tIndex],
                          static_cast<ScalarType>(0),
                          (*mPreviousConstraintGradients)[tIndex]);
        }
    }

    /******************************************************************************//**
     * @brief Compute stopping measures
    **********************************************************************************/
    void computeStoppingMeasures()
    {
        this->computeFeasibilityMeasure();
        this->computeNormObjectiveGradient();
        this->computeControlStagnationMeasure();
        this->computeObjectiveStagnationMeasure();
    }

    /******************************************************************************//**
     * @brief Compute feasibility tolerance
    **********************************************************************************/
    void computeFeasibilityMeasure()
    {
        mConstraintWork->update(static_cast<ScalarType>(1), *mCurrentConstraintValues, static_cast<ScalarType>(0));
        mConstraintWork->modulus();
        mFeasibilityMeasure = mDualReductionOps->max(*mConstraintWork);
    }

    /******************************************************************************//**
     * @brief Compute norm of the objective function gradient
    **********************************************************************************/
    void computeNormObjectiveGradient()
    {
        mNormObjectiveGradient = Plato::norm(*mCurrentObjectiveGradient);
    }

    /******************************************************************************//**
     * @brief Compute control stagnation measure
    **********************************************************************************/
    void computeControlStagnationMeasure()
    {
        OrdinalType tNumVectors = mCurrentControls->getNumVectors();
        std::vector<ScalarType> tStorage(tNumVectors, std::numeric_limits<ScalarType>::min());
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            mControlWork->update(static_cast<ScalarType>(1), (*mCurrentControls)[tIndex], static_cast<ScalarType>(0));
            mControlWork->update(static_cast<ScalarType>(-1), (*mPreviousControls)[tIndex], static_cast<ScalarType>(1));
            mControlWork->modulus();
            tStorage[tIndex] = mControlReductionOps->max(*mControlWork);
        }
        mControlStagnationMeasure = *std::max_element(tStorage.begin(), tStorage.end());
    }

    /******************************************************************************//**
     * @brief Compute objective stagnation measure
    **********************************************************************************/
    void computeObjectiveStagnationMeasure()
    {
        mObjectiveStagnationMeasure = mCurrentObjectiveValue - mPreviousObjectiveValue;
        mObjectiveStagnationMeasure = std::abs(mObjectiveStagnationMeasure);
    }

private:
    /******************************************************************************//**
     * @brief Initialize internal metadata
    **********************************************************************************/
    void initialize(const Plato::DataFactory<ScalarType, OrdinalType> &aDataFactory)
    {
        const OrdinalType tNumConstraints = this->getNumConstraints();
        if(tNumConstraints < static_cast<OrdinalType>(0))
        {
            THROWERR(std::string("INVALID NUMBER OF CONSTRAINTS ") + std::to_string(tNumConstraints) + ". THE NUMBER OF CONSTRAINTS SHOULD BE ZERO OR A POSITIVE NUMBER.\n")
        }

        mConstraintNormalization->fill(1.0);
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            mConstrAppxFunctionP->add(aDataFactory.control().create());
            mConstrAppxFunctionQ->add(aDataFactory.control().create());
            mCurrentConstraintGradients->add(aDataFactory.control().create());
            mPreviousConstraintGradients->add(aDataFactory.control().create());
        }
    }

private:
    ScalarType mFeasibilityMeasure; /*!< current feasibility measure */
    ScalarType mNormObjectiveGradient; /*!< current norm of the objective function gradient */
    ScalarType mCurrentObjectiveValue; /*!< current objective value */
    ScalarType mPreviousObjectiveValue; /*!< previous objective value */
    ScalarType mControlStagnationMeasure; /*!< current control stagnation measure */
    ScalarType mObjectiveStagnationMeasure; /*!< current objective stagnation measure */
    ScalarType mCurrentNormalizedObjectiveValue; /*!< current normalize objective value */

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWork; /*!< 1D container of optimization variables - work */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mConstraintWork; /*!< 1D container of constraint values - work */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mConstraintNormalization; /*!< 1D container of constraint normalization factors */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentConstraintValues; /*!< 1D container of current constraint values */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mPreviousConstraintValues; /*!< 1D container of previous constraint values */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentNormalizedConstraintValue; /*!< 1D container of current normalized constraint values */

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentObjectiveGradient; /*!< 2D container of current objective first-order sensitivities */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousObjectiveGradient; /*!< 2D container of previous objective first-order sensitivities */
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mCurrentConstraintGradients; /*!< 2D container of current constraint first-order sensitivities */
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mPreviousConstraintGradients; /*!< 2D container of previous constraint first-order sensitivities */

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentControls; /*!< 2D container of current optimization variables */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousControls; /*!< 2D container of previous optimization variables */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAntepenultimateControls; /*!< 2D container of antepenultimate optimization variables */

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mLowerAsymptotes; /*!< 2D container of current lower asymptotes */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperAsymptotes; /*!< 2D container of current upper asymptotes */

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mControlLowerBounds; /*!< 2D container of lower bounds on optimization variables */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mControlUpperBounds; /*!< 2D container of upper bounds on optimization variables */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mSubProblemControlLowerBounds; /*!< 2D container of lower bounds on optimization variables for MMA subproblem */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mSubProblemControlUpperBounds; /*!< 2D container of upper bounds on optimization variables for MMA subproblem */

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mObjFuncAppxFunctionP; /*!< 2D container of objective approximation function one (P) */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mObjFuncAppxFunctionQ; /*!< 2D container of objective approximation function two (Q) */
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mConstrAppxFunctionP; /*!< 2D container of constraint approximation function one (P) */
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mConstrAppxFunctionQ; /*!< 2D container of constraint approximation function two (Q) */

    std::shared_ptr<Plato::CommWrapper> mComm; /*!< wrapper to MPI communicator */
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mDualReductionOps; /*!< reduction operation interface for dual variables */
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mControlReductionOps; /*!< reduction operation interface for control variables */

private:
    MethodMovingAsymptotesNewDataMng(const Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType> & aRhs);
    Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType> & operator=(const Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType> & aRhs);
};
// class MethodMovingAsymptotesNewDataMng

/******************************************************************************//**
 * Criterion interface to the approximation function used to solve the Method of
 * Moving Asymptotes (MMA) subproblem.
 *
 * The approximation function for the MMA subproblem is defined as
 * /f$\sum_{j=1}^{N}\left( \alpha\frac{p_{ij}^{k}}{u_j^k - x_j} + \beta \left(
 *    \frac{q_{ij}^{k}}{x_j - l_j} + r_{i} \right) \right)/f$
 *
 * where
 *
 * /f$r_i = f_i(\mathbf{x}^k) - \sum_{j=1}^{N}\left( \frac{p_{ij}^{k}}{u_j^k - x_j^k}
 * + \frac{q_{ij}^{k}}{x_j^k - l_j} \right)/f$
 *
 * /f$p_{ij} = (x_j^k - l_j)^2\left( \alpha_{1}\left(\frac{\partial{f}_i}{\partial{x}_j}
 * (\mathbf{x}^k)\right)^{+} + \alpha_{2}\left(\frac{\partial{f}_i}{\partial{x}_j}
 * (\mathbf{x}^k)\right)^{-} + \frac{\epsilon}{x_j^{\max} - x_j^{\min}}\right)/f$
 *
 * /f$q_{ij} = (x_j^k - l_j)^2\left( \alpha_{2}\left(\frac{\partial{f}_i}{\partial{x}_j}
 * (\mathbf{x}^k)\right)^{+} + \alpha_{1}\left(\frac{\partial{f}_i}{\partial{x}_j}
 * (\mathbf{x}^k)\right)^{-} + \frac{\epsilon}{x_j^{\max} - x_j^{\min}}\right)/f$
 *
 * /f$\frac{\partial{f}_i}{\partial{x}_j}(\mathbf{x}^k)\right)^{+} = \max\left(
 * \frac{\partial{f}_i}{\partial{x}_j}, 0 \right)/f$
 *
 * /f$\frac{\partial{f}_i}{\partial{x}_j}(\mathbf{x}^k)\right)^{-} = \max\left(
 * -\frac{\partial{f}_i}{\partial{x}_j}, 0 \right)/f$
 *
 * Nomenclature:
 *
 * /f$\alpha/f$: positive constant {0}\leq\alpha\leq{1}
 * /f$\beta/f$: positive constant {0}\leq\\beta\leq{1}
 * /f$x_j/f$: trial control j-th value
 * /f$x_j^{\max}/f$: j-th value for control upper bound
 * /f$x_j^{\min}/f$: j-th value for control lower bound
 * /f$u_j/f$: upper asymptote j-th value
 * /f$l_j/f$: lower asymptote j-th value
 * /f$\epsilon/f$: positive coefficient
 * /f$\alpha_{1}, \alpha_{2}/f$: positive coefficients
 * /f$\mathbf{x}^k/f$: current controls
 * /f$f_i(\mathbf{x}^k)/f$: current criterion value
 * /f$\frac{\partial{f}_i}{\partial{x}_j}(\mathbf{x}^k)/f$: current criterion gradient
 *
***********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class MethodMovingAsymptotesNewCriterion : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * Constructor
     * @param [in] aDataFactory constant reference to the core data factory shared pointer.
    ***********************************************************************************/
    explicit MethodMovingAsymptotesNewCriterion(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory) :
        mObjFuncAppxFuncMultiplier(1),
        mConstraintAppxFuncMultiplier(1),
        mCurrentNormalizedCriterionValue(0),
        mControlWork1(aDataFactory->control(0 /* vector index */).create()),
        mControlWork2(aDataFactory->control(0 /* vector index */).create()),
        mAppxFunctionP(aDataFactory->control().create()),
        mAppxFunctionQ(aDataFactory->control().create()),
        mCurrentControls(aDataFactory->control().create()),
        mLowerAsymptotes(aDataFactory->control().create()),
        mUpperAsymptotes(aDataFactory->control().create()),
        mControlReductionOps(aDataFactory->getControlReductionOperations().create())
    {
    }

    /******************************************************************************//**
     * Destructor
    ***********************************************************************************/
    ~MethodMovingAsymptotesNewCriterion()
    {
    }

    /******************************************************************************//**
     * Set objective approximation function multiplier.
    ***********************************************************************************/
    void setObjFuncAppxFuncMultiplier(const ScalarType& aInput)
    {
        mObjFuncAppxFuncMultiplier = aInput;
    }

    /******************************************************************************//**
     * Set constraint approximation functions multiplier.
    ***********************************************************************************/
    void setConstraintAppxFuncMultiplier(const ScalarType& aInput)
    {
        mConstraintAppxFuncMultiplier = aInput;
    }

    /******************************************************************************//**
     * Safely cache application specific data after a new trial control is accepted.
    ***********************************************************************************/
    void cacheData()
    {
        return;
    }

    /******************************************************************************//**
     * Update approximation functions for the next subproblem iteration
     * @param [in] aData struct with approximation function's data
    ***********************************************************************************/
    void update(const Plato::ApproximationFunctionData<ScalarType, OrdinalType> &aData)
    {
        mCurrentNormalizedCriterionValue = aData.mCurrentNormalizedCriterionValue;
        Plato::update(static_cast<ScalarType>(1), *aData.mAppxFunctionP, static_cast<ScalarType>(0), *mAppxFunctionP);
        Plato::update(static_cast<ScalarType>(1), *aData.mAppxFunctionQ, static_cast<ScalarType>(0), *mAppxFunctionQ);
        Plato::update(static_cast<ScalarType>(1), *aData.mCurrentControls, static_cast<ScalarType>(0), *mCurrentControls);
        Plato::update(static_cast<ScalarType>(1), *aData.mLowerAsymptotes, static_cast<ScalarType>(0), *mLowerAsymptotes);
        Plato::update(static_cast<ScalarType>(1), *aData.mUpperAsymptotes, static_cast<ScalarType>(0), *mUpperAsymptotes);
    }

    /******************************************************************************//**
     * Evaluate approximation function.
     * @param [in] aControl: control, i.e. design, variables
     * @return approximation function value
     ***********************************************************************************/
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> &aControl)
    {
        const ScalarType tOutput = this->evaluateApproximationFunction(aControl);
        return (tOutput);
    }

    /******************************************************************************//**
     * Compute approximation function gradient.
     * @param [in] aControl: control, i.e. design, variables
     * @param [in/out] aOutput: gradient of approximation function
     ***********************************************************************************/
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> &aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> &aOutput)
    {
        this->computeApproximationFunctionGradient(aControl, aOutput);
    }

    /******************************************************************************//**
     * Apply vector to approximation function Hessian.
     * @param [in] aControl: control, i.e. design, variables
     * @param [in] aVector: descent direction
     * @param [in/out] aOutput: application of vector to the Hessian of the approximation function
     ***********************************************************************************/
    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> &aControl,
                 const Plato::MultiVector<ScalarType, OrdinalType> &aVector,
                 Plato::MultiVector<ScalarType, OrdinalType> &aOutput)
    {
        this->computeApproximationFunctionHessTimesVec(aControl, aVector, aOutput);
    }

private:
    /******************************************************************************//**
     * Evaluate approximation function
     * @param [in] aControls optimization variables
    ***********************************************************************************/
    ScalarType evaluateApproximationFunction(const Plato::MultiVector<ScalarType, OrdinalType> &aControls)
    {
        mControlWork1->fill(static_cast<ScalarType>(0));
        mControlWork2->fill(static_cast<ScalarType>(0));

        const OrdinalType tNumVectors = aControls.getNumVectors();
        for (OrdinalType tVecIndex = 0; tVecIndex < tNumVectors; tVecIndex++)
        {
            const OrdinalType tNumControls = aControls[tVecIndex].size();
            for (OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                ScalarType tDenominator = (*mUpperAsymptotes)(tVecIndex, tControlIndex) - aControls(tVecIndex, tControlIndex);
                tDenominator = std::abs(tDenominator) > std::numeric_limits<ScalarType>::epsilon()
                        ? tDenominator : std::numeric_limits<ScalarType>::epsilon();
                ScalarType tValueOne = (*mAppxFunctionP)(tVecIndex, tControlIndex) / tDenominator;
                tDenominator = aControls(tVecIndex, tControlIndex) - (*mLowerAsymptotes)(tVecIndex, tControlIndex);
                tDenominator = std::abs(tDenominator) > std::numeric_limits<ScalarType>::epsilon()
                        ? tDenominator : std::numeric_limits<ScalarType>::epsilon();
                ScalarType tValueTwo = (*mAppxFunctionQ)(tVecIndex, tControlIndex) / tDenominator;
                (*mControlWork1)[tControlIndex] += tValueOne + tValueTwo;

                tDenominator = (*mUpperAsymptotes)(tVecIndex, tControlIndex) - (*mCurrentControls)(tVecIndex, tControlIndex);
                tDenominator = std::abs(tDenominator) > std::numeric_limits<ScalarType>::epsilon()
                        ? tDenominator : std::numeric_limits<ScalarType>::epsilon();
                tValueOne = (*mAppxFunctionP)(tVecIndex, tControlIndex) / tDenominator;
                tDenominator = (*mCurrentControls)(tVecIndex, tControlIndex) - (*mLowerAsymptotes)(tVecIndex, tControlIndex);
                tDenominator = std::abs(tDenominator) > std::numeric_limits<ScalarType>::epsilon()
                        ? tDenominator : std::numeric_limits<ScalarType>::epsilon();
                tValueTwo = (*mAppxFunctionQ)(tVecIndex, tControlIndex) / tDenominator;
                (*mControlWork2)[tControlIndex] += tValueOne + tValueTwo;
            }
        }

        const ScalarType tTermOne = mControlReductionOps->sum(*mControlWork1);
        const ScalarType tTermTwo = mControlReductionOps->sum(*mControlWork2);
        const ScalarType tOutput = (mObjFuncAppxFuncMultiplier * tTermOne)
            + (mConstraintAppxFuncMultiplier * (mCurrentNormalizedCriterionValue - tTermTwo));

        return (tOutput);
    }

    /******************************************************************************//**
     * Compute approximation function gradient
     * @param [in] aControls optimization variables
     * @param [out] aGradient approximation function gradient
    ***********************************************************************************/
    void computeApproximationFunctionGradient(const Plato::MultiVector<ScalarType, OrdinalType> &aControls,
                                              Plato::MultiVector<ScalarType, OrdinalType> &aGradient)
    {
        const OrdinalType tNumVectors = aControls.getNumVectors();
        for (OrdinalType tVecIndex = 0; tVecIndex < tNumVectors; tVecIndex++)
        {
            const OrdinalType tNumControls = aControls[tVecIndex].size();
            for (OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                ScalarType tDenominatorValue = (*mUpperAsymptotes)(tVecIndex, tControlIndex) - aControls(tVecIndex, tControlIndex);
                tDenominatorValue = std::abs(tDenominatorValue) > std::numeric_limits<ScalarType>::epsilon()
                        ? tDenominatorValue : std::numeric_limits<ScalarType>::epsilon();
                ScalarType tDenominator = tDenominatorValue * tDenominatorValue;
                ScalarType tValueOne = (*mAppxFunctionP)(tVecIndex, tControlIndex) / tDenominator;

                tDenominatorValue = aControls(tVecIndex, tControlIndex) - (*mLowerAsymptotes)(tVecIndex, tControlIndex);
                tDenominatorValue = std::abs(tDenominatorValue) > std::numeric_limits<ScalarType>::epsilon()
                        ? tDenominatorValue : std::numeric_limits<ScalarType>::epsilon();
                tDenominator = tDenominatorValue * tDenominatorValue;
                ScalarType tValueTwo = (*mAppxFunctionQ)(tVecIndex, tControlIndex) / tDenominator;
                aGradient(tVecIndex, tControlIndex) = (mObjFuncAppxFuncMultiplier * tValueOne) - tValueTwo;
            }
        }
    }

    /******************************************************************************//**
     * Compute approximation function gradient
     * @param [in] aControls optimization variables
     * @param [in] aVector descent direction
     * @param [out] aHessTimesVec vector apply to approximation function's Hessian
    ***********************************************************************************/
    void computeApproximationFunctionHessTimesVec(const Plato::MultiVector<ScalarType, OrdinalType> &aControls,
                                                  const Plato::MultiVector<ScalarType, OrdinalType> &aVector,
                                                  Plato::MultiVector<ScalarType, OrdinalType> &aHessTimesVec)
    {
        const OrdinalType tNumVectors = aControls.getNumVectors();
        for (OrdinalType tVecIndex = 0; tVecIndex < tNumVectors; tVecIndex++)
        {
            const OrdinalType tNumControls = aControls[tVecIndex].size();
            for (OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                ScalarType tDenominatorValue = (*mUpperAsymptotes)(tVecIndex, tControlIndex) - aControls(tVecIndex, tControlIndex);
                tDenominatorValue =
                    std::abs(tDenominatorValue) > std::numeric_limits<ScalarType>::epsilon() ? tDenominatorValue : std::numeric_limits<ScalarType>::epsilon();
                ScalarType tDenominator = tDenominatorValue * tDenominatorValue * tDenominatorValue;
                ScalarType tValueOne = (*mAppxFunctionP)(tVecIndex, tControlIndex) / tDenominator;

                tDenominatorValue = aControls(tVecIndex, tControlIndex) - (*mLowerAsymptotes)(tVecIndex, tControlIndex);
                tDenominatorValue =
                    std::abs(tDenominatorValue) > std::numeric_limits<ScalarType>::epsilon() ? tDenominatorValue : std::numeric_limits<ScalarType>::epsilon();
                tDenominator = tDenominatorValue * tDenominatorValue * tDenominatorValue;
                ScalarType tValueTwo = (*mAppxFunctionQ)(tVecIndex, tControlIndex) / tDenominator;
                aHessTimesVec(tVecIndex, tControlIndex) = static_cast<ScalarType>(2) * ((mObjFuncAppxFuncMultiplier * tValueOne) + tValueTwo)
                    * aVector(tVecIndex, tControlIndex);
            }
        }
    }

private:
    ScalarType mObjFuncAppxFuncMultiplier; /*!< positive constant {0}\leq\alpha\leq{1} */
    ScalarType mConstraintAppxFuncMultiplier; /*!< positive constant {0}\leq\beta\leq{1} */
    ScalarType mCurrentNormalizedCriterionValue; /*!< current normalized objective function value */

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWork1; /*!< optimization variables work vector one */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWork2; /*!< optimization variables work vector two */

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAppxFunctionP; /*!< approximation function one (P) */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAppxFunctionQ; /*!< approximation function two (Q) */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentControls; /*!< current optimization variables */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mLowerAsymptotes; /*!< current lower asymptotes */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperAsymptotes; /*!< current upper asymptotes */

    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mControlReductionOps; /*!< reduction operation interface for controls */

private:
    MethodMovingAsymptotesNewCriterion(const Plato::MethodMovingAsymptotesNewCriterion<ScalarType, OrdinalType> & aRhs);
    Plato::MethodMovingAsymptotesNewCriterion<ScalarType, OrdinalType> & operator=(const Plato::MethodMovingAsymptotesNewCriterion<ScalarType, OrdinalType> & aRhs);
};
// class MethodMovingAsymptotesNewCriterion

/******************************************************************************//**
 * @brief Main operations for the Method Moving Asymptotes (MMA) optimization algorithm
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class MethodMovingAsymptotesOperations
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aDataFactory factory used to allocate internal metadata
     **********************************************************************************/
    explicit MethodMovingAsymptotesOperations(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory) :
        mMoveLimit(0.5),
        mApproxFuncEpsilon(1e-5),
        mAsymptoteExpansion(1.2),
        mAsymptoteContraction(0.7),
        mApproxFuncScalingOne(1.001),
        mApproxFuncScalingTwo(0.001),
        mInitialAymptoteScaling(0.5),
        mSubProblemBoundsScaling(0.1),
        mUpperMinusLowerBounds(aDataFactory->control().create()),
        mCurrentAsymptotesMultipliers(aDataFactory->control().create())
    {
    }

    /******************************************************************************//**
     * @brief Destructor
     **********************************************************************************/
    ~MethodMovingAsymptotesOperations()
    {
    }

    /******************************************************************************//**
     * @brief Set move limit
     * @param [in] aInput move limit
     **********************************************************************************/
    void setMoveLimit(const ScalarType& aInput)
    {
        mMoveLimit = aInput;
    }

    /******************************************************************************//**
     * @brief Set moving asymptotes expansion parameter
     * @param [in] aInput expansion parameter
     **********************************************************************************/
    void setAsymptoteExpansionParameter(const ScalarType& aInput)
    {
        mAsymptoteExpansion = aInput;
    }

    /******************************************************************************//**
     * @brief Set moving asymptotes contraction parameter
     * @param [in] aInput contraction parameter
     **********************************************************************************/
    void setAsymptoteContractionParameter(const ScalarType& aInput)
    {
        mAsymptoteContraction = aInput;
    }

    /******************************************************************************//**
     * @brief Set scaling for initial moving asymptotes
     * @param [in] aInput scaling for initial moving asymptotes
     **********************************************************************************/
    void setInitialAymptoteScaling(const ScalarType& aInput)
    {
        mInitialAymptoteScaling = aInput;
    }

    /******************************************************************************//**
     * @brief Set scaling parameter on upper and lower bounds for subproblem
     * @param [in] aInput scaling parameter on upper and lower bounds for subproblem
     **********************************************************************************/
    void setSubProblemBoundsScaling(const ScalarType& aInput)
    {
        mSubProblemBoundsScaling = aInput;
    }

    /******************************************************************************//**
     * @brief Gather 2D container of upper minus lower bounds
     * @param [in] aData reference to 2D container
     **********************************************************************************/
    void getUpperMinusLowerBounds(Plato::MultiVector<ScalarType, OrdinalType> &aData)
    {
        Plato::update(static_cast<ScalarType>(1), *mUpperMinusLowerBounds, static_cast<ScalarType>(0), aData);
    }

    /******************************************************************************//**
     * @brief Gather 2D container of current moving asymptotes multipliers
     * @param [in] aData reference to 2D container of current moving asymptotes multipliers
     **********************************************************************************/
    void getCurrentAsymptotesMultipliers(Plato::MultiVector<ScalarType, OrdinalType> &aData)
    {
        Plato::update(static_cast<ScalarType>(1), *mCurrentAsymptotesMultipliers, static_cast<ScalarType>(0), aData);
    }

    /******************************************************************************//**
     * @brief Set lower and upper bounds on the optimization variables
     * @param [in] aDataMng MMA data manager interface
     **********************************************************************************/
    void initialize(const Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tControlUpperBounds = aDataMng.getControlUpperBounds();
        Plato::update(static_cast<ScalarType>(1), tControlUpperBounds, static_cast<ScalarType>(0), *mUpperMinusLowerBounds);
        const Plato::MultiVector<ScalarType, OrdinalType> &tControlLowerBounds = aDataMng.getControlLowerBounds();
        Plato::update(static_cast<ScalarType>(-1), tControlLowerBounds, static_cast<ScalarType>(1), *mUpperMinusLowerBounds);
    }

    /******************************************************************************//**
     * @brief Update initial moving asymptotes
     * @param [in] aDataMng MMA data manager interface
     **********************************************************************************/
    void updateInitialAsymptotes(Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = aDataMng.getCurrentControls();
        Plato::MultiVector<ScalarType, OrdinalType> &tLowerAsymptotes = aDataMng.getLowerAsymptotes();
        Plato::update(static_cast<ScalarType>(1), tCurrentControls, static_cast<ScalarType>(0), tLowerAsymptotes);
        Plato::update(-mInitialAymptoteScaling, *mUpperMinusLowerBounds, static_cast<ScalarType>(1), tLowerAsymptotes);

        Plato::MultiVector<ScalarType, OrdinalType> &tUpperAsymptotes = aDataMng.getUpperAsymptotes();
        Plato::update(static_cast<ScalarType>(1), tCurrentControls, static_cast<ScalarType>(0), tUpperAsymptotes);
        Plato::update(mInitialAymptoteScaling, *mUpperMinusLowerBounds, static_cast<ScalarType>(1), tUpperAsymptotes);
    }

    /******************************************************************************//**
     * @brief Update current moving asymptotes multipliers
     * @param [in] aDataMng MMA data manager interface
     **********************************************************************************/
    void updateCurrentAsymptotesMultipliers(Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = aDataMng.getCurrentControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tPreviousControls = aDataMng.getPreviousControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tAntepenultimateControls = aDataMng.getAntepenultimateControls();

        const OrdinalType tNumVectors = tCurrentControls.getNumVectors();
        for (OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const OrdinalType tNumControls = tCurrentControls[tVectorIndex].size();
            for (OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                const ScalarType tMeasure = (tCurrentControls(tVectorIndex, tControlIndex) - tPreviousControls(tVectorIndex, tControlIndex))
                    * (tPreviousControls(tVectorIndex, tControlIndex) - tAntepenultimateControls(tVectorIndex, tControlIndex));
                ScalarType tGammaValue = tMeasure > static_cast<ScalarType>(0) ? mAsymptoteExpansion : mAsymptoteContraction;
                (*mCurrentAsymptotesMultipliers)(tVectorIndex, tControlIndex) =
                        std::abs(tGammaValue) <= std::numeric_limits<ScalarType>::min() ? static_cast<ScalarType>(1) : tGammaValue;
            }
        }
    }

    /******************************************************************************//**
     * @brief Update current moving asymptotes
     * @param [in] aDataMng MMA data manager interface
     **********************************************************************************/
    void updateCurrentAsymptotes(Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        Plato::MultiVector<ScalarType, OrdinalType> &tLowerAsymptotes = aDataMng.getLowerAsymptotes();
        Plato::MultiVector<ScalarType, OrdinalType> &tUpperAsymptotes = aDataMng.getUpperAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = aDataMng.getCurrentControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tPreviousControls = aDataMng.getPreviousControls();

        const OrdinalType tNumVectors = tCurrentControls.getNumVectors();
        for (OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const OrdinalType tNumControls = tCurrentControls[tVectorIndex].size();
            for (OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                tLowerAsymptotes(tVectorIndex, tControlIndex) = tCurrentControls(tVectorIndex, tControlIndex)
                    - ((*mCurrentAsymptotesMultipliers)(tVectorIndex, tControlIndex)
                            * (tPreviousControls(tVectorIndex, tControlIndex) - tLowerAsymptotes(tVectorIndex, tControlIndex)));
                tUpperAsymptotes(tVectorIndex, tControlIndex) = tCurrentControls(tVectorIndex, tControlIndex)
                    + ((*mCurrentAsymptotesMultipliers)(tVectorIndex, tControlIndex)
                            * (tUpperAsymptotes(tVectorIndex, tControlIndex) - tPreviousControls(tVectorIndex, tControlIndex)));
            }
        }
    }

    /******************************************************************************//**
     * @brief Update current objective approximation functions
     * @param [in] aDataMng MMA data manager interface
     **********************************************************************************/
    void updateObjectiveApproximationFunctionData(Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        ScalarType tNormalizationValue = std::abs(aDataMng.getCurrentObjectiveValue());
        if(tNormalizationValue <= std::numeric_limits<ScalarType>::epsilon())
        {
            tNormalizationValue = 1.0;
        }

        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = aDataMng.getCurrentControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tLowerAsymptotes = aDataMng.getLowerAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tUpperAsymptotes = aDataMng.getUpperAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentObjGrad = aDataMng.getCurrentObjectiveGradient();

        Plato::MultiVector<ScalarType, OrdinalType> &tAppxFunctionP = aDataMng.getObjFuncAppxFunctionP();
        Plato::MultiVector<ScalarType, OrdinalType> &tAppxFunctionQ = aDataMng.getObjFuncAppxFunctionQ();
        this->computeApproxFuncCoefficients(tNormalizationValue,
                                            tCurrentControls,
                                            tLowerAsymptotes,
                                            tUpperAsymptotes,
                                            tCurrentObjGrad,
                                            tAppxFunctionP,
                                            tAppxFunctionQ);
    }

    /******************************************************************************//**
     * @brief Update current constraint approximation functions
     * @param [in] aDataMng MMA data manager interface
     **********************************************************************************/
    void updateConstraintApproximationFunctionsData(Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = aDataMng.getCurrentControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tLowerAsymptotes = aDataMng.getLowerAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tUpperAsymptotes = aDataMng.getUpperAsymptotes();

        const OrdinalType tNumConstraints = aDataMng.getNumConstraints();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            const ScalarType tConstraintNormalization = aDataMng.getConstraintNormalization(tIndex);
            const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentConstraintGrad = aDataMng.getCurrentConstraintGradient(tIndex);
            Plato::MultiVector<ScalarType, OrdinalType> &tMyAppxFunctionP = aDataMng.getConstraintAppxFunctionP(tIndex);
            Plato::MultiVector<ScalarType, OrdinalType> &tMyAppxFunctionQ = aDataMng.getConstraintAppxFunctionQ(tIndex);
            this->computeApproxFuncCoefficients(tConstraintNormalization,
                                                tCurrentControls,
                                                tLowerAsymptotes,
                                                tUpperAsymptotes,
                                                tCurrentConstraintGrad,
                                                tMyAppxFunctionP,
                                                tMyAppxFunctionQ);
        }
    }

    /******************************************************************************//**
     * @brief Update subproblem upper and lower bounds
     * @param [in] aDataMng MMA data manager interface
     **********************************************************************************/
    void updateSubProblemBounds(Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = aDataMng.getCurrentControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tLowerAsymptotes = aDataMng.getLowerAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tUpperAsymptotes = aDataMng.getUpperAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tControlLowerBounds = aDataMng.getControlLowerBounds();
        const Plato::MultiVector<ScalarType, OrdinalType> &tControlUpperBounds = aDataMng.getControlUpperBounds();

        Plato::MultiVector<ScalarType, OrdinalType> &tSubProblemLowerBounds = aDataMng.getSubProblemControlLowerBounds();
        Plato::MultiVector<ScalarType, OrdinalType> &tSubProblemUpperBounds = aDataMng.getSubProblemControlUpperBounds();

        std::vector<ScalarType> tCriteria(3/* length */);
        const OrdinalType tNumVectors = tCurrentControls.getNumVectors();
        for (OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const OrdinalType tNumControls = tCurrentControls[tVectorIndex].size();
            for (OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                tCriteria[0] = tControlLowerBounds(tVectorIndex, tControlIndex);
                tCriteria[1] = tLowerAsymptotes(tVectorIndex, tControlIndex)
                    + (mSubProblemBoundsScaling * (tCurrentControls(tVectorIndex, tControlIndex) - tLowerAsymptotes(tVectorIndex, tControlIndex)));
                tCriteria[2] = tCurrentControls(tVectorIndex, tControlIndex) - (mMoveLimit * (*mUpperMinusLowerBounds)(tVectorIndex, tControlIndex));
                tSubProblemLowerBounds(tVectorIndex, tControlIndex) = *std::max_element(tCriteria.begin(), tCriteria.end());

                tCriteria[0] = tControlUpperBounds(tVectorIndex, tControlIndex);
                tCriteria[1] = tUpperAsymptotes(tVectorIndex, tControlIndex)
                    - (mSubProblemBoundsScaling * (tUpperAsymptotes(tVectorIndex, tControlIndex) - tCurrentControls(tVectorIndex, tControlIndex)));
                tCriteria[2] = tCurrentControls(tVectorIndex, tControlIndex) + (mMoveLimit * (*mUpperMinusLowerBounds)(tVectorIndex, tControlIndex));
                tSubProblemUpperBounds(tVectorIndex, tControlIndex) = *std::min_element(tCriteria.begin(), tCriteria.end());
            }
        }
    }

private:
    /******************************************************************************//**
     * @brief Compute approximation functions
     * @param [in] aNormalization normalization factor
     * @param [in] aCurrentControls 2D container of current optimization variables
     * @param [in] aLowerAsymptotes 2D container of current lower asymptotes
     * @param [in] aUpperAsymptotes 2D container of current upper asymptotes
     * @param [in] aCriterionGrad 2D container of current criterion gradient
     * @param [in] aAppxFunctionP 2D container of the first approximation function values
     * @param [in] aAppxFunctionQ 2D container of the second approximation function values
     **********************************************************************************/
    void computeApproxFuncCoefficients(const ScalarType &aNormalization,
                                       const Plato::MultiVector<ScalarType, OrdinalType> &aCurrentControls,
                                       const Plato::MultiVector<ScalarType, OrdinalType> &aLowerAsymptotes,
                                       const Plato::MultiVector<ScalarType, OrdinalType> &aUpperAsymptotes,
                                       const Plato::MultiVector<ScalarType, OrdinalType> &aCriterionGrad,
                                       Plato::MultiVector<ScalarType, OrdinalType> &aAppxFunctionP,
                                       Plato::MultiVector<ScalarType, OrdinalType> &aAppxFunctionQ)
    {
        const ScalarType tAbsNormalization = std::abs(aNormalization);
        const OrdinalType tNumVectors = aCriterionGrad.getNumVectors();
        for (OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const OrdinalType tNumControls = aCriterionGrad[tVectorIndex].size();
            for (OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                const ScalarType tNormalizedGradValue = aCriterionGrad(tVectorIndex, tControlIndex) / tAbsNormalization;
                const ScalarType tGradValuePlus = std::max(tNormalizedGradValue, static_cast<ScalarType>(0));
                const ScalarType tGradValueMinus = std::max(-tNormalizedGradValue, static_cast<ScalarType>(0));

                aAppxFunctionP(tVectorIndex, tControlIndex) = (mApproxFuncScalingOne * tGradValuePlus) + (mApproxFuncScalingTwo * tGradValueMinus)
                    + (mApproxFuncEpsilon / (*mUpperMinusLowerBounds)(tVectorIndex, tControlIndex));
                ScalarType tUpperAsymmMinusCurrentControlSquared = aUpperAsymptotes(tVectorIndex, tControlIndex)
                    - aCurrentControls(tVectorIndex, tControlIndex);
                tUpperAsymmMinusCurrentControlSquared *= tUpperAsymmMinusCurrentControlSquared;
                aAppxFunctionP(tVectorIndex, tControlIndex) *= tUpperAsymmMinusCurrentControlSquared;

                aAppxFunctionQ(tVectorIndex, tControlIndex) = (mApproxFuncScalingTwo * tGradValuePlus) + (mApproxFuncScalingOne * tGradValueMinus)
                    + (mApproxFuncEpsilon / (*mUpperMinusLowerBounds)(tVectorIndex, tControlIndex));
                ScalarType tCurrentControlMinusLowerAsymmSquared = aCurrentControls(tVectorIndex, tControlIndex)
                    - aLowerAsymptotes(tVectorIndex, tControlIndex);
                tCurrentControlMinusLowerAsymmSquared *= tCurrentControlMinusLowerAsymmSquared;
                aAppxFunctionQ(tVectorIndex, tControlIndex) *= tCurrentControlMinusLowerAsymmSquared;
            }
        }
    }

private:
    ScalarType mMoveLimit; /*!< move limit */
    ScalarType mApproxFuncEpsilon; /*!< approximation function epsilon */
    ScalarType mAsymptoteExpansion; /*!< moving asymptotes expansion parameter */
    ScalarType mAsymptoteContraction; /*!< moving asymptotes contraction parameter */
    ScalarType mApproxFuncScalingOne; /*!< scaling factor for approximation function one */
    ScalarType mApproxFuncScalingTwo; /*!< scaling factor for approximation function two */
    ScalarType mInitialAymptoteScaling; /*!< scaling factor for initial moving asymptotes */
    ScalarType mSubProblemBoundsScaling; /*!< scaling factor for subproblem upper and lower bounds */

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperMinusLowerBounds; /*!< 2D container */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentAsymptotesMultipliers; /*!< 2D container of current moving asymptotes multipliers */

private:
    MethodMovingAsymptotesOperations(const Plato::MethodMovingAsymptotesOperations<ScalarType, OrdinalType> & aRhs);
    Plato::MethodMovingAsymptotesOperations<ScalarType, OrdinalType> & operator=(const Plato::MethodMovingAsymptotesOperations<ScalarType, OrdinalType> & aRhs);
};
// class MethodMovingAsymptotesOperations

template<typename ScalarType, typename OrdinalType = size_t>
class NullConstraint : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * Contructor
    ***********************************************************************************/
    NullConstraint()
    {
    }

    /******************************************************************************//**
     * Destructor
    ***********************************************************************************/
    virtual ~NullConstraint()
    {
    }

    /******************************************************************************//**
     * Safely cache application specific data after a new trial control is accepted.
     * For instance, the state solution, i.e. solution to Partial Differential Equation,
     * can be safely cached via the cacheData function.
    ***********************************************************************************/
    void cacheData()
    {
        return;
    }

    /******************************************************************************//**
     * Evaluate criterion function
     * @param [in] aControl: control, i.e. design, variables
     * @return criterion value
    ***********************************************************************************/
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        return (0.0);
    }

    /******************************************************************************//**
     * Evaluate criterion function gradient
     * @param [in] aControl: control, i.e. design, variables
     * @param [in/out] aOutput: function gradient
    ***********************************************************************************/
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        Plato::fill(0.0, aOutput);
    }

    /******************************************************************************//**
     * Evaluate criterion function gradient
     * @param [in] aControl: control, i.e. design, variables
     * @param [in] aVector: descent direction
     * @param [in/out] aOutput: function gradient
    ***********************************************************************************/
    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                         const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                         Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        Plato::fill(0.0, aOutput);
    }

private:
    NullConstraint(const Plato::NullConstraint<ScalarType, OrdinalType> & aRhs);
    Plato::NullConstraint<ScalarType, OrdinalType> & operator=(const Plato::NullConstraint<ScalarType, OrdinalType> & aRhs);
};
// class NullConstraint

template<typename ScalarType, typename OrdinalType = size_t>
class MethodMovingAsymptotesNew
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aObjective objective criterion interface
     * @param [in] aConstraints list of constraint criteria interfaces
     * @param [in] aDataFactory data factory used to allocate core data structure
    **********************************************************************************/
    MethodMovingAsymptotesNew(const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> &aObjective,
                              const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> &aConstraints,
                              const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory) :
        mPrintDiagnostics(false),
        mOutputStream(),
        mIterationCount(0),
        mNumObjFuncEvals(0),
        mMaxNumIterations(100),
        mProblemUpdateFrequency(0),
        mMaxNumTrustRegionIterations(50),
        mMaxNumSubProblemIterations(100),
        mInitialAugLagPenalty(1),
        mAugLagPenaltyMultiplier(1.1),
        mOptimalityTolerance(1e-6),
        mFeasibilityTolerance(1e-4),
        mControlStagnationTolerance(1e-6),
        mObjectiveStagnationTolerance(1e-8),
        mStoppingCriterion(Plato::algorithm::stop_t::NOT_CONVERGED),
        mObjective(aObjective),
        mConstraints(aConstraints),
        mMMAData(std::make_shared<Plato::ApproximationFunctionData<ScalarType, OrdinalType>>(aDataFactory)),
        mDataMng(std::make_shared<Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType>>(aDataFactory)),
        mOperations(std::make_shared<Plato::MethodMovingAsymptotesOperations<ScalarType, OrdinalType>>(aDataFactory)),
        mConstrAppxFuncList(std::make_shared<Plato::CriterionList<ScalarType, OrdinalType>>())
    {
        this->initialize(aDataFactory);
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~MethodMovingAsymptotesNew()
    {
    }

    /******************************************************************************//**
     * @brief Return optimization iteration count
     * @return optimization iteration count
    **********************************************************************************/
    OrdinalType getNumIterations() const
    {
        return mIterationCount;
    }

    /******************************************************************************//**
     * @brief Return number of objective function evaluations
     * @return number of objective function evaluations
    **********************************************************************************/
    OrdinalType getNumObjFuncEvals() const
    {
        return mNumObjFuncEvals;
    }

    /******************************************************************************//**
     * @brief Return optimal objective function value
     * @return optimal objective function value
    **********************************************************************************/
    ScalarType getOptimalObjectiveValue() const
    {
        return (mDataMng->getCurrentObjectiveValue());
    }

    /******************************************************************************//**
     * @brief Return norm of the objective function gradient
     * @return norm of the objective function gradient
    **********************************************************************************/
    ScalarType getNormObjectiveGradient() const
    {
        return (mDataMng->getNormObjectiveGradient());
    }

    /******************************************************************************//**
     * @brief Return control stagnation measure
     * @return control stagnation measure
    **********************************************************************************/
    ScalarType getControlStagnationMeasure() const
    {
        return (mDataMng->getControlStagnationMeasure());
    }

    /******************************************************************************//**
     * @brief Return objective stagnation measure
     * @return objective stagnation measure
    **********************************************************************************/
    ScalarType getObjectiveStagnationMeasure() const
    {
        return (mDataMng->getObjectiveStagnationMeasure());
    }

    /******************************************************************************//**
     * @brief Return optimal constraint value
     * @param constraint index
     * @return optimal constraint value
    **********************************************************************************/
    ScalarType getOptimalConstraintValue(const OrdinalType& aIndex) const
    {
        return (mDataMng->getCurrentConstraintValue(aIndex));
    }

    /******************************************************************************//**
     * @brief Return stopping criterion
     * @return stopping criterion
    **********************************************************************************/
    Plato::algorithm::stop_t getStoppingCriterion() const
    {
        return (mStoppingCriterion);
    }

    /******************************************************************************//**
     * @brief Print algorithm's diagnostics (i.e. optimization problem status)
     * @param [in] aInput print diagnostics if true; if false, do not print diagnostics.
    **********************************************************************************/
    void enableDiagnostics(const bool & aInput)
    {
        mPrintDiagnostics = aInput;
    }

    /******************************************************************************//**
     * @brief Enable bound constraint optimization
    **********************************************************************************/
    void enableBoundConstrainedOptimization()
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            mConstrAppxFuncs[tIndex]->setObjFuncAppxFuncMultiplier(0.0);
            mConstrAppxFuncs[tIndex]->setConstraintAppxFuncMultiplier(0.0);
        }
    }

    /******************************************************************************//**
     * @brief Set maximum number of iterations
     * @param [in] aInput maximum number of iterations
    **********************************************************************************/
    void setMaxNumIterations(const OrdinalType& aInput)
    {
        mMaxNumIterations = aInput;
    }

    /******************************************************************************//**
     * @brief Set update problem, i.e. continuation, frequency
     * @param [in] aInput update problem, i.e. continuation, frequency
    **********************************************************************************/
    void setUpdateFrequency(const OrdinalType& aInput)
    {
        mProblemUpdateFrequency = aInput;
    }

    /******************************************************************************//**
     * @brief Set maximum number of subproblem iterations
     * @param [in] aInput maximum number of subproblem iterations
    **********************************************************************************/
    void setMaxNumSubProblemIterations(const OrdinalType& aInput)
    {
        mMaxNumSubProblemIterations = aInput;
    }

    void setMaxNumTrustRegionIterations(const OrdinalType& aInput)
    {
        mMaxNumTrustRegionIterations = aInput;
    }

    /******************************************************************************//**
     * @brief Set maximum number of trust region iterations
     * @param [in] aInput maximum number of trust region iterations
    **********************************************************************************/
    void setMoveLimit(const ScalarType& aInput)
    {
        mOperations->setMoveLimit(aInput);
    }

    /******************************************************************************//**
     * @brief Set moving asymptotes expansion parameter
     * @param [in] aInput moving asymptotes expansion parameter
    **********************************************************************************/
    void setAsymptoteExpansionParameter(const ScalarType& aInput)
    {
        mOperations->setAsymptoteExpansionParameter(aInput);
    }

    /******************************************************************************//**
     * @brief Set moving asymptotes contraction parameter
     * @param [in] aInput moving asymptotes contraction parameter
    **********************************************************************************/
    void setAsymptoteContractionParameter(const ScalarType& aInput)
    {
        mOperations->setAsymptoteContractionParameter(aInput);
    }

    /******************************************************************************//**
     * @brief Set scaling on initial moving asymptotes
     * @param [in] aInput scaling on initial moving asymptotes
    **********************************************************************************/
    void setInitialAymptoteScaling(const ScalarType& aInput)
    {
        mOperations->setInitialAymptoteScaling(aInput);
    }

    /******************************************************************************//**
     * @brief Set scaling on optimization variables bounds for subproblem
     * @param [in] aInput scaling on optimization variables bounds for subproblem
    **********************************************************************************/
    void setSubProblemBoundsScaling(const ScalarType& aInput)
    {
        mOperations->setSubProblemBoundsScaling(aInput);
    }

    /******************************************************************************//**
     * @brief Set initial penalty on Augmented Lagrangian function
     * @param [in] aInput initial penalty on Augmented Lagrangian function
    **********************************************************************************/
    void setInitialAugLagPenalty(const ScalarType& aInput)
    {
        mInitialAugLagPenalty = aInput;
    }

    /******************************************************************************//**
     * @brief Set reduction parameter for augmented Lagrangian penalty
     * @param [in] aInput reduction parameter for augmented Lagrangian penalty
    **********************************************************************************/
    void setAugLagPenaltyReduction(const ScalarType& aInput)
    {
        mAugLagPenaltyMultiplier = aInput;
    }

    /******************************************************************************//**
     * @brief Set optimality tolerance
     * @param [in] aInput optimality tolerance
    **********************************************************************************/
    void setOptimalityTolerance(const ScalarType& aInput)
    {
        mOptimalityTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set feasibility tolerance
     * @param [in] aInput feasibility tolerance
    **********************************************************************************/
    void setFeasibilityTolerance(const ScalarType& aInput)
    {
        mFeasibilityTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set control stagnation tolerance
     * @param [in] aInput control stagnation tolerance
    **********************************************************************************/
    void setControlStagnationTolerance(const ScalarType& aInput)
    {
        mControlStagnationTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set objective function stagnation tolerance
     * @param [in] aInput objective function stagnation tolerance
    **********************************************************************************/
    void setObjectiveStagnationTolerance(const ScalarType& aInput)
    {
        mObjectiveStagnationTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set 2D container of lower bound on optimization variables
     * @param [in] aInput const reference 2D container of lower bound on optimization variables
    **********************************************************************************/
    void setControlLowerBounds(const Plato::MultiVector<ScalarType, OrdinalType> &aInput)
    {
        mDataMng->setControlLowerBounds(aInput);
    }

    /******************************************************************************//**
     * @brief Set 2D container of upper bound on optimization variables
     * @param [in] aInput const reference 2D container of upper bound on optimization variables
    **********************************************************************************/
    void setControlUpperBounds(const Plato::MultiVector<ScalarType, OrdinalType> &aInput)
    {
        mDataMng->setControlUpperBounds(aInput);
    }

    /******************************************************************************//**
     * @brief Set 2D container of initial optimization variables
     * @param [in] aInput 2D container of initial optimization variables
    **********************************************************************************/
    void setInitialGuess(const Plato::MultiVector<ScalarType, OrdinalType> &aInput)
    {
        mDataMng->setCurrentControls(aInput);
    }

    /******************************************************************************//**
     * @brief Set constraint normalization values
     * @param [in] aIndex constraint index
     * @param [in] aInput constraint normalization values
    **********************************************************************************/
    void setConstraintNormalization(const OrdinalType & aIndex, const ScalarType & aValue)
    {
        mDataMng->setConstraintNormalization(aIndex, aValue);
    }

    /******************************************************************************//**
     * @brief Set constraint normalization parameters
     * @param [in] aInput reference to 1D container of constraint normalization parameters
    **********************************************************************************/
    void setConstraintNormalizationParams(Plato::Vector<ScalarType, OrdinalType>& aInput)
    {
        mDataMng->setConstraintNormalizationParams(aInput);
    }

    /******************************************************************************//**
     * @brief Gather 2D container of optimal optimization variables
     * @param [out] aInput reference to 2D container of optimal optimization variables
    **********************************************************************************/
    void getSolution(Plato::MultiVector<ScalarType, OrdinalType>& aInput) const
    {
        Plato::update(static_cast<ScalarType>(1.0), mDataMng->getCurrentControls(), static_cast<ScalarType>(0.0), aInput);
    }

    /******************************************************************************//**
     * @brief Gather 1D container of optimal constraint values
     * @param [out] aInput reference to 1D container of optimal constraint values
    **********************************************************************************/
    void getOptimalConstraintValues(Plato::Vector<ScalarType, OrdinalType>& aInput) const
    {
        mDataMng->getCurrentConstraintValues(aInput);
    }

    /******************************************************************************//**
     * @brief Solve problem using the Method of Moving Asymptotes (MMA) algorithm.
    **********************************************************************************/
    void solve()
    {
        this->openOutputFile();
        mOperations->initialize(*mDataMng);
        while(true)
        {
            this->updateState();
            this->updateSubProblem();
            this->solveSubProblem();
            mIterationCount++;

            bool tStop = this->checkStoppingCriteria();
            if(tStop == true)
            {
                this->updateState();
                this->printStoppingCriterion();
                this->closeOutputFile();
                break;
            }
        }
    }

private:
    /******************************************************************************//**
     * @brief Initialize core internal operations and data structures.
    **********************************************************************************/
    void initialize(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory)
    {
        this->initializeApproximationFunctions(aDataFactory);
        this->initializeSubProblemSolver(aDataFactory);
    }

    /******************************************************************************//**
     * @brief Initialize solver for Method of Moving Asymptotes (MMA) subproblem.
    **********************************************************************************/
    void initializeSubProblemSolver(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory)
    {
        mSubProblemSolver = std::make_shared<Plato::AugmentedLagrangian<ScalarType, OrdinalType>>(mObjAppxFunc, mConstrAppxFuncList, aDataFactory);
        mSubProblemSolver->disablePostSmoothing();
        mSubProblemSolver->setPenaltyParameter(mInitialAugLagPenalty);
        mSubProblemSolver->setFeasibilityTolerance(mFeasibilityTolerance);
        mSubProblemSolver->setControlStagnationTolerance(mControlStagnationTolerance);
        mSubProblemSolver->setMaxNumOuterIterations(mMaxNumSubProblemIterations);
        mSubProblemSolver->setPenaltyParameterScaleFactor(mAugLagPenaltyMultiplier);
        mSubProblemSolver->setMaxNumTrustRegionSubProblemIterations(mMaxNumTrustRegionIterations);
    }

    /******************************************************************************//**
     * @brief Initialize Method of Moving Asymptotes (MMA) approximation functions.
    **********************************************************************************/
    void initializeApproximationFunctions(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory)
    {
        mObjAppxFunc = std::make_shared<Plato::MethodMovingAsymptotesNewCriterion<ScalarType, OrdinalType>>(aDataFactory);
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            mConstrAppxFuncs.push_back(std::make_shared<Plato::MethodMovingAsymptotesNewCriterion<ScalarType, OrdinalType>>(aDataFactory));
            mConstrAppxFuncList->add(mConstrAppxFuncs[tIndex]);
        }
    }

    /******************************************************************************//**
     * @brief Open output file (i.e. diagnostics file)
    **********************************************************************************/
    void openOutputFile()
    {
        if (mPrintDiagnostics == false)
        {
            return;
        }

        const Plato::CommWrapper &tMyCommWrapper = mDataMng->getCommWrapper();
        if (tMyCommWrapper.myProcID() == 0)
        {
            const OrdinalType tNumConstraints = mDataMng->getNumConstraints();
            mOutputData.mConstraints.clear();
            mOutputData.mConstraints.resize(tNumConstraints);
            mOutputStream.open("plato_mma_algorithm_diagnostics.txt");
            Plato::print_mma_diagnostics_header(mOutputData, mOutputStream);
        }
    }

    /******************************************************************************//**
     * @brief Close output file (i.e. diagnostics file)
    **********************************************************************************/
    void closeOutputFile()
    {
        if (mPrintDiagnostics == false)
        {
            return;
        }

        const Plato::CommWrapper &tMyCommWrapper = mDataMng->getCommWrapper();
        if (tMyCommWrapper.myProcID() == 0)
        {
            mOutputStream.close();
        }
    }

    /******************************************************************************//**
     * @brief Print MMA diagnostics to file
    **********************************************************************************/
    void printDiagnostics()
    {
        if(mPrintDiagnostics == false)
        {
            return;
        }

        const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
        if(tMyCommWrapper.myProcID() == 0)
        {
            mOutputData.mNumIter = mIterationCount;
            mOutputData.mObjFuncCount = mNumObjFuncEvals;
            mOutputData.mObjFuncValue = mDataMng->getCurrentObjectiveValue();
            mOutputData.mNormObjFuncGrad = mDataMng->getNormObjectiveGradient();
            mOutputData.mControlStagnationMeasure = mDataMng->getControlStagnationMeasure();
            mOutputData.mObjectiveStagnationMeasure = mDataMng->getObjectiveStagnationMeasure();

            const OrdinalType tNumConstraints = mDataMng->getNumConstraints();
            for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
            {
                mOutputData.mConstraints[tConstraintIndex] = mDataMng->getCurrentConstraintValue(tConstraintIndex);
            }

            Plato::print_mma_diagnostics(mOutputData, mOutputStream);
        }
    }

    /******************************************************************************//**
     * @brief Print stopping criterion to diagnostics file.
    **********************************************************************************/
    void printStoppingCriterion()
    {
        if(mPrintDiagnostics == true)
        {
            const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
            if(tMyCommWrapper.myProcID() == 0)
            {
                std::string tReason;
                Plato::print_mma_stop_criterion(mStoppingCriterion, tReason);
                mOutputStream << tReason.c_str();
            }
        }
    }

    /******************************************************************************//**
     * @brief Evaluate objective criterion and its gradient.
    **********************************************************************************/
    void evaluateObjective()
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = mDataMng->getCurrentControls();
        ScalarType tObjFuncValue = mObjective->value(tCurrentControls);
        mNumObjFuncEvals++;
        mObjective->cacheData();
        mDataMng->setCurrentObjectiveValue(tObjFuncValue);
        Plato::MultiVector<ScalarType, OrdinalType> &tCurrentObjGrad = mDataMng->getCurrentObjectiveGradient();
        mObjective->gradient(tCurrentControls, tCurrentObjGrad);
    }

    /******************************************************************************//**
     * @brief Evaluate constraint criteria and respective gradients.
    **********************************************************************************/
    void evaluateConstraints()
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = mDataMng->getCurrentControls();
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            const ScalarType tValue = (*mConstraints)[tIndex].value(tCurrentControls);
            (*mConstraints)[tIndex].cacheData();
            mDataMng->setCurrentConstraintValue(tIndex, tValue);
            Plato::MultiVector<ScalarType, OrdinalType> &tCurrentConstraintGrad = mDataMng->getCurrentConstraintGradient(tIndex);
            (*mConstraints)[tIndex].gradient(tCurrentControls, tCurrentConstraintGrad);
        }
    }

    /******************************************************************************//**
     * @brief Update Method of Moving Asymptotes (MMA) subproblem.
    **********************************************************************************/
    void updateSubProblem()
    {
        this->updateAsymptotes();
        mOperations->updateSubProblemBounds(*mDataMng);
        mOperations->updateObjectiveApproximationFunctionData(*mDataMng);
        mOperations->updateConstraintApproximationFunctionsData(*mDataMng);
        this->updateObjectiveApproximationFunction();
        this->updateConstraintApproximationFunctions();
    }

    /******************************************************************************//**
     * @brief Update current state and stopping measures
    **********************************************************************************/
    void updateState()
    {
        this->evaluateObjective();
        this->evaluateConstraints();
        this->performContinuation();
        mDataMng->computeStoppingMeasures();
        mDataMng->cacheState();
        this->printDiagnostics();
    }

    /******************************************************************************//**
     * @brief If active, safely allow application to perform continuation on its data.
    **********************************************************************************/
    void performContinuation()
    {
        const bool tIsContinuationEnabled = mProblemUpdateFrequency > static_cast<OrdinalType>(0);
        bool tPerformContinuation = tIsContinuationEnabled ? (mIterationCount % mProblemUpdateFrequency) == static_cast<OrdinalType>(0) : false;
        if (tPerformContinuation)
        {
            mObjective->updateProblem(mDataMng->getCurrentControls());
            const OrdinalType tNumConstraints = mConstraints->size();
            for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
            {
                (*mConstraints)[tIndex].updateProblem(mDataMng->getCurrentControls());
            }
        }
    }

    /******************************************************************************//**
     * @brief Update moving asymptotes.
    **********************************************************************************/
    void updateAsymptotes()
    {
        if(mIterationCount >= static_cast<OrdinalType>(2))
        {
            mOperations->updateCurrentAsymptotesMultipliers(*mDataMng);
            mOperations->updateCurrentAsymptotes(*mDataMng);
        }
        else
        {
            mOperations->updateInitialAsymptotes(*mDataMng);
        }
    }

    /******************************************************************************//**
     * @brief Update approximation functions associated with the objective criterion.
    **********************************************************************************/
    void updateObjectiveApproximationFunction()
    {
        mMMAData->mCurrentNormalizedCriterionValue = 1.0;
        Plato::update(static_cast<ScalarType>(1), mDataMng->getCurrentControls(), static_cast<ScalarType>(0), *mMMAData->mCurrentControls);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getLowerAsymptotes(), static_cast<ScalarType>(0), *mMMAData->mLowerAsymptotes);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getUpperAsymptotes(), static_cast<ScalarType>(0), *mMMAData->mUpperAsymptotes);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getObjFuncAppxFunctionP(), static_cast<ScalarType>(0), *mMMAData->mAppxFunctionP);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getObjFuncAppxFunctionQ(), static_cast<ScalarType>(0), *mMMAData->mAppxFunctionQ);

        mObjAppxFunc->update(*mMMAData);
    }

    /******************************************************************************//**
     * @brief Update approximation functions associated with the constraint criteria
    **********************************************************************************/
    void updateConstraintApproximationFunctions()
    {
        Plato::update(static_cast<ScalarType>(1), mDataMng->getCurrentControls(), static_cast<ScalarType>(0), *mMMAData->mCurrentControls);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getLowerAsymptotes(), static_cast<ScalarType>(0), *mMMAData->mLowerAsymptotes);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getUpperAsymptotes(), static_cast<ScalarType>(0), *mMMAData->mUpperAsymptotes);

        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            const ScalarType tConstraintValue = mDataMng->getCurrentConstraintValue(tIndex);
            const ScalarType tConstraintNormalization = mDataMng->getConstraintNormalization(tIndex);
            mMMAData->mCurrentNormalizedCriterionValue = tConstraintValue / tConstraintNormalization;
            Plato::update(static_cast<ScalarType>(1), mDataMng->getConstraintAppxFunctionP(tIndex), static_cast<ScalarType>(0), *mMMAData->mAppxFunctionP);
            Plato::update(static_cast<ScalarType>(1), mDataMng->getConstraintAppxFunctionQ(tIndex), static_cast<ScalarType>(0), *mMMAData->mAppxFunctionQ);
            mConstrAppxFuncs[tIndex]->update(*mMMAData);
        }
    }

    /******************************************************************************//**
     * @brief Solve Method of Moving Asymptotes (MMA) subproblem
    **********************************************************************************/
    void solveSubProblem()
    {
        mSubProblemSolver->resetParameters();
        mSubProblemSolver->setInitialGuess(mDataMng->getCurrentControls());
        mSubProblemSolver->setControlLowerBounds(mDataMng->getSubProblemControlLowerBounds());
        mSubProblemSolver->setControlUpperBounds(mDataMng->getSubProblemControlUpperBounds());
        mSubProblemSolver->solve();
        mSubProblemSolver->getSolution(mDataMng->getCurrentControls());
    }

    /******************************************************************************//**
     * @brief Check stopping criteria
     * @return stopping criterion met, yes or no
    **********************************************************************************/
    bool checkStoppingCriteria()
    {
        bool tStop = false;
        const ScalarType tFeasibilityMeasure = mDataMng->getFeasibilityMeasure();
        const ScalarType tOptimalityMeasure = mDataMng->getNormObjectiveGradient();
        const ScalarType tControlStagnation = mDataMng->getControlStagnationMeasure();
        const ScalarType tObjectiveStagnation = mDataMng->getObjectiveStagnationMeasure();

        if(mIterationCount == mMaxNumIterations)
        {
            mStoppingCriterion = Plato::algorithm::MAX_NUMBER_ITERATIONS;
            tStop = true;
        }
        else if(tControlStagnation <= mControlStagnationTolerance)
        {
            mStoppingCriterion = Plato::algorithm::CONTROL_STAGNATION;
            tStop = true;
        }
        else if(tObjectiveStagnation <= mObjectiveStagnationTolerance)
        {
            mStoppingCriterion = Plato::algorithm::OBJECTIVE_STAGNATION;
            tStop = true;
        }
        else if(tOptimalityMeasure <= mOptimalityTolerance && tFeasibilityMeasure <= mFeasibilityTolerance)
        {
            mStoppingCriterion = Plato::algorithm::OPTIMALITY_AND_FEASIBILITY;
            tStop = true;
        }

        return (tStop);
    }

private:
    bool mPrintDiagnostics; /*!< flag to enable output (default=false) */
    std::ofstream mOutputStream; /*!< output string stream with diagnostics */

    OrdinalType mIterationCount; /*!< number of optimization iterations */
    OrdinalType mNumObjFuncEvals; /*!< number of objective function evaluations */
    OrdinalType mMaxNumIterations; /*!< maximum number of optimization iterations */
    OrdinalType mProblemUpdateFrequency; /*!< problem update, i.e. continuation, frequency */
    OrdinalType mMaxNumTrustRegionIterations; /*!< maximum number of trust region subproblem iterations */
    OrdinalType mMaxNumSubProblemIterations; /*!< maximum number of MMA subproblem iterations iterations */

    ScalarType mInitialAugLagPenalty; /*!< initial augmented Lagragian penalty parameter */
    ScalarType mAugLagPenaltyMultiplier; /*!< augmented Lagragian penalty reduction parameter */
    ScalarType mOptimalityTolerance; /*!< tolerance on the norm of the objective gradient - primary stopping tolerance */
    ScalarType mFeasibilityTolerance; /*!< feasibility tolerance - primary stopping tolerance */
    ScalarType mControlStagnationTolerance; /*!< control stagnation tolerance - secondary stopping tolerance */
    ScalarType mObjectiveStagnationTolerance; /*!< objective stagnation tolerance - secondary stopping tolerance */
    Plato::algorithm::stop_t mStoppingCriterion; /*!< stopping criterion */

    Plato::OutputDataMMA<ScalarType, OrdinalType> mOutputData; /*!< output data structure */

    std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> mObjective; /*!< objective criterion interface */
    std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> mConstraints; /*!< constraint criteria interface */

    std::shared_ptr<Plato::ApproximationFunctionData<ScalarType, OrdinalType>> mMMAData; /*!< structure with approximation function's data */
    std::shared_ptr<Plato::AugmentedLagrangian<ScalarType, OrdinalType>> mSubProblemSolver; /*!< MMA subproblem solver */
    std::shared_ptr<Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType>> mDataMng; /*!< MMA data manager */
    std::shared_ptr<Plato::MethodMovingAsymptotesOperations<ScalarType, OrdinalType>> mOperations; /*!< interface to MMA core operations */

    std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> mConstrAppxFuncList; /*!< list of constraint criteria */
    std::shared_ptr<Plato::MethodMovingAsymptotesNewCriterion<ScalarType, OrdinalType>> mObjAppxFunc; /*!< objective criterion approximation function */
    std::vector<std::shared_ptr<Plato::MethodMovingAsymptotesNewCriterion<ScalarType, OrdinalType>>> mConstrAppxFuncs; /*!< list of constraint criteria approximation function */

private:
    MethodMovingAsymptotesNew(const Plato::MethodMovingAsymptotesNew<ScalarType, OrdinalType> & aRhs);
    Plato::MethodMovingAsymptotesNew<ScalarType, OrdinalType> & operator=(const Plato::MethodMovingAsymptotesNew<ScalarType, OrdinalType> & aRhs);
};
// class MethodMovingAsymptotesNew


/******************************************************************************//**
 * @brief Output data structure for the Method of Moving Asymptotes (MMA) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct AlgorithmOutputsMMA
{
    OrdinalType mNumSolverIter; /*!< number of outer iterations */
    OrdinalType mNumObjFuncEval; /*!< number of objective function evaluations */

    ScalarType mObjFuncValue; /*!< objective function value */
    ScalarType mNormObjFuncGrad; /*!< norm of the objective function gradient */
    ScalarType mControlStagnationMeasure; /*!< norm of the difference between two subsequent control fields */
    ScalarType mObjectiveStagnationMeasure; /*!< measures stagnation in two subsequent objective function evaluations */

    std::string mStopCriterion; /*!< stopping criterion */

    std::shared_ptr<Plato::Vector<ScalarType,OrdinalType>> mConstraints; /*!< constraint values */
    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mSolution; /*!< optimal solution */
};
// struct AlgorithmOutputsMMA

/******************************************************************************//**
 * @brief Input data structure for the Method of Moving Asymptotes (MMA) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct AlgorithmInputsMMA
{
    /******************************************************************************//**
     * @brief Default constructor
    **********************************************************************************/
    AlgorithmInputsMMA() :
            mPrintDiagnostics(false),
            mUpdateFrequency(0),
            mMaxNumSolverIter(500),
            mMaxNumSubProblemIter(100),
            mMaxNumTrustRegionIter(50),
            mMoveLimit(0.5),
            mInitialAugLagPenalty(1),
            mAsymptoteExpansion(1.2),
            mAsymptoteContraction(0.7),
            mInitialAymptoteScaling(0.5),
            mSubProblemBoundsScaling(0.1),
            mOptimalityTolerance(1e-6),
            mFeasibilityTolerance(1e-4),
            mControlStagnationTolerance(1e-6),
            mObjectiveStagnationTolerance(1e-8),
            mCommWrapper(),
            mMemorySpace(Plato::MemorySpace::HOST),
            mLowerBounds(nullptr),
            mUpperBounds(nullptr),
            mInitialGuess(nullptr),
            mConstraintNormalizationParams(nullptr),
            mDualReductionOperations(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>()),
            mControlReductionOperations(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>())
    {
        mCommWrapper.useDefaultComm();
    }

    /******************************************************************************//**
     * @brief Default destructor
    **********************************************************************************/
    virtual ~AlgorithmInputsMMA()
    {
    }

    bool mPrintDiagnostics; /*!< flag to enable problem statistics output (default=false) */

    OrdinalType mUpdateFrequency; /*!< continuation frequency (default = disabled = 0) */
    OrdinalType mMaxNumSolverIter; /*!< maximum number of outer iterations */
    OrdinalType mMaxNumSubProblemIter; /*!< maximum number of trust region sub problem iterations */
    OrdinalType mMaxNumTrustRegionIter; /*!< maximum number of trust region iterations */

    ScalarType mMoveLimit; /*!< move limit */
    ScalarType mInitialAugLagPenalty; /*!< initial penalty on augmented Lagrangian function */
    ScalarType mAsymptoteExpansion; /*!< moving asymptotes expansion factor */
    ScalarType mAsymptoteContraction; /*!< moving asymptotes' contraction factor */
    ScalarType mInitialAymptoteScaling; /*!< initial moving asymptotes' scale factor */
    ScalarType mSubProblemBoundsScaling; /*!< scaling on subproblem upper and lower bounds */

    ScalarType mOptimalityTolerance; /*!< optimality tolerance */
    ScalarType mFeasibilityTolerance; /*!< feasibility tolerance */
    ScalarType mControlStagnationTolerance; /*!< control stagnation tolerance */
    ScalarType mObjectiveStagnationTolerance; /*!< objective function stagnation tolerance */

    Plato::CommWrapper mCommWrapper; /*!< distributed memory communication wrapper */
    Plato::MemorySpace::type_t mMemorySpace; /*!< memory space: HOST (default) OR DEVICE */

    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mLowerBounds; /*!< lower bounds */
    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mUpperBounds; /*!< upper bounds */
    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mInitialGuess; /*!< initial guess */
    std::shared_ptr<Plato::Vector<ScalarType,OrdinalType>> mConstraintNormalizationParams; /*!< constraint normalization parameters */

    /*!< operations which require communication across processors, e.g. max, min, global sum */
    std::shared_ptr<Plato::ReductionOperations<ScalarType,OrdinalType>> mDualReductionOperations;
    std::shared_ptr<Plato::ReductionOperations<ScalarType,OrdinalType>> mControlReductionOperations;
};
// struct AlgorithmInputsMMA

/******************************************************************************//**
 * @brief Set Method of Moving Asymptotes (MMA) algorithm inputs
 * @param [in] aInputs Method of Moving Asymptotes algorithm inputs
 * @param [in,out] aAlgorithm Method of Moving Asymptotes algorithm interface
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void set_mma_algorithm_inputs(const Plato::AlgorithmInputsMMA<ScalarType, OrdinalType> & aInputs,
                                     Plato::MethodMovingAsymptotesNew<ScalarType, OrdinalType> & aAlgorithm)
{
    if(aInputs.mPrintDiagnostics == true)
    {
        aAlgorithm.enableDiagnostics(aInputs.mPrintDiagnostics);
    }

    aAlgorithm.setInitialGuess(*aInputs.mInitialGuess);
    aAlgorithm.setControlLowerBounds(*aInputs.mLowerBounds);
    aAlgorithm.setControlUpperBounds(*aInputs.mUpperBounds);
    aAlgorithm.setConstraintNormalizationParams(*aInputs.mConstraintNormalizationParams);

    aAlgorithm.setUpdateFrequency(aInputs.mUpdateFrequency);
    aAlgorithm.setMaxNumIterations(aInputs.mMaxNumSolverIter);
    aAlgorithm.setMaxNumSubProblemIterations(aInputs.mMaxNumSubProblemIter);
    aAlgorithm.setMaxNumTrustRegionIterations(aInputs.mMaxNumTrustRegionIter);

    aAlgorithm.setMoveLimit(aInputs.mMoveLimit);
    aAlgorithm.setInitialAugLagPenalty(aInputs.mInitialAugLagPenalty);
    aAlgorithm.setInitialAymptoteScaling(aInputs.mInitialAymptoteScaling);
    aAlgorithm.setAsymptoteExpansionParameter(aInputs.mAsymptoteExpansion);
    aAlgorithm.setSubProblemBoundsScaling(aInputs.mSubProblemBoundsScaling);
    aAlgorithm.setAsymptoteContractionParameter(aInputs.mAsymptoteContraction);

    aAlgorithm.setOptimalityTolerance(aInputs.mOptimalityTolerance);
    aAlgorithm.setFeasibilityTolerance(aInputs.mFeasibilityTolerance);
    aAlgorithm.setControlStagnationTolerance(aInputs.mControlStagnationTolerance);
    aAlgorithm.setObjectiveStagnationTolerance(aInputs.mObjectiveStagnationTolerance);
}
// function set_mma_algorithm_inputs

/******************************************************************************//**
 * @brief Set Method of Moving Asymptotes (MMA) algorithm outputs
 * @param [in] aAlgorithm Method of Moving Asymptotes algorithm interface
 * @param [in,out] aOutputs Method of Moving Asymptotes algorithm outputs
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void set_mma_algorithm_outputs(const Plato::MethodMovingAsymptotesNew<ScalarType, OrdinalType> & aAlgorithm,
                                      Plato::AlgorithmOutputsMMA<ScalarType, OrdinalType> & aOutputs)
{
    aAlgorithm.getSolution(*aOutputs.mSolution);
    aAlgorithm.getOptimalConstraintValues(*aOutputs.mConstraints);

    aOutputs.mNumSolverIter = aAlgorithm.getNumIterations();
    aOutputs.mNumObjFuncEval = aAlgorithm.getNumObjFuncEvals();

    aOutputs.mObjFuncValue = aAlgorithm.getOptimalObjectiveValue();
    aOutputs.mNormObjFuncGrad = aAlgorithm.getNormObjectiveGradient();
    aOutputs.mControlStagnationMeasure = aAlgorithm.getControlStagnationMeasure();
    aOutputs.mObjectiveStagnationMeasure = aAlgorithm.getObjectiveStagnationMeasure();

    Plato::print_mma_stop_criterion(aAlgorithm.getStoppingCriterion(), aOutputs.mStopCriterion);
}
// function set_mma_algorithm_outputs

/******************************************************************************//**
 * @brief Method of Moving Asymptotes (MMA) algorithm interface
 * @param [in] aObjective user-defined objective function
 * @param [in] aConstraints user-defined list of constraints
 * @param [in] aInputs Method of Moving Asymptotes algorithm inputs
 * @param [in,out] aOutputs Method of Moving Asymptotes algorithm outputs
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void solve_mma(const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & aObjective,
                      const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> & aConstraints,
                      const Plato::AlgorithmInputsMMA<ScalarType, OrdinalType> & aInputs,
                      Plato::AlgorithmOutputsMMA<ScalarType, OrdinalType> & aOutputs)
{
    // ********* ALLOCATE DATA STRUCTURES *********
    std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> tDataFactory;
    tDataFactory = std::make_shared<Plato::DataFactory<ScalarType, OrdinalType>>();
    tDataFactory->setCommWrapper(aInputs.mCommWrapper);
    tDataFactory->allocateControl(*aInputs.mInitialGuess);
    tDataFactory->allocateDual(*aInputs.mConstraintNormalizationParams);
    tDataFactory->allocateDualReductionOperations(*aInputs.mDualReductionOperations);
    tDataFactory->allocateControlReductionOperations(*aInputs.mControlReductionOperations);

    // ********* ALLOCATE OPTIMALITY CRITERIA ALGORITHM AND SOLVE OPTIMIZATION PROBLEM *********
    Plato::MethodMovingAsymptotesNew<ScalarType, OrdinalType> tAlgorithm(aObjective, aConstraints, tDataFactory);
    Plato::set_mma_algorithm_inputs(aInputs, tAlgorithm);
    tAlgorithm.solve();

    aOutputs.mSolution = tDataFactory->control().create();
    aOutputs.mConstraints = tDataFactory->dual(0 /* vector index */).create();
    Plato::set_mma_algorithm_outputs(tAlgorithm, aOutputs);
}
// function solve_mma

}
// namespace Plato

namespace PlatoTest
{

std::vector<double> get_topology_optimization_gold()
{
    std::vector<double> tData = { 1, 1, 0.09963763906, 0.01, 0.01, 0.01, 0.02981858214, 0.122176848, 1, 1, 1, 1, 0.2407984809, 0.0159862795, 0.01, 0.01,
        0.02993454668, 0.1204176079, 0.9984638161, 1, 1, 0.8792774721, 1, 0.1527013574, 0.01, 0.01, 0.03110600992, 0.116003526, 0.99789355, 1, 1, 0.3621251213,
        0.869480942, 0.9222532171, 0.1059821443, 0.01, 0.02726839263, 0.1093306959, 1, 1, 1, 0.229465817, 0.177670247, 0.9608359435, 0.7573956306,
        0.07856082129, 0.01613082746, 0.1038487414, 1, 0.9987520709, 1, 0.213310992, 0.09880097021, 0.4230699414, 0.8618869651, 0.6385901013, 0.05527002214,
        0.09962157642, 0.9991862169, 0.9873343721, 0.9936766576, 0.21332468, 0.0888907914, 0.06665360106, 0.6708894614, 0.8500175685, 0.4072074949,
        0.1368029677, 1, 1, 1, 0.2086912518, 0.07991180519, 0.01759975628, 0.1035920092, 0.7906481264, 0.9033673742, 0.3404269023, 0.9982450947, 1,
        0.9954508096, 0.2127764606, 0.07411368624, 0.01, 0.01, 0.1524722234, 0.9224332142, 0.8073510223, 0.9127129638, 1, 1, 0.2146435157, 0.06006097549, 0.01,
        0.01, 0.01295802044, 0.2550278407, 0.965270516, 0.905787634, 0.9939075568, 0.9945545856, 0.2230887416, 0.05064233306, 0.01, 0.01, 0.01, 0.01774828622,
        0.3203253654, 0.9965550133, 0.9934764039, 0.9981459117, 0.2293810086, 0.04568875546, 0.01634284154, 0.01, 0.01, 0.01044056529, 0.2528440393, 1, 1,
        0.99727598, 0.2413115806, 0.03164221735, 0.01262831739, 0.01841103037, 0.01, 0.1774915334, 0.9692749415, 0.7129325521, 0.8632292503, 1, 0.2580872933,
        0.02539650312, 0.01986806615, 0.01006649374, 0.1385087242, 0.8632392614, 0.8557574523, 0.1701971433, 0.8751502435, 0.9966404439, 0.2888995111,
        0.0168106666, 0.01875154327, 0.112405472, 0.7739732449, 0.893669012, 0.1467913411, 0.07236252735, 0.8757534631, 0.9992574255, 0.3394110602,
        0.01372413809, 0.09592103627, 0.7157707903, 0.815507893, 0.369117658, 0.01567082462, 0.06453770917, 0.8747390771, 1, 0.416004086, 0.1156031579,
        0.6132942877, 0.813874354, 0.5883956644, 0.04770494721, 0.01592927583, 0.06346413713, 0.873894403, 0.9945048142, 0.4007816515, 0.520010458,
        0.8460529553, 0.7240971586, 0.07553824292, 0.01, 0.01, 0.06352301774, 0.873194224, 0.9934189319, 0.5218459518, 0.750271282, 0.8378993636, 0.1098522469,
        0.01257494755, 0.01, 0.01, 0.06361849477, 0.8725756646, 0.9916137817, 0.7458641095, 0.8546970895, 0.1701835009, 0.01735174789, 0.01, 0.01, 0.01,
        0.06371955664, 0.8719892236, 0.9735136467, 0.8210040525, 0.4429160497, 0.02614668387, 0.01338563026, 0.01, 0.01, 0.01, 0.06386002518, 0.8713635522,
        0.8493975383, 0.8719407676, 0.19617933, 0.01490080135, 0.01, 0.01, 0.01, 0.01168024438, 0.06407551131, 0.8706381416, 0.1372458288, 0.881936745,
        0.8051764521, 0.1475906557, 0.01, 0.01317893402, 0.01, 0.01100125746, 0.06447658074, 0.8694408448, 0.01160002611, 0.1483743016, 0.8454816649,
        0.8001792584, 0.1476388708, 0.01, 0.01, 0.01, 0.06523647754, 0.8674728199, 0.01, 0.01262313208, 0.1633494583, 0.8378953242, 0.7977748154, 0.1468819449,
        0.01, 0.01611682195, 0.06690406329, 0.8640400233, 0.01, 0.01, 0.01469643042, 0.167958588, 0.8368900078, 0.7937936722, 0.1471738997, 0.01, 0.07009093675,
        0.8606060962, 0.01, 0.01, 0.01, 0.01002830186, 0.1701030503, 0.8393064961, 0.7847495731, 0.1444929369, 0.07748326246, 0.8597340432, 0.01, 0.01,
        0.02057088789, 0.01, 0.0123472956, 0.1664332417, 0.8605738334, 0.7747448051, 0.2566614261, 0.8579196479, 0.01, 0.01, 0.01, 0.01454491462, 0.01, 0.01,
        0.1540314408, 0.922222503, 0.7825272514, 0.7965536156, 0.01444850231, 0.01, 0.01, 0.01, 0.01, 0.01016863972, 0.01175750541, 0.1550800289, 0.9349465177,
        1 };
    return tData;
}

TEST(PlatoTest, MethodMovingAsymptotes_PrintDiagnosticsOneConstraints)
{

    Plato::CommWrapper tComm(MPI_COMM_WORLD);
    if (tComm.myProcID() == static_cast<int>(0))
    {
        std::ofstream tWriteFile;
        tWriteFile.open("MyFile1.txt");
        Plato::OutputDataMMA<double> tData;
        tData.mNumIter = 0;
        tData.mObjFuncCount = 1;
        tData.mObjFuncValue = 1.0;
        tData.mNormObjFuncGrad = 4.5656e-3;
        tData.mControlStagnationMeasure = 1.2345678e6;
        tData.mObjectiveStagnationMeasure = std::numeric_limits<double>::max();
        const size_t tNumConstraints = 1;
        tData.mConstraints.resize(tNumConstraints);
        tData.mConstraints[0] = 1.23e-5;

        ASSERT_NO_THROW(Plato::print_mma_diagnostics_header(tData, tWriteFile));
        ASSERT_NO_THROW(Plato::print_mma_diagnostics(tData, tWriteFile));
        tData.mNumIter = 1;
        tData.mObjFuncCount = 3;
        tData.mObjFuncValue = 0.298736;
        tData.mNormObjFuncGrad = 3.45656e-1;
        tData.mControlStagnationMeasure = 0.18743;
        tData.mObjectiveStagnationMeasure = 0.7109;
        tData.mConstraints[0] = 8.23e-2;
        ASSERT_NO_THROW(Plato::print_mma_diagnostics(tData, tWriteFile));
        tWriteFile.close();
        std::ifstream tReadFile;
        tReadFile.open("MyFile1.txt");
        std::string tInputString;
        std::stringstream tReadData;
        while (tReadFile >> tInputString)
        {
            tReadData << tInputString.c_str();
        }
        tReadFile.close();
        std::system("rm -f MyFile1.txt");

        std::stringstream tGold;
        tGold << "IterF-countF(X)Norm(F')H1(X)abs(dX)abs(dF)";
        tGold << "011.000000e+004.565600e-031.230000e-051.234568e+061.797693e+308";
        tGold << "132.987360e-013.456560e-018.230000e-021.874300e-017.109000e-01";
        ASSERT_STREQ(tReadData.str().c_str(), tGold.str().c_str());
    }
}

TEST(PlatoTest, MethodMovingAsymptotes_PrintDiagnosticsTwoConstraints)
{
    Plato::CommWrapper tComm(MPI_COMM_WORLD);
    if (tComm.myProcID() == static_cast<int>(0))
    {
        std::ofstream tWriteFile;
        tWriteFile.open("MyFile1.txt");
        Plato::OutputDataMMA<double> tData;
        tData.mNumIter = 0;
        tData.mObjFuncCount = 1;
        tData.mObjFuncValue = 1.0;
        tData.mNormObjFuncGrad = 4.5656e-3;
        tData.mControlStagnationMeasure = 1.2345678e6;
        tData.mObjectiveStagnationMeasure = std::numeric_limits<double>::max();
        const size_t tNumConstraints = 2;
        tData.mConstraints.resize(tNumConstraints);
        tData.mConstraints[0] = 1.23e-5;
        tData.mConstraints[1] = 3.33e-3;

        ASSERT_NO_THROW(Plato::print_mma_diagnostics_header(tData, tWriteFile));
        ASSERT_NO_THROW(Plato::print_mma_diagnostics(tData, tWriteFile));
        tData.mNumIter = 1;
        tData.mObjFuncCount = 3;
        tData.mObjFuncValue = 0.298736;
        tData.mNormObjFuncGrad = 3.45656e-1;
        tData.mControlStagnationMeasure = 0.18743;
        tData.mObjectiveStagnationMeasure = 0.7109;
        tData.mConstraints[0] = 8.23e-2;
        tData.mConstraints[1] = 8.33e-5;
        ASSERT_NO_THROW(Plato::print_mma_diagnostics(tData, tWriteFile));
        tWriteFile.close();

         std::ifstream tReadFile;
         tReadFile.open("MyFile1.txt");
         std::string tInputString;
         std::stringstream tReadData;
         while(tReadFile >> tInputString)
         {
         tReadData << tInputString.c_str();
         }
         tReadFile.close();
         std::system("rm -f MyFile1.txt");

         std::stringstream tGold;
         tGold << "IterF-countF(X)Norm(F')H1(X)H2(X)abs(dX)abs(dF)";
         tGold << "011.000000e+004.565600e-031.230000e-053.330000e-031.234568e+061.797693e+308";
         tGold << "132.987360e-013.456560e-018.230000e-028.330000e-051.874300e-017.109000e-01";
         ASSERT_STREQ(tReadData.str().c_str(), tGold.str().c_str());
    }
}

TEST(PlatoTest, MethodMovingAsymptotes_PrintStoppingCriterion)
{
    std::string tDescription;
    Plato::algorithm::stop_t tFlag = Plato::algorithm::NOT_CONVERGED;
    Plato::print_mma_stop_criterion(tFlag, tDescription);
    std::string tGold("\n\n****** Optimization algorithm did not converge. ******\n\n");
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::OPTIMALITY_AND_FEASIBILITY;
    Plato::print_mma_stop_criterion(tFlag, tDescription);
    tGold.clear();
    tGold = "\n\n****** Optimization stopping due to optimality and feasibility tolerance being met. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::MAX_NUMBER_ITERATIONS;
    Plato::print_mma_stop_criterion(tFlag, tDescription);
    tGold.clear();
    tGold = "\n\n****** Optimization stopping due to exceeding maximum number of iterations. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::CONTROL_STAGNATION;
    Plato::print_mma_stop_criterion(tFlag, tDescription);
    tGold.clear();
    tGold = "\n\n****** Optimization stopping due to control (i.e. design variable) stagnation. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::OBJECTIVE_STAGNATION;
    Plato::print_mma_stop_criterion(tFlag, tDescription);
    tGold.clear();
    tGold = "\n\n****** Optimization stopping due to objective stagnation. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());
}

TEST(PlatoTest, MethodMovingAsymptotesNewCriterion_Objective)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    Plato::ApproximationFunctionData<double> tAppxFuncCoreData(tDataFactory);
    Plato::fill(0.1, *tAppxFuncCoreData.mAppxFunctionP);
    Plato::fill(0.11, *tAppxFuncCoreData.mAppxFunctionQ);
    Plato::fill(1.0, *tAppxFuncCoreData.mCurrentControls);
    Plato::fill(5.0, *tAppxFuncCoreData.mUpperAsymptotes);
    Plato::fill(-5.0, *tAppxFuncCoreData.mLowerAsymptotes);
    tAppxFuncCoreData.mCurrentNormalizedCriterionValue = 1;

    Plato::MethodMovingAsymptotesNewCriterion<double> tAppxFunction(tDataFactory);
    tAppxFunction.setConstraintAppxFuncMultiplier(0.0);
    tAppxFunction.update(tAppxFuncCoreData);

    // TEST GRADIENT
    std::ostringstream tGradDiagnosticMsg;
    Plato::Diagnostics<double> tDiagnostics;
    ASSERT_NO_THROW(tDiagnostics.checkCriterionGradient(tAppxFunction, *tAppxFuncCoreData.mCurrentControls, tGradDiagnosticMsg));
    ASSERT_TRUE(tDiagnostics.didGradientTestPassed());
    Plato::CommWrapper tComm(MPI_COMM_WORLD);
    if(tComm.myProcID() == static_cast<int>(0))
    {
        std::cout << tGradDiagnosticMsg.str().c_str();
    }

    // TEST HESSIAN
    std::ostringstream tHessDiagnosticMsg;
    ASSERT_NO_THROW(tDiagnostics.checkCriterionHessian(tAppxFunction, *tAppxFuncCoreData.mCurrentControls, tHessDiagnosticMsg));
    ASSERT_TRUE(tDiagnostics.didHessianTestPassed());
    if(tComm.myProcID() == static_cast<int>(0))
    {
        std::cout << tHessDiagnosticMsg.str().c_str();
    }
}

TEST(PlatoTest, MethodMovingAsymptotesNewCriterion_Constraint)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    Plato::ApproximationFunctionData<double> tAppxFuncCoreData(tDataFactory);
    Plato::fill(0.1, *tAppxFuncCoreData.mAppxFunctionP);
    Plato::fill(0.11, *tAppxFuncCoreData.mAppxFunctionQ);
    Plato::fill(1.0, *tAppxFuncCoreData.mCurrentControls);
    Plato::fill(5.0, *tAppxFuncCoreData.mUpperAsymptotes);
    Plato::fill(-5.0, *tAppxFuncCoreData.mLowerAsymptotes);
    tAppxFuncCoreData.mCurrentNormalizedCriterionValue = 1;

    Plato::MethodMovingAsymptotesNewCriterion<double> tAppxFunction(tDataFactory);
    tAppxFunction.update(tAppxFuncCoreData);

    // TEST GRADIENT
    std::ostringstream tGradDiagnosticMsg;
    Plato::Diagnostics<double> tDiagnostics;
    ASSERT_NO_THROW(tDiagnostics.checkCriterionGradient(tAppxFunction, *tAppxFuncCoreData.mCurrentControls, tGradDiagnosticMsg));
    ASSERT_TRUE(tDiagnostics.didGradientTestPassed());
    Plato::CommWrapper tComm(MPI_COMM_WORLD);
    if(tComm.myProcID() == static_cast<int>(0))
    {
        std::cout << tGradDiagnosticMsg.str().c_str();
    }

    // TEST HESSIAN
    std::ostringstream tHessDiagnosticMsg;
    ASSERT_NO_THROW(tDiagnostics.checkCriterionHessian(tAppxFunction, *tAppxFuncCoreData.mCurrentControls, tHessDiagnosticMsg));
    ASSERT_TRUE(tDiagnostics.didHessianTestPassed());
    if(tComm.myProcID() == static_cast<int>(0))
    {
        std::cout << tHessDiagnosticMsg.str().c_str();
    }
}

TEST(PlatoTest, MethodMovingAsymptotesNewOperations_initialize)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    // SET DATA
    Plato::MethodMovingAsymptotesNewDataMng<double> tDataMng(tDataFactory);
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(0.25, tData);
    tDataMng.setControlLowerBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlLowerBounds(), tData);
    Plato::fill(1., tData);
    tDataMng.setControlUpperBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlUpperBounds(), tData);

    // CALL FUNCTION
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);

    // TEST OUTPUT
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 0.75);
    tOperations.getUpperMinusLowerBounds(tData);
    PlatoTest::checkMultiVectorData(tData, tGold);
}

TEST(PlatoTest, MethodMovingAsymptotesNewOperations_updateInitialAsymptotes)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    // SET DATA
    Plato::MethodMovingAsymptotesNewDataMng<double> tDataMng(tDataFactory);
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(1.0, tData);
    tDataMng.setCurrentControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControls(), tData);
    Plato::fill(0.25, tData);
    tDataMng.setControlLowerBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlLowerBounds(), tData);
    Plato::fill(1., tData);
    tDataMng.setControlUpperBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlUpperBounds(), tData);

    // CALL FUNCTION
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);
    tOperations.updateInitialAsymptotes(tDataMng);

    // TEST OUTPUT
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 0.625);
    PlatoTest::checkMultiVectorData(tDataMng.getLowerAsymptotes(), tGold);
    Plato::fill(1.375, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getUpperAsymptotes(), tGold);
}

TEST(PlatoTest, MethodMovingAsymptotesNewOperations_updateCurrentAsymptotesMultipliers)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    // SET DATA
    Plato::MethodMovingAsymptotesNewDataMng<double> tDataMng(tDataFactory);
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(1.0, tData);
    tDataMng.setCurrentControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControls(), tData);
    Plato::fill(0.75, tData);
    tDataMng.setPreviousControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getPreviousControls(), tData);
    Plato::fill(0.65, tData);
    tDataMng.setAntepenultimateControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getAntepenultimateControls(), tData);

    // CASE 1: EXPANSION PARAMETER IS CHOSEN
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);
    tOperations.updateCurrentAsymptotesMultipliers(tDataMng);

    // TEST OUTPUT
    tOperations.getCurrentAsymptotesMultipliers(tData);
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 1.2);
    PlatoTest::checkMultiVectorData(tData, tGold);

    // CASE 2: CONTRACTION PARAMETER IS CHOSEN
    Plato::fill(0.55, tData);
    tDataMng.setPreviousControls(tData);
    tOperations.updateCurrentAsymptotesMultipliers(tDataMng);

    // TEST OUTPUT
    tOperations.getCurrentAsymptotesMultipliers(tData);
    Plato::fill(0.7, tGold);
    PlatoTest::checkMultiVectorData(tData, tGold);
}

TEST(PlatoTest, MethodMovingAsymptotesNewOperations_updateCurrentAsymptotes)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    // SET DATA
    Plato::MethodMovingAsymptotesNewDataMng<double> tDataMng(tDataFactory);
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(0.625, tData);
    tDataMng.setLowerAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getLowerAsymptotes(), tData);
    Plato::fill(1.375, tData);
    tDataMng.setUpperAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getUpperAsymptotes(), tData);
    Plato::fill(1.0, tData);
    tDataMng.setCurrentControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControls(), tData);
    Plato::fill(0.75, tData);
    tDataMng.setPreviousControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getPreviousControls(), tData);
    Plato::fill(0.65, tData);
    tDataMng.setAntepenultimateControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getAntepenultimateControls(), tData);

    // CASE 1: EXPANSION PARAMETER IS CHOSEN
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);
    tOperations.updateCurrentAsymptotesMultipliers(tDataMng);
    tOperations.updateCurrentAsymptotes(tDataMng);

    // TEST OUTPUT
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 0.85);
    PlatoTest::checkMultiVectorData(tDataMng.getLowerAsymptotes(), tGold);
    Plato::fill(1.75, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getUpperAsymptotes(), tGold);
}

TEST(PlatoTest, MethodMovingAsymptotesNewOperations_updateSubProblemBounds)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    // SET DATA
    Plato::MethodMovingAsymptotesNewDataMng<double> tDataMng(tDataFactory);
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(0.625, tData);
    tDataMng.setLowerAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getLowerAsymptotes(), tData);
    Plato::fill(1.375, tData);
    tDataMng.setUpperAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getUpperAsymptotes(), tData);
    Plato::fill(1.0, tData);
    tDataMng.setCurrentControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControls(), tData);
    Plato::fill(0.25, tData);
    tDataMng.setControlLowerBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlLowerBounds(), tData);
    Plato::fill(1., tData);
    tDataMng.setControlUpperBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlUpperBounds(), tData);

    // CASE 1: EXPANSION PARAMETER IS CHOSEN
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);
    tOperations.updateSubProblemBounds(tDataMng);

    // TEST OUTPUT
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 0.6625);
    PlatoTest::checkMultiVectorData(tDataMng.getSubProblemControlLowerBounds(), tGold);
    Plato::fill(1., tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getSubProblemControlUpperBounds(), tGold);
}

TEST(PlatoTest, MethodMovingAsymptotesNewOperations_updateObjectiveApproximationFunctionData)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    // CREATE MEANINGFUL CRITERION
    Plato::CcsaTestObjective<double> tCriterion;
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(1.0, tData);
    double tObjFuncValue = tCriterion.value(tData);
    Plato::StandardMultiVector<double> tGradient(1 /* number of vectors */, tNumControls);
    tCriterion.gradient(tData, tGradient);

    // SET DATA
    Plato::MethodMovingAsymptotesNewDataMng<double> tDataMng(tDataFactory);
    tDataMng.setCurrentObjectiveValue(tObjFuncValue);
    const double tTolerance = 1e-6;
    ASSERT_NEAR(0.312, tDataMng.getCurrentObjectiveValue(), tTolerance);
    tDataMng.setCurrentObjectiveGradient(tGradient);
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 0.0624 /* initial value */);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentObjectiveGradient(), tGold);

    Plato::fill(0.625, tData);
    tDataMng.setLowerAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getLowerAsymptotes(), tData);
    Plato::fill(1.375, tData);
    tDataMng.setUpperAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getUpperAsymptotes(), tData);
    Plato::fill(1.0, tData);
    tDataMng.setCurrentControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControls(), tData);
    Plato::fill(0.25, tData);
    tDataMng.setControlLowerBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlLowerBounds(), tData);
    Plato::fill(1., tData);
    tDataMng.setControlUpperBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlUpperBounds(), tData);

    // CASE 1: EXPANSION PARAMETER IS CHOSEN
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);
    tOperations.updateObjectiveApproximationFunctionData(tDataMng);

    // TEST OUTPUT
    Plato::fill(0.028155, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getObjFuncAppxFunctionP(), tGold);
    Plato::fill(3e-5, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getObjFuncAppxFunctionQ(), tGold);
}

TEST(PlatoTest, MethodMovingAsymptotesNewOperations_updateConstraintApproximationFunctionsData)
{
    const size_t tNumControls = 5;
    const size_t tNumConstraints = 2;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateDual(tNumConstraints);
    tDataFactory->allocateControl(tNumControls);

    // CREATE MEANINGFUL CRITERION
    Plato::CcsaTestObjective<double> tCriterion;
    Plato::MethodMovingAsymptotesNewDataMng<double> tDataMng(tDataFactory);
    ASSERT_EQ(2u, tDataMng.getNumConstraints());
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(1.0, tData);

    // SET DATA FOR CONSTRAINT 1
    size_t tContraintIndex = 0;
    double tConstraintValue = tCriterion.value(tData);
    tDataMng.setCurrentConstraintValue(tContraintIndex, tConstraintValue);
    tDataMng.setConstraintNormalization(tContraintIndex, 1);
    const double tTolerance = 1e-6;
    ASSERT_NEAR(0.312, tDataMng.getCurrentConstraintValue(tContraintIndex), tTolerance);

    Plato::StandardMultiVector<double> tConstraintGradient(1 /* number of vectors */, tNumControls);
    tCriterion.gradient(tData, tConstraintGradient);
    tDataMng.setCurrentConstraintGradient(tContraintIndex, tConstraintGradient);
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 0.0624 /* initial value */);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentConstraintGradient(tContraintIndex), tGold);

    // SET DATA FOR CONSTRAINT 2
    tContraintIndex = 1;
    tCriterion.setWeightConstant(0.624);
    tConstraintValue = tCriterion.value(tData);
    tDataMng.setConstraintNormalization(tContraintIndex, 1);
    tDataMng.setCurrentConstraintValue(tContraintIndex, tConstraintValue);
    ASSERT_NEAR(3.12, tDataMng.getCurrentConstraintValue(tContraintIndex), tTolerance);

    tCriterion.gradient(tData, tConstraintGradient);
    tDataMng.setCurrentConstraintGradient(tContraintIndex, tConstraintGradient);
    Plato::fill(0.624, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentConstraintGradient(tContraintIndex), tGold);

    // SET OTHER DATA
    Plato::fill(0.625, tData);
    tDataMng.setLowerAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getLowerAsymptotes(), tData);
    Plato::fill(1.375, tData);
    tDataMng.setUpperAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getUpperAsymptotes(), tData);
    Plato::fill(1.0, tData);
    tDataMng.setCurrentControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControls(), tData);
    Plato::fill(0.25, tData);
    tDataMng.setControlLowerBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlLowerBounds(), tData);
    Plato::fill(1., tData);
    tDataMng.setControlUpperBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlUpperBounds(), tData);

    // CASE 1: EXPANSION PARAMETER IS CHOSEN
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);
    tOperations.updateConstraintApproximationFunctionsData(tDataMng);

    // TEST OUTPUT FOR FIRST CONSTRAINT
    tContraintIndex = 0;
    Plato::fill(0.00878565, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getConstraintAppxFunctionP(tContraintIndex), tGold);
    Plato::fill(0.00001065, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getConstraintAppxFunctionQ(tContraintIndex), tGold);

    // TEST OUTPUT FOR SECOND CONSTRAINT
    tContraintIndex = 1;
    Plato::fill(0.087839625, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getConstraintAppxFunctionP(tContraintIndex), tGold);
    Plato::fill(0.000089625, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getConstraintAppxFunctionQ(tContraintIndex), tGold);
}

TEST(PlatoTest, MethodMovingAsymptotes_5Bars)
{
    // ********* SET OBJECTIVE AND COSNTRAINT *********
    std::shared_ptr<Plato::Criterion<double>> tObjective = std::make_shared<Plato::CcsaTestObjective<double>>();
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    std::shared_ptr<Plato::CcsaTestInequality<double>> tConstraint = std::make_shared<Plato::CcsaTestInequality<double>>();
    tConstraintList->add(tConstraint);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    const size_t tNumVectors = 1;
    const size_t tNumControls = 5;
    const size_t tNumConstraints = 1;
    Plato::AlgorithmInputsMMA<double> tInputs;
    tInputs.mLowerBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 1.0 /* values */);
    tInputs.mUpperBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 10.0 /* values */);
    tInputs.mInitialGuess = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 5.0 /* values */);
    tInputs.mConstraintNormalizationParams = std::make_shared<Plato::StandardVector<double>>(tNumConstraints, 1.0 /* values */);
    Plato::AlgorithmOutputsMMA<double> tOutputs;
    Plato::solve_mma<double, size_t>(tObjective, tConstraintList, tInputs, tOutputs);

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    ASSERT_EQ(28u, tOutputs.mNumSolverIter);
    ASSERT_NEAR(1.33996, tOutputs.mObjFuncValue, tTolerance);
    ASSERT_TRUE(std::abs((*tOutputs.mConstraints)[0]) < tTolerance);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(0,0) = 6.007921063; tGold(0,1) = 5.309376913; tGold(0,2) = 4.497699841; tGold(0,3) = 3.505355435; tGold(0,4) = 2.15340137;
    PlatoTest::checkMultiVectorData(tGold, *tOutputs.mSolution);

    // ********* PRINT SOLUTION *********
    std::cout << "NUMBER OF ITERATIONS = " << tOutputs.mNumSolverIter << "\n" << std::flush;
    std::cout << "BEST OBJECTIVE VALUE = " << tOutputs.mObjFuncValue << "\n" << std::flush;
    std::cout << "BEST CONSTRAINT VALUE = " << (*tOutputs.mConstraints)[0] << "\n" << std::flush;
    std::cout << "SOLUTION\n" << std::flush;
    PlatoTest::printMultiVector(*tOutputs.mSolution);
    std::cout << tOutputs.mStopCriterion.c_str() << "\n" << std::flush;
}

TEST(PlatoTest, MethodMovingAsymptotes_RosenbrockRadius)
{
    // ********* SET OBJECTIVE AND COSNTRAINT *********
    std::shared_ptr<Plato::Rosenbrock<double>> tObjective = std::make_shared<Plato::Rosenbrock<double>>();
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    std::shared_ptr<Plato::Radius<double>> tConstraint = std::make_shared<Plato::Radius<double>>();
    tConstraintList->add(tConstraint);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    const size_t tNumConstraints = 1;
    Plato::AlgorithmInputsMMA<double> tInputs;
    tInputs.mMoveLimit = 0.05;
    tInputs.mLowerBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 0.0 /* values */);
    tInputs.mUpperBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 2.0 /* values */);
    tInputs.mInitialGuess = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 1.0 /* values */);
    tInputs.mConstraintNormalizationParams = std::make_shared<Plato::StandardVector<double>>(tNumConstraints, 2.0 /* values */);
    Plato::AlgorithmOutputsMMA<double> tOutputs;
    Plato::solve_mma<double, size_t>(tObjective, tConstraintList, tInputs, tOutputs);

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    ASSERT_EQ(36u, tOutputs.mNumSolverIter);
    ASSERT_NEAR(0.0456735, tOutputs.mObjFuncValue, tTolerance);
    ASSERT_TRUE(std::abs((*tOutputs.mConstraints)[0]) < tTolerance);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(0,0) = 0.7864850239; tGold(0,1) = 0.617637575;
    PlatoTest::checkMultiVectorData(tGold, *tOutputs.mSolution);

    // ********* PRINT SOLUTION *********
    std::cout << "NUMBER OF ITERATIONS = " << tOutputs.mNumSolverIter << "\n" << std::flush;
    std::cout << "BEST OBJECTIVE VALUE = " << tOutputs.mObjFuncValue << "\n" << std::flush;
    std::cout << "BEST CONSTRAINT VALUE = " << (*tOutputs.mConstraints)[0] << "\n" << std::flush;
    std::cout << "SOLUTION\n" << std::flush;
    PlatoTest::printMultiVector(*tOutputs.mSolution);
    std::cout << tOutputs.mStopCriterion.c_str() << "\n" << std::flush;
}

TEST(PlatoTest, MethodMovingAsymptotes_HimmelblauShiftedEllipse)
{
    // ********* SET OBJECTIVE AND COSNTRAINT *********
    std::shared_ptr<Plato::Himmelblau<double>> tObjective = std::make_shared<Plato::Himmelblau<double>>();
    std::shared_ptr<Plato::ShiftedEllipse<double>> tConstraint = std::make_shared<Plato::ShiftedEllipse<double>>();
    tConstraint->specify(-2., 2., -3., 3.);
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    tConstraintList->add(tConstraint);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    const size_t tNumConstraints = 1;
    Plato::AlgorithmInputsMMA<double> tInputs;
    tInputs.mLowerBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, -5.0 /* values */);
    tInputs.mUpperBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, -1.0 /* values */);
    tInputs.mInitialGuess = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, -2.0 /* values */);
    tInputs.mConstraintNormalizationParams = std::make_shared<Plato::StandardVector<double>>(tNumConstraints, 0.5 /* values */);
    Plato::AlgorithmOutputsMMA<double> tOutputs;
    Plato::solve_mma<double, size_t>(tObjective, tConstraintList, tInputs, tOutputs);

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    ASSERT_EQ(27u, tOutputs.mNumSolverIter);
    ASSERT_NEAR(11.8039164, tOutputs.mObjFuncValue, tTolerance);
    ASSERT_TRUE(std::abs((*tOutputs.mConstraints)[0]) < tTolerance);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(0,0) = -3.99832364; tGold(0,1) = -2.878447094;
    PlatoTest::checkMultiVectorData(tGold, *tOutputs.mSolution);

    // ********* PRINT SOLUTION *********
    std::cout << "NUMBER OF ITERATIONS = " << tOutputs.mNumSolverIter << "\n" << std::flush;
    std::cout << "BEST OBJECTIVE VALUE = " << tOutputs.mObjFuncValue << "\n" << std::flush;
    std::cout << "BEST CONSTRAINT VALUE = " << (*tOutputs.mConstraints)[0] << "\n" << std::flush;
    std::cout << "SOLUTION\n" << std::flush;
    PlatoTest::printMultiVector(*tOutputs.mSolution);
    std::cout << tOutputs.mStopCriterion.c_str() << "\n" << std::flush;
}

TEST(PlatoTest, MethodMovingAsymptotes_GoldsteinPriceShiftedEllipse)
{
    // ********* SET OBJECTIVE AND COSNTRAINT *********
    std::shared_ptr<Plato::GoldsteinPrice<double>> tObjective = std::make_shared<Plato::GoldsteinPrice<double>>();
    std::shared_ptr<Plato::ShiftedEllipse<double>> tConstraint = std::make_shared<Plato::ShiftedEllipse<double>>();
    tConstraint->specify(0., 1., .5, 1.5);
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    tConstraintList->add(tConstraint);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    const size_t tNumConstraints = 1;
    Plato::AlgorithmInputsMMA<double> tInputs;
    tInputs.mLowerBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, -3.0 /* values */);
    tInputs.mUpperBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 0.0 /* values */);
    tInputs.mInitialGuess = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, -0.4 /* values */);
    tInputs.mConstraintNormalizationParams = std::make_shared<Plato::StandardVector<double>>(tNumConstraints, 0.5 /* values */);
    Plato::AlgorithmOutputsMMA<double> tOutputs;
    Plato::solve_mma<double, size_t>(tObjective, tConstraintList, tInputs, tOutputs);

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    ASSERT_EQ(19u, tOutputs.mNumSolverIter);
    ASSERT_NEAR(3, tOutputs.mObjFuncValue, tTolerance);
    ASSERT_TRUE(std::abs((*tOutputs.mConstraints)[0]) < tTolerance);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(0,0) = 0.0; tGold(0,1) = -1.0;
    PlatoTest::checkMultiVectorData(tGold, *tOutputs.mSolution, tTolerance);

    // ********* PRINT SOLUTION *********
    std::cout << "NUMBER OF ITERATIONS = " << tOutputs.mNumSolverIter << "\n" << std::flush;
    std::cout << "BEST OBJECTIVE VALUE = " << tOutputs.mObjFuncValue << "\n" << std::flush;
    std::cout << "BEST CONSTRAINT VALUE = " << (*tOutputs.mConstraints)[0] << "\n" << std::flush;
    std::cout << "SOLUTION\n" << std::flush;
    PlatoTest::printMultiVector(*tOutputs.mSolution);
    std::cout << tOutputs.mStopCriterion.c_str() << "\n" << std::flush;
}

TEST(PlatoTest, MethodMovingAsymptotes_CircleRadius)
{
    // ********* SET OBJECTIVE AND COSNTRAINT *********
    std::shared_ptr<Plato::Circle<double>> tObjective = std::make_shared<Plato::Circle<double>>();
    std::shared_ptr<Plato::Radius<double>> tConstraint = std::make_shared<Plato::Radius<double>>();
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    tConstraintList->add(tConstraint);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    const size_t tNumConstraints = 1;
    Plato::AlgorithmInputsMMA<double> tInputs;
    tInputs.mLowerBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 0.0 /* values */);
    tInputs.mUpperBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 2.0 /* values */);
    tInputs.mInitialGuess = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 0.5 /* values */);
    tInputs.mConstraintNormalizationParams = std::make_shared<Plato::StandardVector<double>>(tNumConstraints, 0.5 /* values */);
    Plato::AlgorithmOutputsMMA<double> tOutputs;
    Plato::solve_mma<double, size_t>(tObjective, tConstraintList, tInputs, tOutputs);

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    ASSERT_EQ(19u, tOutputs.mNumSolverIter);
    ASSERT_NEAR(2.677976067, tOutputs.mObjFuncValue, tTolerance);
    ASSERT_TRUE(std::abs((*tOutputs.mConstraints)[0]) < tTolerance);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(0,0) = 0.3129487804; tGold(0,1) = 0.9497764126;
    PlatoTest::checkMultiVectorData(tGold, *tOutputs.mSolution, tTolerance);

    // ********* PRINT SOLUTION *********
    std::cout << "NUMBER OF ITERATIONS = " << tOutputs.mNumSolverIter << "\n" << std::flush;
    std::cout << "BEST OBJECTIVE VALUE = " << tOutputs.mObjFuncValue << "\n" << std::flush;
    std::cout << "BEST CONSTRAINT VALUE = " << (*tOutputs.mConstraints)[0] << "\n" << std::flush;
    std::cout << "SOLUTION\n" << std::flush;
    PlatoTest::printMultiVector(*tOutputs.mSolution);
    std::cout << tOutputs.mStopCriterion.c_str() << "\n" << std::flush;
}

TEST(PlatoTest, MethodMovingAsymptotes_MinComplianceVolumeConstraint)
{
    // ************** ALLOCATE SIMPLE STRUCTURAL TOPOLOGY OPTIMIZATION SOLVER **************
    const int tNumElementsXDir = 30;
    const int tNumElementsYDir = 10;
    const double tPoissonRatio = 0.3;
    const double tElasticModulus = 1;
    std::shared_ptr<Plato::StructuralTopologyOptimization> tPDE =
            std::make_shared <Plato::StructuralTopologyOptimization>(tPoissonRatio, tElasticModulus, tNumElementsXDir, tNumElementsYDir);
    tPDE->setFilterRadius(1.1);

    // ************** SET FORCE VECTOR **************
    const int tGlobalNumDofs = tPDE->getGlobalNumDofs();
    Epetra_SerialDenseVector tForce(tGlobalNumDofs);
    const int tDOFsIndex = 1;
    tForce[tDOFsIndex] = -1;
    tPDE->setForceVector(tForce);

    // ************** SET FIXED DEGREES OF FREEDOM (DOFs) VECTOR **************
    std::vector<double> tDofs = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 681};
    Epetra_SerialDenseVector tFixedDOFs(Epetra_DataAccess::Copy, tDofs.data(), tDofs.size());
    tPDE->setFixedDOFs(tFixedDOFs);

    // ********* SET OBJECTIVE AND COSNTRAINT *********
    std::shared_ptr<Plato::ProxyVolume<double, int>> tVolume = std::make_shared<Plato::ProxyVolume<double, int>>(tPDE);
    std::shared_ptr<Plato::ProxyCompliance<double, int>> tCompliance = std::make_shared<Plato::ProxyCompliance<double, int>>(tPDE);
    std::shared_ptr<Plato::CriterionList<double, int>> tConstraintList = std::make_shared<Plato::CriterionList<double, int>>();
    tConstraintList->add(tVolume);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    const int tNumVectors = 1;
    const int tNumConstraints = 1;
    const int tNumControls = tPDE->getNumDesignVariables();
    Plato::AlgorithmInputsMMA<double, int> tInputs;
    const double tTargetVolume = tPDE->getVolumeFraction();
    tInputs.mMaxNumSolverIter = 50;
    tInputs.mAsymptoteExpansion = 1.001;
    tInputs.mAsymptoteContraction = 0.999;
    tInputs.mInitialGuess = std::make_shared<Plato::EpetraSerialDenseMultiVector<double, int>>(tNumVectors, tNumControls, tTargetVolume /* values */);
    tInputs.mUpperBounds = std::make_shared<Plato::EpetraSerialDenseMultiVector<double, int>>(tNumVectors, tNumControls, 1.0 /* values */);
    tInputs.mLowerBounds = std::make_shared<Plato::EpetraSerialDenseMultiVector<double, int>>(tNumVectors, tNumControls, 1e-2 /* values */);
    tInputs.mConstraintNormalizationParams = std::make_shared<Plato::EpetraSerialDenseVector<double, int>>(tNumConstraints, 4.0 /* values */);
    Plato::AlgorithmOutputsMMA<double, int> tOutputs;
    Plato::solve_mma<double, int>(tCompliance, tConstraintList, tInputs, tOutputs);

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-2;
    ASSERT_EQ(50u, tOutputs.mNumSolverIter);
    ASSERT_NEAR(0.1485850316, tOutputs.mObjFuncValue, tTolerance);
    ASSERT_TRUE(std::abs((*tOutputs.mConstraints)[0]) < tTolerance);
    std::vector<double> tGoldData = PlatoTest::get_topology_optimization_gold();
    Plato::StandardMultiVector<double, int> tGold(tNumVectors, tGoldData);
    tGold.setData(0, tGoldData);
    PlatoTest::checkMultiVectorData(tGold, *tOutputs.mSolution, tTolerance);

    // ********* PRINT SOLUTION *********
    std::cout << "NUMBER OF ITERATIONS = " << tOutputs.mNumSolverIter << "\n" << std::flush;
    std::cout << "BEST OBJECTIVE VALUE = " << tOutputs.mObjFuncValue << "\n" << std::flush;
    std::cout << "BEST CONSTRAINT VALUE = " << (*tOutputs.mConstraints)[0] << "\n" << std::flush;
    std::cout << tOutputs.mStopCriterion.c_str() << "\n" << std::flush;
}

}
