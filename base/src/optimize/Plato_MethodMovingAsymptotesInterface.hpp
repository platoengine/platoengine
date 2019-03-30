/*
 * Plato_MethodMovingAsymptotesInterface.hpp
 *
 *  Created on: Nov 4, 2017
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

#ifndef PLATO_METHODMOVINGASYMPTOTESINTERFACE_HPP_
#define PLATO_METHODMOVINGASYMPTOTESINTERFACE_HPP_

#include <string>
#include <memory>

#include "Plato_Parser.hpp"
#include "Plato_Interface.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_AlgebraFactory.hpp"
#include "Plato_StandardVector.hpp"
#include "Plato_EngineObjective.hpp"
#include "Plato_EngineConstraint.hpp"
#include "Plato_OptimizerUtilities.hpp"
#include "Plato_OptimizerInterface.hpp"
#include "Plato_PrimalProblemStageMng.hpp"
#include "Plato_MethodMovingAsymptotes.hpp"
#include "Plato_OptimizerEngineStageData.hpp"
#include "Plato_ConservativeConvexSeparableAppxDataMng.hpp"
#include "Plato_ConservativeConvexSeparableAppxAlgorithm.hpp"

namespace Plato
{

template<typename ScalarType>
struct DefaultParametersMMA
{
    DefaultParametersMMA() :
            mMovingAsymptoteExpansionFactor(2),
            mMovingAsymptoteUpperBoundScaleFactor(50),
            mMovingAsymptoteLowerBoundScaleFactor(0.001875)
    {
    }

    ScalarType mMovingAsymptoteExpansionFactor;
    ScalarType mMovingAsymptoteUpperBoundScaleFactor;
    ScalarType mMovingAsymptoteLowerBoundScaleFactor;
};

template<typename ScalarType, typename OrdinalType = size_t>
class MethodMovingAsymptotesInterface : public Plato::OptimizerInterface<ScalarType, OrdinalType>
{
public:
    /******************************************************************************/
    explicit MethodMovingAsymptotesInterface(Plato::Interface* aInterface, const MPI_Comm & aComm) :
            mComm(aComm),
            mInterface(aInterface),
            mInputData(Plato::OptimizerEngineStageData())
    /******************************************************************************/
    {
    }

    /******************************************************************************/
    virtual ~MethodMovingAsymptotesInterface()
    /******************************************************************************/
    {
    }

    /******************************************************************************/
    Plato::optimizer::algorithm_t type() const
    /******************************************************************************/
    {
        return (Plato::optimizer::algorithm_t::METHOD_OF_MOVING_ASYMPTOTES);
    }

    /******************************************************************************/
    void initialize()
    /******************************************************************************/
    {
        Plato::initialize<ScalarType, OrdinalType>(mInterface, mInputData);
    }

    /******************************************************************************/
    void optimize()
    /******************************************************************************/
    {
        mInterface->handleExceptions();

        this->initialize();

        // ********* ALLOCATE LINEAR ALGEBRA FACTORY ********* //
        Plato::AlgebraFactory<ScalarType, OrdinalType> tLinearAlgebraFactory;

        // ********* ALLOCATE OPTIMIZER'S BASELINE DATA STRUCTURES *********
        std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> tDataFactory =
                std::make_shared<Plato::DataFactory<ScalarType, OrdinalType>>();
        this->allocateBaselineDataStructures(tLinearAlgebraFactory, *tDataFactory);

        // ********* ALLOCATE OPTIMIZER'S DATA MANAGER *********
        std::shared_ptr<Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType>> tDataMng =
                std::make_shared<Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType>>(tDataFactory);

        // ********* SET LOWER AND UPPER BOUNDS FOR CONTROLS *********
        this->setLowerBounds(tLinearAlgebraFactory, *tDataFactory, *tDataMng);
        this->setUpperBounds(tLinearAlgebraFactory, *tDataFactory, *tDataMng);

        // ********* SET INITIAL GUESS *********
        this->setInitialGuess(tLinearAlgebraFactory, *tDataMng);

        // ********* SOLVE OPTIMIZATION PROBLEM *********
        this->solveOptimizationProblem(tDataMng, tDataFactory);

        // ********* OUTPUT SOLUTION *********
        Plato::call_finalization_stage(mInterface, mInputData);

        this->finalize();
    }

    /******************************************************************************/
    void finalize()
    /******************************************************************************/
    {
        mInterface->finalize();
    }

private:
    /******************************************************************************/
    void setUpperBounds(const Plato::AlgebraFactory<ScalarType, OrdinalType> & aAlgebraFactory,
                        Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory,
                        Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    /******************************************************************************/
    {
        const OrdinalType tCONTROL_VECTOR_INDEX = 0;
        std::string tControlName = mInputData.getControlName(tCONTROL_VECTOR_INDEX);
        const OrdinalType tNumControls = mInterface->size(tControlName);
        std::vector<ScalarType> tInputBoundsData(tNumControls);

        // ********* GET UPPER BOUNDS INFORMATION *********
        Plato::getUpperBoundsInputData(mInputData, mInterface, tInputBoundsData);

        // ********* SET UPPER BOUNDS FOR OPTIMIZER *********
        std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> tUpperBoundVector =
                aAlgebraFactory.createVector(mComm, tNumControls, mInterface);
        aDataFactory.allocateUpperBoundVector(*tUpperBoundVector);
        Plato::copy(tInputBoundsData, *tUpperBoundVector);
        aDataMng.setControlUpperBounds(tCONTROL_VECTOR_INDEX, *tUpperBoundVector);
    }

    /******************************************************************************/
    void setLowerBounds(const Plato::AlgebraFactory<ScalarType, OrdinalType> & aAlgebraFactory,
                        Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory,
                        Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    /******************************************************************************/
    {
        const OrdinalType tCONTROL_VECTOR_INDEX = 0;
        std::string tControlName = mInputData.getControlName(tCONTROL_VECTOR_INDEX);
        const OrdinalType tNumControls = mInterface->size(tControlName);
        std::vector<ScalarType> tInputBoundsData(tNumControls);

        // ********* GET LOWER BOUNDS INFORMATION *********
        Plato::getLowerBoundsInputData(mInputData, mInterface, tInputBoundsData);

        // ********* SET LOWER BOUNDS FOR OPTIMIZER *********
        std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> tLowerBoundVector =
                aAlgebraFactory.createVector(mComm, tNumControls, mInterface);
        aDataFactory.allocateLowerBoundVector(*tLowerBoundVector);
        Plato::copy(tInputBoundsData, *tLowerBoundVector);
        aDataMng.setControlLowerBounds(tCONTROL_VECTOR_INDEX, *tLowerBoundVector);
    }

    /******************************************************************************/
    void setInitialGuess(const Plato::AlgebraFactory<ScalarType, OrdinalType> & aAlgebraFactory,
                         Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    /******************************************************************************/
    {
        // ********* Allocate Plato::Vector of controls *********
        const OrdinalType tCONTROL_VECTOR_INDEX = 0;
        std::string tControlName = mInputData.getControlName(tCONTROL_VECTOR_INDEX);
        const OrdinalType tNumControls = mInterface->size(tControlName);
        std::vector<ScalarType> tInputIntitalGuessData(tNumControls);
        std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> tVector =
                aAlgebraFactory.createVector(mComm, tNumControls, mInterface);

        // ********* Set initial guess for each control vector *********
        Plato::getInitialGuessInputData(tControlName, mInputData, mInterface, tInputIntitalGuessData);
        Plato::copy(tInputIntitalGuessData, *tVector);
        aDataMng.setInitialGuess(tCONTROL_VECTOR_INDEX, *tVector);
    }

    /******************************************************************************/
    void allocateBaselineDataStructures(const Plato::AlgebraFactory<ScalarType, OrdinalType> & aAlgebraFactory,
                                        Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory)
    /******************************************************************************/
    {
        const OrdinalType tNumDuals = mInputData.getNumConstraints();
        Plato::StandardVector<ScalarType, OrdinalType> tDuals(tNumDuals);
        aDataFactory.allocateDual(tDuals);

        // ********* Allocate control vectors baseline data structures *********
        const OrdinalType tNumVectors = mInputData.getNumControlVectors();
        assert(tNumVectors > static_cast<OrdinalType>(0));
        Plato::StandardMultiVector<ScalarType, OrdinalType> tMultiVector;
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            std::string tControlName = mInputData.getControlName(tIndex);
            const OrdinalType tNumControls = mInterface->size(tControlName);
            std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> tVector =
                    aAlgebraFactory.createVector(mComm, tNumControls, mInterface);
            tMultiVector.add(tVector);
        }
        aDataFactory.allocateControl(tMultiVector);
        std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> tReductionOperations =
                aAlgebraFactory.createReduction(mComm, mInterface);
        aDataFactory.allocateControlReductionOperations(*tReductionOperations);

        Plato::CommWrapper tCommWrapper(mComm);
        aDataFactory.setCommWrapper(tCommWrapper);
    }

    /******************************************************************************/
    void solveOptimizationProblem(const std::shared_ptr<Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType>> & aDataMng,
                                  const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aDataFactory)
    /******************************************************************************/
    {
        // ********* ALLOCATE OBJECTIVE FUNCTION ********* //
        std::shared_ptr<Plato::EngineObjective<ScalarType, OrdinalType>> tObjective =
                std::make_shared<Plato::EngineObjective<ScalarType, OrdinalType>>(*aDataFactory, mInputData, mInterface);

        // ********* ALLOCATE LIST OF CONSTRAINTS ********* //
        const OrdinalType tNumConstraints = mInputData.getNumConstraints();
        std::vector<std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>>> tList(tNumConstraints);
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            tList[tIndex] = std::make_shared<Plato::EngineConstraint<ScalarType, OrdinalType>>(tIndex, *aDataFactory, mInputData, mInterface);
        }
        std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> tConstraints =
                std::make_shared<Plato::CriterionList<ScalarType, OrdinalType>>();
        tConstraints->add(tList);

        // ********* ALLOCATE PRIMAL PROBLEM STAGE MANAGER *********
        std::shared_ptr<Plato::PrimalProblemStageMng<ScalarType, OrdinalType>> tStageMng =
                std::make_shared<Plato::PrimalProblemStageMng<ScalarType, OrdinalType>>(aDataFactory, tObjective, tConstraints);

        // ********* ALLOCATE CCSA METHOD = METHOD OF MOVING ASYMPTOTES *********
        std::shared_ptr<Plato::MethodMovingAsymptotes<ScalarType, OrdinalType>> tSubProblem =
                std::make_shared<Plato::MethodMovingAsymptotes<ScalarType, OrdinalType>>(aDataFactory);

        // ********* ALLOCATE CONSERVATIVE CONVEX SEPARABLE APPROXIMATION (CCSA) ALGORITHM *********
        Plato::ConservativeConvexSeparableAppxAlgorithm<ScalarType, OrdinalType> tAlgorithm(tStageMng, aDataMng, tSubProblem);
        this->setParameters(tAlgorithm);
        tAlgorithm.enableDiagnostics();
        tAlgorithm.solve();
    }

    void setParameters(Plato::ConservativeConvexSeparableAppxAlgorithm<ScalarType, OrdinalType>& aAlgorithm)
    {
        // ********* Get User input ***************
        OrdinalType tMaxNumIterations = mInputData.getMaxNumIterations();
        aAlgorithm.setMaxNumIterations(tMaxNumIterations);
        ScalarType tKKTTolerance = mInputData.getCCSAOuterKKTTolerance();
        aAlgorithm.setKarushKuhnTuckerConditionsTolerance(tKKTTolerance);
        ScalarType tStationarityTolerance = mInputData.getCCSAOuterStationarityTolerance();
        aAlgorithm.setStationarityTolerance(tStationarityTolerance);
        ScalarType tControlStagnationTolerance = mInputData.getCCSAOuterControlStagnationTolerance();
        aAlgorithm.setControlStagnationTolerance(tControlStagnationTolerance);
        ScalarType tObjectiveStagnationTolerance = mInputData.getCCSAOuterObjectiveStagnationTolerance();
        aAlgorithm.setObjectiveStagnationTolerance(tObjectiveStagnationTolerance);
        ScalarType tInitialMovingAsymptoteScaleFactor = mInputData.getInitialMovingAsymptoteScaleFactor();
        aAlgorithm.setInitialMovingAsymptoteScaleFactor(tInitialMovingAsymptoteScaleFactor);

        Plato::DefaultParametersMMA<ScalarType> tDefaultParametersMMA;
        aAlgorithm.setMovingAsymptoteExpansionFactor(tDefaultParametersMMA.mMovingAsymptoteExpansionFactor);
        aAlgorithm.setMovingAsymptoteLowerBoundScaleFactor(tDefaultParametersMMA.mMovingAsymptoteLowerBoundScaleFactor);
        aAlgorithm.setMovingAsymptoteUpperBoundScaleFactor(tDefaultParametersMMA.mMovingAsymptoteUpperBoundScaleFactor);
    }

private:
    MPI_Comm mComm;
    Plato::Interface* mInterface;
    Plato::OptimizerEngineStageData mInputData;

private:
    MethodMovingAsymptotesInterface(const Plato::MethodMovingAsymptotesInterface<ScalarType, OrdinalType>&);
    Plato::MethodMovingAsymptotesInterface<ScalarType, OrdinalType> & operator=(const Plato::MethodMovingAsymptotesInterface<ScalarType, OrdinalType>&);
};

} // namespace Plato

#endif /* PLATO_METHODMOVINGASYMPTOTESINTERFACE_HPP_ */
