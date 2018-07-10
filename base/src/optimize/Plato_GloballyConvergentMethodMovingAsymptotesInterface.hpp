/*
 * Plato_GloballyConvergentMethodMovingAsymptotesInterface.hpp
 *
 *  Created on: Nov 2, 2017
 */

#ifndef PLATO_GLOBALLYCONVERGENTMETHODMOVINGASYMPTOTESINTERFACE_HPP_
#define PLATO_GLOBALLYCONVERGENTMETHODMOVINGASYMPTOTESINTERFACE_HPP_

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
#include "Plato_OptimizerEngineStageData.hpp"
#include "Plato_ConservativeConvexSeparableAppxDataMng.hpp"
#include "Plato_ConservativeConvexSeparableAppxAlgorithm.hpp"
#include "Plato_GloballyConvergentMethodMovingAsymptotes.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class GloballyConvergentMethodMovingAsymptotesInterface : public Plato::OptimizerInterface<ScalarType, OrdinalType>
{
public:
    /******************************************************************************/
    explicit GloballyConvergentMethodMovingAsymptotesInterface(Plato::Interface* aInterface, const MPI_Comm & aComm) :
            mComm(aComm),
            mInterface(aInterface),
            mInputData(Plato::OptimizerEngineStageData())
    /******************************************************************************/
    {
    }

    /******************************************************************************/
    virtual ~GloballyConvergentMethodMovingAsymptotesInterface()
    /******************************************************************************/
    {
    }

    /******************************************************************************/
    Plato::optimizer::algorithm_t type() const
    /******************************************************************************/
    {
        return (Plato::optimizer::algorithm_t::GLOBALLY_CONVERGENT_METHOD_OF_MOVING_ASYMPTOTES);
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
        Plato::AlgebraFactory<ScalarType, OrdinalType> tAlgebraFactory;

        // ********* ALLOCATE OPTIMIZER'S BASELINE DATA STRUCTURES *********
        std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> tDataFactory =
                std::make_shared<Plato::DataFactory<ScalarType, OrdinalType>>();
        this->allocateBaselineDataStructures(tAlgebraFactory, *tDataFactory);

        // ********* ALLOCATE OPTIMIZER'S DATA MANAGER *********
        std::shared_ptr<Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType>> tDataMng =
                std::make_shared<Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType>>(tDataFactory);

        // ********* SET LOWER AND UPPER BOUNDS FOR CONTROLS *********
        this->setLowerBounds(tAlgebraFactory, *tDataFactory, *tDataMng);
        this->setUpperBounds(tAlgebraFactory, *tDataFactory, *tDataMng);

        // ********* SET INITIAL GUESS *********
        this->setInitialGuess(tAlgebraFactory, *tDataMng);

        // ********* SOLVE OPTIMIZATION PROBLEM *********
        this->solveOptimizationProblem(tDataMng, tDataFactory);

        this->finalize();
    }

    /******************************************************************************/
    void finalize()
    /******************************************************************************/
    {
        mInterface->getStage("Terminate");
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
        std::vector<ScalarType> tInputInitialGuessData(tNumControls);
        std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> tVector =
                aAlgebraFactory.createVector(mComm, tNumControls, mInterface);

        // ********* Set initial guess for each control vector *********
        Plato::getInitialGuessInputData(tControlName, mInputData, mInterface, tInputInitialGuessData);
        Plato::copy(tInputInitialGuessData, *tVector);
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

        // ********* ALLOCATE CCSA METHOD = GLOBALLY CONVERGENT METHOD OF MOVING ASYMPTOTES *********
        std::shared_ptr<Plato::GloballyConvergentMethodMovingAsymptotes<ScalarType, OrdinalType>> tSubProblem =
                std::make_shared<Plato::GloballyConvergentMethodMovingAsymptotes<ScalarType, OrdinalType>>(aDataFactory);

        // ********* ALLOCATE CONSERVATIVE CONVEX SEPARABLE APPROXIMATION (CCSA) ALGORITHM *********
        Plato::ConservativeConvexSeparableAppxAlgorithm<ScalarType, OrdinalType> tAlgorithm(tStageMng, aDataMng, tSubProblem);
        this->setOptions(tAlgorithm, *tSubProblem);
        tAlgorithm.solve();

        this->printStoppingReason(tAlgorithm.getStoppingCriterion());

    }

    /******************************************************************************/
    void setOptions(Plato::ConservativeConvexSeparableAppxAlgorithm<ScalarType, OrdinalType> & aAlgorithm,
                    Plato::GloballyConvergentMethodMovingAsymptotes<ScalarType, OrdinalType> & aSubProblem)
    /******************************************************************************/
    {
        // ********* Get User input ***************
        ScalarType tInnerKKTTolerance = mInputData.getGCMMAInnerKKTTolerance();
        ScalarType tOuterKKTTolerance = mInputData.getGCMMAOuterKKTTolerance();
        ScalarType tInnerControlStagnationTolerancer = mInputData.getGCMMAInnerControlStagnationTolerance();
        ScalarType tOuterControlStagnationTolerancer = mInputData.getGCMMAOuterControlStagnationTolerance();
        ScalarType tOuterObjectiveStagnationTolerancer = mInputData.getGCMMAOuterObjectiveStagnationTolerance();
        OrdinalType tMaxNumInnerIterations = mInputData.getGCMMAMaxInnerIterations();
        OrdinalType tMaxNumOuterIterations = mInputData.getMaxNumIterations();
        ScalarType tOuterStationarityTolerance = mInputData.getGCMMAOuterStationarityTolerance();
        ScalarType tInitialMovingAsymptoteScaleFactor = mInputData.getGCMMAInitialMovingAsymptoteScaleFactor();

        aSubProblem.setKarushKuhnTuckerConditionsTolerance(tInnerKKTTolerance);
        aSubProblem.setStagnationTolerance(tInnerControlStagnationTolerancer);
        aSubProblem.setMaxNumIterations(tMaxNumInnerIterations);

        aAlgorithm.setMaxNumIterations(tMaxNumOuterIterations);
        aAlgorithm.setInitialMovingAsymptoteScaleFactor(tInitialMovingAsymptoteScaleFactor);
        aAlgorithm.setKarushKuhnTuckerConditionsTolerance(tOuterKKTTolerance);
        aAlgorithm.setStationarityTolerance(tOuterStationarityTolerance);
        // Using single tolerance for both control and objective right now.
        aAlgorithm.setObjectiveStagnationTolerance(tOuterObjectiveStagnationTolerancer);
        aAlgorithm.setStagnationTolerance(tOuterControlStagnationTolerancer);
    }

    /******************************************************************************/
    void printStoppingReason(Plato::ccsa::stop_t aStopCriteria)
    /******************************************************************************/
    {

        if(aStopCriteria == Plato::ccsa::MAX_NUMBER_ITERATIONS)
        {
            fprintf(stdout,"\n\n GCMMA stopping due to exceeding maximum number of iterations\n");
        }
        else if (aStopCriteria == Plato::ccsa::KKT_CONDITIONS_TOLERANCE)
        {
            fprintf(stdout,"\n\n GCMMA stopping due to meeting KKT conditions within tolerance\n");
        }
        else if (aStopCriteria == Plato::ccsa::OBJECTIVE_STAGNATION  )
        {
            fprintf(stdout,"\n\n GCMMA stopping due to objective stagnation\n");
        }
        else if (aStopCriteria == Plato::ccsa::CONTROL_STAGNATION)
        {
            fprintf(stdout,"\n\n GCMMA stopping due to design variable stagnation\n");
        }
        else
        {
            fprintf(stdout,"\n\nGCMMA stopped for an unknown reason (%i)\n",static_cast<int>(aStopCriteria));
        }

    }

private:
    MPI_Comm mComm;
    Plato::Interface* mInterface;
    Plato::OptimizerEngineStageData mInputData;

private:
    GloballyConvergentMethodMovingAsymptotesInterface(const Plato::GloballyConvergentMethodMovingAsymptotesInterface<ScalarType, OrdinalType>&);
    Plato::GloballyConvergentMethodMovingAsymptotesInterface<ScalarType, OrdinalType> & operator=(const Plato::GloballyConvergentMethodMovingAsymptotesInterface<ScalarType, OrdinalType>&);
};

} // namespace Plato

#endif /* PLATO_GLOBALLYCONVERGENTMETHODMOVINGASYMPTOTESINTERFACE_HPP_ */
