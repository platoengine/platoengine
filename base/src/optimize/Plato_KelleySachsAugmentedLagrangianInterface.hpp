/*
 * Plato_KelleySachsAugmentedLagrangianInterface.hpp
 *
 *  Created on: Dec 21, 2017
 */

#ifndef PLATO_KELLEYSACHSAUGMENTEDLAGRANGIANINTERFACE_HPP_
#define PLATO_KELLEYSACHSAUGMENTEDLAGRANGIANINTERFACE_HPP_

#include <string>
#include <memory>

#include "Plato_Parser.hpp"
#include "Plato_Interface.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_CriterionList.hpp"
#include "Plato_AlgebraFactory.hpp"
#include "Plato_EngineObjective.hpp"
#include "Plato_EngineConstraint.hpp"
#include "Plato_OptimizerUtilities.hpp"
#include "Plato_OptimizerInterface.hpp"
#include "Plato_StandardMultiVector.hpp"
#include "Plato_LinearCriterionHessian.hpp"
#include "Plato_OptimizerEngineStageData.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"
#include "Plato_AugmentedLagrangianStageMng.hpp"
#include "Plato_KelleySachsAugmentedLagrangian.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class KelleySachsAugmentedLagrangianInterface : public Plato::OptimizerInterface<ScalarType, OrdinalType>
{
public:
    /******************************************************************************/
    explicit KelleySachsAugmentedLagrangianInterface(Plato::Interface* aInterface, const MPI_Comm & aComm) :
            mComm(aComm),
            mInterface(aInterface),
            mInputData(Plato::OptimizerEngineStageData())
    /******************************************************************************/
    {
    }

    /******************************************************************************/
    virtual ~KelleySachsAugmentedLagrangianInterface()
    /******************************************************************************/
    {
    }

    /******************************************************************************/
    Plato::optimizer::algorithm_t type() const
    /******************************************************************************/
    {
        return (Plato::optimizer::algorithm_t::KELLEY_SACHS_AUGMENTED_LAGRANGIAN);
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

        // ********* ALLOCATE BASELINE DATA STRUCTURES FOR OPTIMIZER ********* //
        std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> tDataFactory =
                std::make_shared<Plato::DataFactory<ScalarType, OrdinalType>>();
        this->allocateBaselineDataStructures(tAlgebraFactory, *tDataFactory);

        // ********* ALLOCATER DATA MANAGER ********* //
        std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType>> tDataMng =
                std::make_shared<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType>>(tDataFactory);

        // ********* SET UPPER AND LOWER BOUNDS ********* //
        this->setLowerBounds(tAlgebraFactory, *tDataFactory, *tDataMng);
        this->setUpperBounds(tAlgebraFactory, *tDataFactory, *tDataMng);

        // ********* SET INITIAL GUESS ********* //
        this->setInitialGuess(tAlgebraFactory, *tDataMng);

        // ********* ALLOCATE OBJECTIVE FUNCTION ********* //
        std::shared_ptr<Plato::EngineObjective<ScalarType, OrdinalType>> tObjective =
                std::make_shared<Plato::EngineObjective<ScalarType, OrdinalType>>(*tDataFactory, mInputData, mInterface);

        // ********* ALLOCATE CONSTRAINT LIST ********* //
        const OrdinalType tNumConstraints = mInputData.getNumConstraints();
        std::vector<std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>>> tConstraintList(tNumConstraints);
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            tConstraintList[tIndex] = std::make_shared<Plato::EngineConstraint<ScalarType, OrdinalType>>(tIndex, *tDataFactory, mInputData, mInterface);
        }
        std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> tConstraints =
                std::make_shared<Plato::CriterionList<ScalarType, OrdinalType>>();
        tConstraints->add(tConstraintList);

        // ********* ALLOCATE STAGE MANAGER FOR AUGMENTED LAGRANGIAN FORMULATION ********* //
        std::shared_ptr<Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>> tStageMng =
                std::make_shared<Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>>(tDataFactory, tObjective, tConstraints);

        // ********* ALLOCATE HESSIAN FOR EACH CONSTRAINT ********* //
        std::shared_ptr<Plato::LinearOperatorList<ScalarType, OrdinalType>> tConstraintHessianList =
                std::make_shared<Plato::LinearOperatorList<ScalarType, OrdinalType>>();
        this->setConstraintHessianList(*tConstraints, *tConstraintHessianList);
        tStageMng->setConstraintHessians(tConstraintHessianList);

        // ********* SET OBJECTIVE HESSIANS TO IDENTITY IF HESSIAN NAME IS NOT DEFINED ********* //
        // *********    IN THE OBJECTIVE SUBBLOCK DEFINED INSIDE THE OPTIMIZER BLOCK   ********* //
        if(mInputData.getObjectiveHessianOutputName().empty() == true)
        {
            tStageMng->setIdentityObjectiveHessian();
        }

        // ********* ALLOCATE KELLEY-SACHS AUGMENTED LAGRANGIAN TRUST REGION ALGORITHM ********* //
        Plato::KelleySachsAugmentedLagrangian<ScalarType, OrdinalType> tAlgorithm(tDataFactory, tDataMng, tStageMng);
        OrdinalType tMaxNumIterations = mInputData.getMaxNumIterations();
        tAlgorithm.setMaxNumIterations(tMaxNumIterations);
        tAlgorithm.solve();

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
                        Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
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
                        Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
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
    void setConstraintHessianList(const Plato::CriterionList<ScalarType, OrdinalType> & aConstraints,
                                  Plato::LinearOperatorList<ScalarType, OrdinalType> & aHessianList)
    /******************************************************************************/
    {
        const OrdinalType tNumConstraints = mInputData.getNumConstraints();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            std::string tMyHessianName = mInputData.getConstraintHessianName(tIndex);
            if(tMyHessianName.compare("LinearCriterionHessian") == static_cast<int>(0))
            {
                aHessianList.add(std::make_shared<Plato::LinearCriterionHessian<ScalarType, OrdinalType>>());
            }
            else
            {
                aHessianList.add(std::make_shared<Plato::AnalyticalHessian<ScalarType, OrdinalType>>(aConstraints.ptr(tIndex)));
            }
        }
    }

    /******************************************************************************/
    void setInitialGuess(const Plato::AlgebraFactory<ScalarType, OrdinalType> & aAlgebraFactory,
                         Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
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
        // ********* Allocate dual vectors baseline data structures *********
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

private:
    MPI_Comm mComm;
    Plato::Interface* mInterface;
    Plato::OptimizerEngineStageData mInputData;

private:
    KelleySachsAugmentedLagrangianInterface(const Plato::KelleySachsAugmentedLagrangianInterface<ScalarType, OrdinalType>&);
    Plato::KelleySachsAugmentedLagrangianInterface<ScalarType, OrdinalType> & operator=(const Plato::KelleySachsAugmentedLagrangianInterface<ScalarType, OrdinalType>&);
};
// class KelleySachsAugmentedLagrangianInterface

} // namespace Plato

#endif /* PLATO_KELLEYSACHSAUGMENTEDLAGRANGIANINTERFACE_HPP_ */
