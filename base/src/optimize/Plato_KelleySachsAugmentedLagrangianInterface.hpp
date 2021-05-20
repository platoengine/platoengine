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
 * Plato_KelleySachsAugmentedLagrangianInterface.hpp
 *
 *  Created on: Dec 21, 2017
 */

#ifndef PLATO_KELLEYSACHSAUGMENTEDLAGRANGIANINTERFACE_HPP_
#define PLATO_KELLEYSACHSAUGMENTEDLAGRANGIANINTERFACE_HPP_

#include <cstring>

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
#include "Plato_AugmentedLagrangian.hpp"
#include "Plato_LinearCriterionHessian.hpp"
#include "Plato_OptimizerEngineStageData.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"
#include "Plato_AugmentedLagrangianStageMng.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class KelleySachsAugmentedLagrangianInterface : public Plato::OptimizerInterface<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Default constructor
     * @param [in] aInterface PLATO Engine interface
     * @param [in] local application MPI communicator
    **********************************************************************************/
    explicit KelleySachsAugmentedLagrangianInterface(Plato::Interface* aInterface, const MPI_Comm & aComm) :
            mComm(aComm),
            mInterface(aInterface),
            mInputData(Plato::OptimizerEngineStageData())
    {
    }

    /******************************************************************************//**
     * @brief Default destructor
    **********************************************************************************/
    virtual ~KelleySachsAugmentedLagrangianInterface()
    {
    }

    /******************************************************************************//**
     * @brief Return optimization algorithm used to solve optimization problem
     * @return optimization algorithm type
    **********************************************************************************/
    Plato::optimizer::algorithm_t type() const
    {
        return (Plato::optimizer::algorithm_t::KELLEY_SACHS_AUGMENTED_LAGRANGIAN);
    }

    /******************************************************************************//**
     * @brief Read input arguments from input file
    **********************************************************************************/
    void initialize()
    {
        Plato::initialize<ScalarType, OrdinalType>(mInterface, mInputData);
    }

    /******************************************************************************//**
     * @brief Solve optimization problem with Kelley-Sachs augmented Lagrangian (KSAL) algorithm
    **********************************************************************************/
    void optimize()
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

        // ********* SET INITIAL GUESS ********* //
        this->setInitialGuess(tAlgebraFactory, *tDataMng);

        // ********* SET UPPER AND LOWER BOUNDS ********* //
        this->setLowerBounds(tAlgebraFactory, *tDataFactory, *tDataMng);
        this->setUpperBounds(tAlgebraFactory, *tDataFactory, *tDataMng);

        // ********* SOLVE OPTIMIZATION PROBLEM ********* //
        this->solveOptimizationProblem(tDataFactory, tDataMng);

        // ********* OUTPUT SOLUTION *********
        Plato::call_finalization_stage(mInterface, mInputData);

        this->finalize();
    }

    /******************************************************************************//**
     * @brief Notification from optimizer stating that problem has been solved.
    **********************************************************************************/
    void finalize()
    {
        mInterface->finalize();
    }

private:
    /******************************************************************************//**
     * @brief Set numerical method use to compute application of vector to Hessian operator
     * @param [in,out] aAlgorithm interface to optimization algorithm
     **********************************************************************************/
    void setHessianMethod(Plato::AugmentedLagrangian<ScalarType, OrdinalType> & aAlgorithm)
    {
        if(mInputData.getHessianType() == "lbfgs")
        {
            aAlgorithm.setHaveHessian(true);
            aAlgorithm.setCriteriaHessiansLBFGS(mInputData.getLimitedMemoryStorage());
        }
        else if(mInputData.getHessianType() == "analytical")
        {
            aAlgorithm.setHaveHessian(true);
        }
        else
        {
            aAlgorithm.setHaveHessian(false);
        }
    }

    /******************************************************************************//**
     * @brief Solve optimization problem
     * @param [in] aDataFactory linear algebra factory
     * @param [in] aDataMng optimizer data manager
    **********************************************************************************/
    void solveOptimizationProblem(std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aDataFactory,
                                  std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType>> & aDataMng)
    {
        // ********* ALLOCATE OBJECTIVE FUNCTION ********* //
        std::shared_ptr<Plato::EngineObjective<ScalarType, OrdinalType>> tObjective =
                std::make_shared<Plato::EngineObjective<ScalarType, OrdinalType>>(*aDataFactory, mInputData, mInterface);

        // ********* ALLOCATE CONSTRAINT LIST ********* //
        const OrdinalType tNumConstraints = mInputData.getNumConstraints();
        std::vector<std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>>> tConstraintList(tNumConstraints);
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            tConstraintList[tIndex] = std::make_shared<Plato::EngineConstraint<ScalarType, OrdinalType>>(tIndex, *aDataFactory, mInputData, mInterface);
        }
        std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> tConstraints =
                std::make_shared<Plato::CriterionList<ScalarType, OrdinalType>>();
        tConstraints->add(tConstraintList);

        // ********* ALLOCATE STAGE MANAGER FOR AUGMENTED LAGRANGIAN FORMULATION ********* //
        std::shared_ptr<Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>> tStageMng =
                std::make_shared<Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>>(aDataFactory, tObjective, tConstraints);

        // ********* ALLOCATE KELLEY-SACHS AUGMENTED LAGRANGIAN TRUST REGION ALGORITHM ********* //
        Plato::AugmentedLagrangian<ScalarType, OrdinalType> tAlgorithm(aDataFactory, aDataMng, tStageMng);
        this->setAlgorithmParams(tAlgorithm);
        tAlgorithm.solve();
    }

    /******************************************************************************//**
     * @brief Set Kelley-Sachs Augmented Lagrangian (KSAL) trust region algorithm inputs
     * @param [in,out] aAlgorithm Kelley-Sachs Augmented Lagrangian trust region algorithm interface
    **********************************************************************************/
    void setAlgorithmParams(Plato::AugmentedLagrangian<ScalarType, OrdinalType> & aAlgorithm)
    {
        this->setHessianMethod(aAlgorithm);

        aAlgorithm.enableDiagnostics();
        aAlgorithm.disablePostSmoothing();
        aAlgorithm.setMeanNorm(mInputData.getMeanNorm());

        aAlgorithm.setPenaltyParameter(mInputData.getAugLagPenaltyParameter());
        aAlgorithm.setPenaltyParameterScaleFactor(mInputData.getAugLagPenaltyScaleParameter());

        aAlgorithm.setMaxNumOuterIterations(mInputData.getMaxNumIterations());
        aAlgorithm.setMaxNumAugLagSubProbIter(mInputData.getMaxNumAugLagSubProbIter());
        aAlgorithm.setMaxNumTrustRegionSubProblemIterations(mInputData.getKSMaxTrustRegionIterations());

        aAlgorithm.setMaxTrustRegionRadius(mInputData.getMaxTrustRegionRadius());
        aAlgorithm.setMinTrustRegionRadius(mInputData.getMinTrustRegionRadius());
        aAlgorithm.setTrustRegionExpansion(mInputData.getKSTrustRegionExpansionFactor());
        aAlgorithm.setTrustRegionContraction(mInputData.getKSTrustRegionContractionFactor());

        aAlgorithm.setFeasibilityTolerance(mInputData.getFeasibilityTolerance());
        aAlgorithm.setGradientTolerance(mInputData.getKSOuterGradientTolerance());
        aAlgorithm.setStationarityTolerance(mInputData.getKSOuterStationarityTolerance());
        aAlgorithm.setActualReductionTolerance(mInputData.getKSOuterActualReductionTolerance());
        aAlgorithm.setAugLagActualReductionTolerance(mInputData.getKSOuterStagnationTolerance());
        aAlgorithm.setControlStagnationTolerance(mInputData.getKSOuterControlStagnationTolerance());
    }

    /******************************************************************************/
    void setUpperBounds(const Plato::AlgebraFactory<ScalarType, OrdinalType> & aAlgebraFactory,
                        Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory,
                        Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
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
