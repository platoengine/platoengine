/*
 * Plato_KelleySachsBoundConstrainedInterface.hpp
 *
 *  Created on: Dec 21, 2017
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

#ifndef PLATO_KELLEYSACHSBOUNDCONSTRAINEDINTERFACE_HPP_
#define PLATO_KELLEYSACHSBOUNDCONSTRAINEDINTERFACE_HPP_

#include <string>
#include <memory>
#include <sstream>

#include "Plato_Parser.hpp"
#include "Plato_Interface.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_CriterionList.hpp"
#include "Plato_AlgebraFactory.hpp"
#include "Plato_EngineObjective.hpp"
#include "Plato_OptimizerInterface.hpp"
#include "Plato_StandardMultiVector.hpp"
#include "Plato_OptimizerEngineStageData.hpp"
#include "Plato_KelleySachsBoundConstrained.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"
#include "Plato_ReducedSpaceTrustRegionStageMng.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class KelleySachsBoundConstrainedInterface : public Plato::OptimizerInterface<ScalarType, OrdinalType>
{
public:
    /******************************************************************************/
    explicit KelleySachsBoundConstrainedInterface(Plato::Interface* aInterface,
                                                  const MPI_Comm & aComm,
                                                  const Plato::optimizer::algorithm_t & aType) :
            mComm(aComm),
            mInterface(aInterface),
            mType(aType),
            mInputData(Plato::OptimizerEngineStageData())
    /******************************************************************************/
    {
    }

    /******************************************************************************/
    virtual ~KelleySachsBoundConstrainedInterface()
    /******************************************************************************/
    {
    }

    /******************************************************************************/
    Plato::optimizer::algorithm_t type() const
    /******************************************************************************/
    {
        return (mType);
    }

    /******************************************************************************/
    void initialize()
    /******************************************************************************/
    {
        auto tInputData = mInterface->getInputData();
        auto tOptimizationNode = tInputData.get<Plato::InputData>("Optimizer");
        Plato::Parse::parseOptimizerStages(tOptimizationNode, mInputData);
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

        // ********* ALLOCATE DATA MANAGER ********* //
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
        std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> tCriterionList =
                std::make_shared<Plato::CriterionList<ScalarType, OrdinalType>>();
        tCriterionList->add(tObjective);

        // ********* ALLOCATE REDUCED SPACE FORMULATION MANAGER FOR TRUST REGION ALGORITHM ********* //
        std::shared_ptr<Plato::ReducedSpaceTrustRegionStageMng<ScalarType, OrdinalType>> tStageMng =
                std::make_shared<Plato::ReducedSpaceTrustRegionStageMng<ScalarType, OrdinalType>>(tDataFactory, tCriterionList);

        // ********* SET OBJECTIVE HESSIANS TO IDENTITY IF HESSIAN NAME IS NOT DEFINED ********* //
        // *********    IN THE OBJECTIVE SUBBLOCK DEFINED INSIDE THE OPTIMIZER BLOCK   ********* //
        if(mInputData.getObjectiveHessianOutputName().empty() == true)
        {
            tStageMng->setIdentityObjectiveHessian();
        }

        // ********* ALLOCATE KELLEY-SACHS BOUND CONSTRAINED OPTIMIZATION ALGORITHM ********* //
        Plato::KelleySachsBoundConstrained<ScalarType, OrdinalType> tAlgorithm(tDataFactory, tDataMng, tStageMng);
        this->setOptions(tAlgorithm);
        tAlgorithm.solve();

        this->printOutputFile(tAlgorithm, *tDataMng);

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
    void setOptions(Plato::KelleySachsBoundConstrained<ScalarType, OrdinalType> & aAlgorithm)
    /******************************************************************************/
    {
        OrdinalType tMaxNumIterations = mInputData.getMaxNumIterations();
        OrdinalType tMaxTrustRegionIterations = mInputData.getKSMaxTrustRegionIterations();
        ScalarType tContractionScaleFactor = mInputData.getKSTrustRegionContractionFactor();
        ScalarType tExpansionScaleFactor = mInputData.getKSTrustRegionExpansionFactor();
        ScalarType tOuterGradientTolerance = mInputData.getKSOuterGradientTolerance();
        ScalarType tOuterStationarityTolerance = mInputData.getKSOuterStationarityTolerance();
        ScalarType tOuterStagnationTolerance = mInputData.getKSOuterStagnationTolerance();
        ScalarType tOuterControlStagnationTolerance = mInputData.getKSOuterControlStagnationTolerance();
        ScalarType tOuterActualReductionTolerance = mInputData.getKSOuterActualReductionTolerance();

        aAlgorithm.setMaxNumIterations(tMaxNumIterations);
        aAlgorithm.setTrustRegionContraction(tContractionScaleFactor);
        aAlgorithm.setTrustRegionExpansion(tExpansionScaleFactor);
        aAlgorithm.setMaxNumTrustRegionSubProblemIterations(tMaxTrustRegionIterations);
        aAlgorithm.setGradientTolerance(tOuterGradientTolerance);
        aAlgorithm.setStationarityTolerance(tOuterStationarityTolerance);
        aAlgorithm.setStagnationTolerance(tOuterStagnationTolerance);
        aAlgorithm.setControlStagnationTolerance(tOuterControlStagnationTolerance);
        aAlgorithm.setActualReductionTolerance(tOuterActualReductionTolerance);
    }
    /******************************************************************************/


    /******************************************************************************/
    void setUpperBounds(const Plato::AlgebraFactory<ScalarType, OrdinalType> & aAlgebraFactory,
                        Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory,
                        Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
    /******************************************************************************/
    {

        // ********* SET UPPER BOUNDS *********
        const OrdinalType tCONTROL_VECTOR_INDEX = 0;
        std::string tControlName = mInputData.getControlName(tCONTROL_VECTOR_INDEX);
        const OrdinalType tNumControls = mInterface->size(tControlName);
        std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> tUpperBoundVector =
                aAlgebraFactory.createVector(mComm, tNumControls, mInterface);

        ScalarType tUpperBoundValue = std::numeric_limits<ScalarType>::max();
        tUpperBoundVector->fill(tUpperBoundValue);
        aDataFactory.allocateUpperBoundVector(*tUpperBoundVector);

        if(mType == Plato::optimizer::algorithm_t::KELLEY_SACHS_BOUND_CONSTRAINED)
        {
            // ********* GET UPPER BOUNDS INFORMATION *********
            std::vector<ScalarType> tInputBoundsData(tNumControls);
            Plato::getUpperBoundsInputData(mInputData, mInterface, tInputBoundsData);
            Plato::copy(tInputBoundsData, *tUpperBoundVector);
        } // ELSE, PROBLEM IS UNCONSTRAINED

        aDataMng.setControlUpperBounds(tCONTROL_VECTOR_INDEX, *tUpperBoundVector);
    }

    /******************************************************************************/
    void setLowerBounds(const Plato::AlgebraFactory<ScalarType, OrdinalType> & aAlgebraFactory,
                        Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory,
                        Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
    /******************************************************************************/
    {
        // ********* SET LOWER BOUNDS *********
        const OrdinalType tCONTROL_VECTOR_INDEX = 0;
        std::string tControlName = mInputData.getControlName(tCONTROL_VECTOR_INDEX);
        const OrdinalType tNumControls = mInterface->size(tControlName);
        std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> tLowerBoundVector =
                aAlgebraFactory.createVector(mComm, tNumControls, mInterface);

        ScalarType tLowerBoundValue = static_cast<ScalarType>(-1) * std::numeric_limits<ScalarType>::max();
        tLowerBoundVector->fill(tLowerBoundValue);
        aDataFactory.allocateLowerBoundVector(*tLowerBoundVector);

        if(mType == Plato::optimizer::algorithm_t::KELLEY_SACHS_BOUND_CONSTRAINED)
        {
            // ********* GET LOWER BOUNDS INFORMATION *********
            std::vector<ScalarType> tInputBoundsData(tNumControls);
            Plato::getLowerBoundsInputData(mInputData, mInterface, tInputBoundsData);
            Plato::copy(tInputBoundsData, *tLowerBoundVector);
        } // ELSE, PROBLEM IS UNCONSTRAINED

        aDataMng.setControlLowerBounds(tCONTROL_VECTOR_INDEX, *tLowerBoundVector);
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

    /*****************************************************************************/
    void printOutputFile(const Plato::KelleySachsBoundConstrained<ScalarType, OrdinalType> & aAlgorithm,
                         const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
    /*****************************************************************************/
    {
        bool tOutputDiagnostics = mInputData.getOutputDiagnosticsToFile();
        if(tOutputDiagnostics == true)
        {
            std::ofstream tOutputFile;
            tOutputFile.open("solution.txt");

            std::string tStopCriterion;
            Plato::get_stop_criterion(aAlgorithm.getStoppingCriterion(), tStopCriterion);
            tOutputFile << tStopCriterion.c_str();

            tOutputFile << "Number of Outer Iterations = " << aAlgorithm.getNumIterationsDone() << "\n";
            tOutputFile << "Objective Function Value = " << aDataMng.getCurrentObjectiveFunctionValue() << "\n\n";

            bool tOutputControl = mInputData.getOutputControlToFile();
            if(tOutputControl == true)
            {
                const Plato::MultiVector<ScalarType, OrdinalType> & tControl = aDataMng.getCurrentControl();
                for(OrdinalType tIndex = 0; tIndex < tControl.getNumVectors(); tIndex++)
                {
                    tOutputFile << "Solution Vector " << tIndex + static_cast<OrdinalType>(1) << "\n";
                    const Plato::Vector<ScalarType, OrdinalType> & tMyControl = tControl[tIndex];
                    for(OrdinalType tControlIndex = 0; tControlIndex < tMyControl.size(); tControlIndex++)
                    {
                        tOutputFile << "[" << tControlIndex << "] = " << tMyControl[tControlIndex] << "\n";
                    }
                    tOutputFile << "\n";
                }
            }

            tOutputFile.close();
        }
    }

private:
    MPI_Comm mComm;
    Plato::Interface* mInterface;
    Plato::optimizer::algorithm_t mType;
    Plato::OptimizerEngineStageData mInputData;

private:
    KelleySachsBoundConstrainedInterface(const Plato::KelleySachsBoundConstrainedInterface<ScalarType, OrdinalType>&);
    Plato::KelleySachsBoundConstrainedInterface<ScalarType, OrdinalType> & operator=(const Plato::KelleySachsBoundConstrainedInterface<ScalarType, OrdinalType>&);
}; // class KelleySachsBoundConstrainedInterface

} //namespace Plato

#endif /* PLATO_KELLEYSACHSBOUNDCONSTRAINEDINTERFACE_HPP_ */
