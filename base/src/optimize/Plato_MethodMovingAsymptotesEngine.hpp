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
 * Plato_MethodMovingAsymptotesEngine.hpp
 *
 *  Created on: Sep 1, 2019
 */

#pragma once

#include "Plato_Interface.hpp"
#include "Plato_AlgebraFactory.hpp"
#include "Plato_EngineObjective.hpp"
#include "Plato_EngineConstraint.hpp"
#include "Plato_OptimizerInterface.hpp"
#include "Plato_OptimizerUtilities.hpp"
#include "Plato_MethodMovingAsymptotesParser.hpp"
#include "Plato_MethodMovingAsymptotesInterface.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief PLATO Engine interface for Method of Moving AsymptotesEngine (MMA) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class MethodMovingAsymptotesEngine : public Plato::OptimizerInterface<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aInterface PLATO Engine interface
     * @param [in] aComm local MPI communicator
    **********************************************************************************/
    explicit MethodMovingAsymptotesEngine(Plato::Interface *aInterface, const MPI_Comm &aComm) :
        mObjFuncStageName(),
        mConstraintStageNames(),
        mConstraintTargetValues(),
        mConstraintReferenceValues(),
        mComm(aComm),
        mInterface(aInterface),
        mObjFuncStageDataMng(),
        mConstraintStageDataMng(),
        mInputData(Plato::OptimizerEngineStageData())
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~MethodMovingAsymptotesEngine()
    {
    }

    /******************************************************************************//**
     * @brief Return the algorithm type
     * @return algorithm type
    **********************************************************************************/
    Plato::optimizer::algorithm_t type() const
    {
        return (Plato::optimizer::algorithm_t::METHOD_OF_MOVING_ASYMPTOTES);
    }

    /******************************************************************************//**
     * @brief Read Engine stage inputs
    **********************************************************************************/
    void initialize()
    {
        Plato::initialize<ScalarType, OrdinalType>(mInterface, mInputData);
    }

    /******************************************************************************//**
     * @brief Notify PLATO Engine that optimization problem is done.
    **********************************************************************************/
    void finalize()
    {
        // OUTPUT SOLUTION
        Plato::call_finalization_stage(mInterface, mInputData);

        // DEALLOCATE MEMORY
        mInterface->finalize();
    }

    /******************************************************************************//**
     * @brief Solve optimization problem using the MMA optimization algorithm
    **********************************************************************************/
    void optimize()
    {
        mInterface->handleExceptions();
        this->initialize();

        // PARSE INPUT DATA
        Plato::AlgorithmInputsMMA<ScalarType, OrdinalType> tInputs;
        this->parseOptimizerOptions(tInputs);

        // PARSE OBJECTIVE FUNCTION STAGE OPTIONS
        this->parseObjFuncStageOptions();

        // PARSE CONSTRAINT STAGE OPTIONS
        this->parseConstraintStageOptions();

        // ALLOCATE DATA CONTAINER TEMPLATES FOR OPTIMIZATION ALGORITHM
        Plato::AlgebraFactory<ScalarType, OrdinalType> tAlgebraFactory;
        this->allocateDataStructures(tAlgebraFactory, tInputs);

        // SET INITIAL GUESS AND CONSTRAINT NORMALIZATION VALUES
        this->setInitialGuess(tInputs);
        this->setConstraintNormalizationValues(tInputs);

        // SET UPPER AND LOWER BOUNDS
        this->setLowerBounds(tInputs);
        this->setUpperBounds(tInputs);

        // SET PROBLEM CRITERIA
        std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> tConstraints;
        std::shared_ptr<Plato::EngineObjective<ScalarType, OrdinalType>> tObjective;
        const OrdinalType tNumConstraints = tInputs.mConstraintNormalizationParams->size();
        const OrdinalType tNumControls = (*tInputs.mUpperBounds)[0 /* vector index */].size();
        this->setProblemCriteria(tNumControls, tNumConstraints, tObjective, tConstraints);

        // SOLVE OPTIMIZATION PROBLEM
        Plato::AlgorithmOutputsMMA<ScalarType, OrdinalType> tOutputs;
        Plato::solve_mma<ScalarType, OrdinalType>(tObjective, tConstraints, tInputs, tOutputs);

        this->finalize();
    }

private:
    /******************************************************************************//**
     * @brief Parse algorithmic options
     * @param [in/out] aInputs MMA algorithm inputs
    **********************************************************************************/
    void parseOptimizerOptions(Plato::AlgorithmInputsMMA<ScalarType, OrdinalType> & aInput)
    {
        auto tInputData = mInterface->getInputData();
        auto tOptimizerNode = tInputData.get<Plato::InputData>("Optimizer");
        Plato::MethodMovingAsymptotesParser<ScalarType, OrdinalType> tParser;
        mObjFuncStageName = tParser.getObjectiveStageName(tOptimizerNode);
        mConstraintStageNames = tParser.getConstraintStageNames(tOptimizerNode);
        mConstraintTargetValues = tParser.getConstraintTargetValues(tOptimizerNode);
        mConstraintReferenceValues = tParser.getConstraintReferenceValues(tOptimizerNode);
        tParser.parse(tOptimizerNode, aInput);
    }

    /******************************************************************************//**
     * @brief Parse options for objective function stage
    **********************************************************************************/
    void parseObjFuncStageOptions()
    {
        auto tInputData = mInterface->getInputData();
        auto tStages = tInputData.getByName<Plato::InputData>("Stage");
        for(auto tStageNode = tStages.begin(); tStageNode != tStages.end(); ++tStageNode)
        {
            std::string tStageName = tStageNode->get<std::string>("Name");
            if(tStageName == mObjFuncStageName)
            {
                Plato::Parse::parseStageData(*tStageNode, mObjFuncStageDataMng);
            }
        }
    }

    /******************************************************************************//**
     * @brief Parse constraint stage options
    **********************************************************************************/
    void parseConstraintStageOptions()
    {
        auto tInputData = mInterface->getInputData();
        auto tStages = tInputData.getByName<Plato::InputData>("Stage");

        const OrdinalType tNumConstraints = mConstraintStageNames.size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            for(auto tStageNode = tStages.begin(); tStageNode != tStages.end(); ++tStageNode)
            {
                std::string tStageName = tStageNode->get<std::string>("Name");
                if(tStageName == mConstraintStageNames[tIndex])
                {
                    mConstraintStageDataMng.push_back(Plato::StageInputDataMng());
                    Plato::Parse::parseStageData(*tStageNode, mConstraintStageDataMng[tIndex]);
                }
            }
        }
    }

    /******************************************************************************//**
     * @brief Allocate control and dual container templates
     * @param [in] aFactory linear algebra and parallel container factory
     * @param [out] aData MMA algorithm input options
    **********************************************************************************/
    void allocateDataStructures(const Plato::AlgebraFactory<ScalarType, OrdinalType> &aFactory,
                                Plato::AlgorithmInputsMMA<ScalarType, OrdinalType> &aData)
    {
        this->allocateControlSets(aFactory, aData);
        const OrdinalType tNumConstraints = mConstraintStageNames.size();
        aData.mConstraintNormalizationParams = std::make_shared<Plato::StandardVector<ScalarType>>(tNumConstraints);
        aData.mCommWrapper = Plato::CommWrapper(mComm);
        aData.mControlReductionOperations = aFactory.createReduction(mComm, mInterface);
    }

    /******************************************************************************//**
     * @brief Allocate template used for optimization variable containers
     * @param [in] aFactory linear algebra and parallel container factory
     * @param [out] aData MMA algorithm input options
    **********************************************************************************/
    void allocateControlSets(const Plato::AlgebraFactory<ScalarType, OrdinalType> &aFactory,
                             Plato::AlgorithmInputsMMA<ScalarType, OrdinalType> &aData)
    {
        Plato::StandardMultiVector<ScalarType, OrdinalType> tControlSet;
        for(OrdinalType tVectorIndex = 0; tVectorIndex < aData.mNumControlVectors; tVectorIndex++)
        {
            const std::string & tMySharedDataName = mObjFuncStageDataMng.getInput(mObjFuncStageName, tVectorIndex);
            const OrdinalType tNumControls = mInterface->size(tMySharedDataName);
            std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> tVector =
                    aFactory.createVector(mComm, tNumControls, mInterface);
            tControlSet.add(tVector);
        }
        aData.mLowerBounds = tControlSet.create();
        aData.mUpperBounds = tControlSet.create();
        aData.mInitialGuess = tControlSet.create();
    }

    /******************************************************************************//**
     * @brief Set bounds on optimization variables
     * @param [in/out] aData MMA algorithm input options
    **********************************************************************************/
    void setLowerBounds(Plato::AlgorithmInputsMMA<ScalarType, OrdinalType> &aData)
    {
        const OrdinalType tNumControls = (*aData.mLowerBounds)[0 /* vector index */].size();
        std::vector<ScalarType> tBoundsData(tNumControls);
        Plato::getLowerBoundsInputData(mInputData, mInterface, tBoundsData);
        for(OrdinalType tVecIndex = 0; tVecIndex < aData.mNumControlVectors; tVecIndex++)
        {
            Plato::copy(tBoundsData, (*aData.mLowerBounds)[tVecIndex]);
        }
    }

    /******************************************************************************//**
     * @brief Set bounds on optimization variables
     * @param [in/out] aData MMA algorithm input options
    **********************************************************************************/
    void setUpperBounds(Plato::AlgorithmInputsMMA<ScalarType, OrdinalType> &aData)
    {
        const OrdinalType tNumControls = (*aData.mUpperBounds)[0 /* vector index */].size();
        std::vector<ScalarType> tBoundsData(tNumControls);
        Plato::getUpperBoundsInputData(mInputData, mInterface, tBoundsData);
        for(OrdinalType tVecIndex = 0; tVecIndex < aData.mNumControlVectors; tVecIndex++)
        {
            Plato::copy(tBoundsData, (*aData.mUpperBounds)[tVecIndex]);
        }
    }

    /******************************************************************************//**
     * @brief Set constraint normalization parameters
     * @param [in/out] aData MMA algorithm input options
    **********************************************************************************/
    void setConstraintNormalizationValues(Plato::AlgorithmInputsMMA<ScalarType, OrdinalType> &aData)
    {
        const OrdinalType tNumConstraints = aData.mConstraintNormalizationParams->size();
        if(aData.mConstraintNormalizationMultipliers.empty() == true)
        {
            aData.mConstraintNormalizationMultipliers = std::vector<ScalarType>(tNumConstraints, 1.0);
        }

        for (OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            ScalarType tValue = aData.mConstraintNormalizationMultipliers[tIndex] * mConstraintTargetValues[tIndex];
            (*aData.mConstraintNormalizationParams)[tIndex] =
                std::abs(tValue) <= std::numeric_limits<ScalarType>::epsilon() ? static_cast<ScalarType>(1) : tValue;
        }
    }

    /******************************************************************************//**
     * @brief Set initial optimization variables, i,e, initial guess
     * @param [in/out] aData MMA algorithm input options
    **********************************************************************************/
    void setInitialGuess(Plato::AlgorithmInputsMMA<ScalarType, OrdinalType> &aData)
    {
        const OrdinalType tNumControls = (*aData.mInitialGuess)[0 /* vector index */].size();
        std::vector<ScalarType> tData(tNumControls);
        for(OrdinalType tVecIndex = 0; tVecIndex < aData.mNumControlVectors; tVecIndex++)
        {
            const std::string & tMySharedDataName = mObjFuncStageDataMng.getInput(mObjFuncStageName, tVecIndex);
            Plato::getInitialGuessInputData(tMySharedDataName, mInputData, mInterface, tData);
            Plato::copy(tData, (*aData.mInitialGuess)[tVecIndex]);
            std::fill(tData.begin(), tData.end(), static_cast<ScalarType>(0));
        }
    }

    /******************************************************************************//**
     * @brief Set optimization problem criteria
     * @param [in] aNumControls number of controls per control vector
     * @param [in] aNumConstraints number of constraints
     * @param [in/out] aObjective PLATO engine interface for the objective function
     * @param [in/out] aConstraints PLATO engine interface for the constraints
    **********************************************************************************/
    void setProblemCriteria(const OrdinalType& aNumControls,
                            const OrdinalType& aNumConstraints,
                            std::shared_ptr<Plato::EngineObjective<ScalarType, OrdinalType>>& aObjective,
                            std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>>& aConstraints)
    {
        // SET OBJECTIVE FUNCTION
        aObjective = std::make_shared<Plato::EngineObjective<ScalarType, OrdinalType>>(mInputData, mInterface);
        aObjective->allocateControlContainers(aNumControls);

        // SET CONSTRAINTS
        aConstraints = std::make_shared<Plato::CriterionList<ScalarType, OrdinalType>>();
        for(OrdinalType tIndex = 0; tIndex < aNumConstraints; tIndex++)
        {
            std::shared_ptr<Plato::EngineConstraint<ScalarType, OrdinalType>> tConstraint;
            tConstraint = std::make_shared<Plato::EngineConstraint<ScalarType, OrdinalType>>(tIndex, mInputData, mInterface);
            tConstraint->allocateControlContainers(aNumControls);
            aConstraints->add(tConstraint);
        }
    }

private:
    std::string mObjFuncStageName; /*!< objective function stage name */
    std::vector<std::string> mConstraintStageNames; /*!< list of stage names for all constraints */
    std::vector<ScalarType> mConstraintTargetValues; /*!< list of target values for all constraints */
    std::vector<ScalarType> mConstraintReferenceValues; /*!< list of reference values for all constraints */

    MPI_Comm mComm; /*!< MPI communicator */
    Plato::Interface* mInterface; /*!< PLATO Engine interface */
    Plato::StageInputDataMng mObjFuncStageDataMng; /*!< objective function stage data manager */
    std::vector<Plato::StageInputDataMng> mConstraintStageDataMng; /*!< constraint stage data manager */
    Plato::OptimizerEngineStageData mInputData;

private:
    MethodMovingAsymptotesEngine(const Plato::MethodMovingAsymptotesEngine<ScalarType, OrdinalType>&);
    Plato::MethodMovingAsymptotesEngine<ScalarType, OrdinalType> & operator=(const Plato::MethodMovingAsymptotesEngine<ScalarType, OrdinalType>&);
};
// class MethodMovingAsymptotesEngine

}
// namespace Plato
