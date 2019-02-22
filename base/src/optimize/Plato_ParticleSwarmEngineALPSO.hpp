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
 * Plato_ParticleSwarmEngineALPSO.hpp
 *
 *  Created on: Jan 26, 2019
 */

#pragma once

#include "Plato_Interface.hpp"
#include "Plato_AlgebraFactory.hpp"
#include "Plato_StageInputDataMng.hpp"
#include "Plato_OptimizerInterface.hpp"
#include "Plato_ParticleSwarmParser.hpp"
#include "Plato_GradFreeEngineCriterion.hpp"
#include "Plato_ParticleSwarmInterfaceALPSO.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief PLATO Engine interface for Augmented Lagrangian Particle Swarm Optimization (ALPSO) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class ParticleSwarmEngineALPSO : public Plato::OptimizerInterface<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aInterface PLATO Engine interface
     * @param [in] aComm local MPI communicator
    **********************************************************************************/
    explicit ParticleSwarmEngineALPSO(Plato::Interface* aInterface, const MPI_Comm & aComm) :
            mObjFuncStageName(),
            mConstraintStageNames(),
            mConstraintTargetValues(),
            mConstraintReferenceValues(),
            mComm(aComm),
            mInterface(aInterface),
            mObjFuncStageDataMng(),
            mConstraintStageDataMng()
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~ParticleSwarmEngineALPSO()
    {
    }

    /******************************************************************************//**
     * @brief Return optimization algorithm type
     * @return type
    **********************************************************************************/
    Plato::optimizer::algorithm_t type() const
    {
        return (Plato::optimizer::algorithm_t::PARTICLE_SWARM_OPTMIZATION_ALPSO);
    }

    /******************************************************************************//**
     * @brief Read input options from XML file
    **********************************************************************************/
    void initialize()
    {
    }

    /******************************************************************************//**
     * @brief Notifies PLATO Engine that optimization problem finished.
    **********************************************************************************/
    void finalize()
    {
        mInterface->finalize();
    }

    /******************************************************************************//**
     * @brief Solve optimization problem using the ALPSO algorithm
    **********************************************************************************/
    void optimize()
    {
        mInterface->handleExceptions();

        // PARSE INPUT DATA
        Plato::InputDataALPSO<ScalarType, OrdinalType> tInputsALPSO;
        this->parseOptimizerOptions(tInputsALPSO);

        // PARSE OBJECTIVE FUNCTION STAGE OPTIONS
        this->parseObjFuncStageOptions();

        // PARSE CONSTRAINT STAGE OPTIONS
        this->parseConstraintStageOptions();

        // ALLOCATE DATA CONTAINER TEMPLATES FOR OPTIMIZATION ALGORITHM
        Plato::AlgebraFactory<ScalarType, OrdinalType> tAlgebraFactory;
        this->setDataStructures(tAlgebraFactory, tInputsALPSO);

        // SET UPPER AND LOWER BOUNDS
        this->setBounds(tAlgebraFactory, tInputsALPSO);

        // SET GRADIENT FREE OBJECTIVE FUNCTION
        std::shared_ptr<Plato::GradFreeCriterion<ScalarType, OrdinalType>> tObjective;
        this->setEngineObjFuncInterface(tInputsALPSO, tObjective);

        // SET LIST OF GRADIENT FREE CONSTRAINTS
        std::shared_ptr<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>> tConstraints;
        this->setEngineConstraintInterfaces(tInputsALPSO, tConstraints);

        // SOLVE OPTIMIZATION PROBLEM
        Plato::OutputDataALPSO<ScalarType, OrdinalType> tOutputsALPSO;
        Plato::solve_alpso<ScalarType, OrdinalType>(tObjective, tConstraints, tInputsALPSO, tOutputsALPSO);

        this->finalize();
    }

private:
    /******************************************************************************//**
     * @brief Parse objective function stage options
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
     * @brief Parse problem options
     * @param [in] aInputs ALPSO algorithm inputs
    **********************************************************************************/
    void parseOptimizerOptions(Plato::InputDataALPSO<ScalarType, OrdinalType> & aInputs)
    {
        auto tInputData = mInterface->getInputData();
        auto tOptimizerNode = tInputData.get<Plato::InputData>("Optimizer");
        Plato::ParticleSwarmParser<ScalarType, OrdinalType> tParserPSO;
        mObjFuncStageName = tParserPSO.getObjectiveStageName(tOptimizerNode);
        mConstraintStageNames = tParserPSO.getConstraintStageNames(tOptimizerNode);
        mConstraintTargetValues = tParserPSO.getConstraintTargetValues(tOptimizerNode);
        mConstraintReferenceValues = tParserPSO.getConstraintReferenceValues(tOptimizerNode);
        tParserPSO.parse(tOptimizerNode, aInputs);
    }

    /******************************************************************************//**
     * @brief Set gradient free objective function for engine applications
     * @param [in] aInputs ALPSO algorithm inputs
     * @param [out] aObjective gradient free objective function interface
    **********************************************************************************/
    void setEngineObjFuncInterface(const Plato::InputDataALPSO<ScalarType, OrdinalType> & aInputs,
                                   std::shared_ptr<Plato::GradFreeCriterion<ScalarType, OrdinalType>> & aObjective)
    {
        const OrdinalType tNumParticles = aInputs.mNumParticles;
        const OrdinalType tNumControls = aInputs.mParticlesLowerBounds->size();
        std::shared_ptr<Plato::GradFreeEngineCriterion<ScalarType, OrdinalType>> tObjective =
                std::make_shared<Plato::GradFreeEngineCriterion<ScalarType, OrdinalType>>( tNumControls, tNumParticles, mObjFuncStageDataMng);
        tObjective->setInterface(mInterface);
        aObjective = tObjective;
    }

    /******************************************************************************//**
     * @brief Set list of gradient free constraints for engine applications
     * @param [in] aInputs ALPSO algorithm inputs
     * @param [out] aConstraints list of gradient free constraint interfaces
    **********************************************************************************/
    void setEngineConstraintInterfaces(const Plato::InputDataALPSO<ScalarType, OrdinalType> & aInputs,
                                       std::shared_ptr<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>> & aConstraints)
    {
        const OrdinalType tNumParticles = aInputs.mNumParticles;
        const OrdinalType tNumControls = aInputs.mParticlesLowerBounds->size();

        std::shared_ptr<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>> tConstraintList =
                std::make_shared<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>>();
        for(OrdinalType tIndex = 0; tIndex < mConstraintStageNames.size(); tIndex++)
        {
            std::shared_ptr<Plato::GradFreeEngineCriterion<ScalarType, OrdinalType>> tMyConstraint =
                    std::make_shared<Plato::GradFreeEngineCriterion<ScalarType, OrdinalType>>(tNumControls, tNumParticles, mConstraintStageDataMng[tIndex]);
            tMyConstraint->setInterface(mInterface);
            tMyConstraint->setReferenceValue(mConstraintReferenceValues[tIndex]);
            tMyConstraint->setTargetValue(mConstraintTargetValues[tIndex]);
            tConstraintList->add(tMyConstraint, aInputs.mConstraintTypes[tIndex]);
        }

        aConstraints = tConstraintList;
    }

    /******************************************************************************//**
     * @brief Set gradient free objective function for engine applications
     * @param [in] aFactory linear algebra and parallel container factory
     * @param [out] aOutput ALPSO algorithm input options struct
    **********************************************************************************/
    void setDataStructures(const Plato::AlgebraFactory<ScalarType, OrdinalType> & aFactory,
                           Plato::InputDataALPSO<ScalarType, OrdinalType> & aOutput)
    {
        this->allocateParticlesSet(aFactory, aOutput);
        aOutput.mControlReductions = aFactory.createReduction(mComm, mInterface);
        aOutput.mCriteriaEvals = std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(aOutput.mNumParticles);
        const OrdinalType tNumConstraints = mConstraintStageNames.size();
        aOutput.mDual = std::make_shared<Plato::StandardMultiVector<ScalarType>>(tNumConstraints, aOutput.mNumParticles);
    }

    /******************************************************************************//**
     * @brief Allocate template used for particle containers
     * @param [in] aFactory linear algebra and parallel container factory
     * @param [out] aOutput ALPSO algorithm input options struct
    **********************************************************************************/
    void allocateParticlesSet(const Plato::AlgebraFactory<ScalarType, OrdinalType> & aFactory,
                              Plato::InputDataALPSO<ScalarType, OrdinalType> & aInputs)
    {
        Plato::StandardMultiVector<ScalarType, OrdinalType> tParticlesSet;

        for(OrdinalType tParticleIndex = 0; tParticleIndex < aInputs.mNumParticles; tParticleIndex++)
        {
            std::string tMySharedDataName = mObjFuncStageDataMng.getInput(mObjFuncStageName, tParticleIndex);
            const OrdinalType tMyNumControls = mInterface->size(tMySharedDataName);
            std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> tMyParticle =
                    aFactory.createVector(mComm, tMyNumControls, mInterface);
            tParticlesSet.add(tMyParticle);
        }

        aInputs.mParticles = tParticlesSet.create();
    }

    /******************************************************************************//**
     * @brief Set upper and lower bounds
     * @param [in] aFactory linear algebra and parallel container factory
     * @param [out] aOutput ALPSO algorithm input options struct
    **********************************************************************************/
    void setBounds(const Plato::AlgebraFactory<ScalarType, OrdinalType> & aFactory,
                   Plato::InputDataALPSO<ScalarType, OrdinalType> & aOutput)
    {
        const OrdinalType tPARTICLE_INDEX = 0;
        const OrdinalType tNumControls = (*aOutput.mParticles)[tPARTICLE_INDEX].size();

        auto tInputData = mInterface->getInputData();
        auto tOptimizerNode = tInputData.get<Plato::InputData>("Optimizer");
        auto tBoundsNode = tOptimizerNode.get<Plato::InputData>("BoundConstraint");

        std::vector<ScalarType> tLowerBounds = Plato::Get::Doubles(tBoundsNode, "Lower");
        aOutput.mParticlesLowerBounds = aFactory.createVector(mComm, tNumControls, mInterface);
        Plato::copy(tLowerBounds, *aOutput.mParticlesLowerBounds);
        assert(tNumControls == tLowerBounds.size());

        std::vector<ScalarType> tUpperBounds = Plato::Get::Doubles(tBoundsNode, "Upper");
        aOutput.mParticlesUpperBounds = aFactory.createVector(mComm, tNumControls, mInterface);
        assert(tNumControls == tUpperBounds.size());
        Plato::copy(tUpperBounds, *aOutput.mParticlesUpperBounds);
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

private:
    ParticleSwarmEngineALPSO(const Plato::ParticleSwarmEngineALPSO<ScalarType, OrdinalType>&);
    Plato::ParticleSwarmEngineALPSO<ScalarType, OrdinalType> & operator=(const Plato::ParticleSwarmEngineALPSO<ScalarType, OrdinalType>&);
};
// class ParticleSwarmEngineALPSO

} // namespace Plato
