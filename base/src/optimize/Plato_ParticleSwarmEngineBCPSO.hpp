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
 * Plato_ParticleSwarmEngineBCPSO.hpp
 *
 *  Created on: Jan 25, 2019
 */

#pragma once

#include "Plato_Interface.hpp"
#include "Plato_AlgebraFactory.hpp"
#include "Plato_OptimizerInterface.hpp"
#include "Plato_OptimizerUtilities.hpp"
#include "Plato_ParticleSwarmParser.hpp"
#include "Plato_GradFreeEngineCriterion.hpp"
#include "Plato_OptimizerEngineStageData.hpp"
#include "Plato_ParticleSwarmInterfaceBCPSO.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief PLATO Engine interface for Bound Constrained Particle Swarm Optimization (BCPSO) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class ParticleSwarmEngineBCPSO : public Plato::OptimizerInterface<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aInterface PLATO Engine interface
     * @param [in] aComm local MPI communicator
    **********************************************************************************/
    explicit ParticleSwarmEngineBCPSO(Plato::Interface* aInterface, const MPI_Comm & aComm) :
            mComm(aComm),
            mInterface(aInterface),
            mInputData(Plato::OptimizerEngineStageData())
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~ParticleSwarmEngineBCPSO()
    {
    }

    /******************************************************************************//**
     * @brief Return optimization algorithm type
     * @return type
    **********************************************************************************/
    Plato::optimizer::algorithm_t type() const
    {
        return (Plato::optimizer::algorithm_t::PARTICLE_SWARM_OPTMIZATION_BCPSO);
    }

    /******************************************************************************//**
     * @brief Read input options from XML file
    **********************************************************************************/
    void initialize()
    {
        Plato::initialize<ScalarType, OrdinalType>(mInterface, mInputData);
    }

    /******************************************************************************//**
     * @brief Notifies PLATO Engine that optimization problem finished.
    **********************************************************************************/
    void finalize()
    {
        mInterface->getStage("Terminate");
    }

    /******************************************************************************//**
     * @brief Solve optimization problem using the BCPSO algorithm
    **********************************************************************************/
    void optimize()
    {
        mInterface->handleExceptions();

        this->initialize();

        // PARSE INPUT DATA
        auto tInputData = mInterface->getInputData();
        Plato::InputDataBCPSO<ScalarType, OrdinalType> tInputsBCPSO;
        Plato::ParticleSwarmParser<ScalarType, OrdinalType> tParserPSO;
        tParserPSO.parse(tInputData, tInputsBCPSO);

        // ALLOCATE DATA CONTAINER TEMPLATES FOR OPTIMIZATION ALGORITHM
        Plato::AlgebraFactory<ScalarType, OrdinalType> tAlgebraFactory;
        this->setDataStructures(tAlgebraFactory, tInputsBCPSO);

        // SET UPPER AND LOWER BOUNDS
        this->setBounds(tAlgebraFactory, tInputsBCPSO);

        // DEFINE GRADIENT FREE OBJECTIVE FUNCTION
        std::shared_ptr<Plato::GradFreeCriterion<ScalarType, OrdinalType>> tObjective;
        this->setEngineObjFunc(tInputsBCPSO, tObjective);

        // SOLVE OPTIMIZATION PROBLEM
        Plato::OutputDataBCPSO<ScalarType, OrdinalType> tOutputsBCPSO;
        Plato::solve_bcpso<ScalarType, OrdinalType>(tObjective, tInputsBCPSO, tOutputsBCPSO);

        this->finalize();
    }

private:
    /******************************************************************************//**
     * @brief Set gradient free objective function for engine applications
     * @param [in] aInputs BCPSO algorithm inputs
     * @param [out] aObjective gradient free objective function interface
    **********************************************************************************/
    void setEngineObjFunc(const Plato::InputDataBCPSO<ScalarType, OrdinalType> & aInputs,
                          std::shared_ptr<Plato::GradFreeCriterion<ScalarType, OrdinalType>> & aObjective)
    {
        const OrdinalType tNumControls = aInputs.mNumParticles;
        const OrdinalType tNumParticles = aInputs.mNumParticles;
        std::string tStageName = mInputData.getObjectiveValueStageName();
        std::shared_ptr<Plato::GradFreeEngineCriterion<ScalarType, OrdinalType>> tObjective =
                std::make_shared<Plato::GradFreeEngineCriterion<ScalarType, OrdinalType>>(tStageName, tNumControls, tNumParticles);
        tObjective->set(mInterface);
        aObjective = tObjective;
    }

    /******************************************************************************//**
     * @brief Set gradient free objective function for engine applications
     * @param [in] aFactory linear algebra and parallel container factory
     * @param [out] aOutput BCPSO algorithm input options struct
    **********************************************************************************/
    void setDataStructures(const Plato::AlgebraFactory<ScalarType, OrdinalType> & aFactory,
                           Plato::InputDataBCPSO<ScalarType, OrdinalType> & aOutput)
    {
        this->allocateParticlesSet(aFactory, aOutput);
        aOutput.mControlReductions = aFactory.createReduction(mComm, mInterface);
        aOutput.mCriteriaEvals = std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(aOutput.mNumParticles);
    }

    /******************************************************************************//**
     * @brief Allocate template used for particle containers
     * @param [in] aFactory linear algebra and parallel container factory
     * @param [out] aOutput BCPSO algorithm input options struct
    **********************************************************************************/
    void allocateParticlesSet(const Plato::AlgebraFactory<ScalarType, OrdinalType> & aFactory,
                              Plato::InputDataBCPSO<ScalarType, OrdinalType> & aInputs)
    {
        std::string tStageName = mInputData.getObjectiveValueStageName();
        Plato::Stage* tObjFuncStage = mInterface->getStage(tStageName);
        std::vector<std::string> tInputDataNames = tObjFuncStage->getInputDataNames();

        Plato::StandardMultiVector<ScalarType, OrdinalType> tParticlesSet;
        for(OrdinalType tParticleIndex = 0; tParticleIndex < aInputs.mNumParticles; tParticleIndex++)
        {
            std::string tControlSharedDataName = tInputDataNames[tParticleIndex];
            const OrdinalType tNumControls = mInterface->size(tControlSharedDataName);
            std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> tVector =
                    aFactory.createVector(mComm, tNumControls, mInterface);
            tParticlesSet.add(tVector);
        }

        aInputs.mParticles = tParticlesSet.create();
    }

    /******************************************************************************//**
     * @brief Set upper and lower bounds
     * @param [in] aFactory linear algebra and parallel container factory
     * @param [out] aOutput BCPSO algorithm input options struct
    **********************************************************************************/
    void setBounds(const Plato::AlgebraFactory<ScalarType, OrdinalType> & aFactory,
                   Plato::InputDataBCPSO<ScalarType, OrdinalType> & aOutput)
    {
        const OrdinalType tPARTICLE_INDEX = 0;
        const OrdinalType tNumControls = (*aOutput.mParticles)[tPARTICLE_INDEX].size();
        std::vector<ScalarType> tBounds(tNumControls);

        Plato::getLowerBoundsInputData(mInputData, mInterface, tBounds);
        aOutput.mParticlesLowerBounds = aFactory.createVector(mComm, tNumControls, mInterface);
        Plato::copy(tBounds, *aOutput.mParticlesLowerBounds);

        std::fill(tBounds.begin(), tBounds.end(), static_cast<ScalarType>(0));
        Plato::getUpperBoundsInputData(mInputData, mInterface, tBounds);
        aOutput.mParticlesUpperBounds = aFactory.createVector(mComm, tNumControls, mInterface);
        Plato::copy(tBounds, *aOutput.mParticlesUpperBounds);

    }

private:
    MPI_Comm mComm; /*!< MPI communicator */
    Plato::Interface* mInterface; /*!< PLATO Engine interface */
    Plato::OptimizerEngineStageData mInputData; /*!< input data structure */

private:
    ParticleSwarmEngineBCPSO(const Plato::ParticleSwarmEngineBCPSO<ScalarType, OrdinalType>&);
    Plato::ParticleSwarmEngineBCPSO<ScalarType, OrdinalType> & operator=(const Plato::ParticleSwarmEngineBCPSO<ScalarType, OrdinalType>&);
};
// class Plato_ParticleSwarmEngineBCPSO

} // namespace Plato
