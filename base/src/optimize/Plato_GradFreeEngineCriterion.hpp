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
 * Plato_GradFreeEngineCriterion.hpp
 *
 *  Created on: Jan 24, 2019
*/

#pragma once

#include <vector>
#include <memory>

#include "Plato_Stage.hpp"
#include "Plato_Vector.hpp"
#include "Plato_Interface.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_StageInputDataMng.hpp"
#include "Plato_GradFreeCriterion.hpp"
#include "Plato_OptimizerEngineStageData.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief PLATO Engine interface for gradient free criterion
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class GradFreeEngineCriterion : public Plato::GradFreeCriterion<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aNumControls local number of controls
     * @param [in] aNumParticles local number of particles
     * @param [in] aStageDataMng criterion stage data manager
     * @param [in] aInterface interface to data motion coordinator
    **********************************************************************************/
    explicit GradFreeEngineCriterion(const OrdinalType & aNumControls,
                                     const OrdinalType & aNumParticles,
                                     const Plato::StageInputDataMng & aStageDataMng,
                                     Plato::Interface* aInterface = nullptr) :
            mTargetValue(0.0),
            mReferenceValue(1.0),
            mNumControls(aNumControls),
            mNumParticles(aNumParticles),
            mParticles(),
            mCriterionValues(),
            mInterface(aInterface),
            mStageDataMng(aStageDataMng),
            mParameterList(std::make_shared<Teuchos::ParameterList>())
    {
        this->initialize();
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~GradFreeEngineCriterion()
    {
    }

    /******************************************************************************//**
     * @brief Set PLATO interface, coordinates data motion between applications
     * @param [in] aInterface interface to data motion coordinator
    **********************************************************************************/
    void setInterface(Plato::Interface* aInterface)
    {
        assert(aInterface != nullptr);
        mInterface = aInterface;
    }

    /******************************************************************************//**
     * @brief Set reference value, i.e. /f$ f(x) / \alpha /f$ where /f$ \alpha /f$ is the reference value
     * @param [in] aInput reference value
    **********************************************************************************/
    void setReferenceValue(const ScalarType & aInput)
    {
        mReferenceValue = aInput;
    }

    /******************************************************************************//**
     * @brief Set target value, i.e. /f$ f(x) - \alpha /f$ where /f$ \alpha /f$ is the target value
     * @param [in] aInput target value
    **********************************************************************************/
    void setTargetValue(const ScalarType & aInput)
    {
        mTargetValue = aInput;
    }

    /******************************************************************************//**
     * @brief Evaluates generic criterion through the PLATO Engine interface
     * @param [in] aControl set of particles
     * @param [out] aOutput criterion values
     **********************************************************************************/
    void value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
               Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {
        assert(mInterface != nullptr);
        this->exportParticlesSharedData(aControl);
        this->exportCriteriaSharedData();
        this->compute();
        this->importCriterionSharedData(aOutput);
    }

private:
    /******************************************************************************//**
     * @brief Allocate class member containers
    **********************************************************************************/
    void initialize()
    {
        mParticles.resize(mNumParticles);
        mCriterionValues.resize(mNumParticles);

        const OrdinalType tNumValues = 1;
        for(OrdinalType tIndex = 0; tIndex < mNumParticles; tIndex++)
        {
            mParticles[tIndex].resize(mNumControls);
            mCriterionValues[tIndex].resize(tNumValues);
        }
    }

    /******************************************************************************//**
     * @brief Compute gradient free criterion
    **********************************************************************************/
    void compute()
    {
        std::string tMyStageName = mStageDataMng.getStageName();
        std::vector<std::string> tStageNames;
        tStageNames.push_back(tMyStageName);
        mInterface->compute(tStageNames, *mParameterList);
    }

    /******************************************************************************//**
     * @brief Export particles to applications (e.g. simulation driver)
     * @param [in] aControl 2D container of optimization variables
    **********************************************************************************/
    void exportParticlesSharedData(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        std::string tMyStageName = mStageDataMng.getStageName();
        for(OrdinalType tParticleIndex = 0; tParticleIndex < mNumParticles; tParticleIndex++)
        {
            const Plato::Vector<ScalarType> & tMyControls = aControl[tParticleIndex];
            assert(mNumParticles == mParticles.size());
            assert(tMyControls.size() == mParticles[tParticleIndex].size());

            const OrdinalType tNumControls = tMyControls.size();
            for(OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                mParticles[tParticleIndex][tControlIndex] = tMyControls[tControlIndex];
            }

            std::string tMySharedDataName = mStageDataMng.getInput(tMyStageName, tParticleIndex);
            mParameterList->set(tMySharedDataName, mParticles[tParticleIndex].data());
        }
    }

    /******************************************************************************//**
     * @brief Export criterion values to application (e.g. simulation driver)
    **********************************************************************************/
    void exportCriteriaSharedData()
    {
        std::string tMyStageName = mStageDataMng.getStageName();
        for(OrdinalType tParticleIndex = 0; tParticleIndex < mNumParticles; tParticleIndex++)
        {
            assert(mNumParticles == mCriterionValues.size());
            std::fill(mCriterionValues[tParticleIndex].begin(), mCriterionValues[tParticleIndex].end(), 0.0);
            std::string tMySharedDataName = mStageDataMng.getOutput(tMyStageName, tParticleIndex);
            mParameterList->set(tMySharedDataName, mCriterionValues[tParticleIndex].data());
        }
    }

    /******************************************************************************//**
     * @brief Import criteria values from application (e.g. simulation driver)
     * @param [in] aOutput 1D container of criterion values
    **********************************************************************************/
    void importCriterionSharedData(Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {
        assert(mNumParticles == aOutput.size());
        assert(aOutput.size() == mCriterionValues.size());

        const OrdinalType tCRITERION_VALUES_INDEX = 0;
        for(OrdinalType tParticleIndex = 0; tParticleIndex < mNumParticles; tParticleIndex++)
        {
            const ScalarType tNormalizedValue = mCriterionValues[tParticleIndex][tCRITERION_VALUES_INDEX] / mReferenceValue;
            aOutput[tParticleIndex] = tNormalizedValue - mTargetValue;

        }
    }

private:
    ScalarType mTargetValue; /*!< target value, i.e. bound */
    ScalarType mReferenceValue; /*!< reference value, i.e. normalization factor */

    OrdinalType mNumControls; /*!< local number of controls */
    OrdinalType mNumParticles; /*!< local number of particles */

    std::vector<std::vector<ScalarType>> mParticles; /*!< set of particles */
    std::vector<std::vector<ScalarType>> mCriterionValues; /*!< set of criterion values */

    Plato::Interface* mInterface; /*!< interface to data motion coordinator */
    Plato::StageInputDataMng mStageDataMng; /*!< criterion stage data manager */
    std::shared_ptr<Teuchos::ParameterList> mParameterList; /*!< PLATO Engine parameter list */

private:
    GradFreeEngineCriterion(const Plato::GradFreeEngineCriterion<ScalarType, OrdinalType>&);
    Plato::GradFreeEngineCriterion<ScalarType, OrdinalType> & operator=(const Plato::GradFreeEngineCriterion<ScalarType, OrdinalType>&);
};
// class GradFreeEngineCriterion

} // namespace Plato
