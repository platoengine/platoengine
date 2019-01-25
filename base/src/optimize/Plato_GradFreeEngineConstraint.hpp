/*
 * Plato_GradFreeEngineConstraint.hpp
 *
 *  Created on: Jan 25, 2019
 */

#pragma once

#include <vector>
#include <memory>

#include "Plato_Vector.hpp"
#include "Plato_Interface.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_GradFreeCriterion.hpp"
#include "Plato_OptimizerEngineStageData.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Interface for gradient free engine constraint
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class GradFreeEngineConstraint : public Plato::GradFreeCriterion<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aDataFactory data factory
     * @param [in] aInputData data structure with engine's options and input keywords
     * @param [in] aInterface interface to data motion coordinator
    **********************************************************************************/
    explicit GradFreeEngineConstraint(const OrdinalType & aConstraintID,
                                      const Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory,
                                      const Plato::OptimizerEngineStageData & aInputData,
                                      Plato::Interface* aInterface = nullptr) :
            mMyConstraintID(aConstraintID),
            mParticles(),
            mConstraintValues(),
            mInterface(aInterface),
            mEngineInputData(aInputData),
            mParameterList(std::make_shared<Teuchos::ParameterList>())
    {
        this->initialize(aDataFactory);
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~GradFreeEngineConstraint()
    {
    }

    /******************************************************************************//**
     * @brief Set interface to data motion coordinator
     * @param [in] aInterface interface to data motion coordinator
    **********************************************************************************/
    void set(Plato::Interface* aInterface)
    {
        assert(aInterface != nullptr);
        mInterface = aInterface;
    }

    /******************************************************************************//**
     * @brief Evaluates generic objective function through the Plato Engine
     * @param [in] aControl set of particles, each particle denotes a set of optimization variables
     * @param [out] aOutput criterion value for each particle
     **********************************************************************************/
    void value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
               Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {
        assert(mInterface != nullptr);
        this->exportParticlesSharedData(aControl);
        this->exportConstraintSharedData(aOutput);
        this->compute();
        this->importConstraintSharedData(aOutput);
    }

private:
    /******************************************************************************//**
     * @brief Allocate class member containers
    **********************************************************************************/
    void initialize(const Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory)
    {
        const OrdinalType tNumControls = aDataFactory.getNumControls();
        const OrdinalType tNumParticles = aDataFactory.getNumCriterionValues();
        mParticles.resize(tNumParticles, std::vector<ScalarType>(tNumControls));
        const OrdinalType tNumCriterion = 1;
        mConstraintValues.resize(tNumParticles, std::vector<ScalarType>(tNumCriterion));
    }

    /******************************************************************************//**
     * @brief Compute gradient free objective function
    **********************************************************************************/
    void compute()
    {
        std::string tCriterionStageName = mEngineInputData.getConstraintValueStageName(mMyConstraintID);
        assert(tCriterionStageName.empty() == false);
        std::vector<std::string> tConstraintStageName;
        tConstraintStageName.push_back(tCriterionStageName);
        mInterface->compute(tConstraintStageName, *mParameterList);
    }

    /******************************************************************************//**
     * @brief Export particles to applications (e.g. simulation software)
     * @param [in] aControl 2D container of optimization variables
    **********************************************************************************/
    void exportParticlesSharedData(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        std::string tCriterionStageName = mEngineInputData.getConstraintValueStageName(mMyConstraintID);
        Plato::Stage* tConstraintStage = mInterface->getStage(tCriterionStageName);
        std::vector<std::string> tInputDataNames = tConstraintStage->getInputDataNames();

        const OrdinalType tNumParticles = aControl.getNumVectors();
        for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
        {
            const Plato::Vector<ScalarType> & tMyControls = aControl[tParticleIndex];
            assert(tMyControls.size() == mParticles[tParticleIndex].size());

            const OrdinalType tNumControls = tMyControls.size();
            for(OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                mParticles[tParticleIndex][tControlIndex] = tMyControls[tControlIndex];
            }

            std::string tControlSharedDataName = tInputDataNames[tParticleIndex];
            mParameterList->set(tControlSharedDataName, mParticles[tParticleIndex].data());
        }
    }

    /******************************************************************************//**
     * @brief Export objective function values to application (e.g. simulation software)
     * @param [in] aNumParticles number of particles
    **********************************************************************************/
    void exportConstraintSharedData(const OrdinalType & aNumParticles)
    {
        std::string tCriterionStageName = mEngineInputData.getConstraintValueStageName(mMyConstraintID);
        Plato::Stage* tConstraintStage = mInterface->getStage(tCriterionStageName);
        std::vector<std::string> tOutputDataNames = tConstraintStage->getOutputDataNames();

        for(OrdinalType tParticleIndex = 0; tParticleIndex < aNumParticles; tParticleIndex++)
        {
            assert(aNumParticles == mConstraintValues.size());
            std::fill(mConstraintValues[tParticleIndex].begin(), mConstraintValues[tParticleIndex].end(), 0.0);
            std::string tConstraintSharedDataName = tOutputDataNames[tParticleIndex];
            mParameterList->set(tConstraintSharedDataName, mConstraintValues[tParticleIndex].data());
        }
    }

    /******************************************************************************//**
     * @brief Import objective function values from application (e.g. simulation software)
     * @param [in] aOutput 1D container of objective function values
    **********************************************************************************/
    void importConstraintSharedData(Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {
        const OrdinalType tCONSTRAINT_VAL_INDEX = 0;
        const OrdinalType tNumParticles = aOutput.size();
        for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
        {
            assert(aOutput.size() == mConstraintValues.size());
            aOutput[tParticleIndex] = mConstraintValues[tParticleIndex][tCONSTRAINT_VAL_INDEX];
        }
    }

private:
    OrdinalType mMyConstraintID; /*!< constraint index */
    std::vector<std::vector<ScalarType>> mParticles; /*!< set of particles */
    std::vector<std::vector<ScalarType>> mConstraintValues; /*!< set of objective function values */

    Plato::Interface* mInterface; /*!< interface to data motion coordinator */
    Plato::OptimizerEngineStageData mEngineInputData; /*!< holds Plato Engine's options and inputs */
    std::shared_ptr<Teuchos::ParameterList> mParameterList; /*!< Plato Engine parameter list */

private:
    GradFreeEngineConstraint(const Plato::GradFreeEngineConstraint<ScalarType, OrdinalType>&);
    Plato::GradFreeEngineConstraint<ScalarType, OrdinalType> & operator=(const Plato::GradFreeEngineConstraint<ScalarType, OrdinalType>&);
};
// class GradFreeEngineObjective

} // namespace Plato
