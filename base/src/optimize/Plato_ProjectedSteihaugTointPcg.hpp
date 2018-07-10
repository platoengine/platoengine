/*
 * Plato_ProjectedSteihaugTointPcg.hpp
 *
 *  Created on: Oct 21, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_PROJECTEDSTEIHAUGTOINTPCG_HPP_
#define PLATO_PROJECTEDSTEIHAUGTOINTPCG_HPP_

#include <memory>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_SteihaugTointSolver.hpp"
#include "Plato_TrustRegionStageMng.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class ProjectedSteihaugTointPcg : public Plato::SteihaugTointSolver<ScalarType, OrdinalType>
{
public:
    explicit ProjectedSteihaugTointPcg(const Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory) :
            Plato::SteihaugTointSolver<ScalarType, OrdinalType>(),
            mResidual(aDataFactory.control().create()),
            mNewtonStep(aDataFactory.control().create()),
            mCauchyStep(aDataFactory.control().create()),
            mWorkVector(aDataFactory.control().create()),
            mActiveVector(aDataFactory.control().create()),
            mInactiveVector(aDataFactory.control().create()),
            mConjugateDirection(aDataFactory.control().create()),
            mPrecTimesNewtonStep(aDataFactory.control().create()),
            mInvPrecTimesResidual(aDataFactory.control().create()),
            mPrecTimesConjugateDirection(aDataFactory.control().create()),
            mHessTimesConjugateDirection(aDataFactory.control().create())
    {
    }
    virtual ~ProjectedSteihaugTointPcg()
    {
    }

    void solve(Plato::TrustRegionStageMng<ScalarType, OrdinalType> & aStageMng,
               Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        OrdinalType tNumVectors = aDataMng.getNumControlVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            (*mNewtonStep)[tVectorIndex].fill(0);
            (*mConjugateDirection)[tVectorIndex].fill(0);

            const Plato::Vector<ScalarType, OrdinalType> & tCurrentGradient = aDataMng.getCurrentGradient(tVectorIndex);
            (*mResidual)[tVectorIndex].update(static_cast<ScalarType>(-1.), tCurrentGradient, static_cast<ScalarType>(0.));
            const Plato::Vector<ScalarType, OrdinalType> & tInactiveSet = aDataMng.getInactiveSet(tVectorIndex);
            (*mResidual)[tVectorIndex].entryWiseProduct(tInactiveSet);
        }
        ScalarType tNormResidual = Plato::norm(*mResidual);
        this->setNormResidual(tNormResidual);

        this->iterate(aDataMng, aStageMng);

        ScalarType tNormNewtonStep = Plato::norm(*mNewtonStep);
        if(tNormNewtonStep <= static_cast<ScalarType>(0.))
        {
            for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
            {
                const Plato::Vector<ScalarType, OrdinalType> & tCurrentGradient = aDataMng.getCurrentGradient(tVectorIndex);
                (*mNewtonStep)[tVectorIndex].update(static_cast<ScalarType>(-1.), tCurrentGradient, static_cast<ScalarType>(0.));
                const Plato::Vector<ScalarType, OrdinalType> & tInactiveSet = aDataMng.getInactiveSet(tVectorIndex);
                (*mNewtonStep)[tVectorIndex].entryWiseProduct(tInactiveSet);
            }
        }
        aDataMng.setTrialStep(*mNewtonStep);
    }

private:
    void iterate(const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng,
                 Plato::TrustRegionStageMng<ScalarType, OrdinalType> & aStageMng)
    {
        ScalarType tPreviousTau = 0;
        ScalarType tNormResidual = this->getNormResidual();
        ScalarType tCurrentTrustRegionRadius = this->getTrustRegionRadius();

        OrdinalType tIteration = 0;
        OrdinalType tMaxNumIterations = this->getMaxNumIterations();
        while(this->toleranceSatisfied(tNormResidual) == false)
        {
            tIteration++;
            if(tIteration > tMaxNumIterations)
            {
                tIteration = tIteration - static_cast<OrdinalType>(1);
                this->setStoppingCriterion(Plato::krylov_solver::stop_t::MAX_ITERATIONS);
                break;
            }
            this->applyVectorToInvPreconditioner(aDataMng, *mResidual, aStageMng, *mInvPrecTimesResidual);
            //compute scaling
            ScalarType tCurrentTau = Plato::dot(*mResidual, *mInvPrecTimesResidual);
            if(tIteration > 1)
            {
                ScalarType tBeta = tCurrentTau / tPreviousTau;
                Plato::update(static_cast<ScalarType>(1.), *mInvPrecTimesResidual, tBeta, *mConjugateDirection);
            }
            else
            {
                Plato::update(static_cast<ScalarType>(1.), *mInvPrecTimesResidual, static_cast<ScalarType>(0.), *mConjugateDirection);
            }
            this->applyVectorToHessian(aDataMng, *mConjugateDirection, aStageMng, *mHessTimesConjugateDirection);
            ScalarType tCurvature = Plato::dot(*mConjugateDirection, *mHessTimesConjugateDirection);
            if(this->invalidCurvatureDetected(tCurvature) == true)
            {
                // compute scaled inexact trial step
                ScalarType tScaling = this->step(aDataMng, aStageMng);
                Plato::update(tScaling, *mConjugateDirection, static_cast<ScalarType>(1.), *mNewtonStep);
                break;
            }
            ScalarType tRayleighQuotient = tCurrentTau / tCurvature;
            Plato::update(-tRayleighQuotient, *mHessTimesConjugateDirection, static_cast<ScalarType>(1.), *mResidual);
            Plato::update(tRayleighQuotient, *mConjugateDirection, static_cast<ScalarType>(1.), *mNewtonStep);
            tNormResidual = Plato::norm(*mResidual);
            if(this->toleranceSatisfied(tNormResidual) == true)
            {
                break;
            }
            if(tIteration == static_cast<OrdinalType>(1))
            {
                Plato::update(static_cast<ScalarType>(1.), *mNewtonStep, static_cast<ScalarType>(0.), *mCauchyStep);
            }
            ScalarType tNormNewtonStep = Plato::norm(*mNewtonStep);
            if(tNormNewtonStep > tCurrentTrustRegionRadius)
            {
                // compute scaled inexact trial step
                ScalarType tScaleFactor = this->step(aDataMng, aStageMng);
                Plato::update(tScaleFactor, *mConjugateDirection, static_cast<ScalarType>(1), *mNewtonStep);
                this->setStoppingCriterion(Plato::krylov_solver::stop_t::TRUST_REGION_RADIUS);
                break;
            }
            tPreviousTau = tCurrentTau;
        }
        this->setNumIterationsDone(tIteration);
    }
    ScalarType step(const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng,
                     Plato::TrustRegionStageMng<ScalarType, OrdinalType> & aStageMng)
    {
        this->applyVectorToPreconditioner(aDataMng, *mNewtonStep, aStageMng, *mPrecTimesNewtonStep);
        this->applyVectorToPreconditioner(aDataMng, *mConjugateDirection, aStageMng, *mPrecTimesConjugateDirection);

        ScalarType tScaleFactor = this->computeSteihaugTointStep(*mNewtonStep,
                                                                  *mConjugateDirection,
                                                                  *mPrecTimesNewtonStep,
                                                                  *mPrecTimesConjugateDirection);

        return (tScaleFactor);
    }
    void applyVectorToHessian(const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng,
                              const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                              Plato::TrustRegionStageMng<ScalarType, OrdinalType> & aStageMng,
                              Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aVector.getNumVectors() > static_cast<OrdinalType>(0));
        assert(aVector.getNumVectors() == aOutput.getNumVectors());

        const OrdinalType tNumVectors = aVector.getNumVectors();

        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            // Set Active Vector
            (*mActiveVector)[tVectorIndex].update(static_cast<ScalarType>(1.), aVector[tVectorIndex], static_cast<ScalarType>(0.));
            const Plato::Vector<ScalarType, OrdinalType> & tActiveSet = aDataMng.getActiveSet(tVectorIndex);
            (*mActiveVector)[tVectorIndex].entryWiseProduct(tActiveSet);
            // Set Inactive Vector
            (*mInactiveVector)[tVectorIndex].update(static_cast<ScalarType>(1.), aVector[tVectorIndex], static_cast<ScalarType>(0.));
            const Plato::Vector<ScalarType, OrdinalType> & tInactiveSet = aDataMng.getInactiveSet(tVectorIndex);
            (*mInactiveVector)[tVectorIndex].entryWiseProduct(tInactiveSet);
            aOutput[tVectorIndex].fill(0);
        }

        const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = aDataMng.getCurrentControl();
        aStageMng.applyVectorToHessian(tCurrentControl, *mInactiveVector, aOutput);

        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tInactiveSet = aDataMng.getInactiveSet(tVectorIndex);
            aOutput[tVectorIndex].entryWiseProduct(tInactiveSet);
            aOutput[tVectorIndex].update(static_cast<ScalarType>(1.), (*mActiveVector)[tVectorIndex], static_cast<ScalarType>(1.));
        }
    }
    void applyVectorToPreconditioner(const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng,
                                     const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                                     Plato::TrustRegionStageMng<ScalarType, OrdinalType> & aStageMng,
                                     Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aVector.getNumVectors() > static_cast<OrdinalType>(0));
        assert(aVector.getNumVectors() == aOutput.getNumVectors());

        const OrdinalType tNumVectors = aVector.getNumVectors();

        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            // Set Active Vector
            (*mActiveVector)[tVectorIndex].update(static_cast<ScalarType>(1.), aVector[tVectorIndex], static_cast<ScalarType>(0.));
            const Plato::Vector<ScalarType, OrdinalType> & tActiveSet = aDataMng.getActiveSet(tVectorIndex);
            (*mActiveVector)[tVectorIndex].entryWiseProduct(tActiveSet);
            // Set Inactive Vector
            (*mInactiveVector)[tVectorIndex].update(static_cast<ScalarType>(1.), aVector[tVectorIndex], static_cast<ScalarType>(0.));
            const Plato::Vector<ScalarType, OrdinalType> & tInactiveSet = aDataMng.getInactiveSet(tVectorIndex);
            (*mInactiveVector)[tVectorIndex].entryWiseProduct(tInactiveSet);
            aOutput[tVectorIndex].fill(0);
        }

        const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = aDataMng.getCurrentControl();
        aStageMng.applyVectorToPreconditioner(tCurrentControl, *mInactiveVector, aOutput);

        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tInactiveSet = aDataMng.getInactiveSet(tVectorIndex);
            aOutput[tVectorIndex].entryWiseProduct(tInactiveSet);
            aOutput[tVectorIndex].update(static_cast<ScalarType>(1.), (*mActiveVector)[tVectorIndex], static_cast<ScalarType>(1.));
        }
    }
    void applyVectorToInvPreconditioner(const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng,
                                        const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                                        Plato::TrustRegionStageMng<ScalarType, OrdinalType> & aStageMng,
                                        Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aVector.getNumVectors() > static_cast<OrdinalType>(0));
        assert(aVector.getNumVectors() == aOutput.getNumVectors());

        const OrdinalType tNumVectors = aVector.getNumVectors();

        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            // Set Active Vector
            (*mActiveVector)[tVectorIndex].update(static_cast<ScalarType>(1.), aVector[tVectorIndex], static_cast<ScalarType>(0.));
            const Plato::Vector<ScalarType, OrdinalType> & tActiveSet = aDataMng.getActiveSet(tVectorIndex);
            (*mActiveVector)[tVectorIndex].entryWiseProduct(tActiveSet);
            // Set Inactive Vector
            (*mInactiveVector)[tVectorIndex].update(static_cast<ScalarType>(1.), aVector[tVectorIndex], static_cast<ScalarType>(0.));
            const Plato::Vector<ScalarType, OrdinalType> & tInactiveSet = aDataMng.getInactiveSet(tVectorIndex);
            (*mInactiveVector)[tVectorIndex].entryWiseProduct(tInactiveSet);
            aOutput[tVectorIndex].fill(0);
        }

        const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = aDataMng.getCurrentControl();
        aStageMng.applyVectorToInvPreconditioner(tCurrentControl, *mInactiveVector, aOutput);

        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tInactiveSet = aDataMng.getInactiveSet(tVectorIndex);
            aOutput[tVectorIndex].entryWiseProduct(tInactiveSet);
            aOutput[tVectorIndex].update(static_cast<ScalarType>(1.), (*mActiveVector)[tVectorIndex], static_cast<ScalarType>(1.));
        }
    }

private:
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mResidual;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mNewtonStep;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCauchyStep;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mWorkVector;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mActiveVector;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mInactiveVector;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mConjugateDirection;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPrecTimesNewtonStep;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mInvPrecTimesResidual;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPrecTimesConjugateDirection;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mHessTimesConjugateDirection;

private:
    ProjectedSteihaugTointPcg(const Plato::ProjectedSteihaugTointPcg<ScalarType, OrdinalType> & aRhs);
    Plato::ProjectedSteihaugTointPcg<ScalarType, OrdinalType> & operator=(const Plato::ProjectedSteihaugTointPcg<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_PROJECTEDSTEIHAUGTOINTPCG_HPP_ */
