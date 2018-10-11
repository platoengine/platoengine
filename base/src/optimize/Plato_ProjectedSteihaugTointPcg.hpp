/*
 * Plato_ProjectedSteihaugTointPcg.hpp
 *
 *  Created on: Oct 21, 2017
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
        const bool tHaveHessian = aStageMng.getHaveHessian();
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

        if(!tHaveHessian)
        {
            // use Cauchy point

            // step = - gradient
            Plato::update(static_cast<ScalarType>(1.), *mResidual, static_cast<ScalarType>(0.), *mNewtonStep);

            // if strictly positive norm
            ScalarType tNormNewtonStep = Plato::norm(*mNewtonStep);
            if(static_cast<ScalarType>(0.) < tNormNewtonStep)
            {
                // cauchy_point = -1.0 * Trust_Region_Radius * (gradient/norm(gradient))
                ScalarType tCurrentTrustRegionRadius = this->getTrustRegionRadius();
                ScalarType tCauchyScale = tCurrentTrustRegionRadius / tNormNewtonStep;
                Plato::scale(tCauchyScale, *mNewtonStep);
            }
        }
        else
        {
            // use typical Steihaug-CG
            this->iterate(aDataMng, aStageMng);

        }

        // if zero step, use (-gradient)
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
