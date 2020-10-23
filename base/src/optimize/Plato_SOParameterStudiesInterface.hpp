/*
 * Plato_SOParameterStudiesInterface.hpp
 *
 *  Created on: Oct 13, 2020
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

#ifndef PLATO_SOPARAMTERSTUDIESINTERFACE_HPP_
#define PLATO_SOPARAMTERSTUDIESINTERFACE_HPP_

#include <mpi.h>
#include <string>
#include <memory>
#include <cassert>
#include <sstream>

#include "Plato_Macros.hpp"
#include "Plato_Parser.hpp"
#include "Plato_Interface.hpp"
#include "Plato_HostBounds.hpp"
#include "Plato_Diagnostics.hpp"
#include "Plato_CommWrapper.hpp"
#include "Plato_AlgebraFactory.hpp"
#include "Plato_EngineObjective.hpp"
#include "Plato_EngineConstraint.hpp"
#include "Plato_OptimizerUtilities.hpp"
#include "Plato_OptimizerInterface.hpp"
#include "Plato_SOParameterStudies.hpp"
#include "Plato_StandardMultiVector.hpp"
#include "Plato_OptimizerEngineStageData.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class SOParameterStudiesInterface : public Plato::OptimizerInterface<ScalarType, OrdinalType>
{
public:
    explicit SOParameterStudiesInterface(Plato::Interface* aInterface, const MPI_Comm & aComm) :
            mComm(aComm),
            mInterface(aInterface),
            mInputData(Plato::OptimizerEngineStageData())
    {
    }
    virtual ~SOParameterStudiesInterface()
    {
    }

    /******************************************************************************/
    Plato::optimizer::algorithm_t type() const
    /******************************************************************************/
    {
        return (Plato::optimizer::algorithm_t::SO_PARAMETER_STUDIES);
    }
    /******************************************************************************/
    void initialize()
    /******************************************************************************/
    {
        auto tInputData = mInterface->getInputData();
        auto tOptimizationNode = tInputData.get<Plato::InputData>("Optimizer");
        Plato::initialize<ScalarType, OrdinalType>(mInterface, mInputData);
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

        // ********* ALLOCATE DERIVATIVE CHECKER BASELINE DATA STRUCTURES *********
        Plato::DataFactory<ScalarType, OrdinalType> tDataFactory;
        this->allocateBaselineDataStructures(tAlgebraFactory, tDataFactory);

        // ********* SET INITIAL GUESS AND DIAGNOSTICS OPTIONS ********* //
        bool tDidUserDefinedInitialGuess = mInputData.getUserInitialGuess();
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tInitialGuess = tDataFactory.control().create();
        if(tDidUserDefinedInitialGuess == true)
        {
            this->setInitialGuess(tAlgebraFactory, *tInitialGuess);
        }

        // ********* GET UPPER AND LOWER BOUNDS ********* //
        std::vector<ScalarType> tLowerBoundsData;
        std::vector<ScalarType> tUpperBoundsData;
        this->getControlBounds(tLowerBoundsData,tUpperBoundsData);

        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tLowerBounds = tDataFactory.control().create();
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tUpperBounds = tDataFactory.control().create();
        const OrdinalType tCONTROL_VECTOR_INDEX = 0;
        Plato::copy(tLowerBoundsData, (*tLowerBounds)[tCONTROL_VECTOR_INDEX]);
        Plato::copy(tUpperBoundsData, (*tUpperBounds)[tCONTROL_VECTOR_INDEX]);

        // ********* ALLOCATE DIAGNOSTICS TOOL ********* //
        Plato::SOParameterStudies<ScalarType, OrdinalType> tParameterStudies;

        // ********* ENFORCE BOUNDS ********* //
        Plato::HostBounds<ScalarType, OrdinalType> tProjector;
        tProjector.project(*tLowerBounds, *tUpperBounds, *tInitialGuess);

        // ********* CHECK OBJECTIVE FUNCTION ********* //
        this->doParameterStudies(tDataFactory, *tInitialGuess, tParameterStudies);

        this->finalize();
    }
    /******************************************************************************/
    void finalize()
    /******************************************************************************/
    {
        mInterface->finalize();
    }

private:
    void getControlBounds(std::vector<ScalarType> & aLowerBoundsData,
                          std::vector<ScalarType> & aUpperBoundsData)
    {
        // diagnostics does not currently need the control bounds, but if they are
        // not constructed here, then they are not used in other parts of the code.
        const OrdinalType tCONTROL_VECTOR_INDEX = 0;
        std::string tControlName = mInputData.getControlName(tCONTROL_VECTOR_INDEX);
        const OrdinalType tNumControls = mInterface->size(tControlName);
        aLowerBoundsData.resize(tNumControls);
        aUpperBoundsData.resize(tNumControls);
        Plato::getLowerBoundsInputData(mInputData, mInterface, aLowerBoundsData);
        Plato::getUpperBoundsInputData(mInputData, mInterface, aUpperBoundsData);
    }

    /******************************************************************************/
    void setInitialGuess(const Plato::AlgebraFactory<ScalarType, OrdinalType> & aAlgebraFactory,
                        Plato::MultiVector<ScalarType, OrdinalType> & aMultiVector)
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
        aMultiVector[tCONTROL_VECTOR_INDEX].update(1., *tVector, 0.);
    }
    /******************************************************************************/
    void doParameterStudies(const Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory,
                                Plato::MultiVector<ScalarType, OrdinalType> & aInitialGuess,
                                Plato::SOParameterStudies<ScalarType, OrdinalType> & aParameterStudies)
    /******************************************************************************/
    {
        Plato::EngineObjective<ScalarType, OrdinalType> tObjective(aDataFactory, mInputData, mInterface);
        this->parseConstraintReferenceValues();
        //std::string tMyName = mInputData.getConstraintValueName(0);
        Plato::EngineConstraint<ScalarType, OrdinalType> tConstraint(0, aDataFactory, mInputData, mInterface);
        aParameterStudies.doParameterStudies(tObjective, tConstraint, aInitialGuess);
    }
    /******************************************************************************/
    void parseConstraintReferenceValues()
    /******************************************************************************/
    {
        const OrdinalType tNumConstraints = mInputData.getNumConstraints();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            std::string tReferenceValueName = mInputData.getConstraintReferenceValueName(tIndex);
            if(tReferenceValueName.empty() == false)
            {
                ScalarType tReferenceValue = 0;
                Teuchos::ParameterList tArguments;
                tArguments.set(tReferenceValueName, &tReferenceValue);
                mInterface->compute(tReferenceValueName, tArguments);
                std::string tConstraintValueName = mInputData.getConstraintValueName(tIndex);
                mInputData.addConstraintReferenceValue(tConstraintValueName, tReferenceValue);
            }
        }
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

        Plato::CommWrapper tCommWrapper(mComm);
        aDataFactory.setCommWrapper(tCommWrapper);
    }

private:
    MPI_Comm mComm;
    Plato::Interface* mInterface;
    Plato::OptimizerEngineStageData mInputData;

private:
    SOParameterStudiesInterface(const Plato::SOParameterStudiesInterface<ScalarType, OrdinalType> & aRhs);
    Plato::SOParameterStudiesInterface<ScalarType, OrdinalType> & operator=(const Plato::SOParameterStudiesInterface<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_SOPARAMTERSTUDIESINTERFACE_HPP_ */
