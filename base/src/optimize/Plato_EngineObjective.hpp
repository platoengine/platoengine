/*
 * Plato_EngineObjective.hpp
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

#ifndef PLATO_ENGINEOBJECTIVE_HPP_
#define PLATO_ENGINEOBJECTIVE_HPP_

#include <vector>
#include <string>
#include <memory>
#include <cassert>

#include "Plato_Interface.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DistributedVector.hpp"
#include "Plato_OptimizerEngineStageData.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class EngineObjective : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    explicit EngineObjective(const Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory,
                             const Plato::OptimizerEngineStageData & aInputData,
                             Plato::Interface* aInterface = nullptr) :
            mVector(std::vector<ScalarType>(aDataFactory.getNumControls())),
            mControl(std::vector<ScalarType>(aDataFactory.getNumControls())),
            mGradient(std::vector<ScalarType>(aDataFactory.getNumControls())),
            mHessianTimesVector(std::vector<ScalarType>(aDataFactory.getNumControls())),
            mInterface(aInterface),
            mEngineInputData(aInputData),
            mParameterList(std::make_shared<Teuchos::ParameterList>())
    {
    }
    virtual ~EngineObjective()
    {
    }
    /*!
     * Set pointer to Plato engine interface, which provides a link between the user's application and the
     * Plato engine optimizer solving the optimization problem.
     **/
    void setPlatoInterface(Plato::Interface* aInterface)
    {
        assert(aInterface != nullptr);
        mInterface = aInterface;
    }

    //! Directive to cache any criterion specific data once trial control is accepted.
    void cacheData()
    {
        assert(mInterface != nullptr);

        // Tell performers to cache the state
        std::vector<std::string> tStageNames;
        std::string tCacheStageName = mEngineInputData.getCacheStageName();
        if(tCacheStageName.empty() == false)
        {
            tStageNames.push_back(tCacheStageName);
            mInterface->compute(tStageNames, *mParameterList);
        }

        // Temporarily putting output here.
        tStageNames.clear();
        std::string tOutputStageName = mEngineInputData.getOutputStageName();
        if(tOutputStageName.empty() == false)
        {
            tStageNames.push_back(tOutputStageName);
            mInterface->compute(tStageNames, *mParameterList);
        }
    }

    //! Directive to update problem criterion.
    void updateProblem(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(mInterface != nullptr);

        this->setControls(aControl);
        // Tell performers to cache the state
        std::vector<std::string> tStageNames;
        std::string tUpdateProblemName = mEngineInputData.getUpdateProblemStageName();
        if(tUpdateProblemName.empty() == false)
        {
            tStageNames.push_back(tUpdateProblemName);
            mInterface->compute(tStageNames, *mParameterList);
        }
    }

    /*!
     * Evaluates criterion of type f(\mathbf{u}(\mathbf{z}),\mathbf{z})\colon\mathbb{R}^{n_u}\times\mathbb{R}^{n_z}
     * \rightarrow\mathbb{R}, where u denotes the state and z denotes the control variables. This criterion
     * is typically associated with nonlinear programming optimization problems. For instance, PDE constrasize_t
     * optimization problems.
     *  Parameters:
     *    \param In
     *          aControl: control variables (i.e. design variables)
     *
     *  \return Objective function value
     **/
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(mInterface != nullptr);

        // ********* Set view to each control vector entry ********* //
        this->setControls(aControl);

        // ********* Set view to objective function value ********* //
        ScalarType tObjectiveValue = 0;
        std::string tValueOutputName = mEngineInputData.getObjectiveValueOutputName();
        mParameterList->set(tValueOutputName, &tObjectiveValue);

        // ********* Compute objective function value ********* //
        std::string tMyStageName = mEngineInputData.getObjectiveValueStageName();
        assert(tMyStageName.empty() == false);
        std::vector<std::string> tStageNames;
        tStageNames.push_back(tMyStageName);
        mInterface->compute(tStageNames, *mParameterList);

        return (tObjectiveValue);
    }
    /*!
     * Computes the gradient of a criterion of type f(\mathbf{u}(\mathbf{z}),\mathbf{z})\colon\mathbb{R}^{n_u}
     * \times\mathbb{R}^{n_z}\rightarrow\mathbb{R}, where u denotes the state and z denotes the control variables.
     * This criterion is typically associated with nonlinear programming optimization problems. For instance, PDE
     * constraint optimization problems.
     *  Parameters:
     *    \param In
     *          aControl: control variables (i.e. design variables)
     *    \param Out
     *          aOutput: gradient
     **/
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(mInterface != nullptr);
        // ********* Set view to each control vector entry ********* //
        this->setControls(aControl);

        // ********* Set view to each output vector ********* //
        std::string tGradientOutputName = mEngineInputData.getObjectiveGradientOutputName();
        std::fill(mGradient.begin(), mGradient.end(), static_cast<ScalarType>(0));
        mParameterList->set(tGradientOutputName, mGradient.data());

        // ********* Compute objective function gradient ********* //
        std::string tMyStageName = mEngineInputData.getObjectiveGradientStageName();
        assert(tMyStageName.empty() == false);
        std::vector<std::string> tStageNames;
        tStageNames.push_back(tMyStageName);
        mInterface->compute(tStageNames, *mParameterList);
        this->copy(mGradient, aOutput);
    }
    /*!
     * Computes the application of a vector to the Hessian of a criterion of type f(\mathbf{u}(\mathbf{z}),\mathbf{z})
     * \colon\mathbb{R}^{n_u}\times\mathbb{R}^{n_z}\rightarrow\mathbb{R}, where u denotes the state and z denotes the
     * control variables. This criterion is typically associated with nonlinear programming optimization problems.
     * For instance, PDE constraint optimization problems.
     *  Parameters:
     *    \param In
     *          aControl: control variables (i.e. design variables)
     *    \param In
     *          aVector:  vector
     *    \param Out
     *          aOutput:  Hessian times vector
     **/
    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                 const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(mInterface != nullptr);
        // ********* Set view to each control and descent direction vector entry ********* //
        const OrdinalType tControlVectorIndex = 0;
        assert(mVector.size() == mControl.size());
        assert(aVector[tControlVectorIndex].size() == mVector.size());
        assert(aControl[tControlVectorIndex].size() == mControl.size());
        for(OrdinalType tControlIndex = 0; tControlIndex < mControl.size(); tControlIndex++)
        {
            mVector[tControlIndex] = aVector(tControlVectorIndex, tControlIndex);
            mControl[tControlIndex] = aControl(tControlVectorIndex, tControlIndex);
        }
        std::string tControlName = mEngineInputData.getControlName(tControlVectorIndex);
        mParameterList->set(tControlName, mControl.data());
        std::string tDescentDirectionlName = mEngineInputData.getDescentDirectionName(tControlVectorIndex);
        mParameterList->set(tDescentDirectionlName, mVector.data());

        // ********* Set view to each output vector ********* //
        std::string tHessianOutputName = mEngineInputData.getObjectiveHessianOutputName();
        std::fill(mHessianTimesVector.begin(), mHessianTimesVector.end(), static_cast<ScalarType>(0));
        mParameterList->set(tHessianOutputName, mHessianTimesVector.data());

        // ********* Apply vector to Hessian operator ********* //
        std::string tMyStageName = mEngineInputData.getObjectiveHessianStageName();
        assert(tMyStageName.empty() == false);
        std::vector<std::string> tStageNames;
        tStageNames.push_back(tMyStageName);
        mInterface->compute(tStageNames, *mParameterList);
        this->copy(mHessianTimesVector, aOutput);
    }

private:
    /******************************************************************************//**
     * @brief Copy elements in standard vector into a Plato multi-vector
     * @param [in] aFrom input data
     * @param [out] aTo output data
    **********************************************************************************/
    void copy(const std::vector<ScalarType> & aFrom, Plato::MultiVector<ScalarType, OrdinalType> & aTo)
    {
        const OrdinalType tVectorIndex = 0;
        Plato::Vector<ScalarType, OrdinalType> & tToVector = aTo[tVectorIndex];
        assert(tToVector.size() == aFrom.size());
        for(OrdinalType tIndex = 0; tIndex < aFrom.size(); tIndex++)
        {
            tToVector[tIndex] = aFrom[tIndex];
        }
    }

    /******************************************************************************//**
     * @brief Make control variables available to the parameter list
     * @param [in] aControl design variables
    **********************************************************************************/
    void setControls(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        // ********* Set view to each control vector entry ********* //
        const OrdinalType tControlVectorIndex = 0;
        assert(aControl[tControlVectorIndex].size() == mControl.size());
        for(OrdinalType tControlIndex = 0; tControlIndex < mControl.size(); tControlIndex++)
        {
            mControl[tControlIndex] = aControl(tControlVectorIndex, tControlIndex);
        }
        std::string tControlName = mEngineInputData.getControlName(tControlVectorIndex);
        mParameterList->set(tControlName, mControl.data());
    }

private:
    std::vector<ScalarType> mVector; /*!< local copy of a descent direction vector */
    std::vector<ScalarType> mControl; /*!< local copy of the control variables */
    std::vector<ScalarType> mGradient; /*!< local copy of the gradient vector */
    std::vector<ScalarType> mHessianTimesVector; /*!< local copy of the application of a vector to the Hessian */

    Plato::Interface* mInterface; /*!< PLATO Engine interface */
    Plato::OptimizerEngineStageData mEngineInputData; /*!< Parsed input data */
    std::shared_ptr<Teuchos::ParameterList> mParameterList; /*!< parameter list with data to be communicated through the PLATO Engine interface */

private:
    EngineObjective(const Plato::EngineObjective<ScalarType, OrdinalType>&);
    Plato::EngineObjective<ScalarType, OrdinalType> & operator=(const Plato::EngineObjective<ScalarType, OrdinalType>&);
}; // class EngineObjective

} // namespace Plato

#endif /* PLATO_ENGINEOBJECTIVE_HPP_ */
