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
 * Plato_GradFreeEngineObjective.hpp
 *
 *  Created on: Jan 24, 2019
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
 * @brief Interface for gradient free algebraic rocket criterion
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class GradFreeEngineObjective : public Plato::GradFreeCriterion<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aDataFactory data factory
     * @param [in] aInputData data structure with engine's options and input keywords
     * @param [in] aInterface interface to data motion coordinator
    **********************************************************************************/
    explicit GradFreeEngineObjective(const Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory,
                                     const Plato::OptimizerEngineStageData & aInputData,
                                     Plato::Interface* aInterface = nullptr) :
            mControl(std::vector<ScalarType>(aDataFactory.getNumControls())),
            mObjFuncValues(std::vector<ScalarType>(aDataFactory.getNumCriterionValues())),
            mInterface(aInterface),
            mEngineInputData(aInputData),
            mParameterList(std::make_shared<Teuchos::ParameterList>())
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
     * @brief Destructor
    **********************************************************************************/
    virtual ~GradFreeEngineObjective()
    {
    }

    /******************************************************************************//**
     * @brief Evaluates generic objective function through the Plato Engine
     * @param [in] aControl set of particles, each particle denotes a set of optimization variables
     * @param [out] aOutput criterion value for each particle
     **********************************************************************************/
    virtual void value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                       Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {

    }

private:
    std::vector<ScalarType> mControl; /*!< set of particles */
    std::vector<ScalarType> mObjFuncValues; /*!< set of objective function values */

    Plato::Interface* mInterface; /*!< interface to data motion coordinator */
    Plato::OptimizerEngineStageData mEngineInputData; /*!< holds Plato Engine's options and inputs */
    std::shared_ptr<Teuchos::ParameterList> mParameterList; /*!< Plato Engine parameter list */

private:
    GradFreeEngineObjective(const Plato::GradFreeEngineObjective<ScalarType, OrdinalType>&);
    Plato::GradFreeEngineObjective<ScalarType, OrdinalType> & operator=(const Plato::GradFreeEngineObjective<ScalarType, OrdinalType>&);
};
// class GradFreeEngineObjective

} // namespace Plato
