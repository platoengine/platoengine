/*
 * Plato_EngineOutputStage.hpp
 *
 *  Created on: Feb 13, 2019
 */

#pragma once

#include "Plato_CustomOutput.hpp"
#include "Plato_Interface.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Interface to PLATO Engine output stage
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class EngineOutputStage : public Plato::CustomOutput<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aInterface PLATO Engine interface
     * @param [in] aName output stage name
    **********************************************************************************/
    EngineOutputStage(Plato::Interface* aInterface, const std::string & aName) :
            mStageName(aName),
            mInterface(aInterface)
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~EngineOutputStage()
    {
    }

    /******************************************************************************//**
     * @brief Return PLATO Engine output stage name
     * @return output stage name
    **********************************************************************************/
    std::string name() const
    {
        return (mStageName);
    }

    /******************************************************************************//**
     * @brief Output PLATO Engine specific data
    **********************************************************************************/
    void output() override
    {
        if(mStageName.empty() == false)
        {
            Teuchos::ParameterList tParams;
            std::vector<std::string> tStageNames;
            tStageNames.push_back(mStageName);
            mInterface->compute(tStageNames, tParams);
        }
    }

private:
    std::string mStageName; /*!< output stage name */
    Plato::Interface* mInterface; /*!< PLATO Engine interface */
};
// class EngineOutputStage

} // namespace Plato
