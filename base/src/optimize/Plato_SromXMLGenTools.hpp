/*
 * Plato_SromXMLGenTools.hpp
 *
 *  Created on: May 11, 2020
 */

#pragma once

#include "Plato_SromXMLGenMaterial.hpp"
#include "Plato_SromXMLGenLoad.hpp"

namespace Plato
{

namespace srom
{

/******************************************************************************//**
 * \fn preprocess_srom_problem_inputs
 * \brief Pre-process Stochastic Reduced Order Model (SROM) problem inputs.
 * \param [in/out] aInputMetadata  Plato problem input metadata
 * \param [in/out] aSromInputs     SROM problem metadata
**********************************************************************************/
inline void preprocess_srom_problem_inputs
(XMLGen::InputData& aInputMetadata,
 Plato::srom::InputMetaData& aSromInputs)
{
    switch(aSromInputs.usecase())
    {
        case Plato::srom::usecase::LOAD:
        {
            Plato::srom::preprocess_load_inputs(aInputMetadata, aSromInputs);
            break;
        }
        case Plato::srom::usecase::MATERIAL:
        {
            Plato::srom::preprocess_material_inputs(aInputMetadata, aSromInputs);
            break;
        }
        case Plato::srom::usecase::MATERIAL_PLUS_LOAD:
        {
            Plato::srom::preprocess_load_inputs(aInputMetadata, aSromInputs);
            Plato::srom::preprocess_material_inputs(aInputMetadata, aSromInputs);
            break;
        }
        default:
        case Plato::srom::usecase::UNDEFINED:
        {
            THROWERR("Pre-Process SROM Problem Inputs: Detected an undefined SROM problem use case.")
            break;
        }
    }
}
// function preprocess_srom_problem_inputs

/******************************************************************************//**
 * \fn postprocess_srom_problem_outputs
 * \brief Post-process Stochastic Reduced Order Model (SROM) problem outputs.
 * \param [in/out] aSromOutputs    SROM problem output metadata
 * \param [in/out] aInputMetadata  Plato problem input metadata
**********************************************************************************/
inline void postprocess_srom_problem_outputs
(const Plato::srom::OutputMetaData& aSromOutputs,
 XMLGen::InputData& aInputMetadata)
{
    switch(aSromOutputs.usecase())
    {
        case Plato::srom::usecase::LOAD:
        {
            Plato::srom::postprocess_load_outputs(aSromOutputs, aInputMetadata);
            break;
        }
        case Plato::srom::usecase::MATERIAL:
        {
            Plato::srom::postprocess_material_outputs(aSromOutputs, aInputMetadata);
            break;
        }
        case Plato::srom::usecase::MATERIAL_PLUS_LOAD:
        {
            Plato::srom::postprocess_load_outputs(aSromOutputs, aInputMetadata);
            Plato::srom::postprocess_material_outputs(aSromOutputs, aInputMetadata);
            break;
        }
        default:
        case Plato::srom::usecase::UNDEFINED:
        {
            THROWERR("Post-Process SROM Problem Outputs: Detected an undefined SROM problem use case.")
            break;
        }
    }
}
// function postprocess_srom_problem_outputs

}
// namespace srom

}
// namespace Plato
