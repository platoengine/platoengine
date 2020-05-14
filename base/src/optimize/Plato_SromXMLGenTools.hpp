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
 * \fn check_output
 * \brief Check Stochastic Reduced Order Model (SROM) problem outputs are properly formatted.
 * \param [in] aMetaData Random use case metadata
**********************************************************************************/
inline void check_output(const XMLGen::RandomMetaData& aMetaData)
{
    Plato::srom::check_output_load_set_size(aMetaData);
    Plato::srom::check_output_load_set_types(aMetaData);
    Plato::srom::check_output_load_set_application_name(aMetaData);
}
// function check_output

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
 * \param [in/out] aSromOutputs     SROM problem output metadata
 * \param [in/out] aXMLGenMetaData  Plato problem input metadata
**********************************************************************************/
inline void postprocess_srom_problem_outputs
(const Plato::srom::OutputMetaData& aSromOutputs,
 XMLGen::InputData& aXMLGenMetaData)
{
    aXMLGenMetaData.mRandomMetaData.clear();

    switch(aSromOutputs.usecase())
    {
        case Plato::srom::usecase::LOAD:
        {
            Plato::srom::postprocess_load_outputs(aSromOutputs, aXMLGenMetaData);
            break;
        }
        case Plato::srom::usecase::MATERIAL:
        {
            Plato::srom::postprocess_material_outputs(aSromOutputs, aXMLGenMetaData);
            break;
        }
        case Plato::srom::usecase::MATERIAL_PLUS_LOAD:
        {
            Plato::srom::postprocess_load_outputs(aSromOutputs, aXMLGenMetaData);
            Plato::srom::postprocess_material_outputs(aSromOutputs, aXMLGenMetaData);
            break;
        }
        default:
        case Plato::srom::usecase::UNDEFINED:
        {
            THROWERR("Post-Process SROM Problem Outputs: Detected an undefined SROM problem use case.")
            break;
        }
    }

    aXMLGenMetaData.mRandomMetaData.finalize();
}
// function postprocess_srom_problem_outputs

}
// namespace srom

}
// namespace Plato
