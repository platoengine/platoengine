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
//    Plato::srom::check_output_load_set_size(aMetaData);
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

/******************************************************************************//**
 * \fn set_stochastic_problem_usecase
 * \brief Set stochastic problem use case.
 * \param [in]  aXMLMetaData  Plato problem input metadata
 * \param [out] aSromInputs   Stochastic Reduced Order Model (SROM) problem input metadata
**********************************************************************************/
inline void set_stochastic_problem_usecase
(const XMLGen::InputData& aXMLMetaData,
 Plato::srom::InputMetaData& aSromInputs)
{
    std::vector<std::string> tCategories;
    for(auto& tCase : aXMLMetaData.uncertainties)
    {
        auto tLowerCategory = Plato::tolower(tCase.category());
        tCategories.push_back(tLowerCategory);
    }

    auto tItrLoad = std::find(tCategories.begin(), tCategories.end(), "load");
    auto tItrMaterial = std::find(tCategories.begin(), tCategories.end(), "material");
    if(tItrLoad != tCategories.end() && tItrMaterial == tCategories.end())
    {
        aSromInputs.usecase(Plato::srom::usecase::LOAD);
    }
    else if(tItrLoad == tCategories.end() && tItrMaterial != tCategories.end())
    {
        aSromInputs.usecase(Plato::srom::usecase::MATERIAL);
    }
    else if(tItrLoad != tCategories.end() && tItrMaterial != tCategories.end())
    {
        aSromInputs.usecase(Plato::srom::usecase::MATERIAL_PLUS_LOAD);
    }
    else
    {
        THROWERR(std::string("Set Stochastic Problem Use Case: Stochasti Reduced Order Model problem use case was not defined. ")
            + "The following use case are supported: load uncertainty, material uncertainty, or material plus load uncertainties.")
    }
}
// function set_stochastic_problem_usecase

/******************************************************************************//**
 * \fn solve
 * \brief Solve optimization problem to form stochastic reduced order model.
 * \param [in/out] aXMLGenMetaData  Plato problem input metadata
**********************************************************************************/
inline void solve(XMLGen::InputData& aXMLGenMetaData)
{
    Plato::srom::InputMetaData tSromInputs;
    Plato::srom::set_stochastic_problem_usecase(aXMLGenMetaData, tSromInputs);
    Plato::srom::preprocess_srom_problem_inputs(aXMLGenMetaData, tSromInputs);
    Plato::srom::OutputMetaData tSromOutputs;
    Plato::srom::build_sroms(tSromInputs, tSromOutputs);
    Plato::srom::postprocess_srom_problem_outputs(tSromOutputs, aXMLGenMetaData);
    Plato::srom::check_output(aXMLGenMetaData.mRandomMetaData);
}
// function solve

}
// namespace srom

}
// namespace Plato
