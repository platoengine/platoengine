/*
 * Plato_SromXMLGenHelpers.hpp
 *
 *  Created on: May 11, 2020
 */

#pragma once

#include "XMLGeneratorDataStruct.hpp"
#include "Plato_SromXML.hpp"

namespace Plato
{

namespace srom
{

/******************************************************************************//**
 * \enum Non-deterministic variable's category.  Is the non-deterministic variable \n
 * a material property or a load?
**********************************************************************************/
enum struct category
{
    MATERIAL = 0, LOAD = 1, UNDEFINED = 2,
};
// enum struct category

/*!< map from non-deterministic variable's category to list of non-deterministic variables' metadata */
using UncertaintyCategories = std::map<Plato::srom::category, std::vector<XMLGen::Uncertainty>>;
/******************************************************************************//**
 * \fn split_uncertainty_categories
 * \brief Split input uncertainties into categories.
 * \param [in]  aMetadata Plato problem input metadata
 * \return map from non-deterministic variable category to list of non-deterministic variables' metadata
**********************************************************************************/
inline UncertaintyCategories split_uncertainties_into_categories(const XMLGen::InputData& aMetadata)
{
    if(aMetadata.uncertainties.empty())
    {
        THROWERR(std::string("Split Uncertainty Into Categories: Input list of uncertainties is empty. ")
            + "Requested a stochastic use case but no uncertainty block was defined.")
    }

    Plato::srom::UncertaintyCategories tCategories;
    for(auto& tUncertainty : aMetadata.uncertainties)
    {
        auto tCategory = Plato::srom::tolower(tUncertainty.variable_type);
        auto tIsLoad = tCategory == "load" ? true : false;
        auto tIsMaterial = tCategory == "material" ? true : false;
        if (tIsLoad && !tIsMaterial)
        {
            tCategories[Plato::srom::category::LOAD].push_back(tUncertainty);
        }
        else if (tIsMaterial && !tIsLoad)
        {
            tCategories[Plato::srom::category::MATERIAL].push_back(tUncertainty);
        }
        else if (!tIsMaterial && !tIsLoad)
        {
            THROWERR(std::string("Split Uncertainty Into Categories: Non-deterministic category '")
                + tCategory + "' is not supported.")
        }
    }
    return tCategories;
}
// function split_uncertainty_categories

}
// namespace srom

}
// namespace Plato
