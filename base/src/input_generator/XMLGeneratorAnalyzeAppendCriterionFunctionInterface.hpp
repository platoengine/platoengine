/*
 * XMLGeneratorAnalyzeAppendCriterionFunctionInterface.hpp
 *
 *  Created on: Jun 15, 2020
 */

#pragma once

#include "XMLGeneratorAnalyzeFunctionMapTypes.hpp"
#include "XMLGeneratorAnalyzeCriterionUtilities.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \struct The goal of this C++ struct is to provide an interface for the \n
 * functions used to append design criteria paraemters to plato_analyze_input_deck.xml. \n
 * This interface reduces cyclomatic complexity due to having multiple design \n
 * criteria in Plato Analyze.
**********************************************************************************/
template<typename CriterionType>
struct AppendCriterionParameters
{
private:
    /*!< map from design criterion category to function used to append design criterion and respective parameters */
    XMLGen::Analyze::CriterionFuncMap mMap;

    /******************************************************************************//**
     * \fn insert
     * \brief Insert functions used to append design criterion parameters to design \n
     *   criterion function map.
     **********************************************************************************/
    void insert()
    {
        // volume
        auto tFuncIndex = std::type_index(typeid(XMLGen::Private::append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("volume",
          std::make_pair((XMLGen::Analyze::CriterionFunc)XMLGen::Private::append_scalar_function_criterion<CriterionType>, tFuncIndex)));

        // maximize stiffness
        tFuncIndex = std::type_index(typeid(XMLGen::Private::append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("mechanical_compliance",
          std::make_pair((XMLGen::Analyze::CriterionFunc)XMLGen::Private::append_scalar_function_criterion<CriterionType>, tFuncIndex)));

        // stress_and_mass (same as stress_constraint_general)
        tFuncIndex = std::type_index(typeid(XMLGen::Private::append_stress_constrained_mass_minimization_criterion));
        mMap.insert(std::make_pair("stress_and_mass",
          std::make_pair((XMLGen::Analyze::CriterionFunc)XMLGen::Private::append_stress_constrained_mass_minimization_criterion, tFuncIndex)));

        // stress_constraint_general
        tFuncIndex = std::type_index(typeid(XMLGen::Private::append_stress_constrained_mass_minimization_criterion));
        mMap.insert(std::make_pair("stress_constraint_general",
          std::make_pair((XMLGen::Analyze::CriterionFunc)XMLGen::Private::append_stress_constrained_mass_minimization_criterion, tFuncIndex)));

        // stress p-norm
        tFuncIndex = std::type_index(typeid(XMLGen::Private::append_pnorm_criterion<CriterionType>));
        mMap.insert(std::make_pair("stress_p-norm",
          std::make_pair((XMLGen::Analyze::CriterionFunc)XMLGen::Private::append_pnorm_criterion<CriterionType>, tFuncIndex)));

        // effective energy
        tFuncIndex = std::type_index(typeid(XMLGen::Private::append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("effective_energy",
          std::make_pair((XMLGen::Analyze::CriterionFunc)XMLGen::Private::append_scalar_function_criterion<CriterionType>, tFuncIndex)));

        // maximize heat conduction
        tFuncIndex = std::type_index(typeid(XMLGen::Private::append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("thermal_compliance",
          std::make_pair((XMLGen::Analyze::CriterionFunc)XMLGen::Private::append_scalar_function_criterion<CriterionType>, tFuncIndex)));

        // thermo-elastic energy
        tFuncIndex = std::type_index(typeid(XMLGen::Private::append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("thermomechanical_compliance",
          std::make_pair((XMLGen::Analyze::CriterionFunc)XMLGen::Private::append_scalar_function_criterion<CriterionType>, tFuncIndex)));

        // electro-elastic energy
        tFuncIndex = std::type_index(typeid(XMLGen::Private::append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("electromechanical_compliance",
          std::make_pair((XMLGen::Analyze::CriterionFunc)XMLGen::Private::append_scalar_function_criterion<CriterionType>, tFuncIndex)));

        // flux p-norm
        tFuncIndex = std::type_index(typeid(XMLGen::Private::append_pnorm_criterion<CriterionType>));
        mMap.insert(std::make_pair("flux_p-norm",
          std::make_pair((XMLGen::Analyze::CriterionFunc)XMLGen::Private::append_pnorm_criterion<CriterionType>, tFuncIndex)));
        
        // elastic_work
        tFuncIndex = std::type_index(typeid(XMLGen::Private::append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("elastic_work",
          std::make_pair((XMLGen::Analyze::CriterionFunc)XMLGen::Private::append_scalar_function_criterion<CriterionType>, tFuncIndex)));

        // plastic_work
        tFuncIndex = std::type_index(typeid(XMLGen::Private::append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("plastic_work",
          std::make_pair((XMLGen::Analyze::CriterionFunc)XMLGen::Private::append_scalar_function_criterion<CriterionType>, tFuncIndex)));
        
        // total_work
        tFuncIndex = std::type_index(typeid(XMLGen::Private::append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("total_work",
          std::make_pair((XMLGen::Analyze::CriterionFunc)XMLGen::Private::append_scalar_function_criterion<CriterionType>, tFuncIndex)));
        
        // thermoplasticity_thermal_energy
        tFuncIndex = std::type_index(typeid(XMLGen::Private::append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("thermoplasticity_thermal_energy",
          std::make_pair((XMLGen::Analyze::CriterionFunc)XMLGen::Private::append_scalar_function_criterion<CriterionType>, tFuncIndex)));
    }

public:
    /******************************************************************************//**
     * \fn AppendCriterionParameters
     * \brief Default constructor
    **********************************************************************************/
    AppendCriterionParameters()
    {
        this->insert();
    }

    /******************************************************************************//**
     * \fn call
     * \tparam CriterionType criterion function metadata C++ structure type
     * \brief Append criterion function parameters to plato_analyze_input_deck.xml file.
     * \param [in]     Criterion    criterion metadata
     * \param [in/out] aParentNode  pugi::xml_node
    **********************************************************************************/
    pugi::xml_node call(const CriterionType& aCriterion, pugi::xml_node &aParentNode) const
    {
/* Code is not a member of Criterion and we should never get in here except for plato_analyze if
 * things are being called correctly
        auto tLowerPerformer = Plato::tolower(aCriterion.code());
        if(tLowerPerformer.compare("plato_analyze") != 0)
        {
            return;
        }
*/

        auto tLowerCategory = Plato::tolower(aCriterion.type());
        auto tMapItr = mMap.find(tLowerCategory);
        if(tMapItr == mMap.end())
        {
            THROWERR(std::string("Criterion Function Interface: Did not find criterion function with tag '") + tLowerCategory + "' in list.")
        }
        auto tTypeCastedFunc = reinterpret_cast<pugi::xml_node(*)(const CriterionType&, pugi::xml_node&)>(tMapItr->second.first);
        if(tMapItr->second.second == std::type_index(typeid(tTypeCastedFunc)))
        {
            THROWERR(std::string("Criterion Function Interface: Reinterpret cast for criterion function with tag '") + tLowerCategory + "' failed.")
        }
        return tTypeCastedFunc(aCriterion, aParentNode);
    }
};
// struct AppendCriterionParameters

}
// namespace XMLGen
