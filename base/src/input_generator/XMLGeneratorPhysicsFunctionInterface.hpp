/*
 * XMLGeneratorPhysicsFunctionInterface.hpp
 *
 *  Created on: Jul 23, 2020
 */

#pragma once

#include "pugixml.hpp"

#include "XMLGeneratorOutputMetadata.hpp"
#include "XMLGeneratorScenarioMetadata.hpp"
#include "XMLGeneratorAnalyzeFunctionMapTypes.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \struct The goal of this C++ struct is to provide an interface for the \n
 * functions used to append material models to plato_analyze_input_deck.xml. \n
 * This interface reduces cyclomatic complexity due to having multiple material \n
 * models in Plato Analyze.
**********************************************************************************/
struct PhysicsFunctionInterface
{
// private member data
private:
    /*!< map from physics category to function used to append physics parameters */
    XMLGen::Analyze::PhysicsFuncMap mMap;

// private member functions
private:
    /******************************************************************************//**
     * \fn insertEllipticPhysics
     * \brief Insert elliptic physics to physics functions map.
    **********************************************************************************/
    void insertEllipticPhysics();

    /******************************************************************************//**
     * \fn insertParabolicPhysics
     * \brief Insert parabolic physics to physics functions map.
    **********************************************************************************/
    void insertParabolicPhysics();

    /******************************************************************************//**
     * \fn insertHyperbolicPhysics
     * \brief Insert hyperbolic physics to physics functions map.
    **********************************************************************************/
    void insertHyperbolicPhysics();

    /******************************************************************************//**
     * \fn insertPlasticityPhysics
     * \brief Insert infinitesimal strain plasticity physics to physics functions map.
    **********************************************************************************/
    void insertPlasticityPhysics();

    /******************************************************************************//**
     * \fn insertStabilizedEllipticPhysics
     * \brief Insert stabilized physics to physics functions map.
    **********************************************************************************/
    void insertStabilizedEllipticPhysics();

    /******************************************************************************//**
     * \fn insert
     * \brief Insert physics functions to physics functions map.
     **********************************************************************************/
    void insert();

// public member functions
public:
    /******************************************************************************//**
     * \fn AppendPhysicsParameters
     * \brief Default constructor
    **********************************************************************************/
    PhysicsFunctionInterface();

    /******************************************************************************//**
     * \fn call
     * \brief Append physics and respective parameters to plato_analyze_input_deck.xml file.
     * \param [in]     aScenarioMetaData    physics scenario metadata
     * \param [in]     aOutputMetaData      physics output metadata
     * \param [in/out] aParentNode          pugi::xml_node
    **********************************************************************************/
    void call(const XMLGen::Scenario& aScenarioMetaData,
              const XMLGen::Output& aOutputMetaData,
              pugi::xml_node &aParentNode) const;
};
// struct AppendMaterialModelParameters

}
// namespace XMLGen
