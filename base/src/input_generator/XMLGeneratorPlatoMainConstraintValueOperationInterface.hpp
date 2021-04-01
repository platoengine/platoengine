/*
 * XMLGeneratorPlatoMainConstraintValueOperationInterface.hpp
 *
 *  Created on: Jun 28, 2020
 */

#pragma once

#include "pugixml.hpp"

#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorPlatoMainFunctionMapTypes.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \struct ConstraintValueOperation
 * \brief The goal of this C++ struct is to provide a ligh-weight interface for \n
 * the functions used to append constraint value operations to interface.xml.
**********************************************************************************/
struct ConstraintValueOperation
{
private:
    /*!< map from 'code' keyword to function used to append constraint value operation to interface.xml file */
    XMLGen::PlatoMain::ConstraintValueOpMap mMap;

    /******************************************************************************//**
     * \fn insert
     * \brief Insert constraint value operations to map.
    **********************************************************************************/
    void insert();

public:
    /******************************************************************************//**
     * \fn ConstraintValueOperation
     * \brief Default constructor
    **********************************************************************************/
    ConstraintValueOperation() { this->insert(); }

    /******************************************************************************//**
     * \fn call
     * \brief Append constraint value operation to XML node.
     * \param [in]     aConstraint  constraint metadata
     * \param [in/out] aParentNode  pugi::xml_node
    **********************************************************************************/
    void call(const XMLGen::Constraint& aConstraint, const std::string &aPerformer,
              const std::string &aDesignVariableName, const std::string &aCode, 
              pugi::xml_node& aParentNode) const;
};
// struct ConstraintValueOperation

}
// namespace XMLGen
