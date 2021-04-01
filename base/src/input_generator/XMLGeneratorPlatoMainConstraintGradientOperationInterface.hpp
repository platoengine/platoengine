/*
 * XMLGeneratorPlatoMainConstraintGradientOperationInterface.hpp
 *
 *  Created on: Jun 28, 2020
 */

#pragma once

#include "pugixml.hpp"

#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorPlatoMainFunctionMapTypes.hpp"

namespace XMLGen
{

struct ConstraintGradientOperation
{
private:
    /*!< map from 'code' keyword to function used to append constraint value operation to interface.xml file */
    XMLGen::PlatoMain::ConstraintGradientOpMap mMap;

    /******************************************************************************//**
     * \fn insert
     * \brief Insert constraint value operations to map.
    **********************************************************************************/
    void insert();

public:
    /******************************************************************************//**
     * \fn ConstraintGradientOperation
     * \brief Default constructor
    **********************************************************************************/
    ConstraintGradientOperation() { this->insert(); }

    /******************************************************************************//**
     * \fn call
     * \brief Append constraint gradient operation to XML node.
     * \param [in]     aConstraint  constraint metadata
     * \param [in/out] aParentNode  pugi::xml_node
    **********************************************************************************/
    void call(const XMLGen::Constraint& aConstraint, const std::string &aPerformer,
              const std::string &aDesignVariableName, const std::string &aCode, 
              pugi::xml_node& aParentNode) const;
};
// struct ConstraintGradientOperation

}
// namespace XMLGen
