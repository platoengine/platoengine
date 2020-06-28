/*
 * XMLGeneratorPlatoMainFunctionMapTypes.hpp
 *
 *  Created on: Jun 28, 2020
 */

#pragma once

#include <typeindex>
#include <unordered_map>

namespace XMLGen
{

namespace PlatoMain
{

/*!< constraint value operation pointer type */
typedef void (*ConstraintValueOp)(void);

/*!< constraint gradient operation pointer type */
typedef void (*ConstraintGradientOp)(void);

/*!< map from simulation code to function used to append the constraint value operation, \n
 * i.e. map<code, append_constraint_value_operation_function> */
typedef std::unordered_map<std::string, std::pair<XMLGen::PlatoMain::ConstraintValueOp, std::type_index>> ConstraintValueOpMap;

/*!< map from simulation code to function used to append the constraint gradient operation, \n
 * i.e. map<code, append_constraint_gradient_operation_function> */
typedef std::unordered_map<std::string, std::pair<XMLGen::PlatoMain::ConstraintGradientOp, std::type_index>> ConstraintGradientOpMap;

}
// namespace PlatoMain

}
// namespace XMLGen
