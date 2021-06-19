/*
 * XMLGeneratorAnalyzeFunctionMapTypes.hpp
 *
 *  Created on: Jun 15, 2020
 */

#pragma once

#include <typeindex>
#include <unordered_map>

namespace XMLGen
{

namespace Analyze
{

/*!< physics function pointer type */
typedef void (*PhysicsFunc)(void);

/*!< criterion function pointer type */
typedef void (*CriterionFunc)(void);

/*!< material function pointer type */
typedef void (*MaterialModelFunc)(void);

/*!< function pointer type */
typedef void (*MaterialOperationFunc)(void);

/*!< load function pointer type */
typedef void (*LoadFunc)(void);

/*!< define essential boundary condition function pointer type */
typedef void (*EssentialBCFunc)(void);

/*!< define load tag function pointer type */
typedef std::string (*LoadTagFunc)(void);

/*!< define essential boundary condition tag function pointer type */
typedef std::string (*EssentialBCTagFunc)(void);


/*!< map from physics category to physics function used to append PDE and respective \n
 * parameters, i.e. map<physics_category, physics_function> */
typedef std::unordered_map<std::string, std::pair<XMLGen::Analyze::PhysicsFunc, std::type_index>> PhysicsFuncMap;

/*!< map from design criterion category to design criterion function used to append design \n
 * criterion and respective parameters, i.e. map<design_criterion_category, criterion_function> */
typedef std::unordered_map<std::string, std::pair<XMLGen::Analyze::CriterionFunc, std::type_index>> CriterionFuncMap;

/*!< map from material model category to material function used to append material properties and \n
 * respective values, i.e. map<material_model, material_function> */
typedef std::unordered_map<std::string, std::pair<XMLGen::Analyze::MaterialModelFunc, std::type_index>> MaterialModelFuncMap;

/*!< map from material model key to material function used to append material \n
 * parameter tags and corresponding values, \n
 * i.e. map<material_model_key, pair<material_model_function, function_id>> */
typedef std::unordered_map<std::string, std::pair<XMLGen::Analyze::MaterialOperationFunc, std::type_index>> MaterialFunctionMap;

/*!< map from load category to load function used to append, load parameters, i.e. map<load_category, append_load_function> */
typedef std::unordered_map<std::string, std::pair<XMLGen::Analyze::LoadFunc, std::type_index>> LoadFuncMap;

/*!< map from load category to load function used to define load tags, i.e. map<load_category, define_load_tag_function> */
typedef std::unordered_map<std::string, std::pair<XMLGen::Analyze::LoadTagFunc, std::type_index>> LoadTagFuncMap;

/*!< map from essential boundary condition (bc) category to essential bc function used to append, \n
 * essential bc parameters, i.e. map<essential_bc_category, append_essential_bc_function> */
typedef std::unordered_map<std::string, std::pair<XMLGen::Analyze::EssentialBCFunc, std::type_index>> EssentialBCFuncMap;

/*!< map from essential boundary condition (bc) category to essential bc function used to define \n
 * the essential bcs tags, i.e. map<essential_bc_category, define_essential_bc_tag_function> */
typedef std::unordered_map<std::string, std::pair<XMLGen::Analyze::EssentialBCTagFunc, std::type_index>> EssentialBCTagFuncMap;

}
// namespace Analyze

}
// namespace XMLGen
