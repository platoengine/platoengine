/*
 * Plato_OperationFactory.hpp
 *
 *  Created on: March 30, 2018
 *
 */

#ifndef SRC_OPERATION_FACTORY_HPP_
#define SRC_OPERATION_FACTORY_HPP_

#include <map>
#include <string>
#include <vector>

namespace Plato
{

class Performer;
class SharedData;
class Operation;
class OperationInputDataMng;

//! Performer with input and output shared fields.
/*!
 */
class OperationFactory
{
public:
    Operation*
    create(const Plato::OperationInputDataMng & aOperationDataMng,
           const std::vector<Plato::Performer*>& aPerformers,
           const std::vector<Plato::SharedData*>& aSharedData);
};

} // End namespace Plato

#endif
