/*
 * Plato_SingleOperation.hpp
 *
 *  Created on: March 30, 2018
 *
 */

#ifndef SRC_SINGLE_OPERATION_HPP_
#define SRC_SINGLE_OPERATION_HPP_

#include <map>
#include <string>
#include <vector>

#include "Plato_Operation.hpp"

namespace Plato
{

class Performer;
class SharedData;
class OperationInputDataMng;

//! Performer with input and output shared fields.
/*!
 */
class SingleOperation : public Operation
{
public:
    SingleOperation(const Plato::OperationInputDataMng & aOperationDataMng,
                    const std::vector<Plato::Performer*>& aPerformers,
                    const std::vector<Plato::SharedData*>& aSharedData);
};

} // End namespace Plato

#endif
