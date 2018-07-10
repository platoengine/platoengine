/*
 * Plato_Stage.hpp
 *
 *  Created on: April 23, 2017
 *
 */

#ifndef SRC_STAGE_HPP_
#define SRC_STAGE_HPP_

#include <string>
#include <vector>

namespace Plato
{

class Performer;
class SharedData;
class Operation;
class StageInputDataMng;

//! Sequence of Operations that correspond to a call to Plato::Interface::compute()
/*!
 */
class Stage
{
public:
    Stage(const Plato::StageInputDataMng & aStageInputData,
          const std::vector<Plato::Performer*>& aPerformers,
          const std::vector<Plato::SharedData*>& aSharedData);
    ~Stage();

    Plato::Operation* getNextOperation();
    void begin();
    void end();

    std::string getName() const
    {
        return m_name;
    }
    std::vector<std::string> getInputDataNames() const;
    std::vector<std::string> getOutputDataNames() const;

private:
    std::string m_name;
    std::vector<Plato::Operation*> m_operations;
    std::vector<Plato::SharedData*> m_inputData;
    std::vector<Plato::SharedData*> m_outputData;

    int currentOperationIndex;
};

} // End namespace Plato

#endif
