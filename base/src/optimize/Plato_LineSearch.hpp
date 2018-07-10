/*
 * Plato_LineSearch.hpp
 *
 *  Created on: Oct 22, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_LINESEARCH_HPP_
#define PLATO_LINESEARCH_HPP_

namespace Plato
{

template<typename ScalarType, typename OrdinalType>
class StateManager;

template<typename ScalarType, typename OrdinalType = size_t>
class LineSearch
{
public:
    virtual ~LineSearch()
    {
    }

    virtual OrdinalType getNumIterationsDone() const = 0;
    virtual void setMaxNumIterations(const OrdinalType & aInput) = 0;
    virtual void setContractionFactor(const ScalarType & aInput) = 0;
    virtual void step(Plato::StateManager<ScalarType, OrdinalType> & aStateMng) = 0;
};

} // namespace Plato

#endif /* PLATO_LINESEARCH_HPP_ */
