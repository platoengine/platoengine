/*
 * Plato_SharedData.hpp
 *
 *  Created on: April 23, 2017
 *
 */

#ifndef SRC_SHAREDDATA_HPP_
#define SRC_SHAREDDATA_HPP_

#include <string>
#include <vector>
#include <cassert>

namespace Plato
{

//!  Inter-program shared data
/*!
 */

struct data
{
    enum layout_t
    {
        SCALAR = 0, SCALAR_FIELD = 1, VECTOR_FIELD = 2, TENSOR_FIELD = 3, ELEMENT_FIELD = 4, SCALAR_PARAMETER = 5
    };
};

class SharedData
{
public:
    virtual ~SharedData()
    {
    }

    virtual int size() const = 0;
    virtual std::string myName() const = 0;
    virtual std::string myContext() const { return std::string(); }
    virtual Plato::data::layout_t myLayout() const = 0;

    virtual void transmitData() = 0;
    virtual void setData(const std::vector<double> & aData) = 0;
    virtual void getData(std::vector<double> & aData) const = 0;
};

} // End namespace Plato

#endif
