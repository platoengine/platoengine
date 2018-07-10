/*
 * Plato_SharedValue.hpp
 *
 *  Created on: April 23, 2017
 *
 */

#ifndef SRC_SHAREDVALUE_HPP_
#define SRC_SHAREDVALUE_HPP_

#include <string>
#include <vector>

#include "mpi.h"
#include "Plato_SharedData.hpp"

namespace Plato
{

struct CommunicationData;

class SharedValue : public SharedData
{
public:
    SharedValue(const std::string & aMyName, const std::string & aProviderName, const Plato::CommunicationData & aCommData, int aSize = 1);
    virtual ~SharedValue();

    int size() const;
    std::string myName() const;
    Plato::data::layout_t myLayout() const;

    void transmitData();
    void setData(const std::vector<double> & aData);
    void getData(std::vector<double> & aData) const;

private:
    std::string mMyName;
    std::string mProviderName;
    std::string mLocalCommName;

    MPI_Comm mMyComm;
    MPI_Comm mInterComm;

    int mNumData;
    std::vector<double> mData;
    Plato::data::layout_t mMyLayout;

private:
    SharedValue(const SharedValue& aRhs);
    SharedValue& operator=(const SharedValue& aRhs);
};

} // End namespace Plato

#endif
