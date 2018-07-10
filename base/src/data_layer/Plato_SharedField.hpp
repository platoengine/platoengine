/*
 * Plato_SharedField.hpp
 *
 *  Created on: April 23, 2017
 *
 */

#ifndef SRC_SHAREDFIELD_HPP_
#define SRC_SHAREDFIELD_HPP_

#include <string>
#include <vector>
#include <memory>

#include <Epetra_Vector.h>
#include <Epetra_MpiComm.h>
#include <Epetra_Map.h>
#include <Epetra_Import.h>

#include "Plato_SharedData.hpp"
#include "Plato_Communication.hpp"
#include "Plato_SharedDataInfo.hpp"

namespace Plato
{

struct communication;
struct CommunicationData;

class SharedField : public SharedData
{
public:
    SharedField(const std::string & aMyName,
                const Plato::communication::broadcast_t & aMyBroadcast,
                const Plato::CommunicationData & aCommData,
                Plato::data::layout_t aMyLayout);
    virtual ~SharedField();

    int size() const;
    std::string myName() const;
    Plato::data::layout_t myLayout() const;
    Plato::communication::broadcast_t myBroadcast() const;

    void transmitData();
    void setData(const std::vector<double> & aData);
    void getData(std::vector<double> & aData) const;

    void setData(const double & aDataVal, const int & aGlobalIndex);
    void getData(double & dataVal, const int & aGlobalIndex) const;

private:
    void initialize(const Plato::CommunicationData & aCommData);

private:
    std::string mMyName;
    Plato::data::layout_t mMyLayout;
    Plato::communication::broadcast_t mMyBroadcast;

    std::shared_ptr<Epetra_MpiComm> mEpetraComm;
    std::shared_ptr<Epetra_Map> mGlobalIDsProvided;
    std::shared_ptr<Epetra_Map> mGlobalIDsReceived;

    std::shared_ptr<Epetra_Import> mNodeImporter;

    std::shared_ptr<Epetra_Vector> mSendDataVector;
    std::shared_ptr<Epetra_Vector> mRecvDataVector;

private:
    SharedField(const SharedField& aRhs);
    SharedField& operator=(const SharedField& aRhs);
};

} // End namespace Plato

#endif
