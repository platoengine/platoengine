/*
 * Plato_PhysicsProxyApp.hpp
 *
 *  Created on: Oct 17, 2017
 */

#ifndef PLATO_PHYSICSPROXYAPP_HPP_
#define PLATO_PHYSICSPROXYAPP_HPP_

#include <mpi.h>

#include "Plato_Application.hpp"

namespace stk { namespace mesh { class Selector; } }
namespace stk { namespace io { class StkMeshIoBroker; } }

namespace Plato
{

class SharedData;

class PhysicsProxyApp : public Plato::Application
{
public:
    PhysicsProxyApp(const std::string & aInputMeshFile, const MPI_Comm & aAppComm);
    virtual ~PhysicsProxyApp();

    void finalize();
    void initialize();
    void compute(const std::string & aOperationName);
    void exportData(const std::string & aArgumentName, Plato::SharedData & aExportData);
    void importData(const std::string & aArgumentName, const Plato::SharedData & aImportData);
    void exportDataMap(const Plato::data::layout_t & aDataLayout, std::vector<int> & aMyOwnedGlobalIDs);

    int getLocalNumNodes() const;
    int getLocalNumElements() const;
    int getGlobalNumNodes() const;
    int getGlobalNumElements() const;

    void getSubDomainOwnedGlobalIDs(std::vector<int> & aInput) const;
    void getSubDomainOwnedAndSharedGlobalIDs(std::vector<int> & aInput) const;

private:
    MPI_Comm mMyComm;
    std::string mInputMeshFile;
    stk::mesh::Selector mSelector;
    stk::io::StkMeshIoBroker* mMeshData;

    std::vector<int> mGlobalIDsOwned;
    std::vector<int> mGlobalIDsOwnedAndShared;

    std::vector<double> mObjectiveValue;
    std::vector<double> mDesignVariables;
    std::vector<double> mObjectiveGradient;

private:
    PhysicsProxyApp(const Plato::PhysicsProxyApp& aRhs);
    Plato::PhysicsProxyApp& operator=(const Plato::PhysicsProxyApp& aRhs);
};

} // namespace Plato

#endif /* PLATO_PHYSICSPROXYAPP_HPP_ */
