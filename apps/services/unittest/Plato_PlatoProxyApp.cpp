/*
 * Plato_PlatoProxyApp.cpp
 *
 *  Created on: Oct 17, 2017
 */

#include <algorithm>

#include <stk_mesh/base/Comm.hpp>
#include <stk_io/StkMeshIoBroker.hpp>
#include <stk_mesh/base/GetEntities.hpp>

#include "Plato_StkMeshUtils.hpp"
#include "Plato_PlatoProxyApp.hpp"

namespace Plato
{

PlatoProxyApp::PlatoProxyApp(const std::string & aInputMeshFile, const MPI_Comm & aAppComm) :
        mAppComm(aAppComm),
        mInputMeshFile(aInputMeshFile),
        mObjectiveValue(std::vector<double>(1, std::numeric_limits<double>::max())),
        mDesignVariables(),
        mObjectiveGradient(),
        mGlobalIDsOwned(),
        mGlobalIDsOwnedAndShared()
{
}

PlatoProxyApp::~PlatoProxyApp()
{
}

void PlatoProxyApp::exportDataMap(const Plato::data::layout_t & aDataLayout, std::vector<int> & aMyOwnedGlobalIDs)
{
    if(aDataLayout == Plato::data::layout_t::SCALAR_FIELD)
    {
        aMyOwnedGlobalIDs = mGlobalIDsOwned;
    }
}

void PlatoProxyApp::finalize()
{
}

void PlatoProxyApp::initialize()
{
    // Build Temporary Mesh Database
    stk::io::StkMeshIoBroker tMeshData(mAppComm);
    tMeshData.add_mesh_database(mInputMeshFile, stk::io::READ_MESH);
    tMeshData.create_input_mesh();
    tMeshData.populate_bulk_data();

    // Build Communication Maps
    const stk::mesh::BulkData & tBulkData = tMeshData.bulk_data();
    const stk::mesh::MetaData & tMetaData = tMeshData.meta_data();
    Plato::buildCommunicationOwnedNodeMap(tBulkData, tMetaData, mGlobalIDsOwned);
    Plato::buildCommunicationOwnedAndSharedNodeMap(tBulkData, tMetaData, mGlobalIDsOwnedAndShared);

    int tNumOwnedDesignVariables = mGlobalIDsOwned.size();
    mDesignVariables.resize(tNumOwnedDesignVariables, 0);
    mObjectiveGradient.resize(tNumOwnedDesignVariables, 0);
}

void PlatoProxyApp::compute(const std::string & aOperationName)
{
    if(std::strcmp(aOperationName.c_str(), "UpdateDesignVariables") == 0)
    {
        int tLocalProcID = -1;
        MPI_Comm_rank(mAppComm, &tLocalProcID);
        std::fill(mDesignVariables.begin(), mDesignVariables.end(), static_cast<double>(tLocalProcID + 1));
    }
}

void PlatoProxyApp::exportData(const std::string & aArgumentName, Plato::SharedData& aExportData)
{
    if(std::strcmp(aArgumentName.c_str(), "DesignVariables") == 0)
    {
        // Get locally owned data
        aExportData.setData(mDesignVariables);
    }
}

void PlatoProxyApp::importData(const std::string & aArgumentName, const Plato::SharedData& aImportData)
{
    if(std::strcmp(aArgumentName.c_str(), "ObjectiveValue") == 0)
    {
        // Get locally owned data
        double tValue = 0;
        std::fill(mObjectiveValue.begin(), mObjectiveValue.end(), tValue);
        aImportData.getData(mObjectiveValue);
    }
    if(std::strcmp(aArgumentName.c_str(), "ObjectiveGradient") == 0)
    {
        // Get locally owned data
        double tValue = 0;
        std::fill(mObjectiveGradient.begin(), mObjectiveGradient.end(), tValue);
        aImportData.getData(mObjectiveGradient);
    }
}

void PlatoProxyApp::getSubDomainOwnedGlobalIDs(std::vector<int> & aInput) const
{
    assert(mGlobalIDsOwned.empty() == false);

    aInput.clear();
    const int tSize = mGlobalIDsOwned.size();
    aInput.resize(tSize);
    std::copy(mGlobalIDsOwned.begin(), mGlobalIDsOwned.end(), aInput.begin());
}

void PlatoProxyApp::getSubDomainOwnedAndSharedGlobalIDs(std::vector<int> & aInput) const
{
    assert(mGlobalIDsOwnedAndShared.empty() == false);

    aInput.clear();
    const int tSize = mGlobalIDsOwnedAndShared.size();
    aInput.resize(tSize);
    std::copy(mGlobalIDsOwnedAndShared.begin(), mGlobalIDsOwnedAndShared.end(), aInput.begin());
}

} // namespace Plato
