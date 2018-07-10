/*
 * Plato_PhysicsProxyApp.cpp
 *
 *  Created on: Oct 17, 2017
 */

#include <stk_mesh/base/Comm.hpp>
#include <stk_io/StkMeshIoBroker.hpp>
#include <stk_mesh/base/GetEntities.hpp>

#include "Plato_SharedData.hpp"
#include "Plato_StkMeshUtils.hpp"
#include "Plato_PhysicsProxyApp.hpp"

namespace Plato
{

PhysicsProxyApp::PhysicsProxyApp(const std::string & aInputMeshFile, const MPI_Comm & aAppComm) :
        mMyComm(aAppComm),
        mInputMeshFile(aInputMeshFile),
        mSelector(),
        mMeshData(new stk::io::StkMeshIoBroker(mMyComm)),
        mGlobalIDsOwned(),
        mGlobalIDsOwnedAndShared(),
        mObjectiveValue(std::vector<double>(1, std::numeric_limits<double>::max())),
        mDesignVariables(),
        mObjectiveGradient()
{
}

PhysicsProxyApp::~PhysicsProxyApp()
{
    delete mMeshData;
}

void PhysicsProxyApp::exportDataMap(const Plato::data::layout_t & aDataLayout, std::vector<int> & aMyOwnedGlobalIDs)
{
    if(aDataLayout == Plato::data::layout_t::SCALAR_FIELD)
    {
        aMyOwnedGlobalIDs = mGlobalIDsOwned;
    }
}

void PhysicsProxyApp::finalize()
{
}

void PhysicsProxyApp::initialize()
{
    mMeshData->add_mesh_database(mInputMeshFile, stk::io::READ_MESH);
    mMeshData->create_input_mesh();
    mMeshData->populate_bulk_data();

    stk::mesh::BulkData & tBulkData = mMeshData->bulk_data();
    stk::mesh::MetaData & tMetaData = mMeshData->meta_data();
    mSelector = tMetaData.locally_owned_part() | tMetaData.globally_shared_part();
    Plato::buildCommunicationOwnedNodeMap(tBulkData, tMetaData, mGlobalIDsOwned);
    Plato::buildCommunicationOwnedAndSharedNodeMap(tBulkData, tMetaData, mGlobalIDsOwnedAndShared);

    assert(this->getLocalNumNodes() == static_cast<int>(mGlobalIDsOwnedAndShared.size()));

    int tNumDesignVariables = mGlobalIDsOwned.size();
    mDesignVariables.resize(tNumDesignVariables, 0);
    mObjectiveGradient.resize(tNumDesignVariables, 0);
}

void PhysicsProxyApp::compute(const std::string & aOperationName)
{
    if(std::strcmp(aOperationName.c_str(), "ObjectiveFunctionEvaluation") == 0)
    {
        std::fill(mObjectiveValue.begin(), mObjectiveValue.end(), 0);
        double tLocalObjectiveFunctionValue = 1;
        int tCount = mObjectiveValue.size();
        MPI_Allreduce(&tLocalObjectiveFunctionValue, mObjectiveValue.data(), tCount, MPI_DOUBLE, MPI_SUM, mMyComm);
    }
    if(std::strcmp(aOperationName.c_str(), "ComputeObjectiveGradient") == 0)
    {
        int tMyProcID = -1;
        MPI_Comm_rank(mMyComm, &tMyProcID);
        std::fill(mObjectiveGradient.begin(), mObjectiveGradient.end(), static_cast<double>(tMyProcID + 1));
    }
}

void PhysicsProxyApp::importData(const std::string & aArgumentName, const Plato::SharedData & aImportData)
{
    if(std::strcmp(aArgumentName.c_str(), "DesignVariables") == 0)
    {
        // Get locally owned data
        double tValue = 0;
        std::fill(mDesignVariables.begin(), mDesignVariables.end(), tValue);
        aImportData.getData(mDesignVariables);
    }
}

void PhysicsProxyApp::exportData(const std::string & aArgumentName, Plato::SharedData & aExportData)
{
    if(std::strcmp(aArgumentName.c_str(), "ObjectiveValue") == 0)
    {
        aExportData.setData(mObjectiveValue);
    }
    if(std::strcmp(aArgumentName.c_str(), "ObjectiveGradient") == 0)
    {
        aExportData.setData(mObjectiveGradient);
    }
}

int PhysicsProxyApp::getLocalNumNodes() const
{
    assert(mMeshData != nullptr);

    stk::topology::rank_t tRank = stk::topology::NODE_RANK;
    stk::mesh::BulkData & tBulkData = mMeshData->bulk_data();
    const stk::mesh::BucketVector & tNodeBuckets = tBulkData.buckets(tRank);
    int tNumLocalNodes = stk::mesh::count_selected_entities(mSelector, tNodeBuckets);
    return (tNumLocalNodes);
}

int PhysicsProxyApp::getLocalNumElements() const
{
    assert(mMeshData != nullptr);

    stk::topology::rank_t tRank = stk::topology::ELEMENT_RANK;
    stk::mesh::BulkData & tBulkData = mMeshData->bulk_data();
    const stk::mesh::BucketVector & tNodeBuckets = tBulkData.buckets(tRank);
    int tNumLocalElements = stk::mesh::count_selected_entities(mSelector, tNodeBuckets);
    return (tNumLocalElements);
}

int PhysicsProxyApp::getGlobalNumNodes() const
{
    assert(mMeshData != nullptr);

    std::vector<size_t> tCommMeshCounts;
    stk::mesh::BulkData & tBulkData = mMeshData->bulk_data();
    stk::mesh::comm_mesh_counts(tBulkData, tCommMeshCounts);
    const size_t tGlobalNumNodes = tCommMeshCounts[stk::topology::NODE_RANK];
    return (tGlobalNumNodes);
}

int PhysicsProxyApp::getGlobalNumElements() const
{
    assert(mMeshData != nullptr);

    std::vector<size_t> tCommMeshCounts;
    stk::mesh::BulkData & tBulkData = mMeshData->bulk_data();
    stk::mesh::comm_mesh_counts(tBulkData, tCommMeshCounts);
    const size_t tGlobalNumElements = tCommMeshCounts[stk::topology::ELEMENT_RANK];
    return (tGlobalNumElements);
}

void PhysicsProxyApp::getSubDomainOwnedGlobalIDs(std::vector<int> & aInput) const
{
    assert(mGlobalIDsOwned.empty() == false);

    aInput.clear();
    const int tSize = mGlobalIDsOwned.size();
    aInput.resize(tSize);
    std::copy(mGlobalIDsOwned.begin(), mGlobalIDsOwned.end(), aInput.begin());
}

void PhysicsProxyApp::getSubDomainOwnedAndSharedGlobalIDs(std::vector<int> & aInput) const
{
    assert(mGlobalIDsOwnedAndShared.empty() == false);

    aInput.clear();
    const int tSize = mGlobalIDsOwnedAndShared.size();
    aInput.resize(tSize);
    std::copy(mGlobalIDsOwnedAndShared.begin(), mGlobalIDsOwnedAndShared.end(), aInput.begin());
}

} // namespace Plato
