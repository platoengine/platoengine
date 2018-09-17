/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
*/

/*
 * Plato_PlatoProxyApp.cpp
 *
 *  Created on: Oct 17, 2017
 */

#ifdef STK_ENABLED

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

#endif // STK_ENABLED
