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
