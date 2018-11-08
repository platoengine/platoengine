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
 * Plato_StructuralTopologyOptimizationProxyApp.hpp
 *
 *  Created on: Nov 16, 2017
 */

#ifndef PLATO_STRUCTURALTOPOLOGYOPTIMIZATIONPROXYAPP_HPP_
#define PLATO_STRUCTURALTOPOLOGYOPTIMIZATIONPROXYAPP_HPP_

#include <map>
#include <vector>
#include <string>
#include <memory>

#include "Plato_Application.hpp"

class Epetra_SerialDenseVector;

namespace Plato
{

class SharedData;
class StructuralTopologyOptimization;

class StructuralTopologyOptimizationProxyApp : public Plato::Application
{
public:
    StructuralTopologyOptimizationProxyApp();
    StructuralTopologyOptimizationProxyApp(int aArgc, char **aArgv);
    virtual ~StructuralTopologyOptimizationProxyApp();

    int getNumDesignVariables() const;
    int getGlobalNumDofs() const;

    /******************************************************************************//**
     * @brief Set perturbation parameter used for finite difference gradient approximation.
     * @param [in] aInput perturbation parameter
     **********************************************************************************/
    void disableObjectiveNormalization();

    // ************* DELLOCATE APPLICATION MEMORY *************
    void finalize();
    // ************* ALLOCATE APPLICATION MEMORY *************
    void initialize();
    // ************* PERFORM OPERATION *************
    void compute(const std::string & aOperationName);
    // ************* EXPORT DATA FROM APPLICATION *************
    void exportData(const std::string & aArgumentName, Plato::SharedData & aExportData);
    // ************* IMPORT DATA FROM APPLICATION *************
    void importData(const std::string & aArgumentName, const Plato::SharedData & aImportData);
    // ************* EXPORT OWNED DATA MAP (I.E. GRAPH) *************
    void exportDataMap(const Plato::data::layout_t & aDataLayout, std::vector<int> & aMyOwnedGlobalIDs);

private:
    void makeGraph();
    void makeDataMap();

    void inputData(const std::string & aArgumentName, const Plato::SharedData & aImportData);
    void outputData(const std::string & aArgumentName, Plato::SharedData & aExportData);

    void solvePartialDifferentialEquation();
    void evaluateConstraint();
    void evaluateObjective();
    void computeObjectiveGradient();
    void computeConstraintGradient();
    void computeFilteredObjectiveGradient();

private:
    bool mNormalizeObjFunc;
    bool mIsFirstObjFuncValueSet;

    int mNumElemXDirection;
    int mNumElemYDirection;

    double mPoissonRatio;
    double mElasticModulus;
    double mFirstObjFuncValue;

    std::vector<int> mMyStateOwnedGlobalIDs;
    std::vector<int> mMyControlOwnedGlobalIDs;

    std::shared_ptr<Plato::StructuralTopologyOptimization> mPDE;
    std::map<std::string, std::shared_ptr<Epetra_SerialDenseVector>> mDataMap;

private:
    StructuralTopologyOptimizationProxyApp(const Plato::StructuralTopologyOptimizationProxyApp & aRhs);
    Plato::StructuralTopologyOptimizationProxyApp & operator=(const Plato::StructuralTopologyOptimizationProxyApp & aRhs);
};

} // namespace Plato

#endif /* PLATO_STRUCTURALTOPOLOGYOPTIMIZATIONPROXYAPP_HPP_ */
