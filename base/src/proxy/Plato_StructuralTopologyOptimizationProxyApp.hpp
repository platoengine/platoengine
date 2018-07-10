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
    int mNumElemXDirection;
    int mNumElemYDirection;

    double mPoissonRatio;
    double mElasticModulus;

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
