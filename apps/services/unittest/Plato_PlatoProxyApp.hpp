/*
 * Plato_PlatoProxyApp.hpp
 *
 *  Created on: Oct 17, 2017
 */

#ifndef PLATO_PLATOPROXYAPP_HPP_
#define PLATO_PLATOPROXYAPP_HPP_

#include <mpi.h>

#include "Plato_Application.hpp"

namespace Plato
{

class SharedData;

class PlatoProxyApp : public Plato::Application
{
public:
    PlatoProxyApp(const std::string & aInputMeshFile, const MPI_Comm & aAppComm);
    virtual ~PlatoProxyApp();

    void exportDataMap(const Plato::data::layout_t & aDataLayout, std::vector<int> & aMyOwnedGlobalIDs);

    void finalize();
    void initialize();
    void compute(const std::string & aOperationName);
    void exportData(const std::string & aArgumentName, Plato::SharedData& aExportData);
    void importData(const std::string & aArgumentName, const Plato::SharedData& aImportData);

    void getSubDomainOwnedGlobalIDs(std::vector<int> & aInput) const;
    void getSubDomainOwnedAndSharedGlobalIDs(std::vector<int> & aInput) const;

private:
    MPI_Comm mAppComm;
    std::string mInputMeshFile;

    std::vector<double> mObjectiveValue;
    std::vector<double> mDesignVariables;
    std::vector<double> mObjectiveGradient;

    std::vector<int> mGlobalIDsOwned;
    std::vector<int> mGlobalIDsOwnedAndShared;

private:
    PlatoProxyApp(const Plato::PlatoProxyApp& aRhs);
    Plato::PlatoProxyApp& operator=(const Plato::PlatoProxyApp& aRhs);
};

} // namespace Plato

#endif /* PLATO_PLATOPROXYAPP_HPP_ */
