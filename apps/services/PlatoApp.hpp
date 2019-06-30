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

#include "lightmp.hpp"

#include "Plato_LocalOperation.hpp"
#include "Plato_Application.hpp"
#include "Plato_Exceptions.hpp"
#include "Plato_TimersTree.hpp"

#ifdef GEOMETRY
#include "Plato_MLS.hpp"
#endif

namespace pugi
{

class xml_document;

}

namespace Plato
{

class SharedData;
class AbstractFilter;

}

/******************************************************************************//**
 * @brief PLATO Application
**********************************************************************************/
class PlatoApp : public Plato::Application
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aLocalComm local communicator
    **********************************************************************************/
    PlatoApp(MPI_Comm& aLocalComm);

    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aArgc input arguments
     * @param [in] aArgv input arguments
     * @param [in] aLocalComm local communicator
    **********************************************************************************/
    PlatoApp(int aArgc, char **aArgv, MPI_Comm& aLocalComm);

    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aPhysics_XML_File interface input file
     * @param [in] aApp_XML_File PLATO application input file
     * @param [in] aLocalComm local communicator
    **********************************************************************************/
    PlatoApp(const std::string &aPhysics_XML_File, const std::string &aApp_XML_File, MPI_Comm& aLocalComm);

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~PlatoApp();

    /******************************************************************************//**
     * @brief Safely deallocate local memory
    **********************************************************************************/
    void finalize();

    /******************************************************************************//**
     * @brief Safely allocate local memory
    **********************************************************************************/
    void initialize();

    /******************************************************************************//**
     * @brief Perform local operation
     * @param [in] aOperationName local operation name
    **********************************************************************************/
    void compute(const std::string & aOperationName);

    /******************************************************************************//**
     * @brief Import data
     * @param [in] aArgumentName argument name used to identify import data
     * @param [in] aImportData data
    **********************************************************************************/
    void importData(const std::string & aArgumentName, const Plato::SharedData & aImportData);

    /******************************************************************************//**
     * @brief Export local data
     * @param [in] aArgumentName argument name used to identify export data
     * @param [in] aImportData data
    **********************************************************************************/
    void exportData(const std::string & aArgumentName, Plato::SharedData & aImportData);

    /******************************************************************************//**
     * @brief Export parallel graph
     * @param [in] aDataLayout data layout
     * @param [in] aMyOwnedGlobalIDs local rank owned identifiers
    **********************************************************************************/
    void exportDataMap(const Plato::data::layout_t & aDataLayout, std::vector<int> & aMyOwnedGlobalIDs);

    /******************************************************************************//**
     * @brief Return pointer to application-specific services
     * @return pointer to application-specific services
    **********************************************************************************/
    LightMP* getLightMP();

    /******************************************************************************//**
     * @brief Return reference to local communicator
     * @return reference to local communicator
    **********************************************************************************/
    const MPI_Comm& getComm() const;

    /******************************************************************************//**
     * @brief Return pointer to local filter operator
     * @return pointer to local filter operator
    **********************************************************************************/
    Plato::AbstractFilter* getFilter();

    /******************************************************************************//**
     * @brief Return pointer to parallel graph
     * @return pointer to parallel graph
    **********************************************************************************/
    SystemContainer* getSysGraph();

    /******************************************************************************//**
     * @brief Return pointer to local mesh services
     * @return pointer to local mesh services
    **********************************************************************************/
    MeshServices* getMeshServices();

    /******************************************************************************//**
     * @brief Return pointer to timer services
     * @return pointer to timer services
    **********************************************************************************/
    Plato::TimersTree* getTimersTree();

    /******************************************************************************//**
     * @brief Return element field
     * @param [in] aName field name
     * @return element field
    **********************************************************************************/
    VarIndex getElementField(const std::string & aName);

    /******************************************************************************//**
     * @brief Return distributed node field
     * @param [in] aName field name
     * @return pointer to node field
    **********************************************************************************/
    DistributedVector* getNodeField(const std::string & aName);

    /******************************************************************************//**
     * @brief Return scalar values
     * @param [in] aName quantity name
     * @return pointer to array of values
    **********************************************************************************/
    std::vector<double>* getValue(const std::string & aName);

#ifdef GEOMETRY

    /******************************************************************************//**
     * @brief Return Moving Least Squared (MLS) data
     * @return reference to MLS data
    **********************************************************************************/
    std::map<std::string,std::shared_ptr<Plato::MLSstruct>>& getMovingLeastSquaredData()
        {return mMLS;}
#endif

private:
    /******************************************************************************//**
     * @brief Import data operation
     * @param [in] aArgumentName name used to identify data
     * @param [in] aImportData data
    **********************************************************************************/
    template<typename SharedDataT>
    void importDataT(const std::string& aArgumentName, const SharedDataT& aImportData);

    /******************************************************************************//**
     * @brief Export local data operation
     * @param [in] aArgumentName name used to identify data
     * @param [in/out] aExportData data
    **********************************************************************************/
    template<typename SharedDataT>
    void exportDataT(const std::string& aArgumentName, SharedDataT& aExportData);

    /******************************************************************************//**
     * @brief Parsing exception handler
     * @param [in] aName input data name
     * @param [in] aValueMap name-data map
    **********************************************************************************/
    template<typename ValueType>
    void throwParsingException(const std::string & aName, const std::map<std::string, ValueType> & aValueMap);

    /******************************************************************************//**
     * @brief Create local operation
     * @param [in] aOperation local operation
    **********************************************************************************/
    void createLocalData(Plato::LocalOp* aOperation);

    /******************************************************************************//**
     * @brief Create local arguments
     * @param [in] aArguments arguments associated with an operation
    **********************************************************************************/
    void createLocalData(Plato::LocalArg aArguments);

private:
    MPI_Comm mLocalComm; /*!< local communicator */
    LightMP* mLightMp; /*!< application-specific services */
    SystemContainer* mSysGraph; /*!< parallel graph services */
    MeshServices* mMeshServices; /*!< mesh services */
    Plato::AbstractFilter* mFilter; /*!< filter services */
    Plato::InputData mAppfileData; /*!< PLATO application input data */
    Plato::InputData mInputfileData; /*!< Shared input data */

#ifdef GEOMETRY
    std::map<std::string,std::shared_ptr<Plato::MLSstruct>> mMLS;  /*!< Moving Least Squared (MLS) metadata */
#endif

    std::map<std::string, VarIndex> mElementFieldMap; /*!< Name - Element Field map */
    std::map<std::string, DistributedVector*> mNodeFieldMap; /*!< Name - Node Field map */
    std::map<std::string, std::vector<double>*> mValueMap; /*!< Name - Scalar values map */
    std::map<std::string, Plato::LocalOp*> mOperationMap; /*!< Name - Operation map */

    Plato::TimersTree* mTimersTree; /*!< timer tools/services */
};
// class PlatoApp

