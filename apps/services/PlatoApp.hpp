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

//class Interface;
class SharedData;
class AbstractFilter;

}

/******************************************************************************/
class PlatoApp : public Plato::Application
/******************************************************************************/
{
public:
    PlatoApp(MPI_Comm& aLocalComm);
    PlatoApp(int aArgc, char **aArgv, MPI_Comm& aLocalComm);
    PlatoApp(const std::string &aPhysics_XML_File, const std::string &aApp_XML_File, MPI_Comm& aLocalComm);
    virtual ~PlatoApp();

    void finalize();
    void initialize();
    void compute(const std::string & aOperationName);
    void importData(const std::string & aArgumentName, const Plato::SharedData & aImportData);
    void exportData(const std::string & aArgumentName, Plato::SharedData & aImportData);
    void exportDataMap(const Plato::data::layout_t & aDataLayout, std::vector<int> & aMyOwnedGlobalIDs);

    /******************************************************************************/
    template<typename SharedDataT>
    void importDataT(const std::string& aArgumentName, const SharedDataT& aImportData)
    /******************************************************************************/
    {
        if(aImportData.myLayout() == Plato::data::layout_t::SCALAR_FIELD)
        {
            DistributedVector* tLocalData = getNodeField(aArgumentName);
    
            int tMyLength = tLocalData->getEpetraVector()->MyLength();
            assert(tMyLength == aImportData.size());
            std::vector<double> tImportData(tMyLength);
            aImportData.getData(tImportData);
    
            double* tDataView;
            tLocalData->getEpetraVector()->ExtractView(&tDataView);
            std::copy(tImportData.begin(), tImportData.end(), tDataView);
    
            tLocalData->Import();
            tLocalData->DisAssemble();
        }
        else if(aImportData.myLayout() == Plato::data::layout_t::ELEMENT_FIELD)
        {
            auto dataContainer = mLightMp->getDataContainer();
            double* tDataView; dataContainer->getVariable(getElementField(aArgumentName), tDataView);
            int tMyLength = mLightMp->getMesh()->getNumElems();
    
            assert(tMyLength == aImportData.size());

            std::vector<double> tImportData(tMyLength);
            aImportData.getData(tImportData);

            std::copy(tImportData.begin(), tImportData.end(), tDataView);
        }
        else if(aImportData.myLayout() == Plato::data::layout_t::SCALAR)
        {
            std::vector<double>* tLocalData = getValue(aArgumentName);
            tLocalData->resize(aImportData.size());
            aImportData.getData(*tLocalData);
        }
    }

    /******************************************************************************/
    template <typename SharedDataT>
    void exportDataT(const std::string& aArgumentName, SharedDataT& aExportData)
    /******************************************************************************/
    {
        if(aExportData.myLayout() == Plato::data::layout_t::SCALAR_FIELD)
        {
            DistributedVector* tLocalData = getNodeField(aArgumentName);
    
            tLocalData->LocalExport();
            double* tDataView;
            tLocalData->getEpetraVector()->ExtractView(&tDataView);

            int tMyLength = tLocalData->getEpetraVector()->MyLength();
            assert(tMyLength == aExportData.size());
            std::vector<double> tExportData(tMyLength);
            std::copy(tDataView, tDataView + tMyLength, tExportData.begin());
    
            aExportData.setData(tExportData);
        }
        else if(aExportData.myLayout() == Plato::data::layout_t::ELEMENT_FIELD)
        {
            auto dataContainer = mLightMp->getDataContainer();
            double* tDataView; dataContainer->getVariable(getElementField(aArgumentName), tDataView);
            int tMyLength = mLightMp->getMesh()->getNumElems();
    
            assert(tMyLength == aExportData.size());
            std::vector<double> tExportData(tMyLength);
            std::copy(tDataView, tDataView + tMyLength, tExportData.begin());

            aExportData.setData(tExportData);
        }
        else if(aExportData.myLayout() == Plato::data::layout_t::SCALAR)
        {
            std::vector<double>* tLocalData = getValue(aArgumentName);
            if( int(tLocalData->size()) == aExportData.size() )
            {
                aExportData.setData(*tLocalData);
            } else
            if( tLocalData->size() == 1u )
            {
                std::vector<double> retVec(aExportData.size(), (*tLocalData)[0]);
                aExportData.setData(retVec);
            } else
            {
                throw Plato::ParsingException("SharedValued length mismatch.");
            }
        }
    }

    LightMP* getLightMP();
    Plato::AbstractFilter* getFilter();
    SystemContainer* getSysGraph() {return mSysGraph;}

    std::vector<double>* getValue(const std::string & aName);

private:
    DistributedVector*   getNodeField(const std::string & aName);
    VarIndex             getElementField(const std::string & aName);

    template<typename ValueType>
    void throwParsingException(const std::string & aName, const std::map<std::string, ValueType> & aValueMap);

    MPI_Comm mLocalComm;
    LightMP* mLightMp;
    SystemContainer* mSysGraph;
    MeshServices* mMeshServices;
    Plato::AbstractFilter* mFilter;
  #ifdef GEOMETRY
    struct MLSstruct { Plato::any mls; int dimension; };
    std::map<std::string,std::shared_ptr<MLSstruct>> mMLS;
  #endif
    Plato::InputData mAppfileData;
    Plato::InputData mInputfileData;

    struct LocalArg
    {
        LocalArg(const Plato::data::layout_t & aLayout, const std::string & aName, int aLength = 0, bool aWrite = false) :
                mLayout(aLayout),
                mName(aName),
                mLength(aLength),
                mWrite(aWrite)
        {
        }
        Plato::data::layout_t mLayout;
        std::string mName;
        int mLength;
        bool mWrite;
    };

    class LocalOp
    {
    public:
        LocalOp(PlatoApp* p) :
                mPlatoApp(p)
        {
        }
        virtual ~LocalOp()
        {
        }
        virtual void operator()()=0;
        virtual void getArguments(std::vector<LocalArg>& localArgs)=0;
    protected:
        PlatoApp* mPlatoApp;
    };

    class WriteGlobalValue : public LocalOp
    {
    public:
        WriteGlobalValue(PlatoApp* aPlatoApp, Plato::InputData& aNode);
        void operator()();
        void getArguments(std::vector<LocalArg>& aLocalArgs);
      private:
        std::string m_inputName;
        std::string m_filename;
        int m_size;
    };
    friend class WriteGlobalValue;

    class ReciprocateObjectiveValue : public LocalOp
    {
    public:
        ReciprocateObjectiveValue(PlatoApp* aPlatoApp, Plato::InputData& aNode);
        void operator()();
        void getArguments(std::vector<LocalArg>& aLocalArgs);
      private:
        std::string m_outputName;
        std::string m_inputName;
    };
    friend class ReciprocateObjectiveValue;

    class ReciprocateObjectiveGradient : public LocalOp
    {
    public:
        ReciprocateObjectiveGradient(PlatoApp* aPlatoApp, Plato::InputData& aNode);
        void operator()();
        void getArguments(std::vector<LocalArg>& aLocalArgs);
      private:
        std::string m_refValName;
        std::string m_outputName;
        std::string m_inputName;
    };
    friend class ReciprocateObjectiveGradient;

    class NormalizeObjectiveValue : public LocalOp
    {
    public:
        NormalizeObjectiveValue(PlatoApp* aPlatoApp, Plato::InputData& aNode);
        void operator()();
        void getArguments(std::vector<LocalArg>& aLocalArgs);
      private:
        std::string m_refValName;
        std::string m_outputName;
        std::string m_inputName;
    };
    friend class NormalizeObjectiveValue;

    class NormalizeObjectiveGradient : public LocalOp
    {
    public:
        NormalizeObjectiveGradient(PlatoApp* aPlatoApp, Plato::InputData& aNode);
        void operator()();
        void getArguments(std::vector<LocalArg>& aLocalArgs);
      private:
        std::string m_refValName;
        std::string m_outputName;
        std::string m_inputName;
    };
    friend class NormalizeObjectiveGradient;

    class Aggregator : public LocalOp
    {
    public:
        Aggregator(PlatoApp* aPlatoApp, Plato::InputData& aNode);
        void operator()();
        void getArguments(std::vector<LocalArg>& aLocalArgs);
      private:
        struct AggStruct 
        {
          Plato::data::layout_t layout;
          std::string outputName;
          std::vector<std::string> inputNames;
        };
        double m_limitWeight;
        std::vector<std::string> m_weightBases;
        std::string m_weightMethod;
        std::vector<double> m_weights;
        std::vector<AggStruct> m_aggStructs;
    };
    friend class Aggregator;

    class PlatoMainOutput : public LocalOp
    {
    public:
        PlatoMainOutput(PlatoApp* aPlatoApp, Plato::InputData& aNode);
        ~PlatoMainOutput();
        void operator()();
        void getArguments(std::vector<LocalArg> & aLocalArgs);
    private:
        void extract_iso_surface(int aIteration);
        std::vector<LocalArg> m_outputData;
        int m_outputFrequency;
        int m_outputMethod;
        std::string mDiscretization;
    };
    friend class PlatoMainOutput;

#ifdef GEOMETRY
    template<int SpaceDim, typename ScalarType=double>
    class ComputeMLSField : public LocalOp
    {
    public:
        ComputeMLSField( PlatoApp* aPlatoApp, Plato::InputData& aNode) : 
          LocalOp(aPlatoApp), m_inputName("MLS Point Values"), m_outputName("MLS Field Values") 
        { 
            auto tName = Plato::Get::String(aNode,"MLSName");
            auto& tMLS = mPlatoApp->mMLS;
            if( tMLS.count(tName) == 0 )
            {
                throw Plato::ParsingException("Requested PointArray that doesn't exist.");
            }
            m_MLS = mPlatoApp->mMLS[tName];
        }

        ~ComputeMLSField() { }

        void operator()()
        { 
            std::vector<double>* tLocalData = mPlatoApp->getValue(m_inputName);
            Kokkos::View<ScalarType*, Kokkos::HostSpace, Kokkos::MemoryUnmanaged> t_pointsHost(tLocalData->data(),tLocalData->size());
            Kokkos::View<ScalarType*, Kokkos::DefaultExecutionSpace::memory_space> t_pointValues("point values",tLocalData->size());
            Kokkos::deep_copy(t_pointValues, t_pointsHost);

            auto& tLocalOutput = *(mPlatoApp->getNodeField(m_outputName));
            int tMyLength = tLocalOutput.MyLength();

            Kokkos::View<ScalarType*, Kokkos::DefaultExecutionSpace::memory_space> t_nodeValues("values", tMyLength);
            Kokkos::View<ScalarType**, Kokkos::LayoutRight, Kokkos::DefaultExecutionSpace::memory_space> t_nodeCoords("coords", tMyLength, SpaceDim);
            auto t_nodeCoordsHost = Kokkos::create_mirror_view(t_nodeCoords);
            auto mesh = mPlatoApp->mLightMp->getMesh();
            {
                auto tCoordsSub = Kokkos::subview(t_nodeCoordsHost, Kokkos::ALL(), /*dim_index=*/ 0);
                Kokkos::View<ScalarType*, Kokkos::HostSpace, Kokkos::MemoryUnmanaged> t_coord(mesh->getX(),tMyLength);
                Kokkos::deep_copy(tCoordsSub, t_coord);
            }
            if( SpaceDim > 1 )
            {
                auto tCoordsSub = Kokkos::subview(t_nodeCoordsHost, Kokkos::ALL(), /*dim_index=*/ 1);
                Kokkos::View<ScalarType*, Kokkos::HostSpace, Kokkos::MemoryUnmanaged> t_coord(mesh->getY(),tMyLength);
                Kokkos::deep_copy(tCoordsSub, t_coord);
            }
            if( SpaceDim > 2 )
            {
                auto tCoordsSub = Kokkos::subview(t_nodeCoordsHost, Kokkos::ALL(), /*dim_index=*/ 2);
                Kokkos::View<ScalarType*, Kokkos::HostSpace, Kokkos::MemoryUnmanaged> t_coord(mesh->getZ(),tMyLength);
                Kokkos::deep_copy(tCoordsSub, t_coord);
            }
            Kokkos::deep_copy(t_nodeCoords, t_nodeCoordsHost);

            Plato::any_cast<MLS_Type>(m_MLS->mls).f(t_pointValues, t_nodeCoords, t_nodeValues);

            double* tDataView;
            tLocalOutput.ExtractView(&tDataView);
            Kokkos::View<ScalarType*, Kokkos::HostSpace, Kokkos::MemoryUnmanaged> t_nodeValuesHost(tDataView,tMyLength);
            Kokkos::deep_copy(t_nodeValuesHost, t_nodeValues);
        }

        void getArguments(std::vector<LocalArg>& aLocalArgs)
        { 
            int tNumPoints = Plato::any_cast<MLS_Type>(m_MLS->mls).getNumPoints();
            aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR_FIELD, m_outputName}); 
            aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR, m_inputName, tNumPoints }); 
        }

    private:
        shared_ptr<MLSstruct> m_MLS;
        typedef typename Plato::Geometry::MovingLeastSquares<SpaceDim, ScalarType> MLS_Type;
        const std::string m_inputName;
        const std::string m_outputName;
    };
    template<int SpaceDim, typename ScalarType> friend class ComputeMLSField;

    template<int SpaceDim, typename ScalarType=double>
    class InitializeMLSPoints : public LocalOp
    {
    public:
        InitializeMLSPoints( PlatoApp* aPlatoApp, Plato::InputData& aNode) : 
          LocalOp(aPlatoApp), m_outputName("MLS Point Values")
        { 
            auto tName = Plato::Get::String(aNode,"MLSName");
            auto& tMLS = mPlatoApp->mMLS;
            if( tMLS.count(tName) == 0 )
            {
                throw Plato::ParsingException("Requested PointArray that doesn't exist.");
            }
            m_MLS = mPlatoApp->mMLS[tName];

            m_fieldName = Plato::Get::String(aNode, "Field");
        }

        ~InitializeMLSPoints() { }

        void operator()()
        { 
            auto tFields = Plato::any_cast<MLS_Type>(m_MLS->mls).getPointFields();
            auto tField  = tFields[m_fieldName];

            std::vector<double>* tLocalData = mPlatoApp->getValue(m_outputName);
            Kokkos::View<ScalarType*, Kokkos::HostSpace, Kokkos::MemoryUnmanaged> tLocalDataHost(tLocalData->data(),tLocalData->size());
            Kokkos::deep_copy(tLocalDataHost, tField);
        }

        void getArguments(std::vector<LocalArg>& aLocalArgs)
        { 
            int tNumPoints = Plato::any_cast<MLS_Type>(m_MLS->mls).getNumPoints();
            aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR, m_outputName, tNumPoints }); 
        }

    private:
        shared_ptr<MLSstruct> m_MLS;
        typedef typename Plato::Geometry::MovingLeastSquares<SpaceDim, ScalarType> MLS_Type;
        const std::string m_outputName;
        std::string m_fieldName;
    };
    template<int SpaceDim, typename ScalarType> friend class InitializeMLSPoints;

    template<int SpaceDim, typename ScalarType=double>
    class MapMLSField : public LocalOp
    {
    public:
        MapMLSField( PlatoApp* aPlatoApp, Plato::InputData& aNode) : 
          LocalOp(aPlatoApp), m_inputName("MLS Field Values"), m_outputName("Mapped MLS Point Values")
        { 
            auto tName = Plato::Get::String(aNode,"MLSName");
            auto& tMLS = mPlatoApp->mMLS;
            if( tMLS.count(tName) == 0 )
            {
                throw Plato::ParsingException("Requested PointArray that doesn't exist.");
            }
            m_MLS = mPlatoApp->mMLS[tName];
        }

        ~MapMLSField() { }

        void operator()()
        { 
            // pull field values into Kokkos::View
            //
            auto& tLocalInput = *(mPlatoApp->getNodeField(m_inputName));
            int tMyLength = tLocalInput.MyLength();
            double* tDataView; tLocalInput.ExtractView(&tDataView);
            Kokkos::View<ScalarType*, Kokkos::HostSpace, Kokkos::MemoryUnmanaged> t_nodeValuesHost(tDataView,tMyLength);
            Kokkos::View<ScalarType*, Kokkos::DefaultExecutionSpace::memory_space> t_nodeValues("values", tMyLength);
            Kokkos::deep_copy(t_nodeValues, t_nodeValuesHost);


            // pull node coordinates into Kokkos::View
            //
            Kokkos::View<ScalarType**, Kokkos::LayoutRight, Kokkos::DefaultExecutionSpace::memory_space> t_nodeCoords("coords", tMyLength, SpaceDim);
            auto t_nodeCoordsHost = Kokkos::create_mirror_view(t_nodeCoords);
            auto mesh = mPlatoApp->mLightMp->getMesh();
            {
                auto tCoordsSub = Kokkos::subview(t_nodeCoordsHost, Kokkos::ALL(), /*dim_index=*/ 0);
                Kokkos::View<ScalarType*, Kokkos::HostSpace, Kokkos::MemoryUnmanaged> t_coord(mesh->getX(),tMyLength);
                Kokkos::deep_copy(tCoordsSub, t_coord);
            }
            if( SpaceDim > 1 )
            {
                auto tCoordsSub = Kokkos::subview(t_nodeCoordsHost, Kokkos::ALL(), /*dim_index=*/ 1);
                Kokkos::View<ScalarType*, Kokkos::HostSpace, Kokkos::MemoryUnmanaged> t_coord(mesh->getY(),tMyLength);
                Kokkos::deep_copy(tCoordsSub, t_coord);
            }
            if( SpaceDim > 2 )
            {
                auto tCoordsSub = Kokkos::subview(t_nodeCoordsHost, Kokkos::ALL(), /*dim_index=*/ 2);
                Kokkos::View<ScalarType*, Kokkos::HostSpace, Kokkos::MemoryUnmanaged> t_coord(mesh->getZ(),tMyLength);
                Kokkos::deep_copy(tCoordsSub, t_coord);
            }
            Kokkos::deep_copy(t_nodeCoords, t_nodeCoordsHost);

            // create output View
            //
            std::vector<double>* tLocalData = mPlatoApp->getValue(m_outputName);
            Kokkos::View<ScalarType*, Kokkos::DefaultExecutionSpace::memory_space> t_mappedValues("mapped values",tLocalData->size());
            
            Plato::any_cast<MLS_Type>(m_MLS->mls).mapToPoints(t_nodeCoords, t_nodeValues, t_mappedValues);

            // pull from View to local data
            //
            Kokkos::View<ScalarType*, Kokkos::HostSpace, Kokkos::MemoryUnmanaged> tLocalDataHost(tLocalData->data(),tLocalData->size());
            Kokkos::deep_copy(tLocalDataHost, t_mappedValues);
        }

        void getArguments(std::vector<LocalArg>& aLocalArgs)
        { 
            int tNumPoints = Plato::any_cast<MLS_Type>(m_MLS->mls).getNumPoints();
            aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR_FIELD, m_inputName }); 
            aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR, m_outputName, tNumPoints }); 
        }

    private:
        shared_ptr<MLSstruct> m_MLS;
        typedef typename Plato::Geometry::MovingLeastSquares<SpaceDim, ScalarType> MLS_Type;
        const std::string m_inputName;
        const std::string m_outputName;
    };
    template<int SpaceDim, typename ScalarType> friend class MapMLSField;


#endif

    class Filter : public LocalOp
    {
    public:
        Filter(PlatoApp* aPlatoApp, Plato::InputData& aNode);
        ~Filter();
        void operator()();
        void getArguments(std::vector<LocalArg>& aLocalArgs);
    private:
        Plato::AbstractFilter* mFilter;
        const std::string m_inputName;
        const std::string m_outputName;
        bool m_transpose;
    };
    friend class Filter;

    class Roughness : public LocalOp
    {
    public:
        Roughness(PlatoApp* aPlatoApp, Plato::InputData & aNode);
        void operator()();
        void getArguments(std::vector<LocalArg> & aLocalArgs);
    private:
        std::string m_topologyName;
        std::string m_roughnessName;
        std::string m_gradientName;
    };
    friend class Roughness;

    class InitializeValues : public LocalOp
    {
    public:
        InitializeValues(PlatoApp* aPlatoApp, Plato::InputData & aNode);
        void operator()();
        void getArguments(std::vector<LocalArg> & aLocalArgs);
    private:
        std::string m_valuesName;
        double m_value;
    };
    friend class InitializeValues;

    class InitializeField : public LocalOp
    {
    public:
        InitializeField(PlatoApp* aPlatoApp, Plato::InputData & aNode);
        void operator()();
        void getArguments(std::vector<LocalArg> & aLocalArgs);
    private:
        void getInitialValuesForRestart(DistributedVector &field, std::vector<double> &values);
        void getInitialValuesForSwissCheeseLevelSet(DistributedVector &field, std::vector<double> &values);
        void getInitialValuesForPrimitivesLevelSet(DistributedVector &field, std::vector<double> &values);
        double evaluateSwissCheeseLevelSet(const double &aX,
                                           const double &aY,
                                           const double &aZ,
                                           std::vector<double> aLowerCoordBoundsOfDomain,
                                           std::vector<double> aUpperCoordBoundsOfDomain,
                                           double aAverageElemLength);
        std::string m_strMethod;
        std::string m_outputFieldName;
        Plato::data::layout_t mOutputLayout;
        double      m_uniformValue;
        std::string m_fileName;
        std::string m_sphereRadius;
        std::string m_spherePackingFactor;
        bool m_createSpheres;
        std::string m_sphereSpacingX;
        std::string m_sphereSpacingY;
        std::string m_sphereSpacingZ;
        std::string m_variableName;
        std::vector<int> m_levelSetNodesets;
        int m_iteration;
        double m_minCoords[3];
        double m_maxCoords[3];
    };
    friend class InitializeField;

    class EnforceBounds : public LocalOp
    {
    public:
        EnforceBounds(PlatoApp* p, Plato::InputData& node);
        void operator()();
        void getArguments(std::vector<LocalArg>& localArgs);
    private:
        std::string mLowerBoundVectorFieldName;
        std::string mUpperBoundVectorFieldName;
        std::string mTopologyFieldName;
    };
    friend class EnforceBounds;

    class SetLowerBounds : public LocalOp
    {
    public:
        SetLowerBounds(PlatoApp* p, Plato::InputData& node);
        void operator()();
        void getArguments(std::vector<LocalArg>& localArgs);
    private:
        std::string mOutputName;
        Plato::data::layout_t mOutputLayout;
        int mOutputSize;
        std::string mInputName;
        std::string mDiscretization;
        std::vector<int> mFixedBlocks;
        std::vector<int> mFixedSidesets;
        std::vector<int> mFixedNodesets;
    };
    friend class SetLowerBounds;

    class SetUpperBounds : public LocalOp
    {
    public:
        SetUpperBounds(PlatoApp* aPlatoApp, Plato::InputData& aXML_Node);

        void operator()();
        void getArguments(std::vector<LocalArg> & aLocalArgs);
    private:
        std::string mOutputName;
        std::string mDiscretization;
        Plato::data::layout_t mOutputLayout;
        int mOutputSize;
        std::string mInputName;
        std::vector<int> mFixedBlocks;
        std::vector<int> mFixedSidesets;
        std::vector<int> mFixedNodesets;
    };
    friend class SetUpperBounds;

    class DesignVolume : public LocalOp
    {
    public:
        DesignVolume(PlatoApp* p, Plato::InputData& node);
        void operator()();
        void getArguments(std::vector<LocalArg>& aLocalArgs);
    private:
        std::string m_outValueName;
    };
    friend class DesignVolume;

    class ComputeVolume : public LocalOp
    {
    public:
        ComputeVolume(PlatoApp* p, Plato::InputData& node);
        ~ComputeVolume();
        void operator()();
        void getArguments(std::vector<LocalArg>& aLocalArgs);
    private:
        std::string m_topologyName;
        std::string m_volumeName;
        std::string m_gradientName;
        Plato::PenaltyModel* m_penaltyModel;
    };
    friend class ComputeVolume;

    void createLocalData(LocalOp* op);
    void createLocalData(LocalArg arg);

    std::map<std::string, VarIndex> mElementFieldMap;
    std::map<std::string, DistributedVector*> mNodeFieldMap;
    std::map<std::string, std::vector<double>*> mValueMap;
    std::map<std::string, LocalOp*> mOperationMap;

    Plato::TimersTree* mTimersTree;

};

