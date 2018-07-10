#include <lightmp.hpp>
#include <Plato_Application.hpp>
#include <Plato_Exceptions.hpp>
#include <Plato_Interface.hpp>
#include <Plato_PenaltyModel.hpp>
#include <Plato_SharedData.hpp>

#ifndef NDEBUG
#include <fenv.h>
#endif

/******************************************************************************/
class LocalApp : public Plato::Application
/******************************************************************************/
{
public:
    LocalApp(int aArgc, char **aArgv);
    virtual ~LocalApp();

    void finalize();
    void initialize();
    void compute(const std::string & aOperationName);
    void importData(const std::string & aArgumentName, const Plato::SharedData & aImportData);
    void exportData(const std::string & aArgumentName, Plato::SharedData & aExportData);
    void exportDataMap(const Plato::data::layout_t & aDataLayout, std::vector<int> & aMyOwnedGlobalIDs);

private:
    // functions
    void throwParsingException(std::string aArgumentName, const std::map<std::string, std::vector<double>*>& aValueMap);
    void throwParsingException(std::string aArgumentName, const std::map<std::string, DistributedVector*>& aFieldMap);

    // data
    LightMP* m_lightmp;
    SystemContainer* m_sysGraph_3D;
    SystemContainer* m_sysGraph_1D;
    SolidStatics* m_statics;
    DistributedVector* m_displacement;
    DistributedVector* m_forcingVector;
    DistributedCrsMatrix* m_stiffnessMatrix;
    Plato::InputData m_inputData;

    Plato::PenaltyModel* m_penaltyModel;

    std::map<std::string, DistributedVector*> m_fieldMap;
    std::map<std::string, std::vector<double>*> m_valueMap;

    bool m_firstTime;
    double m_firstValue;
};

/******************************************************************************/
int main(int argc, char **argv)
/******************************************************************************/
{
#ifndef NDEBUG
    feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif

    MPI_Init(&argc, &argv);

    Plato::Interface* platoInterface = nullptr;
    try
    {
        platoInterface = new Plato::Interface();
    }
    catch(...)
    {
        MPI_Finalize();
        exit(0);
    }

    MPI_Comm localComm;
    platoInterface->getLocalComm(localComm);
    WorldComm.init(localComm);

    LocalApp* myApp = nullptr;
    try
    {
        myApp = new LocalApp(argc, argv);
    }
    catch(...)
    {
        MPI_Finalize();
        exit(0);
    }

    try
    {
        platoInterface->registerPerformer(myApp);
    }
    catch(...)
    {
        MPI_Finalize();
        exit(0);
    }

    try
    {
        platoInterface->perform();
    }
    catch(...)
    {
    }

    delete myApp;

    MPI_Finalize();
}

/******************************************************************************/
void LocalApp::importData(const std::string & aArgumentName, const Plato::SharedData & aImportData)
/******************************************************************************/
{
    if(aImportData.myLayout() == Plato::data::layout_t::SCALAR_FIELD)
    {
        auto tIterator = m_fieldMap.find(aArgumentName);
        if(tIterator == m_fieldMap.end())
        {
            throwParsingException(aArgumentName, m_fieldMap);
        }

        DistributedVector* tLocalData = tIterator->second;
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
    else if(aImportData.myLayout() == Plato::data::layout_t::SCALAR)
    {
        auto tIterator = m_valueMap.find(aArgumentName);
        if(tIterator == m_valueMap.end())
        {
            throwParsingException(aArgumentName, m_valueMap);
        }

        std::vector<double>* tLocalData = tIterator->second;
        aImportData.getData(*tLocalData);
    }
}

/******************************************************************************/
void LocalApp::exportData(const std::string & aArgumentName, Plato::SharedData & aExportData)
/******************************************************************************/
{
    if(aExportData.myLayout() == Plato::data::layout_t::SCALAR_FIELD)
    {
        auto tIterator = m_fieldMap.find(aArgumentName);
        if(tIterator == m_fieldMap.end())
        {
            throwParsingException(aArgumentName, m_fieldMap);
        }

        DistributedVector* tLocalData = tIterator->second;

        tLocalData->LocalExport();
        double* tDataView;
        tLocalData->getEpetraVector()->ExtractView(&tDataView);

        int tMyLength = tLocalData->getEpetraVector()->MyLength();
        //assert(tMyLength == aExportData.size());
        std::vector<double> tExportData(tMyLength);
        std::copy(tDataView, tDataView + tMyLength, tExportData.begin());

        aExportData.setData(tExportData);
    }
    else if(aExportData.myLayout() == Plato::data::layout_t::SCALAR)
    {
        auto tIterator = m_valueMap.find(aArgumentName);
        if(tIterator == m_valueMap.end())
        {
            throwParsingException(aArgumentName, m_valueMap);
        }

        std::vector<double>* tLocalData = tIterator->second;
        aExportData.setData(*tLocalData);
    }
}

/******************************************************************************/
void LocalApp::exportDataMap(const Plato::data::layout_t & aDataLayout, std::vector<int> & aMyOwnedGlobalIDs)
/******************************************************************************/
{
    aMyOwnedGlobalIDs.clear();

    if(aDataLayout == Plato::data::layout_t::VECTOR_FIELD)
    {
        // Plato::data::layout_t = VECTOR_FIELD
        int tMyNumElements = m_sysGraph_3D->getRowMap()->NumMyElements();
        aMyOwnedGlobalIDs.resize(tMyNumElements);
        m_sysGraph_3D->getRowMap()->MyGlobalElements(aMyOwnedGlobalIDs.data());
    }
    else if(aDataLayout == Plato::data::layout_t::SCALAR_FIELD)
    {
        // Plato::data::layout_t = SCALAR_FIELD
        int tMyNumElements = m_sysGraph_3D->getNodeRowMap()->NumMyElements();
        aMyOwnedGlobalIDs.resize(tMyNumElements);
        m_sysGraph_3D->getNodeRowMap()->MyGlobalElements(aMyOwnedGlobalIDs.data());
    }
    else
    {
        // TODO: THROW
    }
}

/******************************************************************************/
void LocalApp::finalize()
{
}
/******************************************************************************/

/******************************************************************************/
void LocalApp::initialize()
/******************************************************************************/
{
    // define the system graph for solid mechanics
    const int dofsPerNode_3D = 3;
    m_sysGraph_3D = new SystemContainer(m_lightmp->getMesh(), dofsPerNode_3D, m_lightmp->getInput());

    const int dofsPerNode_1D = 1;
    m_sysGraph_1D = new SystemContainer(m_lightmp->getMesh(), dofsPerNode_1D, m_lightmp->getInput());

    // define a distributed global stiffness matrix
    m_stiffnessMatrix = new DistributedCrsMatrix(m_sysGraph_3D);

    // define a distributed global forcing matrix
    vector<VarIndex> forcing(dofsPerNode_3D);
    DataContainer* dataContainer = m_lightmp->getDataContainer();
    bool plottable = true;
    forcing[0] = dataContainer->registerVariable(RealType, "fvecx", NODE, !plottable);
    forcing[1] = dataContainer->registerVariable(RealType, "fvecy", NODE, !plottable);
    forcing[2] = dataContainer->registerVariable(RealType, "fvecz", NODE, !plottable);
    m_forcingVector = new DistributedVector(m_sysGraph_3D, forcing);

    // define displacement vector
    vector<VarIndex> displacements(dofsPerNode_3D);
    displacements[0] = dataContainer->registerVariable(RealType, "dispx", NODE, plottable);
    displacements[1] = dataContainer->registerVariable(RealType, "dispy", NODE, plottable);
    displacements[2] = dataContainer->registerVariable(RealType, "dispz", NODE, plottable);
    m_displacement = new DistributedVector(m_sysGraph_3D, displacements);

    // build system
    m_statics = new SolidStatics(*m_sysGraph_3D, *m_lightmp);

    m_lightmp->finalizeSetup();

    // parse Operation spec

    if( m_inputData.size<Plato::InputData>("Operation") > 1 )
    {
        throw Plato::ParsingException("Plato::Statics: multiple Operations defined. currently only one allowed");
    }

    // get penalty model
    Plato::PenaltyModelFactory pmFactory;
    auto tNode = Plato::Get::InputData(m_inputData, "Operation");
    m_penaltyModel = pmFactory.create(tNode);

    // create local storage for arguments

    string name;
    vector<VarIndex> newData(1);
    name = "Topology";
    newData[0] = dataContainer->registerVariable(RealType, name, NODE, plottable);
    m_fieldMap[name] = new DistributedVector(m_sysGraph_1D, newData);

    name = "Internal Energy Gradient";
    newData[0] = dataContainer->registerVariable(RealType, name, NODE, plottable);
    m_fieldMap[name] = new DistributedVector(m_sysGraph_1D, newData);

    name = "Internal Energy";
    vector<double>* newValue = new vector<double>(1, 0.0);
    m_valueMap[name] = newValue;
}

/******************************************************************************/
void LocalApp::throwParsingException(std::string aArgumentName, const std::map<std::string, std::vector<double>*>& aValueMap)
/******************************************************************************/
{
    std::stringstream message;
    message << "Cannot find specified Argument: '" << aArgumentName << "'" << std::endl;
    message << "    Available Arguments: " << std::endl;
    for(auto f : aValueMap)
    {
        message << "\t" << f.first << std::endl;
    }
    Plato::ParsingException pe(message.str());
    throw pe;
}

/******************************************************************************/
void LocalApp::throwParsingException(std::string aArgumentName, const std::map<std::string, DistributedVector*>& aFieldMap)
/******************************************************************************/
{
    std::stringstream message;
    message << "Cannot find specified Argument: '" << aArgumentName << "'" << std::endl;
    message << "    Available Arguments: " << std::endl;
    for(auto f : aFieldMap)
    {
        message << "\t" << f.first << std::endl;
    }
    Plato::ParsingException pe(message.str());
    throw pe;
}

/******************************************************************************/
void LocalApp::compute(const std::string & aOperationName)
/******************************************************************************/
{
    DistributedVector *topology = m_fieldMap["Topology"];

    DistributedVector *iegradient = m_fieldMap["Internal Energy Gradient"];

    double* internalEnergy = m_valueMap["Internal Energy"]->data();

    // compute divergence and update forcing
    m_forcingVector->PutScalar(0.0);
    m_statics->computeExternalForces(*m_forcingVector, /*currentTime=*/0.0);

    // compute stiffness 
    m_statics->buildStiffnessMatrix(*m_stiffnessMatrix, *topology, m_penaltyModel);
    m_statics->applyConstraints(*m_stiffnessMatrix, *m_forcingVector, /*currentTime=*/0.0);

    m_statics->updateDisplacement(*m_displacement, *m_forcingVector, *m_stiffnessMatrix,
    /*currentTime=*/0.0);

    // update stress field for output and to check residual
    m_statics->updateMaterialState( /*currentTime=*/0.0);

    // compute internal energy and gradient
    iegradient->PutScalar(0.0);
    m_statics->computeInternalEnergy(topology, m_penaltyModel, internalEnergy, iegradient);

    // if first time, save first value
    if(m_firstTime)
    {
        m_firstTime = false;
        m_firstValue = *internalEnergy;
    }

    // normalize
    *internalEnergy = (*internalEnergy) / m_firstValue;
    double* gradient_data;
    iegradient->ExtractView(&gradient_data);
    int gradLen = iegradient->MyLength();
    for(int i = 0; i < gradLen; i++)
    {
        gradient_data[i] /= m_firstValue;
    }

    m_lightmp->WriteOutput();
}

/******************************************************************************/
LocalApp::~LocalApp()
/******************************************************************************/
{
    if(m_lightmp)
        delete m_lightmp;
    if(m_sysGraph_3D)
        delete m_sysGraph_3D;
    if(m_sysGraph_1D)
        delete m_sysGraph_1D;
    if(m_stiffnessMatrix)
        delete m_stiffnessMatrix;
    if(m_forcingVector)
        delete m_forcingVector;
    if(m_displacement)
        delete m_displacement;
    if(m_statics)
        delete m_statics;
    if(m_penaltyModel)
        delete m_penaltyModel;
}

/******************************************************************************/
LocalApp::LocalApp(int aArgc, char **aArgv) :
        m_lightmp(NULL),
        m_sysGraph_3D(NULL),
        m_sysGraph_1D(NULL),
        m_statics(NULL),
        m_displacement(NULL),
        m_forcingVector(NULL),
        m_stiffnessMatrix(NULL),
        m_inputData("Input Data"),
        m_penaltyModel(NULL),
        m_fieldMap(),
        m_valueMap(),
        m_firstTime(true),
        m_firstValue(0.)
/******************************************************************************/
{
    const char* input_char = getenv("PLATO_APP_FILE");
    Plato::Parser* parser = new Plato::PugiParser();
    m_inputData = parser->parseFile(input_char);

    // create the FEM utility object
    string tInputFile;
    if(aArgc == 2)
    {
        tInputFile = aArgv[1];
    }
    else
    {
        tInputFile = "lightmp.xml";
    }

    m_lightmp = new LightMP(tInputFile);
}

