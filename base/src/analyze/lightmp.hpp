#ifndef LIGHTMP
#define LIGHTMP

#include <memory>

// Intrepid includes
#include "Intrepid_FunctionSpaceTools.hpp"
#include "Intrepid_FieldContainer.hpp"
#include "Intrepid_CellTools.hpp"
#include "Intrepid_ArrayTools.hpp"
#include "Intrepid_HGRAD_HEX_C1_FEM.hpp"
#include "Intrepid_RealSpaceTools.hpp"
#include "Intrepid_DefaultCubatureFactory.hpp"
#include "Intrepid_Utils.hpp"

// Epetra includes
#include "Epetra_Time.h"
#include "Epetra_Map.h"
#include "Epetra_FECrsMatrix.h"
#include "Epetra_FEVector.h"
#include "Epetra_SerialComm.h"
#include "Epetra_LinearProblem.h"
#include "AztecOO.h"

// Anasazi includes
#include "AnasaziConfigDefs.hpp"
#include "AnasaziBlockKrylovSchurSolMgr.hpp"
#include "AnasaziBasicEigenproblem.hpp"
#include "AnasaziEpetraAdapter.hpp"

// Teuchos includes
#include "Teuchos_oblackholestream.hpp"
#include "Teuchos_RCP.hpp"
#include "Teuchos_BLAS.hpp"

// Shards includes
#include "Shards_CellTopology.hpp"

// EpetraExt includes
#include "EpetraExt_RowMatrixOut.h"
#include "EpetraExt_MultiVectorOut.h"

#include "Plato_Parser.hpp"
#include "data_mesh.hpp"
#include "data_container.hpp"
#include "topological_element.hpp"
#include "communicator.hpp"
#include "matrix_container.hpp"
#include "mesh_io.hpp"
#include "solid_statics.hpp"
#include "mesh_services.hpp"
#include "material_container.hpp"

#include <string>
#include <vector>

typedef struct { 
  std::vector<std::string> plotName;
  std::vector<int> plotIndex;
  VarIndex varIndex;
} PlotVariable;

void setupSolver( pugi::xml_node& config, AztecOO&, int&, Real&, DistributedCrsMatrix* A=NULL );

/*! 
  LightMP is a top level class that manages the mesh, data, IO, etc.  
*/
class LightMP{
  public:
    LightMP();
    LightMP(std::string inputfile);
    LightMP(std::shared_ptr<pugi::xml_document> inputTree);
    ~LightMP();

    void Parse(std::string inputfile);
    void Setup();
    void finalizeSetup();

    void WriteOutput();
    void closeOutput();

    Real advanceTime();

    void setMaterialContainer( MaterialContainer* mcon ){ myMaterialContainer = mcon; }
    void setCurrentTime(Real t) {currentTime = t; }

    // access functions
    DataMesh*          getMesh()              {return myMesh;}
    DataContainer*     getDataContainer()     {return myDataContainer;}
    MaterialContainer* getMaterialContainer() {return myMaterialContainer;}
    Real               getCurrentTime()       {return currentTime; }
    int                getCurrentStep()       {return stepIndex; }
    Real               getTermTime()          {return termTime; }

    pugi::xml_document&
    getInput() {return *myInputTree;}
    void setWriteTimeStepDuringSetup(bool value) { mWriteTimeStepDuringSetup = value; }


  private:
    bool mWriteTimeStepDuringSetup;
    DataMesh* myMesh;
    DataContainer* myDataContainer;
    MaterialContainer* myMaterialContainer;
    MeshIO* myMeshOutput;
    std::shared_ptr<pugi::xml_document> myInputTree;

    void InitializeOutput();
    bool initPlot();
    bool closePlot(MeshIO* io);
    bool initPlotVars(MeshIO* io);
    bool writePlotVars(MeshIO* io);

    void init();
    bool ParseMesh();
    bool ParseFunctions();
    bool ParseMaterials();
    void ParseOutput();
    void ParseControl();

    std::vector<PlotVariable> myPlotVars;

    bool appendOutput;
    int stepIndex;
    int plotIndex;
    Real currentTime;
    Real timeStep;
    Real termTime;

    void addPlotVariable( AbstractData* d, VarIndex index);
    void writeNodeData(MeshIO* io, AbstractData* d, PlotVariable& plotvar, int time);
    void writeElemData(MeshIO* io, AbstractData* d, PlotVariable& plotvar, int time);

    template <class myType>
    void integrateOnElement(myType *mpdata, myType* eldata);

};

using namespace std;
using namespace Intrepid;

/******************************************************************************/
template <class myType>
void LightMP::integrateOnElement(myType *mpdata, myType* eldata)
/******************************************************************************/
{
  MaterialContainer& mc = *myMaterialContainer;

  int elementIndex = 0;

  // *** Block loop ***
  int nblocks = myMesh->getNumElemBlks();
  for(int ib=0; ib<nblocks; ib++){
    Topological::Element& elblock = *(myMesh->getElemBlk(ib));

    // not all blocks will be present on all processors
    if( elblock.getNumElem() == 0 ) continue;

    Intrepid::FieldContainer<int>& bfc = mc.getBlockDataMap(ib);
    FieldContainer<double>& cubWeights = elblock.getCubatureWeights();
    Real elementMeasure = 0.0;
    int numCubPoints = elblock.getNumIntPoints();
    for (int ipoint=0; ipoint<numCubPoints; ipoint++) 
      elementMeasure += cubWeights(ipoint);

    // *** Element loop ***
    int numElemsThisBlock = elblock.getNumElem();
    for (int iel=0; iel<numElemsThisBlock; iel++) {
      eldata[elementIndex] = 0.0;
      for (int ipoint=0; ipoint<numCubPoints; ipoint++) {
        int dataIndex = bfc(iel, ipoint);
        eldata[elementIndex] += cubWeights(ipoint)*mpdata[dataIndex];
      }
      eldata[elementIndex] /= elementMeasure;
      elementIndex++;
    }
  }
}



#endif
