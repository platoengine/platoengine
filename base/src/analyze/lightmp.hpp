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
