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
#include <stdlib.h>
#include <exodusII.h>
#include <string>
#include <sstream>
#include "data_mesh.hpp"
#include "data_container.hpp"
#include "mesh_io.hpp"
#include "exception_handling.hpp"
#include "types.hpp"
#include "communicator.hpp"
#include "material_container.hpp"

#include "ml_include.h"
#include "ml_MultiLevelPreconditioner.h"

/******************************************************************************/
LightMP::~LightMP()
/******************************************************************************/
{
  if(myMeshOutput) delete myMeshOutput;
  if(myDataContainer) delete myDataContainer;
  if(myMesh) delete myMesh;
  if(myMaterialContainer) delete myMaterialContainer;
}

/******************************************************************************/
void LightMP::init()
/******************************************************************************/
{
  //  myMeshOutput = new ExodusIO();
  myDataContainer = new DataContainer();
  myMaterialContainer = nullptr;

  currentTime = 0.0;
  plotIndex = 0;
  mWriteTimeStepDuringSetup = true;
}

/******************************************************************************/
LightMP::LightMP() :
        myMesh(nullptr),
        myDataContainer(nullptr),
        myMaterialContainer(nullptr),
        myMeshOutput(nullptr),
        myInputTree(),
        myPlotVars(),
        appendOutput(),
        stepIndex(),
        plotIndex(),
        currentTime(),
        timeStep(),
        termTime()
/******************************************************************************/
{
  init();
}

/******************************************************************************/
void LightMP::Parse(string inputfile)
/******************************************************************************/
{
    myInputTree = std::make_shared<pugi::xml_document>();
    myInputTree->load_file(inputfile.c_str());
    Setup();
}

/******************************************************************************/
void LightMP::Setup()
/******************************************************************************/
{
  
    ParseFunctions();

    // parse and create the mesh object, myMesh
    ParseMesh();

    // parse and create the materials in myMaterialContainer
    ParseMaterials();

    ParseOutput();
    ParseControl();

    
}

/******************************************************************************/
LightMP::LightMP(string inputfile) :
        myMesh(nullptr),
        myDataContainer(nullptr),
        myMaterialContainer(nullptr),
        myMeshOutput(nullptr),
        myInputTree(),
        myPlotVars(),
        appendOutput(),
        stepIndex(),
        plotIndex(),
        currentTime(),
        timeStep(),
        termTime()
/******************************************************************************/
{
  init();
  Parse(inputfile);
}

/******************************************************************************/
LightMP::LightMP(std::shared_ptr<pugi::xml_document> inputTree) :
        myMesh(nullptr),
        myDataContainer(nullptr),
        myMaterialContainer(nullptr),
        myMeshOutput(nullptr),
        myInputTree(inputTree),
        myPlotVars(),
        appendOutput(),
        stepIndex(),
        plotIndex(),
        currentTime(),
        timeStep(),
        termTime()
/******************************************************************************/
{
  init();
  Setup();
}

/******************************************************************************/
bool LightMP::ParseMaterials()
/******************************************************************************/
{

  int ntopos = Plato::Parse::numChildren(*myInputTree, "material_topology");
  int nmatls = Plato::Parse::numChildren(*myInputTree, "material");

  if(ntopos + nmatls == 0) return false;

  // if the material container hasn't been set with setup functions,
  // use the default
  if( myMaterialContainer == nullptr ){
    MCFactory mcf;
    myMaterialContainer = mcf.create(*myInputTree);
  }

  // The material container is responsible for the material topology:
  // 1.  creates and maintains a map from (blockIndex, elemIndex, gaussIndex)
  //     to an index into the material array.
  // 2.  creates and maintains a map from (blockIndex, elemIndex, gaussIndex)
  //     to an index into the data arrays.
  // This service is encapsulated in a virtual class so that external code
  // can be used to specify the material topology and material data access.
  myMaterialContainer->SetUp(myMesh, myDataContainer, *myInputTree);

  myMaterialContainer->Initialize(myMesh, myDataContainer);

  return true;
}

/******************************************************************************/
Real LightMP::advanceTime()
/******************************************************************************/
{
  myDataContainer->advanceState();
  currentTime += timeStep;
  stepIndex++;

  return currentTime;
}


/******************************************************************************/
bool LightMP::ParseFunctions()
/******************************************************************************/
{
 return true;
}

/******************************************************************************/
bool LightMP::ParseMesh()
/******************************************************************************/
{
#ifdef VERBOSE_DEBUG_LOCATION
  _print_entering_location(__AXSIS_FUNCTION_NAMER__);
#endif //VERBOSE_DEBUG_LOCATION

  // read mesh specification block
  pugi::xml_node meshspec = myInputTree->child("mesh");
 
  string meshType = Plato::Parse::getString(meshspec,"type");
  if( meshType == "unstructured" )
    myMesh = new UnsMesh( myDataContainer );
  else if( meshType == "structured" )
    myMesh = new StrMesh( myDataContainer );

  myMesh->parseMesh(meshspec);

  myDataContainer->setNumNodes(myMesh->getNumNodes());
  myDataContainer->setNumElems(myMesh->getNumElems());

  return true;

}

/******************************************************************************/
void LightMP::ParseControl()
/******************************************************************************/
{
  pugi::xml_node control = myInputTree->child("control");
  if( control ){
    pugi::xml_node timespec = control.child("time");
    termTime = Plato::Parse::getDouble( timespec, "end" );
    timeStep = Plato::Parse::getDouble( timespec, "timestep" );
    stepIndex = Plato::Parse::getInt( timespec, "starting_step");
  }
}
  
/******************************************************************************/
void LightMP::ParseOutput()
/******************************************************************************/
{
  pugi::xml_node output = myInputTree->child("output");
  if( output ){

    string append = Plato::Parse::getString(output, "append");
    if( append == "true" ) appendOutput = true;
    else appendOutput = false;

    string outputfile;

    // determine file basename
    string outfilename = Plato::Parse::getString(output,"file");
    outputfile = Plato::Parse::getString(output,"file");

    // determine format and create the file
    string fileformat = Plato::Parse::getString(output,"format");
    if( fileformat == "exodus" || fileformat == "format_not_set" ){
        outputfile += ".exo";
        ostringstream buffer;
        ostringstream zeros;
        buffer << outputfile;
        int my_pid = WorldComm.GetPID();
        int my_sze = WorldComm.GetSize();
        int div = 10;
        int quo = my_sze/div;
        int count = 0;
        while(quo>0) {
          count++;
          div*=10;
          quo = my_sze/div;
        }
        div = 10;
        quo = (my_pid/div);
        while(quo>0) {
          count--;
          div*=10;
          quo = my_pid/div;
        }
        for(int i=0;i<count;i++) zeros << '0';
        buffer << "." << WorldComm.GetSize() << "." << zeros.str() << WorldComm.GetPID();
        outputfile = buffer.str();
        if( WorldComm.GetSize() == 1 ) {
            myMeshOutput = new ExodusIO();
        } else {
            myMeshOutput = new NemesisIO();
        }
        MeshIO::Type outputType;
        if( appendOutput ) outputType = MeshIO::WRITE;
        else outputType = MeshIO::CLOBBER;
        myMeshOutput->initMeshIO( myMesh, 
                                  myDataContainer,
                                  outputfile.c_str(),
                                  outputType);
        
    }
    else {
        throw ParsingException("unrecognized output format");
    }
  }
}
/******************************************************************************/
void LightMP::WriteOutput()
/******************************************************************************/
{
  MeshIO *io = myMeshOutput;
  if(io){
    if(!writePlotVars(io)) {
      pXcout << "!!!ERROR: Problem writing output to plot file"
             << endl;
    }
  }
}

/******************************************************************************/
void LightMP::closeOutput()
/******************************************************************************/
{
  closePlot(myMeshOutput);
}

/******************************************************************************/
bool LightMP::writePlotVars(MeshIO* io)
/******************************************************************************/
{
#ifdef DEBUG_LOCATION
  _print_entering_location(__AXSIS_FUNCTION_NAMER__);
#endif //DEBUG_LOCATION
  io->writeTime(++plotIndex, currentTime);

  // write data
  size_t num_plot = myPlotVars.size();
  for(size_t i=0; i<num_plot; i++) {
    PlotVariable& plotVar = myPlotVars[i];
    VarIndex vid = plotVar.varIndex;
    AbstractData* d = myDataContainer->getAbstractVariable(vid);
    if( d->getCentering() == NODE )
      writeNodeData( io, d, plotVar, plotIndex );
    else
      writeElemData( io, d, plotVar, plotIndex );
  }

  return true;
}

/******************************************************************************/
void LightMP::writeNodeData(MeshIO* io, AbstractData* d, PlotVariable& plotvar, int timestep)
/******************************************************************************/
{
  AbstractData& ad = *d;
  DataType dt = ad.getDataType();
  string base = ad.getName();
  if( dt == RealType ){
    Real *data; ad.getData( data );
    assert( plotvar.plotIndex.size() == 1 );
    io->writeNodePlot(data, plotvar.plotIndex[0], timestep);
  } else if( dt == VectorType ){
    int numNodes = myDataContainer->getNumNodes();
    Vector *data; ad.getData( data );
    Real *vx = new Real[numNodes];
    Real *vy = new Real[numNodes];
    Real *vz = new Real[numNodes];
    for(int i=0; i<numNodes; i++){
      vx[i] = data[i][VEC::X];
      vy[i] = data[i][VEC::Y];
      vz[i] = data[i][VEC::Z];
    }
    io->writeNodePlot(vx, plotvar.plotIndex[0], timestep);
    io->writeNodePlot(vy, plotvar.plotIndex[1], timestep);
    io->writeNodePlot(vz, plotvar.plotIndex[2], timestep);
 
    delete [] vx;
    delete [] vy;
    delete [] vz;
  }
  //TODO: implement other types.  symtensor and tensor nodal fields?
}
/******************************************************************************/
void LightMP::writeElemData(MeshIO* io, AbstractData* d, PlotVariable& plotvar, int timestep)
/******************************************************************************/
{
  AbstractData& ad = *d;
  DataType dt = ad.getDataType();
  string base = ad.getName();
  if( dt == RealType ){
    Real *data; 
    if( ad.getCentering() == MATPNT ){
      int numElems = myDataContainer->getNumElems();
      data = new Real [numElems];
      Real* mpdata; ad.getData( mpdata );
      integrateOnElement( mpdata, data );
    } else
      ad.getData( data );
    assert( plotvar.plotIndex.size() == 1 );
    io->writeElemPlot(data, plotvar.plotIndex[0], timestep);
  } else if( dt == VectorType ){
    int numElems = myDataContainer->getNumElems();
    Vector *data; 
    if( ad.getCentering() == MATPNT ){
      data = new Vector [numElems];
      Vector* mpdata; ad.getData( mpdata );
      integrateOnElement( mpdata, data );
    } else
      ad.getData( data );
    Real *vx = new Real[numElems];
    Real *vy = new Real[numElems];
    Real *vz = new Real[numElems];
    for(int i=0; i<numElems; i++){
      vx[i] = data[i][VEC::X];
      vy[i] = data[i][VEC::Y];
      vz[i] = data[i][VEC::Z];
    }
    io->writeElemPlot(vx, plotvar.plotIndex[0], timestep);
    io->writeElemPlot(vy, plotvar.plotIndex[1], timestep);
    io->writeElemPlot(vz, plotvar.plotIndex[2], timestep);
 
    if( ad.getCentering() == MATPNT ) 
      delete [] data;
    delete [] vx;
    delete [] vy;
    delete [] vz;

  } else if( dt == SymTensorType ){
    int numElems = myDataContainer->getNumElems();
    SymTensor *data; 
    if( ad.getCentering() == MATPNT ){
      data = new SymTensor [numElems];
      SymTensor* mpdata; ad.getData( mpdata );
      integrateOnElement( mpdata, data );
    } else
      ad.getData( data );
    Real *vxx = new Real[numElems];
    Real *vyy = new Real[numElems];
    Real *vzz = new Real[numElems];
    Real *vyz = new Real[numElems];
    Real *vxz = new Real[numElems];
    Real *vxy = new Real[numElems];
    for(int i=0; i<numElems; i++){
      vxx[i] = data[i][SYM::XX];
      vyy[i] = data[i][SYM::YY];
      vzz[i] = data[i][SYM::ZZ];
      vyz[i] = data[i][SYM::YZ];
      vxz[i] = data[i][SYM::XZ];
      vxy[i] = data[i][SYM::XY];
    }
    io->writeElemPlot(vxx, plotvar.plotIndex[0], timestep);
    io->writeElemPlot(vyy, plotvar.plotIndex[1], timestep);
    io->writeElemPlot(vzz, plotvar.plotIndex[2], timestep);
    io->writeElemPlot(vyz, plotvar.plotIndex[3], timestep);
    io->writeElemPlot(vxz, plotvar.plotIndex[4], timestep);
    io->writeElemPlot(vxy, plotvar.plotIndex[5], timestep);
 
    if( ad.getCentering() == MATPNT )
      delete [] data;
    delete [] vxx;
    delete [] vyy;
    delete [] vzz;
    delete [] vyz;
    delete [] vxz;
    delete [] vxy;
  }
  // TODO: implement symtensor output
}

/******************************************************************************/
void LightMP::finalizeSetup()
/******************************************************************************/
{
  myDataContainer->advanceState();
  InitializeOutput();
}


/******************************************************************************/
void LightMP::InitializeOutput()
/******************************************************************************/
{
  if(!initPlot())
     pXcout << "!!!ERROR: Initializing plot file"
            << endl;
}

/******************************************************************************/
bool LightMP::initPlot()
/******************************************************************************/
{
    bool status = true;
    if( myMeshOutput ) {
        MeshIO* io = myMeshOutput;
        if(!(io->openMeshIO())) {
            status = false;
            return status;
        }

        if( appendOutput ) return status;

        //! Write connectivity and node coordinates
        if(!(io->writePrologue())) {
            status = false;
            return status;
        }
        //! Write number of plot variables and names
        if(!initPlotVars(io)) {
            status = false;
            return status;
        }
        if(mWriteTimeStepDuringSetup)
        {
            //! write initial data to plot file
            if(!writePlotVars(io))
                return false;
        }
    }

    return status;
}

/******************************************************************************/
bool LightMP::initPlotVars(MeshIO* io)
/******************************************************************************/
{ 
#ifdef DEBUG_LOCATION
  _print_entering_location(__AXSIS_FUNCTION_NAMER__);
#endif //DEBUG_LOCATION
  

  // find plot variables in data container
  int total_num_vars = myDataContainer->getNumVariables();
  for(int var=0; var<total_num_vars; var++) {
    AbstractData& d = *(myDataContainer->getAbstractVariable(var));
    if(d.isPlottable()) addPlotVariable( &d, var );
  }
  
  // extract list of variable names and assign plot numbers for output
  vector<string> elemPlots, nodePlots;
  int nodePlotNumber = 0, elemPlotNumber = 0;
  int nvars = myPlotVars.size();
  for(int ivar=0; ivar<nvars; ivar++){
    PlotVariable& var = myPlotVars[ivar];
    VarIndex vari = var.varIndex;
    AbstractData& d = *(myDataContainer->getAbstractVariable(vari));
    DataCentering dc = d.getCentering();
    if( dc == NODE ){
      int nplots = var.plotName.size();
      for(int iplot=0; iplot<nplots; iplot++){
        nodePlots.push_back(var.plotName[iplot]);
        var.plotIndex.push_back(nodePlotNumber++);
      }
    } else if( dc == ELEM || dc == MATPNT ){
      int nplots = var.plotName.size();
      for(int iplot=0; iplot<nplots; iplot++){
        elemPlots.push_back(var.plotName[iplot]);
        var.plotIndex.push_back(elemPlotNumber++);
      }
    }
  }
  //!< initialize elem output if any
  if( elemPlots.size() )
    if(!(io->initVars(ELEM, elemPlots.size(), elemPlots)))
      return false;

  //!< initialize node output if any
  if( nodePlots.size() )
    if(!(io->initVars(NODE, nodePlots.size(), nodePlots)))
      return false;

  return true;
}
/******************************************************************************/
void LightMP::addPlotVariable( AbstractData* d, VarIndex index)
/******************************************************************************/
{

  PlotVariable addplot;
  myPlotVars.push_back(addplot);
  PlotVariable& newplot = myPlotVars.back();
  newplot.varIndex = index;
  vector<string>&  names = newplot.plotName;

  DataType dt = d->getDataType();
  string base = d->getName();
  if( dt == RealType ){
    names.push_back( base );
  } else if( dt == VectorType ){
    string name;
    name = base + "X";
    names.push_back( name );
    name = base + "Y";
    names.push_back( name );
    name = base + "Z";
    names.push_back( name );
  } else if( dt == TensorType ){
    string name;
    name = base + "XX";
    names.push_back( name );
    name = base + "YY";
    names.push_back( name );
    name = base + "ZZ";
    names.push_back( name );
    name = base + "YZ";
    names.push_back( name );
    name = base + "XZ";
    names.push_back( name );
    name = base + "XY";
    names.push_back( name );
    name = base + "ZY";
    names.push_back( name );
    name = base + "ZX";
    names.push_back( name );
    name = base + "YX";
    names.push_back( name );
  } else if( dt == SymTensorType ){
    string name;
    name = base + "XX";
    names.push_back( name );
    name = base + "YY";
    names.push_back( name );
    name = base + "ZZ";
    names.push_back( name );
    name = base + "YZ";
    names.push_back( name );
    name = base + "XZ";
    names.push_back( name );
    name = base + "XY";
    names.push_back( name );
  } else if( dt == ArrayType ){
  }
}

/******************************************************************************/
bool LightMP::closePlot(MeshIO *io)
/******************************************************************************/
{
#ifdef DEBUG_LOCATION
  _print_entering_location(__AXSIS_FUNCTION_NAMER__);
#endif //DEBUG_LOCATION

  if(!(io->closeMeshIO()))
    return false;

  return true;

}

/******************************************************************************/
void setupSolver( pugi::xml_node& config, AztecOO& solver, 
                           int& iterations, Real& tolerance,
                           DistributedCrsMatrix* A)
/******************************************************************************/
{

  iterations = Plato::Parse::getInt( config, "iterations" );
  tolerance = Plato::Parse::getDouble( config, "tolerance" );

//    bool useML = true;
    bool useML = false;
    if( useML && A ){
      // ML setup
      Teuchos::ParameterList MLList;
      ML_Epetra::SetDefaults("SA",MLList);
      MLList.set("ML output", 0);
      MLList.set("max levels",6);
      MLList.set("increasing or decreasing","increasing");
      MLList.set("aggregation: type", "Uncoupled");
  
      // smoother is Chebyshev. Example file 
      // `ml/examples/TwoLevelDD/ml_2level_DD.cpp' shows how to use
      // AZTEC's preconditioners as smoothers
  
      MLList.set("smoother: type","Chebyshev");
      MLList.set("smoother: sweeps",3);
      MLList.set("smoother: pre or post", "both");
      MLList.set("coarse: type","Amesos-KLU");

      ML_Epetra::MultiLevelPreconditioner* MLPrec = 
        new ML_Epetra::MultiLevelPreconditioner( 
             *(A->getEpetraCrsMatrix()), MLList);

      solver.SetPrecOperator(MLPrec);
      solver.SetAztecOption(AZ_scaling, AZ_row_sum);
      solver.SetAztecOption(AZ_solver, AZ_gmres);
      solver.SetAztecOption(AZ_output, 100);
    } else {

      // solver configuration
      int output_interval = Plato::Parse::getInt( config, "output_interval" );
      solver.SetAztecOption(AZ_output, output_interval);
      solver.SetAztecOption(AZ_precond, AZ_ilu);
      solver.SetAztecOption(AZ_subdomain_solve, AZ_ilu);
      solver.SetAztecOption(AZ_precond, AZ_dom_decomp);
      solver.SetAztecOption(AZ_scaling, AZ_row_sum);
      solver.SetAztecOption(AZ_solver, AZ_gmres);
    }
}


