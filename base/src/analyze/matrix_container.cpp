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

#define HAVE_CONFIG_H
#include "matrix_container.hpp"
#include "data_mesh.hpp"
#include "data_container.hpp"
#include "topological_element.hpp"
#include "communicator.hpp"
#include "bc.hpp"

#include <vector>


/*****************************************************************************/
SystemContainer::SystemContainer(DataMesh* mesh, int dofs_per_node, pugi::xml_node& input)
/*****************************************************************************/
{
  Comm = new Epetra_MpiComm( WorldComm.getComm() );

  Initialize(mesh, dofs_per_node);
}

/*****************************************************************************/
SystemContainer::~SystemContainer()
/*****************************************************************************/
{
    if(RowMap)
    {
        delete RowMap;
        RowMap = NULL;
    }
    if(OverlapRowMap)
    {
        delete OverlapRowMap;
        OverlapRowMap = NULL;
    }
    if(NodeRowMap)
    {
        delete NodeRowMap;
        NodeRowMap = NULL;
    }
    if(NodeOverlapRowMap)
    {
        delete NodeOverlapRowMap;
        NodeOverlapRowMap = NULL;
    }
    if(Comm)
    {
        delete Comm;
        Comm = NULL;
    }
    if(k_graph)
    {
        delete k_graph;
        k_graph = NULL;
    }
    if(k_overlap_graph)
    {
        delete k_overlap_graph;
        k_overlap_graph = NULL;
    }
    if(assemblyExporter)
    {
        delete assemblyExporter;
        assemblyExporter = NULL;
    }
    if(myImporter)
    {
        delete myImporter;
        myImporter = NULL;
    }
    if(myNodeImporter)
    {
        delete myNodeImporter;
        myNodeImporter = NULL;
    }
    myMesh = NULL;
}


/*****************************************************************************/
void SystemContainer::Initialize(DataMesh* mesh, int dofs_per_node)
/*****************************************************************************/
{

  zeroSet();

  dofsPerNode = dofs_per_node;

  myMesh = mesh;

  if(Comm)
    delete Comm;

  Comm = new Epetra_MpiComm( WorldComm.getComm() );


#ifdef DEBUG_LOCATION
  _print_entering_location(__AXSIS_FUNCTION_NAMER__);
#endif //DEBUG_LOCATION

  int num_nodes       = mesh->getNumNodes();

  int *total_nodes    = mesh->nodeGlobalIds;

  //make a list of global ids for owned nodes
  vector<int> owned_nodes;
  for(int j=0; j<num_nodes; j++) {
    // add dofs if any, otherwise empty for that node_gid...
    if(mesh->nodeOwnership[j]==1) {
      owned_nodes.push_back(total_nodes[j]);
    } 
  }

  NodeRowMap  = new Epetra_Map(-1, owned_nodes.size(),
                               &owned_nodes[0], 0, *Comm);

  NodeOverlapRowMap  = new Epetra_Map(-1, num_nodes,
                                  total_nodes, 0, *Comm);

  myNodeImporter = new Epetra_Import(*NodeOverlapRowMap, *NodeRowMap);



  //make a list of global ids for owned and total nodes
  vector<int> owned_dofs, total_dofs;
  for(int j=0; j<num_nodes; j++) {
    int node_gid = total_nodes[j];
    int start_dof_id = node_gid*dofsPerNode;
    if(mesh->nodeOwnership[j]==1) {
      for(int ndpn=0; ndpn<dofsPerNode; ndpn++) {
        owned_dofs.push_back(start_dof_id+ndpn);
      }
    }
    for(int ndpn=0; ndpn<dofsPerNode; ndpn++)
      total_dofs.push_back(start_dof_id+ndpn);
  }

  int num_total_dofs = total_dofs.size();
  int num_owned_dofs = owned_dofs.size();

  RowMap  = new Epetra_Map(-1, num_owned_dofs,
                            &owned_dofs[0], 0, *Comm);

  OverlapRowMap  = new Epetra_Map(-1, num_total_dofs,
                                  &total_dofs[0], 0, *Comm);



  Comm->Barrier();

  k_overlap_graph = new Epetra_CrsGraph(Copy, *OverlapRowMap, 0);

  k_graph = new Epetra_CrsGraph(Copy, *RowMap, 0);

  Comm->Barrier();
  // build graph ...
  int  num_elem_blocks = mesh->getNumElemBlks();
  for(int elem_blk=0; elem_blk<num_elem_blocks; elem_blk++) {
    int num_elems = mesh->getNumElemInBlk(elem_blk);
    if(num_elems == 0) continue;
    int Nnpe = mesh->getNnpeInBlk(elem_blk);
    int *p_connect = new int[Nnpe];
    for(int elem_plid=0; elem_plid<num_elems; elem_plid++) {
      mesh->Connect(p_connect, elem_blk, elem_plid);
      for(int lrnode=0; lrnode<Nnpe; lrnode++) {
        int local_node = p_connect[lrnode]; //p_connect returns processor-local id...
	int global_node = total_nodes[local_node];
        int global_row = global_node*dofsPerNode;
          int row_size = dofsPerNode;
	  for(int row_dofs=0; row_dofs<row_size; row_dofs++) {
	    for(int lcnode=0; lcnode<Nnpe; lcnode++) {
	      int local_node2 = p_connect[lcnode];
	      int global_node2 = total_nodes[local_node2];
              int global_column = global_node2*dofsPerNode;
	      int column_size = dofsPerNode;
	      for(int column_dofs=0; column_dofs<column_size; column_dofs++) {
		k_overlap_graph->InsertGlobalIndices(global_row,1,&global_column);
		global_column++;
              }
            }
            global_row++;
         }
      }
    }
    delete [] p_connect;
  }

  Comm->Barrier();
  k_overlap_graph->FillComplete();

  assemblyExporter = new Epetra_Export(*OverlapRowMap, *RowMap);
  k_graph->Export(*(k_overlap_graph), *assemblyExporter, Insert);
  k_graph->FillComplete();

  myImporter = new Epetra_Import(*OverlapRowMap, *RowMap);


}

/******************************************************************************/
void SystemContainer::zeroSet()
/******************************************************************************/
{
 myMesh            = NULL;

 RowMap            = NULL;
 OverlapRowMap     = NULL;
 NodeRowMap        = NULL;
 NodeOverlapRowMap = NULL;
 //Comm              = NULL;
 k_graph           = NULL;
 k_overlap_graph   = NULL;
 assemblyExporter  = NULL;
 myImporter        = NULL;
 myNodeImporter    = NULL;

}

/******************************************************************************/
DistributedCrsMatrix::DistributedCrsMatrix(SystemContainer *sys) : DistributedEntity(sys)
/******************************************************************************/
{
  Values = new Real[sys->dofsPerNode];
  Indices = new int[sys->dofsPerNode];

  globalMatrix = new Epetra_CrsMatrix(Copy, *(sys->k_graph));
  globalMatrix->PutScalar(0.);

  assemblyMatrix = new Epetra_CrsMatrix(Copy, *(sys->k_overlap_graph));
  assemblyMatrix->PutScalar(0.);
}


/******************************************************************************/
void DistributedCrsMatrix::Assemble( Intrepid::FieldContainer<double>& localMatrix, 
                                 int* elemConnect, int numFieldsG )
/******************************************************************************/
{

    int dofsPerNode = mySystem->dofsPerNode;
    int* node_global_ids = mySystem->myMesh->nodeGlobalIds;


    for(int iNode=0; iNode<numFieldsG; iNode++){
      int node_plid  = elemConnect[iNode];
      int node_gid   = node_global_ids[node_plid];
      int global_row = node_gid*dofsPerNode;
      for(int iDof=0; iDof<dofsPerNode; iDof++){
        for(int jNode=0; jNode<numFieldsG; jNode++){
          int node_plid2 = elemConnect[jNode];
          int node_gid2  = node_global_ids[node_plid2];
          int global_column = node_gid2*dofsPerNode;
          for(int jDof=0; jDof<dofsPerNode; jDof++) {
            Values[jDof] = localMatrix(0, iNode, iDof, jNode, jDof);
            Indices[jDof] = global_column;
            global_column++;
          }
          assemblyMatrix->SumIntoGlobalValues(global_row, dofsPerNode, Values, Indices);
        }
        global_row++;
      }
    }

}

/******************************************************************************/
void DistributedCrsMatrix::replaceGlobalValue(int node1_plid, int node2_plid, int dof_id, Real val)
/******************************************************************************/
{
  //This function replace the value in the assemblyMatrix that corresponds with
  //...the two given nodes and the given degree of freedom. It is used to place
  //...off-diagonal penalty values for periodic boundary conditions.
 
  int dofsPerNode = mySystem->dofsPerNode;
  int* node_global_ids = mySystem->myMesh->nodeGlobalIds;

  int node1_gid = node_global_ids[node1_plid];
  int global_row = node1_gid*dofsPerNode + dof_id;

  int node2_gid = node_global_ids[node2_plid];
  int global_column = node2_gid*dofsPerNode + dof_id;

  assemblyMatrix->ReplaceGlobalValues(global_row, 1, &val, &global_column);
}

/******************************************************************************/
void DistributedCrsMatrix::replaceDiagonal(DistributedVector& diag)
/******************************************************************************/
{
  globalMatrix->ReplaceDiagonalValues(*(diag.globalVector));
}

/******************************************************************************/
void DistributedCrsMatrix::getDiagonal(DistributedVector& diag)
/******************************************************************************/
{
  globalMatrix->ExtractDiagonalCopy(*(diag.globalVector));
}

/******************************************************************************/
void DistributedCrsMatrix::FillComplete()
/******************************************************************************/
{

  assemblyMatrix->FillComplete();
  globalMatrix->Export( *assemblyMatrix, *(mySystem->assemblyExporter), Add);
  globalMatrix->FillComplete();

}

/******************************************************************************/
DistributedVector::DistributedVector(SystemContainer *sys) :
        DistributedEntity(sys),
        globalVector(NULL),
        assemblyVector(NULL),
        plotIndices()
/******************************************************************************/
{
  globalVector = new Epetra_Vector( *(sys->RowMap) );
  globalVector->PutScalar(0.0);
  assemblyVector = new Epetra_Vector( *(sys->OverlapRowMap) );
  assemblyVector->PutScalar(0.0);
}

/******************************************************************************/
DistributedVector::DistributedVector(SystemContainer *sys, vector<VarIndex> varIndices) :
        DistributedEntity(sys),
        globalVector(NULL),
        assemblyVector(NULL),
        plotIndices()
/******************************************************************************/
{
  globalVector = new Epetra_Vector( *(sys->RowMap) );
  globalVector->PutScalar(0.0);
  assemblyVector = new Epetra_Vector( *(sys->OverlapRowMap) );
  assemblyVector->PutScalar(0.0);

  if( varIndices.size() == (unsigned int) mySystem->dofsPerNode )
    plotIndices = varIndices;
  else
    throw RunTimeError( "Number of plot indices != number of DOFs per node. Cannot Disassemble." );
}

/******************************************************************************/
DistributedVector::DistributedVector(SystemContainer *sys, VarIndex varIndex) :
        DistributedEntity(sys),
        globalVector(NULL),
        assemblyVector(NULL),
        plotIndices()
/******************************************************************************/
{
  globalVector = new Epetra_Vector( *(sys->RowMap) );
  globalVector->PutScalar(0.0);
  assemblyVector = new Epetra_Vector( *(sys->OverlapRowMap) );
  assemblyVector->PutScalar(0.0);

  if( mySystem->dofsPerNode == 1 )
    plotIndices.push_back(varIndex);
}

/******************************************************************************/
DistributedVector::DistributedVector() :
/******************************************************************************/
        DistributedEntity(),
        globalVector(NULL),
        assemblyVector(NULL),
        plotIndices()
{
}

/******************************************************************************/
DistributedVector::~DistributedVector()
/******************************************************************************/
{
    if(assemblyVector)
    {
        delete assemblyVector;
        assemblyVector = NULL;
    }
    if(globalVector)
    {
        delete globalVector;
        globalVector = NULL;
    }
}

/******************************************************************************/
void DistributedVector::Assemble( Intrepid::FieldContainer<double>& localVector, 
                                 int* elemConnect, int numFieldsG )
/******************************************************************************/
{

    int dofsPerNode = mySystem->dofsPerNode;
    int* node_global_ids = mySystem->myMesh->nodeGlobalIds;


    for(int iNode=0; iNode<numFieldsG; iNode++){
      int node_plid  = elemConnect[iNode];
      int node_gid   = node_global_ids[node_plid];
      int global_row = node_gid*dofsPerNode;
      for(int iDof=0; iDof<dofsPerNode; iDof++){
        int dof_plid = mySystem->OverlapRowMap->LID(global_row);
        if( dof_plid < 0 ) 
           throw RunTimeError( "requested a DOF that isn't on this processor" );
        (*assemblyVector)[dof_plid] += localVector(iNode, iDof);
        global_row++;
      }
    }
}
/******************************************************************************/
void DistributedVector::LocalAssemble( Real value, int nodeId, int dofId)
/******************************************************************************/
{

// TODO:  this should assemble into the assembly vector
// shouldn't this be '+=' ?

  int dofsPerNode = mySystem->dofsPerNode;


  int node_gid  = mySystem->myMesh->nodeGlobalIds[nodeId];
  int dof_gid  = dofsPerNode*node_gid + dofId;
  int dof_plid = mySystem->RowMap->LID(dof_gid);
  if( dof_plid != -1 ) 
    (*globalVector)[dof_plid] += value;
  
}
/******************************************************************************/
void DistributedVector::Assemble( Real value, int nodeId, int dofId)
/******************************************************************************/
{

// TODO:  this should assemble into the assembly vector
// shouldn't this be '+=' ?

  int dofsPerNode = mySystem->dofsPerNode;


  int node_gid  = mySystem->myMesh->nodeGlobalIds[nodeId];
  int dof_gid  = dofsPerNode*node_gid + dofId;
  int dof_plid = mySystem->OverlapRowMap->LID(dof_gid);
  if( dof_plid < 0 ) 
     throw RunTimeError( "requested a DOF that isn't on this processor" );
  (*assemblyVector)[dof_plid] += value;
  
}

/******************************************************************************/
void DistributedVector::DisAssemble()
/******************************************************************************/
{

  int dofsPerNode = mySystem->dofsPerNode;

  

  if( plotIndices.size() != (unsigned int) dofsPerNode ) 
    throw RunTimeError( "Number of plot indices != number of DOFs per node. Cannot Disassemble." );
 
  DataContainer* dc = mySystem->myMesh->getDataContainer();
  Real* vdata; 
  assemblyVector->ExtractView(&vdata);

  int* nodeGlobalIds = mySystem->myMesh->nodeGlobalIds;
  Epetra_Map* myOverlapRowMap = mySystem->OverlapRowMap;
  
  Real** cdata = new Real*[dofsPerNode];
  for(int ic=0; ic<dofsPerNode; ic++)
    dc->getVariable( plotIndices[ic], cdata[ic] );

  
  int numNodes = mySystem->myMesh->getNumNodes();
  for(int inode=0; inode<numNodes; inode++)
    for(int idof=0; idof<dofsPerNode; idof++){
      int node_gid = nodeGlobalIds[inode];
      int dof_gid = dofsPerNode*node_gid+idof;
      int dof_plid = myOverlapRowMap->LID(dof_gid);
      cdata[idof][inode] = vdata[dof_plid];
    }

  delete [] cdata;

}


/******************************************************************************/
void DistributedVector::Import()
/******************************************************************************/
{
  assemblyVector->PutScalar(0.0);
  assemblyVector->Import(*globalVector, *(mySystem->myImporter), Insert);
}

/******************************************************************************/
void DistributedVector::Export()
/******************************************************************************/
{
  globalVector->Export(*assemblyVector, *(mySystem->myImporter), Add);
  assemblyVector->Import(*globalVector, *(mySystem->myImporter), Insert);
}

/******************************************************************************/
void DistributedVector::LocalExport()
/******************************************************************************/
{
  globalVector->Export(*assemblyVector, *(mySystem->myImporter), Insert);
}
