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

#include <iostream>
#include <string>
#include <sstream>
using std::cout;
using std::endl;
using std::ostringstream;

#include "types.hpp"
#include "communicator.hpp"
#include "data_mesh.hpp"
#include "mpi.h"

Communicator WorldComm;

MPI_Datatype MPI_Vector;
bool built_MPI_Vector = false;

Communicator::Communicator() : rootPID(0)
{
  myIndices = NULL;
  end[0] = -1; begin[0] = -1;
  end[1] = -1; begin[1] = -1;
  end[2] = -1; begin[2] = -1;
}

Communicator::~Communicator()
{
}

void Communicator::init()
{
  setComm();
  setSize();
  setPID();
  addVectorType();
}

void Communicator::init(MPI_Comm comm, bool a_preConstructed)
{
  preConstructed = a_preConstructed;
  setComm(comm);
  setSize();
  setPID();
  addVectorType();
}

void Communicator::sync()
{
  MPI_Barrier(myComm);
}

void Communicator::setComm()
{
  myComm = MPI_COMM_WORLD;
}

void Communicator::setComm(MPI_Comm comm)
{
  myComm = comm;
}


void
Communicator::setSize()
{
  mySize = 1;
  MPI_Comm_size(myComm, &mySize);
}

int
Communicator::globalSum(int value)
{
  int return_value = 0;
  MPI_Allreduce(&value, &return_value, 1, MPI_INT, MPI_SUM, myComm);
  return return_value;
}


void 
Communicator::globalArraySum(Real* values, int len ){

  Real* return_values = new Real[len];

  MPI_Allreduce(values, return_values, len, MPI_DOUBLE, MPI_SUM, myComm);

  for(int i=0; i<len; i++)
    values[i] = return_values[i];

  delete [] return_values;
 
  return;
}




Real
Communicator::globalSum(Real value)
{
  Real return_value = 0.;
  MPI_Allreduce(&value, &return_value, 1, MPI_DOUBLE, MPI_SUM, myComm);
  return return_value;
}

int
Communicator::globalMax(int *value, int size)
{
  int return_value = -INT_MAX;
  for (int isize=0; isize<size; isize++) {
    if(return_value < value[isize])
       return_value = value[isize];
  }
  int tmp_value = return_value;
  MPI_Allreduce(&tmp_value, &return_value, 1, MPI_INT, MPI_MAX, myComm);
  return return_value;
}

Real
Communicator::globalMax(Real *value, int size)
{
  Real return_value = -DBL_MAX;
  for (int isize=0; isize<size; isize++) {
    if(return_value < value[isize])
       return_value = value[isize];
  }
  Real tmp_value = return_value;
  MPI_Allreduce(&tmp_value, &return_value, 1, MPI_DOUBLE, MPI_MAX, myComm);
  return return_value;
}

Real
Communicator::globalMax(Real value)
{
  Real return_value = value;
  Real tmp_value = return_value;
  MPI_Allreduce(&tmp_value, &return_value, 1, MPI_DOUBLE, MPI_MAX, myComm);
  return return_value;
}

int
Communicator::globalMax(int value)
{
  int return_value = value;
  int tmp_value = return_value;
  MPI_Allreduce(&tmp_value, &return_value, 1, MPI_INT, MPI_MAX, myComm);
  return return_value;
}

int
Communicator::globalMin(int value)
{
  int return_value = value;
  int tmp_value = return_value;
  MPI_Allreduce(&tmp_value, &return_value, 1, MPI_INT, MPI_MIN, myComm);
  return return_value;
}

Real
Communicator::globalMin(Real value)
{
  Real return_value = value;
  Real tmp_value = return_value;
  MPI_Allreduce(&tmp_value, &return_value, 1, MPI_DOUBLE, MPI_MIN, myComm);
  return return_value;
}

int
Communicator::Broadcast(int &value)
{
  int return_value = 0;
  return_value = MPI_Bcast(&value, 1, MPI_INT, 0, myComm);
  return return_value;
}

int
Communicator::Broadcast(Real &value)
{
  int return_value = 0;
  return_value = MPI_Bcast(&value, 1, MPI_DOUBLE, 0, myComm);
  return return_value;
}

int
Communicator::Broadcast(Real *value, int size)
{
  int return_value = 0;
  return_value = MPI_Bcast(value, size, MPI_DOUBLE, 0, myComm);
  return return_value;
}

int
Communicator::Broadcast(Vector *value, int size)
{
  int return_value = 0;
  return_value = MPI_Bcast(value, size, MPI_Vector, 0, myComm);
  return return_value;
}

int 
Communicator::BroadcastFromRank(int fromRank, Real *data, int numData)
{
  int return_value = 0;
  return_value = MPI_Bcast(data, numData, MPI_DOUBLE, fromRank, myComm);
  return return_value;
}

int Communicator::Import(vector<int>* vec, DataMesh* mesh){

  // loop on numNodeCommMaps and complete vec with off-processor data

  int **numEntries_send = new int* [mesh->numNodeCommMaps];
  int **numEntries_recv = new int* [mesh->numNodeCommMaps];
  // loop on nodes in each node_comm_map and count the entries in vec
  for(int iNodeMap=0; iNodeMap<mesh->numNodeCommMaps; iNodeMap++){
    int send_to = mesh->commNodeProcIds[iNodeMap][0];
    int recv_from = send_to;
    int numNodesThisMap = mesh->nodeCmapNodeCnts[iNodeMap];
    int *nodesThisMap = mesh->commNodeIds[iNodeMap];
    numEntries_send[iNodeMap] = new int [numNodesThisMap];
    numEntries_recv[iNodeMap] = new int [numNodesThisMap];
    int* send = numEntries_send[iNodeMap];
    int* recv = numEntries_recv[iNodeMap];
    for(int iNode=0; iNode<numNodesThisMap; iNode++){
      send[iNode] = vec[nodesThisMap[iNode]].size();
    }

    // do a sendrecv to communicate the number of entries for each node
    MPI_Status status;
    MPI_Sendrecv(send, numNodesThisMap, MPI_INT, send_to, 0,
                 recv, numNodesThisMap, MPI_INT, recv_from, 0,
                 myComm, &status);
  }

  for(int iNodeMap=0; iNodeMap<mesh->numNodeCommMaps; iNodeMap++){
    int totalNumEntries_send = 0;
    int totalNumEntries_recv = 0;
    int* send = numEntries_send[iNodeMap];
    int* recv = numEntries_recv[iNodeMap];
    int numNodesThisMap = mesh->nodeCmapNodeCnts[iNodeMap];
    for(int iNode=0; iNode<numNodesThisMap; iNode++){
      totalNumEntries_send += send[iNode];
      totalNumEntries_recv += recv[iNode];
    }

    int send_to = mesh->commNodeProcIds[iNodeMap][0];
    int recv_from = send_to;

    int* vecEntries_send = new int [totalNumEntries_send];
    int* vecEntries_recv = new int [totalNumEntries_recv];
    
    int* vec_send = vecEntries_send;

    int *nodesThisMap = mesh->commNodeIds[iNodeMap];
    for(int iNode=0; iNode<numNodesThisMap; iNode++){
      int nsend = send[iNode];
      memcpy(vec_send, &(vec[nodesThisMap[iNode]][0]), nsend*sizeof(int));
      vec_send += nsend;
    }
    
    MPI_Status status;
    MPI_Sendrecv(vecEntries_send, totalNumEntries_send, MPI_INT, send_to, 0,
                 vecEntries_recv, totalNumEntries_recv, MPI_INT, recv_from, 0,
                 myComm, &status);

    int* vec_recv = vecEntries_recv;

    for(int iNode=0; iNode<numNodesThisMap; iNode++){
      int nrecv = recv[iNode];
      for(int inew=0;inew<nrecv;inew++)
        vec[nodesThisMap[iNode]].push_back(*(vec_recv++));
    }

    delete [] vecEntries_send;
    delete [] vecEntries_recv;
  }

  for(int iNodeMap=0; iNodeMap<mesh->numNodeCommMaps; iNodeMap++){
    delete [] numEntries_send[iNodeMap];
    delete [] numEntries_recv[iNodeMap];
  }
  delete [] numEntries_send;
  delete [] numEntries_recv;

 return 0;
}

int Communicator::Import(vector<double>* vec, DataMesh* mesh){
  // loop on numNodeCommMaps and complete vec with off-processor data

  int **numEntries_send = new int* [mesh->numNodeCommMaps];
  int **numEntries_recv = new int* [mesh->numNodeCommMaps];
  // loop on nodes in each node_comm_map and count the entries in vec
  for(int iNodeMap=0; iNodeMap<mesh->numNodeCommMaps; iNodeMap++){
    int send_to = mesh->commNodeProcIds[iNodeMap][0];
    int recv_from = send_to;
    int numNodesThisMap = mesh->nodeCmapNodeCnts[iNodeMap];
    int *nodesThisMap = mesh->commNodeIds[iNodeMap];
    numEntries_send[iNodeMap] = new int [numNodesThisMap];
    numEntries_recv[iNodeMap] = new int [numNodesThisMap];
    int* send = numEntries_send[iNodeMap];
    int* recv = numEntries_recv[iNodeMap];
    for(int iNode=0; iNode<numNodesThisMap; iNode++){
      send[iNode] = vec[nodesThisMap[iNode]].size();
    }

    // do a sendrecv to communicate the number of entries for each node
    MPI_Status status;
    MPI_Sendrecv(send, numNodesThisMap, MPI_INT, send_to, 0,
                 recv, numNodesThisMap, MPI_INT, recv_from, 0,
                 myComm, &status);
  }

  for(int iNodeMap=0; iNodeMap<mesh->numNodeCommMaps; iNodeMap++){
    int totalNumEntries_send = 0;
    int totalNumEntries_recv = 0;
    int* send = numEntries_send[iNodeMap];
    int* recv = numEntries_recv[iNodeMap];
    int numNodesThisMap = mesh->nodeCmapNodeCnts[iNodeMap];
    for(int iNode=0; iNode<numNodesThisMap; iNode++){
      totalNumEntries_send += send[iNode];
      totalNumEntries_recv += recv[iNode];
    }

    int send_to = mesh->commNodeProcIds[iNodeMap][0];
    int recv_from = send_to;

    double* vecEntries_send = new double [totalNumEntries_send];
    double* vecEntries_recv = new double [totalNumEntries_recv];
    
    double* vec_send = vecEntries_send;

    int *nodesThisMap = mesh->commNodeIds[iNodeMap];
    for(int iNode=0; iNode<numNodesThisMap; iNode++){
      int nsend = send[iNode];
      memcpy(vec_send, &(vec[nodesThisMap[iNode]][0]), nsend*sizeof(double));
      vec_send += nsend;
    }
    
    MPI_Status status;
    MPI_Sendrecv(vecEntries_send, totalNumEntries_send, MPI_DOUBLE, send_to, 0,
                 vecEntries_recv, totalNumEntries_recv, MPI_DOUBLE, recv_from, 0,
                 myComm, &status);

    double* vec_recv = vecEntries_recv;

    for(int iNode=0; iNode<numNodesThisMap; iNode++){
      int nrecv = recv[iNode];
      for(int inew=0;inew<nrecv;inew++)
        vec[nodesThisMap[iNode]].push_back(*(vec_recv++));
    }

    delete [] vecEntries_send;
    delete [] vecEntries_recv;
  }

  for(int iNodeMap=0; iNodeMap<mesh->numNodeCommMaps; iNodeMap++){
    delete [] numEntries_send[iNodeMap];
    delete [] numEntries_recv[iNodeMap];
  }
  delete [] numEntries_send;
  delete [] numEntries_recv;

 return 0;
}

void
Communicator::setPID()
{
  myPID = 0;
  MPI_Comm_rank(myComm, &myPID);
}

void Communicator::cartCreate()
{
  const int numDimensions = 3;
  int periodic[3] = {0, 0, 0};
  if( !preConstructed ){
    MPI_Dims_create( mySize, numDimensions, procDomain );
    MPI_Comm oldComm = myComm;
    const int reorder = 1;
    MPI_Cart_create( oldComm, numDimensions, procDomain, 
                     periodic, reorder, &myComm );
    MPI_Comm_rank( myComm, &myPID );
  }
  if( myIndices ) delete [] myIndices;
  myIndices = new int[numDimensions];
  MPI_Cart_get( myComm, numDimensions, procDomain, periodic, myIndices );
}

void Communicator::setElementDomain( int dim, int& ibegin, int& iend)
{
  if( dim>=0 && dim<3 ){
    begin[dim] = ibegin;
    end[dim] = iend;
  }
}

void Communicator::getElementDomain( int dim, int numElem, int& ibegin, int& iend)
{
  if( begin[dim] == -1 ){
    if( dim>=0 && dim<3 ){
      int myid = myIndices[dim];
      int size = procDomain[dim];
      int nlocal = numElem / size;
      ibegin = myid * nlocal;
      int deficit = numElem % size;
      int offset = myid < deficit ? myid : deficit;
      ibegin += offset;
      if( myid < deficit ) nlocal++;
      iend = ibegin + nlocal;
      if( iend > numElem || myid == size ) iend = size;
      begin[dim] = ibegin;
      end[dim] = iend;
    } else {
      ibegin = 0;
      iend = 0;
    }
  } else {
    ibegin = begin[dim];
    iend = end[dim];
  }
}

void
Communicator::dumpComm()
{
  pXcout << "Communicator data dump..." << endl;
  pXcout << "\tmyComm = " << myComm << endl;
  pXcout << "\tmySize = " << mySize << endl;
  pXcout << "\tmyPID = " << myPID << endl;
}

void Communicator::pAcout(const char* message)
{
    ostringstream buffer;
    buffer << message;
    const int rootPID = 0;
    int data = 0;
    if(mySize > 1)
    {
        if(myPID != rootPID)
            blockingRecv(myPID - 1, &data);

        cout << buffer.str();
        if(buffer.str().size())
            cout << std::endl;
        cout.flush();

        if(myPID < (mySize - 1))
            blockingSend(myPID + 1, &data);
        sync();
    }
    else
        cout << buffer.str() << endl;
}

void
Communicator::BeginProcStaging()
{
  const int TAG_STAGE_PROCS = 1111;
  const int rootPID = 0;
  int data = 0;
  if( mySize > 1 ) {
    if( myPID != rootPID )
      blockingRecv(myPID-1, &data, 1, TAG_STAGE_PROCS);
  }
}

void
Communicator::EndProcStaging()
{
  const int TAG_STAGE_PROCS = 1111;
  int data = 0;
  if( mySize > 1 ) {
    if( myPID < (mySize-1) )
      blockingSend(myPID+1, &data, 1, TAG_STAGE_PROCS);
  }
}

void
Communicator::BeginBlockForIO()
{
  if( myPID != AXSIS_IO_PROC) {
    int data = 0;
    blockingRecv(AXSIS_IO_PROC, &data, 1, 1112);
  }
}

void
Communicator::EndBlockForIO()
{
  // this is awfully inefficient but i'm not good enough
  //   with mpi yet to figger out how to cascade these.
  if( myPID == AXSIS_IO_PROC) {
    int data = 0;
    for(int i=0; i<AXSIS_IO_PROC; i++)
      blockingSend(i, &data, 1, 1112);
    for(int i=AXSIS_IO_PROC+1; i<mySize; i++)
      blockingSend(i, &data, 1, 1112);
  }
}


void
Communicator::blockingRecv(int source, int* data, int size, int tag)
{
  MPI_Status status;
  MPI_Recv( data, size, MPI_INT, source, tag, myComm, &status );
}
 
void
Communicator::blockingSend(int dest, int* data, int size, int tag)
{
  MPI_Send( data, size, MPI_INT, dest, tag, myComm );
}

void Communicator::addVectorType()
{
    if (built_MPI_Vector) {
        return;
    }
    built_MPI_Vector = true;

    Vector p;
    int blockcounts[1] = {MLib::VEC::NUM_TERMS};
    MPI_Aint offsets[1] = {(MPI_Aint)(p.getData()) - (MPI_Aint) & p};
    MPI_Datatype oldtypes[1] = {MPI_DOUBLE};
    MPI_Type_create_struct(1, blockcounts, offsets, oldtypes, &MPI_Vector);
    MPI_Type_commit(&MPI_Vector);
}
