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

#include "topological_element.hpp"
#include "types.hpp"
#include "mesh_io.hpp"
#include "communicator.hpp"
#include "data_container.hpp"
#include "data_mesh.hpp"

#include "exodusII.h"
#include "ne_nemesisI.h"

#include <iostream>
#include <fstream>
#include <string>
using std::ifstream;
using std::string;
using std::cout;
using std::endl;


NemesisIO::NemesisIO() 
{ 
  myType = READ; 
  myMesh = NULL;
  myData = NULL;
  myName = "NemesisIO";
  myTitle = "Fester NemesisIO";

}

NemesisIO::~NemesisIO() 
{ 

  closeMeshIO();

}

bool
NemesisIO::openMeshIO()
{
#ifdef DEBUG_LOCATION
  _print_entering_location(__AXSIS_FUNCTION_NAMER__);
#endif //DEBUG_LOCATION

  bool status = true;

  status = ExodusIO::openMeshIO();

  return status;
}

bool
NemesisIO::closeMeshIO()
{
  return true;//do nada...
}

bool
NemesisIO::readMeshIO()
{
#ifdef DEBUG_LOCATION
  _print_entering_location(__AXSIS_FUNCTION_NAMER__);
#endif //DEBUG_LOCATION
  
  if(!ExodusIO::readMeshIO())
    return false;

  int global_Nnp = 0;
  int global_Nel = 0;
  int global_Neblock = 0;
  int global_Nnset = 0;
  int global_Nsset = 0;

  WorldComm.BeginBlockForIO();

  if(WorldComm.getPID() == AXSIS_IO_PROC) {
    ne_get_init_global(myFileID, &global_Nnp, &global_Nel, &global_Neblock,
                       &global_Nnset, &global_Nsset);
  }

  WorldComm.EndBlockForIO();

  readParallelData();

  return true;
}

bool
NemesisIO::readParallelData()
{

  int my_pid = WorldComm.GetPID();
  int err = 0;

  err += ne_get_init_info(myFileID,
                          &(myMesh->num_procs),
                          &(myMesh->num_proc_in_file),
                          &(myMesh->ftype));

  err += ne_get_loadbal_param(myFileID, 
	                      &(myMesh->num_internal_nodes),
                              &(myMesh->num_border_nodes), 
                              &(myMesh->num_external_nodes),
                              &(myMesh->num_internal_elems), 
                              &(myMesh->num_border_elems),
                              &(myMesh->numNodeCommMaps),
                              &(myMesh->numElemCommMaps),
                              my_pid);

  if(myMesh->num_internal_nodes)
     myMesh->internalNodes  = new int[myMesh->num_internal_nodes];
  if(myMesh->num_border_nodes)
     myMesh->borderNodes    = new int[myMesh->num_border_nodes];
  if(myMesh->num_external_nodes)
     myMesh->externalNodes  = new int[myMesh->num_external_nodes];

  err += ne_get_node_map(myFileID, 
                         myMesh->internalNodes,   //
                         myMesh->borderNodes,     //  these are local node numbers
                         myMesh->externalNodes,   // 
                         my_pid);

  for(int i=0;i<myMesh->num_internal_nodes;i++)
    myMesh->internalNodes[i] -= 1;
  for(int i=0;i<myMesh->num_border_nodes;i++)
    myMesh->borderNodes[i] -= 1;
  for(int i=0;i<myMesh->num_external_nodes;i++)
    myMesh->externalNodes[i] -= 1;


  if(myMesh->num_internal_elems)
    myMesh->internalElems  = new int[myMesh->num_internal_elems];
  if(myMesh->num_border_elems)
    myMesh->borderElems    = new int[myMesh->num_border_elems];

  err += ne_get_elem_map(myFileID,
                         myMesh->internalElems,
                         myMesh->borderElems,
                         my_pid);


  int num_node_comm_maps = myMesh->numNodeCommMaps;
  int num_elem_comm_maps = myMesh->numElemCommMaps;

  if(num_node_comm_maps)
  {
    myMesh->nodeCmapNodeCnts = new int[num_node_comm_maps];
    myMesh->nodeCmapIds      = new int[num_node_comm_maps];
    myMesh->elemCmapElemCnts = new int[num_elem_comm_maps];
    myMesh->elemCmapIds      = new int[num_elem_comm_maps];
    myMesh->commNodeIds      = new int*[num_node_comm_maps];
    myMesh->commNodeProcIds  = new int*[num_node_comm_maps];
    myMesh->commElemIds      = new int*[num_elem_comm_maps];
    myMesh->commSideIds      = new int*[num_elem_comm_maps];
    myMesh->commElemProcIds  = new int*[num_elem_comm_maps];

    err += ne_get_cmap_params(myFileID, 
			      myMesh->nodeCmapIds,
			      (int*)(myMesh->nodeCmapNodeCnts), 
			      myMesh->elemCmapIds,
			      (int*)(myMesh->elemCmapElemCnts), 
			      my_pid);

    for(int j = 0; j < num_node_comm_maps; j++)
    {
      (myMesh->commNodeIds)[j]      = new int[(myMesh->nodeCmapNodeCnts)[j]];
      (myMesh->commNodeProcIds)[j]  = new int[(myMesh->nodeCmapNodeCnts)[j]];
      err += ne_get_node_cmap(myFileID, 
			      (myMesh->nodeCmapIds)[j], 
			      (myMesh->commNodeIds)[j], 
			      (myMesh->commNodeProcIds)[j],
			      my_pid);
      for(int i=0; i<myMesh->nodeCmapNodeCnts[j]; i++) //zero based local-node numbering
	myMesh->commNodeIds[j][i] = myMesh->commNodeIds[j][i]-1;
      // simple ownership determination... no communication required but decomposition
      // isn't as well balanced... why can't nemesis do this?
      int *proc_ids = myMesh->commNodeProcIds[j];
      for(int i=0; i<(myMesh->nodeCmapNodeCnts)[j]; i++)
      {
        if(my_pid>proc_ids[i]) {
	  myMesh->nodeOwnership[myMesh->commNodeIds[j][i]] = 0;  //not owned
        }
      }
    }
    // no element map needed. nodal decomposition... for now
    for(int j = 0; j < num_elem_comm_maps; j++)
    {
      (myMesh->commElemIds)[j]      = new int[(myMesh->elemCmapElemCnts)[j]];
      (myMesh->commSideIds)[j]      = new int[(myMesh->elemCmapElemCnts)[j]];
      (myMesh->commElemProcIds)[j]  = new int[(myMesh->elemCmapElemCnts)[j]];
      err += ne_get_elem_cmap(myFileID,
         		      myMesh->elemCmapIds[j], 
    		              myMesh->commElemIds[j], 
    		              myMesh->commSideIds[j],
    		              myMesh->commElemProcIds[j],
   		              my_pid);
    }
  }
  
  if(err == 0)
    return true;
  else
    return false;
}

bool
NemesisIO::writeHeader()
{
#ifdef DEBUG_LOCATION
  _print_entering_location(__AXSIS_FUNCTION_NAMER__);
#endif //DEBUG_LOCATION

  int err = 0;

  if(!ExodusIO::writeHeader())
      err++;

  if(!writeParallelData())
      err++;

  if(err == 0)
    return true;
  else
    return false;
}

bool
NemesisIO::writeParallelData()
{

  int my_pid = WorldComm.GetPID();
  int err = 0;

  err += ne_put_init_info(myFileID,
                          myMesh->num_procs,
                          myMesh->num_proc_in_file,
                          &(myMesh->ftype));

  err += ne_put_loadbal_param(myFileID,
                              myMesh->num_internal_nodes,
                              myMesh->num_border_nodes,
                              myMesh->num_external_nodes,
                              myMesh->num_internal_elems,
                              myMesh->num_border_elems,
                              myMesh->numNodeCommMaps,
                              myMesh->numElemCommMaps,
                              my_pid);

  // create tmp vars to offset node numbering
  int tmpInternalNodes[myMesh->num_internal_nodes];
  int tmpBorderNodes[myMesh->num_border_nodes];
  int tmpExternalNodes[myMesh->num_external_nodes];

  for(int i=0;i<myMesh->num_internal_nodes;i++)
    tmpInternalNodes[i] = myMesh->internalNodes[i] + 1;
  for(int i=0;i<myMesh->num_border_nodes;i++)
    tmpBorderNodes[i] = myMesh->borderNodes[i] + 1;
  for(int i=0;i<myMesh->num_external_nodes;i++)
    tmpExternalNodes[i] = myMesh->externalNodes[i] + 1;

  err += ne_put_node_map(myFileID,
                         tmpInternalNodes,
                         tmpBorderNodes,
                         tmpExternalNodes,
                         my_pid);
  err += ne_put_elem_map(myFileID,
                         myMesh->internalElems,
                         myMesh->borderElems,
                         my_pid);

  int num_node_comm_maps = myMesh->numNodeCommMaps;
  int num_elem_comm_maps = myMesh->numElemCommMaps;

  if(num_node_comm_maps)
  {
    err += ne_put_cmap_params(myFileID,
                              myMesh->nodeCmapIds,
                              (int*)(myMesh->nodeCmapNodeCnts),
                              myMesh->elemCmapIds,
                              (int*)(myMesh->elemCmapElemCnts),
                              my_pid);

    for(int j = 0; j < num_node_comm_maps; j++)
    {
      int tmpCommNodeIds[myMesh->nodeCmapNodeCnts[j]];
      for(int i=0; i<myMesh->nodeCmapNodeCnts[j]; i++) //zero based local-node numbering
        tmpCommNodeIds[i] = myMesh->commNodeIds[j][i]+1;
      err += ne_put_node_cmap(myFileID,
                              (myMesh->nodeCmapIds)[j],
                              tmpCommNodeIds,
                              (myMesh->commNodeProcIds)[j],
                              my_pid);
    }
    // no element map needed. nodal decomposition... for now
    for(int j = 0; j < num_elem_comm_maps; j++) {
      err += ne_put_elem_cmap(myFileID,
                              myMesh->elemCmapIds[j],
                              myMesh->commElemIds[j],
                              myMesh->commSideIds[j],
                              myMesh->commElemProcIds[j],
                              my_pid);
    }
  }

  if(err == 0)
    return true;
  else
    return false;
}

