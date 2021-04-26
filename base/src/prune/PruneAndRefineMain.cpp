
/*--------------------------------------------------------------------*/
/*    Copyright 2002 - 2008, 2010, 2011 National Technology &         */
/*    Engineering Solutions of Sandia, LLC (NTESS). Under the terms   */
/*    of Contract DE-NA0003525 with NTESS, there is a                 */
/*    non-exclusive license for use of this work by or on behalf      */
/*    of the U.S. Government.  Export of this program may require     */
/*    a license from the United States Government.                    */
/*--------------------------------------------------------------------*/

#if defined( STK_HAS_MPI )
#include <mpi.h>
#endif

#include <iostream>
#include <memory>

#include <stdio.h>
#include <stdlib.h>

#include "MeshManager.hpp"
#include "MeshTransfer.hpp"
#include "MeshRefine.hpp"
#include "MeshPrune.hpp"
#include "MeshBalance.hpp"


int main(int argc,  char **argv)
{
  stk::parallel_machine_init(&argc,&argv);

  MeshManager tMeshManager;
  MeshTransfer tMeshTransfer;
  MeshPrune tMeshPrune;
  MeshRefine tRefineMesh;
  MeshBalance tMeshBalance;

  tMeshManager.setup_mesh(argc, argv);

  if(tMeshManager.get_transfer_flag())
      tMeshTransfer.transfer_mesh(tMeshManager);

  for(int cntr = 0; cntr<tMeshManager.get_refines(); cntr ++)
  {
      if(tMeshManager.get_prune_flag())
      {
          tMeshPrune.prune_mesh(tMeshManager,argc,argv,1);

          tMeshBalance.balance_mesh(tMeshManager);
      }

      tRefineMesh.refine_mesh(tMeshManager);

      if(tMeshManager.get_transfer_flag())
          tMeshTransfer.transfer_mesh(tMeshManager);
  }

  if(tMeshManager.get_prune_flag())
      tMeshPrune.prune_mesh(tMeshManager,argc,argv,1);

  tMeshManager.write_mesh();

  stk::parallel_machine_finalize();

  return 0;
}
