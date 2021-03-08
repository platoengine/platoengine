
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

#include "STKPrune.hpp"


using namespace prune;

int main(int argc,  char **argv)
{
  STKPrune pruner;
  if(pruner.create_mesh_apis_stand_alone(argc, argv, "", "", "LSD", "", 1e-5,
                     0.0, 0, 1, 0, 0))
  {
    pruner.run_stand_alone();
  }

  return 0;
}
