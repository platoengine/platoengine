
/*------------------------------------------------------------------------*/
/*                 Copyright 2010, 2011 Sandia Corporation.                     */
/*  Under terms of Contract DE-AC04-94AL85000, there is a non-exclusive   */
/*  license for use of this work by or on behalf of the U.S. Government.  */
/*  Export of this program may require a license from the                 */
/*  United States Government.                                             */
/*------------------------------------------------------------------------*/

#if defined( STK_HAS_MPI )
#include <mpi.h>
#endif

#include <iostream>

#include "STKExtract.hpp"


using namespace iso;

int main(int argc,  char **argv)
{
  STKExtract ex;
  if(ex.create_mesh_apis_stand_alone(argc, argv, "", "", "LSD", "", 1e-5,
                     0.0, 0, 1, 0, 0))
  {
      ex.run_stand_alone();
  }

  return 0;
}
