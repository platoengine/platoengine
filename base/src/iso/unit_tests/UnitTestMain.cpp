/*------------------------------------------------------------------------*/
/*                 Copyright 2010 Sandia Corporation.                     */
/*  Under terms of Contract DE-AC04-94AL85000, there is a non-exclusive   */
/*  license for use of this work by or on behalf of the U.S. Government.  */
/*  Export of this program may require a license from the                 */
/*  United States Government.                                             */
/*------------------------------------------------------------------------*/

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <utility>

#if defined( STK_HAS_MPI )
#include <mpi.h>
#endif

//#include <percept/fixtures/Fixture.hpp>
//#include <percept/RunEnvironment.hpp>

//#include <percept/pyencore.h>

#if !PY_PERCEPT 

#include <gtest/gtest.h>
#include <mpi.h>

int gl_argc=0;
char** gl_argv=0;

int main(int argc, char **argv)
{
    MPI_Init(&argc, &argv);

    testing::InitGoogleTest(&argc, argv);

    gl_argc = argc;
    gl_argv = argv;

    int returnVal = RUN_ALL_TESTS();

    MPI_Finalize();

    return returnVal;
}

#else
  int main() {return 0;}
#endif
