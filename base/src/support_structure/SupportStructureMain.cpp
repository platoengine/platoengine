
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

#include "SupportStructure.hpp"


using namespace plato;
using namespace support_structure;

int main(int argc,  char **argv)
{
    SupportStructure support_structure_generator;
    if(support_structure_generator.createMeshAPIsStandAlone(argc, argv))
    {
        support_structure_generator.run();
    }

    return 0;
}
