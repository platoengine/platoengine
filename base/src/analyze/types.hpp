#ifndef TYPES_H
#define TYPES_H

#include <float.h>

#include "mpi.h"

typedef double Real;

typedef int VarIndex;
typedef unsigned int uint;

const int UNSET_VAR_INDEX = -1;

#define DEFAULT_INT  999;
#define DEFAULT_REAL 999.;

#ifdef __GNUC__
#define __GIBLET_FUNCTION_NAMER__ __PRETTY_FUNCTION__
#else 
#define __GIBLET_FUNCTION_NAMER__ __FILE__ << ":" << __LINE__
#endif //__GNUCC__"

#endif //TYPES_H
