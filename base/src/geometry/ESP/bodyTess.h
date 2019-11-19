/*!
 * bodyTess.h
 *
 * Provided by Bob Haimes
 *
 */

#include "egads.h"


typedef struct {
  int ptype;                            /* vertex type */
  int pindex;                           /* vertex index */
} verTags;


extern int 
bodyTess(ego tess, int *nfaca, int *nedga, int *nvert, double **verts, 
         verTags **vtags, int *ntriang, int **triang);
