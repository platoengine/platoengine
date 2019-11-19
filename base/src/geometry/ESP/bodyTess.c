/*!
 * bodyTess.c
 *
 * Provided by Bob Haimes
 *
 */
#include <stdio.h>
#include <stdlib.h>

#include "bodyTess.h"


/*
 * 	calculates and returns a complete Body tessellation
 *
 * 	where:	body	- ego of a body tessellation
 *              nfaca   - number of faces in the body (returned)
 *              nedga   - number of edges in the body (returned)
 * 		nvert	- Number of vertices (returned)
 * 		verts	- coordinates (returned) 3*nverts in len -- freeable
 *              tags    - type/index/parameter tags (returned)
 *                        nverts in len -- freeable
 * 		ntriang	- number of triangles (returned)
 * 		triang	- triangle indices (returned) 4*ntriang in len
 *			  freeable
 */

int
bodyTess(ego tess, int *nfaca, int *nedga, int *nvert, double **verts,
         verTags **vtags, int *ntriang, int **triang)
{
  int          status, i, j, ntri, sta, *tri, nface, nedge, plen, tlen, nGlobal;
  const int    *tris, *tric, *ptype, *pindex;
  double       *xyzs;
  verTags      *tags;
  const double *points, *uv;
  egTessel     *btess;
  ego          ref;

  *nvert  = *ntriang = 0;
  *verts  = NULL;
  *vtags  = NULL;
  *triang = NULL;

  if (tess == NULL)                 return EGADS_NULLOBJ;
  if (tess->magicnumber != MAGIC)   return EGADS_NOTOBJ;
  if (tess->oclass != TESSELLATION) return EGADS_NOTTESS;
  if (tess->blind == NULL)          return EGADS_NOTFOUND;
  
  status  = EG_statusTessBody(tess, &ref, &sta, &nGlobal);
  if ((status < EGADS_SUCCESS) || (nGlobal == 0)) {
    printf(" Error: EG_statusTessBody %d status = %d (bodyTess)!\n",
           nGlobal, status);
    return status;
  }
  
  btess = (egTessel *) tess->blind;
  nface = btess->nFace;
  nedge = btess->nEdge;
  ntri  = 0;
  for (i = 1; i <= nface; i++) {
    status = EG_getTessFace(tess, i, &plen, &points, &uv, &ptype, &pindex,
                            &tlen, &tris, &tric);
    if (status != EGADS_SUCCESS) {
      printf(" Face %d: EG_getTessFace status = %d (bodyTess)!\n", i, status);
    } else {
      ntri += tlen;
    }
  }
  
  /* get the memory associated with the points */
  
  xyzs = (double *) EG_alloc(3*nGlobal*sizeof(double));
  if (xyzs == NULL) {
    printf(" Error: Can not allocate %d XYZs (bodyTess)!\n", nGlobal);
    return EGADS_MALLOC;
  }
  tags = (verTags *) EG_alloc(nGlobal*sizeof(verTags));
  if (tags == NULL) {
    printf(" Error: Can not allocate %d tags (bodyTess)!\n", nGlobal);
    EG_free(xyzs);
    return EGADS_MALLOC;
  }
  /* get the global data */
  for (i = 0; i < nGlobal; i++) {
    status = EG_getGlobal(tess, i+1, &tags[i].ptype, &tags[i].pindex,
                          &xyzs[3*i]);
    if (status != EGADS_SUCCESS) {
      printf(" Error: EG_getGlobal %d status = %d (bodyTessellation)!\n",
             i+1, status);
      EG_free(tags);
      EG_free(xyzs);
      return status;
    }
  }

  /* fill up the whole triangle list -- a Face at a time */
  
  tri = (int *) EG_alloc(4*ntri*sizeof(int));
  if (tri == NULL) {
    printf(" Error: Can not allocate triangles (bodyTess)!\n");
    EG_free(tags);
    EG_free(xyzs);
    return EGADS_MALLOC;
  }
  ntri = 0;
  for (j = 1; j <= nface; j++) {
    /* get the face tessellation and store it away */
    status = EG_getTessFace(tess, j, &plen, &points, &uv, &ptype, &pindex,
                            &tlen, &tris, &tric);
    if (status != EGADS_SUCCESS) continue;
    for (i = 0; i < tlen; i++, ntri++) {
      status = EG_localToGlobal(tess, j, tris[3*i  ], &tri[4*ntri  ]);
      if (status != EGADS_SUCCESS) {
        printf(" Face %d  %d/0 Error: EG_localToGlobal = %d (bodyTess)!\n",
               j, i+1, status);
        EG_free(tri);
        EG_free(tags);
        EG_free(xyzs);
        return status;
      }
      status = EG_localToGlobal(tess, j, tris[3*i+1], &tri[4*ntri+1]);
      if (status != EGADS_SUCCESS) {
        printf(" Face %d  %d/1 Error: EG_localToGlobal = %d (bodyTess)!\n",
               j, i+1, status);
        EG_free(tri);
        EG_free(tags);
        EG_free(xyzs);
        return status;
      }
      status = EG_localToGlobal(tess, j, tris[3*i+2], &tri[4*ntri+2]);
      if (status != EGADS_SUCCESS) {
        printf(" Face %d  %d/2 Error: EG_localToGlobal = %d (bodyTess)!\n",
               j, i+1, status);
        EG_free(tri);
        EG_free(tags);
        EG_free(xyzs);
        return status;
      }
      tri[4*ntri+3] = j;
    }
  }
  
  *nfaca   = nface;
  *nedga   = nedge;
  *nvert   = nGlobal;
  *vtags   = tags;
  *verts   = xyzs;
  *ntriang = ntri;
  *triang  = tri;
/*@-mustfreefresh@*/
  return EGADS_SUCCESS;
}
