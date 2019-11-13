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

#include <Python.h>
#include "exodusII.h"

/*****************************************************************************/
// Define an error object that is unique to the expy module
/*****************************************************************************/
static PyObject *ExpyErrorObject;




/*****************************************************************************/
// CONVERSION FUNCTIONS
/*****************************************************************************/

ex_entity_type get_enum_from_var_type(const char * var_type);


/*****************************************************************************/
// create a C double array from a Python list
/*****************************************************************************/
static void double_array_from_list(PyObject* list, double* array) {
  int i;
  int length = PyList_Size(list);
  for(i = 0; i < length; i++) {
    PyObject *v = PyList_GetItem(list,i);
    if(!PyFloat_Check(v)) {
      PyErr_SetString(PyExc_TypeError, "list must contain only reals");
      return;
    }
    array[i] = PyFloat_AsDouble(v);
  }
}

/*****************************************************************************/
// create a python list from C double array
/*****************************************************************************/
static PyObject* list_from_double_array(double* array, int array_length)
{
  int i;

  PyObject *newlist = PyList_New(array_length);

  for(i=0; i<array_length; i++)
    PyList_SetItem(newlist, i, PyFloat_FromDouble(array[i]));

  return newlist;
  
}

/*****************************************************************************/
// create a python list from C int array
/*****************************************************************************/
static PyObject* list_from_int_array(int* array, int array_length)
{
  int i;

  PyObject *newlist = PyList_New(array_length);

  for(i=0; i<array_length; i++)
    PyList_SetItem(newlist, i, PyLong_FromLong(array[i]));

  return newlist;
  
}

/*****************************************************************************/
// create a C string array from a Python list
/*****************************************************************************/
static void string_array_from_list(PyObject* list, char** array) {
  int i;
  int length = PyList_Size(list);
  for(i = 0; i < length; i++) {
    PyObject *v = PyList_GetItem(list,i);
    if(!PyString_Check(v)) {
      PyErr_SetString(PyExc_TypeError, "list must contain only strings");
      return;
    }
    array[i] = PyBytes_AsString(v);
  }
}


/*****************************************************************************/
// create a C int array from a Python list
/*****************************************************************************/
static void int_array_from_list(PyObject* list, int* array) {
  int i;
  int length = PyList_Size(list);
  for(i = 0; i < length; i++) {
    PyObject *v = PyList_GetItem(list,i);
    if(!PyLong_Check(v)) {
      PyErr_SetString(PyExc_TypeError, "list must contain only integers");
      return;
    }
    array[i] = (int)PyLong_AsLong(v);
  }
}








/*****************************************************************************/
// EXODUS II FUNCTIONS
/*****************************************************************************/


/******************************************************************************/
static PyObject*
expy_open(PyObject *self, PyObject *args)
/******************************************************************************/
{
  const char* path;
  const char* mode;
  int comp_ws;
  int io_ws;
  float ver;

  int file_id, imode;

  if(!PyArg_ParseTuple(args,"ssii:open",&path, &mode, &comp_ws, &io_ws))
    return NULL;

  if(!strcmp(mode,"write")) imode = EX_WRITE;
  else if(!strcmp(mode,"read")) imode = EX_READ;
  else imode = EX_READ;

  file_id = ex_open(path, imode, &comp_ws, &io_ws, &ver);

  return PyLong_FromLong(file_id);
        
}


/******************************************************************************/
static PyObject *
expy_create(PyObject *self, PyObject *args)
/******************************************************************************/
{
  const char* path;
  int comp_ws = 4;
  int io_ws = 4;
  int genesis_id = 0;
  if (!PyArg_ParseTuple(args, "sii:create", &path, &comp_ws, &io_ws))
    return NULL;

  genesis_id = ex_create(path, EX_CLOBBER, &comp_ws, &io_ws);

  return PyLong_FromLong(genesis_id);
}

/******************************************************************************/
static PyObject *
expy_close(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid;
  int return_value;
  if (!PyArg_ParseTuple(args, "i:close", &exoid))
    return NULL;

  return_value = ex_close(exoid);

  return PyLong_FromLong(return_value);
}

/******************************************************************************/
static PyObject *
expy_update(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid;
  int return_value;
  if (!PyArg_ParseTuple(args, "i:update", &exoid))
    return NULL;

  return_value = ex_update(exoid);

  return PyLong_FromLong(return_value);
}


/******************************************************************************/
static PyObject *
expy_get_init(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid;
  char title[MAX_LINE_LENGTH+1];
  int num_dim;
  int num_nodes;
  int num_elem;
  int num_elem_blk;
  int num_node_sets;
  int num_side_sets;

  int return_status;

  if (!PyArg_ParseTuple(args, "i:get_init", &exoid))
     return NULL;

  return_status = ex_get_init(exoid, title, &num_dim, &num_nodes, &num_elem,
                              &num_elem_blk, &num_node_sets, &num_side_sets);
  
  if(!return_status)
    return Py_BuildValue("siiiiii",title,num_dim,num_nodes,num_elem,
                       num_elem_blk,num_node_sets,num_side_sets);
  else return NULL;
}

/******************************************************************************/
static PyObject *
expy_put_init(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid;
  char* title;
  int num_dim;
  int num_nodes;
  int num_elem;
  int num_elem_blk;
  int num_node_sets;
  int num_side_sets;
  int return_status;
  if (!PyArg_ParseTuple(args, "isiiiiii:put_init", &exoid, &title, &num_dim,
               &num_nodes, &num_elem, &num_elem_blk, &num_node_sets, &num_side_sets))
    return NULL;

  return_status = ex_put_init(exoid, title, num_dim, num_nodes, num_elem,
      num_elem_blk, num_node_sets, num_side_sets);


  return Py_BuildValue("i",return_status);
}

/******************************************************************************/
static PyObject*
expy_get_names(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int i, exoid;
  char* var_type;
  int num_vars;
  char** names;
  PyObject *pynames;

  if(!PyArg_ParseTuple(args, "isi", &exoid, &var_type, &num_vars))
    return NULL;

        pynames = PyList_New(num_vars);
  names = (char**)calloc(num_vars,sizeof(char*));

        for(i=0;i<num_vars;i++)
          names[i] = (char*)calloc((MAX_STR_LENGTH+1),sizeof(char));

  ex_entity_type obj_type = get_enum_from_var_type(var_type);

  ex_get_names(exoid, obj_type, names);

        for(i=0;i<num_vars;i++)
          PyList_SetItem(pynames,i,Py_BuildValue("s",names[i]));

  return pynames;
}

/******************************************************************************/
static PyObject *
expy_put_names(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int i;
  int exoid;
  char *var_type;
  PyObject *pynames;
  char **names;
  int num_names;
  int return_status;

  if (!PyArg_ParseTuple(args, "isO:put_names", &exoid, &var_type, &pynames))
    return NULL;

  if(!PyList_Check(pynames)) {
    PyErr_SetString(PyExc_TypeError, "arg must be list");
    return NULL;
  }

  num_names = PyList_Size(pynames);

  names = (char**) calloc(num_names,sizeof(char*));

  for(i = 0; i < num_names; i++)
    names[i] = (char*) calloc((MAX_LINE_LENGTH+1),sizeof(char));

  string_array_from_list(pynames, names);

  ex_entity_type obj_type = get_enum_from_var_type(var_type);

  return_status = ex_put_names(exoid, obj_type, names);

  return Py_BuildValue("i",return_status);
}
/******************************************************************************/
static PyObject *
expy_put_info(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int i;
  int exoid;
  int return_status;
  int num_info;
  PyObject  *pyinfo;
  char **info;

  if (!PyArg_ParseTuple(args, "iiO:put_info", &exoid, &num_info, &pyinfo))
    return NULL;

  if(!PyList_Check(pyinfo)) {
    PyErr_SetString(PyExc_TypeError, "arg must be list");
    return NULL;
  }

  num_info = PyList_Size(pyinfo);

  info = (char**) calloc(num_info,sizeof(char*));

  for(i = 0; i < num_info; i++) 
    info[i] = (char*) calloc((MAX_LINE_LENGTH+1),sizeof(char));

  string_array_from_list(pyinfo, info);

  return_status = ex_put_info(exoid, num_info, info);

  return Py_BuildValue("i",return_status);
}

/******************************************************************************/
static PyObject *
expy_get_coord(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid, num_dim, num_nodes;
  double *x_coor, *y_coor, *z_coor;

  PyObject *px_coor, *py_coor, *pz_coor;

  if(!PyArg_ParseTuple(args, "iii:get_coord", &exoid, &num_dim, &num_nodes))
    return NULL;

  x_coor = (double*) calloc(num_nodes, sizeof(double));
  y_coor = (double*) calloc(num_nodes, sizeof(double));
  if (num_dim == 3) 
    z_coor = (double*) calloc(num_nodes, sizeof(double));
  else z_coor = NULL;

  ex_get_coord(exoid, x_coor, y_coor, z_coor);

  px_coor = list_from_double_array(x_coor, num_nodes);
  py_coor = list_from_double_array(y_coor, num_nodes);
  if(num_dim == 3)
    pz_coor = list_from_double_array(z_coor, num_nodes);

  if(num_dim == 3)
    return Py_BuildValue("[OOO]", px_coor, py_coor, pz_coor);
  else 
    return Py_BuildValue("[OO]", px_coor, py_coor);

}


/******************************************************************************/
static PyObject *
expy_put_coord(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid;
  int nx, ny, nz, ndim=0;
  int return_status;
  PyObject  *x_coor, *y_coor, *z_coor;
        PyObject *coordinates;
  double *xc_array=NULL, *yc_array=NULL, *zc_array=NULL;

  if (!PyArg_ParseTuple(args, "iiO", &exoid, &ndim, &coordinates))
    return NULL;

        x_coor = PyList_GetItem(coordinates, 0);
  nx = PyList_Size(x_coor);
  xc_array = (double*) calloc(nx,sizeof(double));
  if (nx > 0) double_array_from_list(x_coor, xc_array);

        y_coor = PyList_GetItem(coordinates, 1);
  ny = PyList_Size(y_coor);
  yc_array = (double*) calloc(ny,sizeof(double));
  if (ny > 0) double_array_from_list(y_coor, yc_array);

  if(ndim == 3){
          z_coor = PyList_GetItem(coordinates, 2);
    nz = PyList_Size(z_coor);
    zc_array = (double*) calloc(nz,sizeof(double));
    if (nz > 0) double_array_from_list(z_coor, zc_array);
        } else nz = 0;



  return_status = ex_put_coord(exoid, xc_array, yc_array, zc_array);

  free(xc_array);
  free(yc_array);
  if(ndim == 3) free(zc_array);

  return Py_BuildValue("i",return_status);
}

/******************************************************************************/
static PyObject*
expy_get_var_param(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid;
  char* var_type;
  int num_vars;
  
  if(!PyArg_ParseTuple(args, "is", &exoid, &var_type) )
    return NULL;

  ex_entity_type obj_type = get_enum_from_var_type(var_type);
  
  ex_get_variable_param(exoid, obj_type, &num_vars);
  
  return Py_BuildValue("i",num_vars);

}

/******************************************************************************/
static PyObject*
expy_put_var_param(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid;
  char* var_type;
  int num_vars;
  int return_status;
  
  if(!PyArg_ParseTuple(args, "isi", &exoid, &var_type, &num_vars))
    return NULL;
  
  ex_entity_type obj_type = get_enum_from_var_type(var_type);

  return_status = ex_put_variable_param(exoid, obj_type, num_vars);
  
  return Py_BuildValue("i",return_status);

}

/******************************************************************************/
static PyObject *
expy_get_elem_var(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid, num_elems;
  int time_step, var_index, block_id;
  double *values;
  PyObject *pyvalues;

  if(!PyArg_ParseTuple(args, "iiiii:get_elem_var", &exoid, &time_step, 
                       &var_index, &block_id, &num_elems) )
    return NULL;

  values = (double*) calloc(num_elems, sizeof(double));

  ex_get_var(exoid, time_step, EX_ELEM_BLOCK, var_index, block_id, num_elems, values);

  pyvalues = list_from_double_array(values, num_elems);

  return pyvalues;

}

/******************************************************************************/
static PyObject *
expy_get_nodal_var(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid, num_nodes;
  int time_step, var_index;
  double *values;
  PyObject *pyvalues;

  if(!PyArg_ParseTuple(args, "iiii:get_nodal_var", &exoid, &time_step, 
                       &var_index, &num_nodes) )
    return NULL;

  values = (double*) calloc(num_nodes, sizeof(double));

  ex_get_var(exoid, time_step, EX_NODAL, var_index, 1, num_nodes, values);

  pyvalues = list_from_double_array(values, num_nodes);

  return pyvalues;

}

/******************************************************************************/
static PyObject *
expy_get_node_num_map(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid, num_nodes;
  int *map;
  PyObject *pymap;

  if(!PyArg_ParseTuple(args, "ii:get_node_num_map", &exoid, &num_nodes) )
    return NULL;

  map = (int*) calloc(num_nodes, sizeof(int));

  ex_get_id_map(exoid, EX_NODE_MAP, map);

  pymap = list_from_int_array(map, num_nodes);

  return pymap;

}

/******************************************************************************/
static PyObject *
expy_get_elem_num_map(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid, num_elems;
  int *map;
  PyObject *pymap;

  if(!PyArg_ParseTuple(args, "ii:get_elem_num_map", &exoid, &num_elems) )
    return NULL;

  map = (int*) calloc(num_elems, sizeof(int));

  ex_get_id_map(exoid, EX_ELEM_MAP, map);

  pymap = list_from_int_array(map, num_elems);

  return pymap;

}


/******************************************************************************/
static PyObject *
expy_get_all_times(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid, num_times;
  double *times;
  float fdum;
  char cdum[1];
  PyObject *pytimes;

  if(!PyArg_ParseTuple(args, "i:get_all_times", &exoid) )
    return NULL;

  ex_inquire(exoid, EX_INQ_TIME, &num_times, &fdum, cdum);

  times = (double*) calloc(num_times, sizeof(double));

  ex_get_all_times(exoid, times);

  pytimes = list_from_double_array(times, num_times);

  return pytimes;

}

/******************************************************************************/
static PyObject*
expy_get_var_names(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int i, exoid;
  char* var_type;
  int num_vars;
  char** names;
  PyObject *pynames;

  if(!PyArg_ParseTuple(args, "isi", &exoid, &var_type, &num_vars))
    return NULL;

        pynames = PyList_New(num_vars);
  names = (char**)calloc(num_vars,sizeof(char*));

        for(i=0;i<num_vars;i++) 
          names[i] = (char*)calloc((MAX_STR_LENGTH+1),sizeof(char));
  
  ex_entity_type obj_type = get_enum_from_var_type(var_type);

  ex_get_variable_names(exoid, obj_type, num_vars, names);

        for(i=0;i<num_vars;i++) 
          PyList_SetItem(pynames,i,Py_BuildValue("s",names[i]));

  return pynames;
}

/******************************************************************************/
static PyObject*
expy_put_var_names(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid;
  char* var_type;
  int num_vars;
  int return_status;
  char** names;
  PyObject *pynames;

  if(!PyArg_ParseTuple(args, "isiO", &exoid, &var_type, &num_vars, &pynames))
    return NULL;

  names = (char**)calloc(num_vars,sizeof(char*));
  string_array_from_list(pynames, names);
  
  ex_entity_type obj_type = get_enum_from_var_type(var_type);

  return_status = ex_put_variable_names(exoid, obj_type, num_vars, names);

  return Py_BuildValue("i",return_status);
}

/******************************************************************************/
static PyObject*
expy_put_time(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid;
  int time_step;
  double time_value;
  int return_status;

  if(!PyArg_ParseTuple(args,"iid",&exoid, &time_step, &time_value))
    return NULL;

  return_status = ex_put_time(exoid, time_step, &time_value);

  return Py_BuildValue("i",return_status);
}

/******************************************************************************/
static PyObject*
expy_put_elem_var(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid, time_step, elem_var_index, return_status;
  int elem_blk_id, num_elem_this_blk;
  double* elem_var_vals;
  PyObject *py_elem_var_vals;

  if(!PyArg_ParseTuple(args, "iiiiiO", &exoid, &time_step, &elem_var_index,
                             &elem_blk_id, &num_elem_this_blk, &py_elem_var_vals))
    return NULL;

  elem_var_vals = (double*)calloc(num_elem_this_blk,sizeof(double));
  double_array_from_list(py_elem_var_vals, elem_var_vals);
  
  return_status = ex_put_var(exoid, time_step, EX_ELEM_BLOCK, elem_var_index, elem_blk_id, 
                                        num_elem_this_blk, elem_var_vals);

  return Py_BuildValue("i",return_status);
}

/******************************************************************************/
static PyObject*
expy_put_nodal_var(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid, time_step, node_var_index, return_status;
  int num_nodes;
  double* node_var_vals;
  PyObject *py_node_var_vals;

  if(!PyArg_ParseTuple(args, "iiiiO", &exoid, &time_step, &node_var_index,
                             &num_nodes, &py_node_var_vals))
    return NULL;

  node_var_vals = (double*)calloc(num_nodes,sizeof(double));
  double_array_from_list(py_node_var_vals, node_var_vals);
  
  return_status = ex_put_var(exoid, time_step, EX_NODAL, node_var_index, 
                                        1, num_nodes, node_var_vals);

  return Py_BuildValue("i",return_status);
}

/******************************************************************************/
static PyObject*
expy_put_elem_attr(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid, return_status;
  int block_id, num_attr, num_elem_in_blk;
  double* attrib;
  PyObject* py_attrib;
  
  if(!PyArg_ParseTuple(args,"iiiiO", &exoid, &block_id, &num_attr, 
                       &num_elem_in_blk, &py_attrib))
    return NULL;

  if(num_attr*num_elem_in_blk != PyList_Size(py_attrib))
    return NULL;

  attrib = (double*)calloc(num_attr*num_elem_in_blk, sizeof(double));
  double_array_from_list(py_attrib, attrib);

  return_status = ex_put_attr(exoid, EX_ELEM_BLOCK, block_id, attrib);

  return Py_BuildValue("i",return_status);
}

/******************************************************************************/
static PyObject *
expy_get_coord_names(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int i;
  int exoid, num_dim;
  char *coord_names[3];
  PyObject *names[3];

  if (!PyArg_ParseTuple(args, "ii:get_coord_names", &exoid, &num_dim))
    return NULL;

  for(i=0;i<num_dim;i++) 
    coord_names[i] = (char*)calloc((MAX_STR_LENGTH+1),sizeof(char));

  ex_get_coord_names(exoid, coord_names);

  for(i=0;i<num_dim;i++) 
    names[i] = Py_BuildValue("s",coord_names[i]);

  if(num_dim == 3)
    return Py_BuildValue("[OOO]", names[0], names[1], names[2]);
  else
    return Py_BuildValue("[OO]", names[0], names[1]);
  
}

/******************************************************************************/
static PyObject *
expy_put_coord_names(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid;
  int return_status;
  PyObject  *pynames;
  char *names[3];

  if (!PyArg_ParseTuple(args, "iO:put_coord_names", &exoid, &pynames))
    return NULL;

  if(!PyList_Check(pynames)) {
    PyErr_SetString(PyExc_TypeError, "arg must be list");
    return NULL;
  }

  string_array_from_list(pynames, names);

  return_status = ex_put_coord_names(exoid, names);

  return Py_BuildValue("i",return_status);
}

/******************************************************************************/
static PyObject *
expy_get_elem_block(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid, elem_blk_id;
  char elem_type[MAX_STR_LENGTH+1];
  int num_elem_this_blk, num_nodes_per_elem, num_attr;
  PyObject *py_elem_type;
  PyObject *py_num_elem_this_blk, *py_num_nodes_per_elem, *py_num_attr;

  if (!PyArg_ParseTuple(args, "ii", &exoid, &elem_blk_id))
    return NULL;

  ex_get_block(exoid, EX_ELEM_BLOCK, elem_blk_id, elem_type,
                    &num_elem_this_blk, &num_nodes_per_elem, NULL, NULL, &num_attr);

  py_elem_type = Py_BuildValue("s",elem_type);
  py_num_elem_this_blk = Py_BuildValue("i",num_elem_this_blk);
  py_num_nodes_per_elem = Py_BuildValue("i",num_nodes_per_elem);
  py_num_attr = Py_BuildValue("i",num_attr);

  return Py_BuildValue("OOOO", py_elem_type, py_num_elem_this_blk, 
                       py_num_nodes_per_elem, py_num_attr);
}

/******************************************************************************/
static PyObject *
expy_put_elem_block(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid;
  int elem_blk_id;
  char *elem_type;
  int num_elem_this_blk;
  int num_nodes_per_elem;
  int num_attr;
  int return_status;

  if (!PyArg_ParseTuple(args, "iisiii:put_elem_conn", &exoid, 
      &elem_blk_id, &elem_type, &num_elem_this_blk, 
      &num_nodes_per_elem, &num_attr))
    return NULL;

  return_status = ex_put_block(exoid, EX_ELEM_BLOCK, elem_blk_id, elem_type,
      num_elem_this_blk,num_nodes_per_elem, 0, 0, num_attr);

  return Py_BuildValue("i",return_status);
}

/******************************************************************************/
static PyObject *
expy_get_elem_blk_ids(PyObject *self, PyObject *args)
/******************************************************************************/
{
 
  int exoid, num_elem_blks;
  int *elem_blk_ids;
  PyObject *py_elem_blk_ids;

  if(!PyArg_ParseTuple(args,"ii",&exoid,&num_elem_blks))
    return NULL;

  elem_blk_ids = (int*)calloc(num_elem_blks,sizeof(int));

  ex_get_ids(exoid, EX_ELEM_BLOCK, elem_blk_ids);

  py_elem_blk_ids = list_from_int_array(elem_blk_ids, num_elem_blks);

  return py_elem_blk_ids;
  
}

/******************************************************************************/
static PyObject *
expy_get_elem_conn(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int *connect;
  int exoid, elem_blk_id, num_elem_this_blk, num_nodes_per_elem;
  int connect_length;
  PyObject *py_connect;

  if(!PyArg_ParseTuple(args, "iiii", &exoid, &elem_blk_id, &num_elem_this_blk, 
                       &num_nodes_per_elem))
     return NULL;

  connect_length = num_elem_this_blk*num_nodes_per_elem;
  connect = (int*)calloc(connect_length,sizeof(int));

  ex_get_conn(exoid, EX_ELEM_BLOCK, elem_blk_id, connect, 0, 0);

  py_connect = list_from_int_array(connect, connect_length);
  free(connect);

  return py_connect;

}

/******************************************************************************/
static PyObject *
expy_put_elem_conn(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid;
  int return_status;
  PyObject  *pyintarray;
  int* connect;
  int elem_blk_id;

  if (!PyArg_ParseTuple(args, "iiO:put_elem_conn", &exoid,&elem_blk_id,&pyintarray))
    return NULL;

  if(!PyList_Check(pyintarray)) {
    PyErr_SetString(PyExc_TypeError, "arg must be list");
    return NULL;
  }

  connect = (int*) calloc(PyList_Size(pyintarray),sizeof(int));
  int_array_from_list(pyintarray, connect);

  return_status = ex_put_conn(exoid, EX_ELEM_BLOCK, elem_blk_id, connect, 0, 0);

  free(connect);

  return Py_BuildValue("i",return_status);
}

/******************************************************************************/
static PyObject *
expy_put_node_set_param(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid, node_set_id, num_nodes_in_set, num_dist_in_set;
  int return_status;

  if(!PyArg_ParseTuple(args,"iiii",&exoid, &node_set_id, &num_nodes_in_set,
                       &num_dist_in_set))
    return NULL;

  return_status = ex_put_set_param(exoid, EX_NODE_SET, node_set_id, num_nodes_in_set,
                                        num_dist_in_set);

  return Py_BuildValue("i",return_status);
}

/******************************************************************************/
static PyObject *
expy_get_node_set_param(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid, node_set_id, num_nodes_in_set, num_dist_in_set;
  PyObject *py_num_nodes_in_set, *py_num_dist_in_set;

  if(!PyArg_ParseTuple(args, "ii", &exoid, &node_set_id))
    return NULL;

	ex_get_set_param(exoid, EX_NODE_SET, node_set_id, &num_nodes_in_set, &num_dist_in_set);

  py_num_nodes_in_set = Py_BuildValue("i",num_nodes_in_set);
  py_num_dist_in_set = Py_BuildValue("i",num_dist_in_set);

  return Py_BuildValue("OO", py_num_nodes_in_set, py_num_dist_in_set);
}

/******************************************************************************/
static PyObject *
expy_get_node_set_ids(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid, num_node_sets;
  int *node_set_ids;
  PyObject *py_node_set_ids;

  if(!PyArg_ParseTuple(args, "ii", &exoid, &num_node_sets))
    return NULL;

  node_set_ids = (int*)calloc(num_node_sets, sizeof(int));

  ex_get_ids(exoid, EX_NODE_SET, node_set_ids);

        py_node_set_ids = list_from_int_array(node_set_ids, num_node_sets);
        free(node_set_ids);

  return py_node_set_ids;
  
}

/******************************************************************************/
static PyObject *
expy_put_node_set(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid, return_status, node_set_id, num_nodes;
  int* node_set_node_list;
  PyObject* py_node_set_node_list;

  if(!PyArg_ParseTuple(args,"iiO", &exoid, &node_set_id, &py_node_set_node_list))
    return NULL;

  num_nodes = PyList_Size(py_node_set_node_list);
  node_set_node_list = (int*)calloc(num_nodes,sizeof(int));
  int_array_from_list(py_node_set_node_list, node_set_node_list);

  return_status = ex_put_set(exoid, EX_NODE_SET, node_set_id, node_set_node_list, NULL);

  return Py_BuildValue("i",return_status);
}

/******************************************************************************/
static PyObject *
expy_get_node_set(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid, node_set_id, num_nodes_in_set;
  int* node_set_node_list;
  PyObject *py_node_set_node_list;

  if(!PyArg_ParseTuple(args, "iii", &exoid, &node_set_id, &num_nodes_in_set))
    return NULL;
         
  node_set_node_list = (int*)calloc(num_nodes_in_set,sizeof(int));

  ex_get_set(exoid, EX_NODE_SET, node_set_id, node_set_node_list, NULL);

  py_node_set_node_list = list_from_int_array(node_set_node_list, num_nodes_in_set);
  free(node_set_node_list);

  return py_node_set_node_list;
        
}

/******************************************************************************/
static PyObject *
expy_get_side_set_param(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid, side_set_id, num_side_in_set, num_dist_in_set;
  PyObject *py_num_side_in_set, *py_num_dist_in_set;

  if(!PyArg_ParseTuple(args, "ii", &exoid, &side_set_id))
    return NULL;

  ex_get_set_param(exoid, EX_SIDE_SET, side_set_id, &num_side_in_set, &num_dist_in_set);

  py_num_side_in_set = Py_BuildValue("i",num_side_in_set);
  py_num_dist_in_set = Py_BuildValue("i",num_dist_in_set);

  return Py_BuildValue("OO", py_num_side_in_set, py_num_dist_in_set);
}

/******************************************************************************/
static PyObject *
expy_get_side_set_ids(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid, num_side_sets;
  int *side_set_ids;
  PyObject *py_side_set_ids;

  if(!PyArg_ParseTuple(args, "ii", &exoid, &num_side_sets))
    return NULL;

  side_set_ids = (int*)calloc(num_side_sets, sizeof(int));

  ex_get_ids(exoid, EX_SIDE_SET, side_set_ids);

  py_side_set_ids = list_from_int_array(side_set_ids, num_side_sets);
  free(side_set_ids);

  return py_side_set_ids;
  
}

/******************************************************************************/
static PyObject *
expy_get_side_set(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid, side_set_id, num_sides_in_set;
  int *side_set_elem_list, *side_set_side_list;
  PyObject *py_side_set_elem_list, *py_side_set_side_list;

  if(!PyArg_ParseTuple(args, "iii", &exoid, &side_set_id, &num_sides_in_set))
    return NULL;
         
  side_set_elem_list = (int*)calloc(num_sides_in_set,sizeof(int));
  side_set_side_list = (int*)calloc(num_sides_in_set,sizeof(int));

  ex_get_set(exoid, EX_SIDE_SET, side_set_id, side_set_elem_list, side_set_side_list);

  py_side_set_elem_list = list_from_int_array(side_set_elem_list, num_sides_in_set);
  py_side_set_side_list = list_from_int_array(side_set_side_list, num_sides_in_set);
  free(side_set_elem_list);
  free(side_set_side_list);

  return Py_BuildValue("OO",py_side_set_elem_list, py_side_set_side_list);
        
}


/******************************************************************************/
static PyObject *
expy_put_side_set_param(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid;
  int side_set_id;
  int num_side_in_set;
  int num_dist_fact_in_set;
  int return_status;

  if (!PyArg_ParseTuple(args, "iiii:put_side_set_param", &exoid,&side_set_id,
                        &num_side_in_set,&num_dist_fact_in_set))
    return NULL;

  return_status = ex_put_set_param(exoid, EX_SIDE_SET, side_set_id, num_side_in_set, 
                                              num_dist_fact_in_set);

  return Py_BuildValue("i",return_status);
}

/******************************************************************************/
static PyObject *
expy_put_side_set(PyObject *self, PyObject *args)
/******************************************************************************/
{
  int exoid;
  int side_set_id;
  PyObject  *pyelemlist;
  PyObject  *pysidelist;
  int* elemlist;
  int* sidelist;
  int return_status;

  if (!PyArg_ParseTuple(args, "iiOO:put_elem_conn", &exoid,&side_set_id,&pyelemlist,&pysidelist))
    return NULL;

  if(!PyList_Check(pyelemlist)) {
    PyErr_SetString(PyExc_TypeError, "arg must be list");
    return NULL;
  }

  if(!PyList_Check(pysidelist)) {
    PyErr_SetString(PyExc_TypeError, "arg must be list");
    return NULL;
  }

  elemlist = (int*) calloc(PyList_Size(pyelemlist),sizeof(int));
  int_array_from_list(pyelemlist, elemlist);

  sidelist = (int*) calloc(PyList_Size(pysidelist),sizeof(int));
  int_array_from_list(pysidelist, sidelist);

  return_status = ex_put_set(exoid, EX_SIDE_SET, side_set_id, elemlist, sidelist);

  free(elemlist);
  free(sidelist);

  return Py_BuildValue("i",return_status);
}


/*****************************************************************************/
// List of functions defined in the module 
/*****************************************************************************/
static PyMethodDef expy_methods[] = {
  {"open",                expy_open,                METH_VARARGS},
  {"create",              expy_create,              METH_VARARGS},
  {"close",               expy_close,               METH_VARARGS},
  {"update",              expy_update,              METH_VARARGS},
  {"put_init",            expy_put_init,            METH_VARARGS},
  {"get_init",            expy_get_init,            METH_VARARGS},
  {"put_coord",           expy_put_coord,           METH_VARARGS},
  {"get_coord",           expy_get_coord,           METH_VARARGS},
  {"put_coord_names",     expy_put_coord_names,     METH_VARARGS},
  {"get_coord_names",     expy_get_coord_names,     METH_VARARGS},
  {"put_elem_block",      expy_put_elem_block,      METH_VARARGS},
  {"get_elem_block",      expy_get_elem_block,      METH_VARARGS},
  {"get_elem_blk_ids",    expy_get_elem_blk_ids,    METH_VARARGS},
  {"put_elem_conn",       expy_put_elem_conn,       METH_VARARGS},
  {"get_elem_conn",       expy_get_elem_conn,       METH_VARARGS},
  {"get_node_set",        expy_get_node_set,        METH_VARARGS},
  {"get_node_set_param",  expy_get_node_set_param,  METH_VARARGS},
  {"get_node_set_ids",    expy_get_node_set_ids,    METH_VARARGS},
  {"put_node_set",        expy_put_node_set,        METH_VARARGS},
  {"put_node_set_param",  expy_put_node_set_param,  METH_VARARGS},
  {"get_side_set",        expy_get_side_set,        METH_VARARGS},
  {"get_side_set_param",  expy_get_side_set_param,  METH_VARARGS},
  {"get_side_set_ids",    expy_get_side_set_ids,    METH_VARARGS},
  {"put_side_set",        expy_put_side_set,        METH_VARARGS},
  {"put_side_set_param",  expy_put_side_set_param,  METH_VARARGS},
  {"put_info",            expy_put_info,            METH_VARARGS},
  {"put_var_param",       expy_put_var_param,       METH_VARARGS},
  {"put_var_names",       expy_put_var_names,       METH_VARARGS},
  {"get_var_param",       expy_get_var_param,       METH_VARARGS},
  {"get_var_names",       expy_get_var_names,       METH_VARARGS},
  {"get_all_times",       expy_get_all_times,       METH_VARARGS},
  {"get_node_num_map",    expy_get_node_num_map,    METH_VARARGS},
  {"get_elem_num_map",    expy_get_elem_num_map,    METH_VARARGS},
  {"get_elem_var",        expy_get_elem_var,        METH_VARARGS},
  {"get_nodal_var",       expy_get_nodal_var,       METH_VARARGS},
  {"put_elem_attr",       expy_put_elem_attr,       METH_VARARGS},
  {"put_time",            expy_put_time,            METH_VARARGS},
  {"put_elem_var",        expy_put_elem_var,        METH_VARARGS},
  {"put_nodal_var",       expy_put_nodal_var,       METH_VARARGS},
  {"put_names",           expy_put_names,           METH_VARARGS},
  {"get_names",           expy_get_names,           METH_VARARGS},
  {NULL,NULL}/* sentinel */
};


/*****************************************************************************/
// Required initialization function.  This is the only non-static item in the
// module.  
/*****************************************************************************/
PyMODINIT_FUNC
initexpy(void)
{
PyObject *m, *d;

  /* Initialize the type of the new type object here; doing it here
   * is required for portability to Windows without requiring C++. */
  //Exo_Type.ob_type = &PyType_Type;

  /* Create the module and add the functions */
  m = Py_InitModule("expy", expy_methods);

  /* Add some symbolic constants to the module */
  d = PyModule_GetDict(m);
  ExpyErrorObject = PyErr_NewException("expy.error", NULL, NULL);
  PyDict_SetItemString(d, "error", ExpyErrorObject);
}

ex_entity_type get_enum_from_var_type(const char * var_type)
{
   if(!strcmp(var_type,"n")) 
     return EX_NODAL;
   else if(!strcmp(var_type,"e"))
     return EX_ELEM_BLOCK;
   else if(!strcmp(var_type,"ns"))
     return EX_NODE_SET;
   else if(!strcmp(var_type,"ss"))
     return EX_SIDE_SET;
   else if(!strcmp(var_type,"g"))
     return EX_GLOBAL;
   else
     return 0;
}
