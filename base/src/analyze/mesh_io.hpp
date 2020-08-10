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

#ifndef _MESH_IO_H_
#define _MESH_IO_H_

#include "types.hpp"
#include "data_container.hpp"

#include <string>
#include <vector>
using std::string;
using std::vector;

class DataMesh;
class DataContainer;

  
class MeshIO {

public:
  MeshIO();
  virtual ~MeshIO();
  
  enum Type {READ = 0,
             WRITE,
             CLOBBER,
             NUMBER};

  //! The following MUST be called before MeshIO object may be used.
  bool initMeshIO( DataMesh* mesh, 
                   DataContainer* dc,
                   const char* name,
                   Type type = READ ); 

  virtual void setName(const char* name) { myName = name; }
  virtual void setIgnoreNodeMap(bool ignore_node_map) { myIgnoreNodeMap = ignore_node_map; }
  virtual void setIgnoreElemMap(bool ignore_elem_map) { myIgnoreElemMap = ignore_elem_map; }
  virtual void setMode(Type type) { myType = type; }
  virtual void setData(DataContainer *dc) { myData = dc; }
  virtual void setMesh(DataMesh *mesh) { myMesh = mesh; }
  virtual const char* getName() { return myName.c_str(); }
  //! pure virtuals
  virtual bool openMeshIO()    = 0;         //! Opens a mesh-file
  virtual bool readMeshIO()    = 0;         //! Reads from a mesh-file
  virtual bool writePrologue() = 0;         //! Initial write to mesh file (set-up)
  virtual bool writeTitle()    = 0;
  virtual bool writeQA()       = 0;
  virtual bool writeTime(int,Real)     = 0;
  virtual bool writeNodePlot(Real*, int, int) = 0;
  virtual bool readNodePlot(Real*, string) = 0;
  virtual bool readElemPlot(double* data, string name) = 0;
  virtual bool writeElemPlot(Real*, int, int) = 0;
  virtual bool closeMeshIO()   = 0;
  virtual bool initVars(DataCentering, 
                        int,
                        vector<string>
                        )      = 0;
   
protected:
  int myType;
  string myName;
  bool myIgnoreNodeMap;
  bool myIgnoreElemMap;
  string myTitle;
  int myFileID;
  DataMesh* myMesh;
  DataContainer* myData;
  
private: //!no copy allowed
  MeshIO(const MeshIO&);
  MeshIO& operator=(const MeshIO&);
};

class ExodusIO : public MeshIO {
  
public:
  ExodusIO();
  virtual ~ExodusIO();
  
  virtual bool openMeshIO();
  virtual bool readMeshIO();
  virtual bool writePrologue();
  virtual bool writeTitle();
  virtual bool writeQA();
  virtual bool writeTime(int,Real);
  virtual bool writeNodePlot(Real*, int, int);
  virtual bool readNodePlot(Real*, string);
  virtual bool readElemPlot(double* data, string name);
  virtual bool writeElemPlot(Real*, int, int);
  virtual bool closeMeshIO();
  virtual bool initVars(DataCentering, 
                        int,
                        vector<string>);

protected:
  virtual int  testFile(const char *file_name);
  virtual bool readHeader();
  virtual bool readCoord();
  virtual bool readConn();
  virtual bool writeHeader();
  virtual bool writeCoord();
  virtual bool writeConn();
  void GetExodusNodeIds(int * a_NodeIds, int a_MyFileId);
  void GetSerialNodeIds(int * a_NodeIds, int a_MyFileId);
  void GetParallelNodeIds(int * a_NodeIds, int a_MyFileId);

  void GetExodusElementIds(int * a_ElemIds, int a_MyField);
  void GetSerialElementIds(int * a_ElemIds, int a_MyFileId);
  void GetParallelElementIds(int * a_ElemIds, int a_MyFileId);

private: //!no copy allowed
  ExodusIO(const ExodusIO&);
  ExodusIO& operator=(const ExodusIO&);

};

class NemesisIO : public ExodusIO {
public:
  NemesisIO();
  virtual ~NemesisIO();

  virtual bool openMeshIO();
  virtual bool readMeshIO();
  virtual bool closeMeshIO();

protected:
  virtual bool writeHeader();

private: //functions
  bool readParallelData();
  bool writeParallelData();

private: //!no copy allowed
  NemesisIO(const ExodusIO&);
  NemesisIO& operator=(const ExodusIO&);
};

#endif //_MESH_IO_H_




