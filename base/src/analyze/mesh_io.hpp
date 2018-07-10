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
  void GetExodusElementIds(int * a_ElemIds, int a_MyField);
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




