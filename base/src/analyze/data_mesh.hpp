#ifndef _DATA_MESH_H_
#define _DATA_MESH_H_

#include <assert.h>

#include "types.hpp"
#include "Plato_Parser.hpp"

#include <string>
using std::string;

class DataMesh;
class DataContainer;

struct DMNodeSet {
  int id;
  int numNodes;
  int numDist;
  string setName;
  VarIndex NODE_LIST;
};

struct DMSideSet {
  int id;
  int numSides;       //!< Number of elements in sideset
  int numNodes;       //!< Number of nodes in sideset
  int nodesPerFace;   //!< Number of nodes per face
  string setName;
  VarIndex FACE_ID_LIST; //!< List of local side ids
  VarIndex ELEM_ID_LIST; //!< List of local faces for each element in sideset
  VarIndex FACE_NODE_LIST;
};


namespace Topological {
  class Element;
}
class DataContainer;
class MeshIO;
class Element;

#include <vector>
using std::vector;

#include <string>
using std::string;

enum IndexOrder { Cstyle=0, FortranStyle };


enum DataMeshType {
                    DATAMESH,
                    STR_DM,
                    UNS_DM,
                    NUM_DM
                  };

class BlockElemPair {
  public: 
  BlockElemPair() : blockid(-1), elemid(-1) {}
  BlockElemPair(int b, int e) : blockid(b), elemid(e) {}
  int blockid;
  int elemid;

  bool operator==(const BlockElemPair &bp){
    return ((bp.blockid == this->blockid) && (bp.elemid == this->elemid));
  }
  bool operator!=(const BlockElemPair &bp){
    return ((bp.blockid != this->blockid) || (bp.elemid != this->elemid));
  }
};

class DataMesh {
friend class MeshIO;
public:
  DataMesh();
  virtual ~DataMesh();

  virtual bool parseMesh(pugi::xml_node& mesh_spec) { return false; }

  virtual void Connect(int* node_gid_list, int block_index, int nlid_in_blk);

  virtual bool hasContactBlock(){return false; }
  virtual int getContactBlockIndex(){ assert(0); return 0; }

  virtual bool Initialize(Element**){return true;}

  virtual void setTitle( const char* title );
  virtual const char* getTitle();
  virtual void setNumNodes( int Nnp );      /*! sets total number of nodes (ghost + owned) on proc */
  virtual DataMeshType getMeshType() { return myMeshType; }

  virtual bool registerNodeSet(int ids, int number_in_set);
  virtual int  getNumNodeSets() { int nns = nodeSets.size(); return nns; }
  virtual const vector<DMNodeSet>& getNodeSets() { return nodeSets; }
  virtual DMNodeSet* getNodeSet(int i) { return &nodeSets[i]; };
  virtual bool registerSideSet(int ids, int number_faces_in_set, int number_nodes_in_set);
  virtual vector<DMSideSet> getSideSets() { return sideSets; }
  virtual DMSideSet* getSideSet(int i) { return &sideSets[i]; };
  virtual int  getNumSideSets() { int nss = sideSets.size(); return nss; }
  virtual int getNumNodes();      /*! returns total number of nodes on proc (owned + ghosted)*/
  virtual int getNumElems();      /*! returns total number of elements of proc (owned) */

  virtual double* getVolumeFractions(){ return NULL; }

  int getNumElemInBlk(int blk);
  int getNnpeInBlk(int blk);

  virtual Topological::Element* getElemBlk(int);
  virtual const char*  getElemTypeInBlk(int blk) = 0;
  virtual int  getNumElemBlks() = 0;
  virtual int  getBlockId(int blk) = 0;     // given an index find the id
  virtual int  getBlockIndex(int blk) = 0;  // given an id find the index
  virtual int* getElemToNodeConnInBlk(int blk) = 0;
  virtual bool readNodePlot(Real*, string) = 0;
  virtual void addElemBlk(Topological::Element*){ return; }

  virtual void setDimensions(int);
  virtual int  getDimensions();
  virtual bool registerData();
  virtual void setDataContainer(DataContainer*);
  virtual DataContainer* getDataContainer(){ return myData; }
  virtual Real* getX();
  virtual Real* getY();
  virtual Real* getZ();
  virtual Real* getX0();
  virtual Real* getY0();
  virtual Real* getZ0();
  virtual void getCoords(Real** X);

  virtual bool isExplicit(int global_element_id){ return true; }

public: //!data
  int *nodeGlobalIds;
  int *elemGlobalIds;
  int *nodeOwnership;    //1 if owned by this processor, 0 if not
  int *startingDofPlid;  //index into RowMap for first dof of node_lid index
  int globalNnp;         //global number of nodes in mesh
  int globalNel;         //global number of elements in mesh
  int globalNeblock;     //global number of element blocks in mesh
  int global_num_node_sets;
  int global_num_side_sets;

  int num_procs;
  int num_proc_in_file;
  char ftype;
  int num_internal_nodes;
  int num_border_nodes;
  int num_external_nodes;
  int num_internal_elems;
  int num_border_elems;
  int numNodeCommMaps;
  int numElemCommMaps;
  int *internalElems;
  int *borderElems;
  int *internalNodes;
  int *borderNodes;
  int *externalNodes;
  int *nodeCmapNodeCnts;
  int *nodeCmapIds;
  int *elemCmapElemCnts;
  int *elemCmapIds;
  int **commNodeIds;
  int **commNodeProcIds;
  int **commElemIds;
  int **commSideIds;
  int **commElemProcIds;

protected: //!data
  DataMeshType myMeshType;     //! meshtype for RTTI
  int  myDimensions;           //! dimensionality of problem
  int  numNodes;               //! number of (total=ghost+local) nodes in datamesh 
  int  numElems;               //! number of elements in topo
  int *numDofsOnNodes;         //! vector of number of dofs on each node (totalNumNodes long)
  vector<DMNodeSet> nodeSets;    //! all node sets
  vector<DMSideSet> sideSets;    //! all side sets

  string myTitle;              //! topology title
  DataContainer* myData;       //! copy of pointer to data container

  VarIndex XMATCOOR;           //! variable index to x material coordinate
  VarIndex YMATCOOR;           //! variable index to y material coordinate
  VarIndex ZMATCOOR;           //! variable index to z material coordinate
  VarIndex XMATCOOR0;          //! variable index to initial x material coordinate
  VarIndex YMATCOOR0;          //! variable index to initial y material coordinate
  VarIndex ZMATCOOR0;          //! variable index to initial z material coordinate

protected: //!functions
  vector <Topological::Element*> myElemBlk; //! groups of elements

private: //!data

private: //!functions
  DataMesh(const DataMesh&); //!no copy allowed
  DataMesh& operator=(const DataMesh&);
};

class UnsMesh : public DataMesh
{
public:
  UnsMesh(){ zeroSet(); }
  UnsMesh(DataContainer* dc){ zeroSet(); myData = dc; }
  virtual ~UnsMesh();

  virtual bool parseMesh(pugi::xml_node& mesh_spec);

  int getNumElemInBlk(int);
  int getNnpeInBlk(int blk);
  virtual const char*  getElemTypeInBlk(int blk);
  virtual int  getNumElemBlks();
  virtual int  getBlockId(int blk);
  virtual int  getBlockIndex(int blk);
  virtual int* getElemToNodeConnInBlk(int blk);
  virtual void addElemBlk(Topological::Element*);
  virtual bool readNodePlot(Real*, string);

protected: //!data

private: //!data
  MeshIO* myMeshInput;         //! mesh input - if required

private: //! member functions
  void zeroSet();
private: //!no copy allowed
  UnsMesh(const UnsMesh&);
  UnsMesh& operator=(const UnsMesh&);
};

class StrMesh : public DataMesh
{
public:
  StrMesh(){ zeroSet(); }
  StrMesh(DataContainer* dc){ zeroSet(); myData = dc; }
  virtual ~StrMesh(){}

  virtual bool parseMesh(pugi::xml_node& mesh_spec);

  int getNumElemInBlk(int);
  int getNnpeInBlk(int blk);
  virtual const char*  getElemTypeInBlk(int blk);
  virtual int  getNumElemBlks();
  virtual int  getBlockId(int blk);
  virtual int  getBlockIndex(int blk);
  virtual int* getElemToNodeConnInBlk(int blk);
  virtual bool readNodePlot(Real*, string);

protected: //!data

private: //!data

  int indexMap( int, int, int, int, int, int);

  IndexOrder indexOrdering;

  bool nodeLocationsExternal;

  int numGlobalNodesInX;
  int numGlobalNodesInY;
  int numGlobalNodesInZ;

  int numLocalNodesInX;
  int numLocalNodesInY;
  int numLocalNodesInZ;

  int iBegin, iEnd;
  int jBegin, jEnd;
  int kBegin, kEnd;

  int numLocalElementsInX;
  int numLocalElementsInY;
  int numLocalElementsInZ;

  int numGlobalElementsInX;
  int numGlobalElementsInY;
  int numGlobalElementsInZ;

  double xBeginGlobal, xEndGlobal;
  double yBeginGlobal, yEndGlobal;
  double zBeginGlobal, zEndGlobal;

  double xBegin, xEnd;
  double yBegin, yEnd;
  double zBegin, zEnd;

private: //! member functions
  void zeroSet();
private: //!no copy allowed
  StrMesh(const StrMesh&);
  StrMesh& operator=(const StrMesh&);
};


#endif //_DATA_MESH_H_
