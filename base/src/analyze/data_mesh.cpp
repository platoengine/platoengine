#include "communicator.hpp"
#include "data_container.hpp"
#include "data_mesh.hpp"
#include "mesh_io.hpp"
#include "types.hpp"
#include "topological_element.hpp"
#include "exception_handling.hpp"

#ifdef VERBOSE_DEBUG_LOCATION
#include "utilities.hpp"
#endif

#include <ostream>
#include <sstream>
#include <math.h>
using std::ostringstream;
using std::cout;
using std::endl;


DataMesh::DataMesh()
{
#ifdef VERBOSE_DEBUG_LOCATION
  _print_entering_location(__AXSIS_FUNCTION_NAMER__);
#endif //VERBOSE_DEBUG_LOCATION

  XMATCOOR  = UNSET_VAR_INDEX;
  YMATCOOR  = UNSET_VAR_INDEX;
  ZMATCOOR  = UNSET_VAR_INDEX;
  XMATCOOR0 = UNSET_VAR_INDEX;
  YMATCOOR0 = UNSET_VAR_INDEX;
  ZMATCOOR0 = UNSET_VAR_INDEX;
  myDimensions = 3;
  numElems = 0;
  numNodes = 0;
  myMeshType      = DATAMESH;
  numDofsOnNodes  = NULL;
  startingDofPlid = NULL;

// parallel data stuff... comms etc...
  internalElems    = NULL;
  num_internal_elems = 0;
  borderElems      = NULL;
  num_border_elems = 0;
  internalNodes    = NULL;
  num_internal_nodes = 0;
  borderNodes      = NULL;
  num_border_nodes = 0;
  externalNodes    = NULL;
  num_external_nodes = 0;
  commNodeIds      = NULL;
  commNodeProcIds  = NULL;
  commElemIds      = NULL;
  commSideIds      = NULL;
  commElemProcIds  = NULL;
  nodeCmapNodeCnts = NULL;
  nodeCmapIds      = NULL;
  numNodeCommMaps = 0;
  numElemCommMaps = 0;

  elemCmapElemCnts = NULL;
  elemCmapIds = NULL;
}

UnsMesh::~UnsMesh()
{
  // delete elements
//  for(size_t i=0; i<myElemBlk.size(); ++i) {
//    Topological::Element* eb = myElemBlk[i];
//    if( eb )
//      delete eb;
//  }

  if(myMeshInput)
    delete myMeshInput;

}

DataMesh::~DataMesh()
{
#ifdef VERBOSE_DEBUG_LOCATION
  _print_entering_location(__AXSIS_FUNCTION_NAMER__);
#endif //VERBOSE_DEBUG_LOCATION

  delete [] numDofsOnNodes;
  delete [] startingDofPlid;

  delete [] internalElems;
  delete [] borderElems;
  delete [] internalNodes;
  delete [] borderNodes;
  delete [] externalNodes;

  for(int j=0; j<numNodeCommMaps; j++) {
    delete [] commNodeIds[j];
    delete [] commNodeProcIds[j];
  }
  delete [] commNodeIds;
  delete [] commNodeProcIds;

  for(int j=0; j<numElemCommMaps; j++) {
    delete [] commElemIds[j];
    delete [] commSideIds[j];
    delete [] commElemProcIds[j];
  }

  delete [] commElemIds;
  delete [] commSideIds;
  delete [] commElemProcIds;

  delete [] nodeCmapNodeCnts;
  delete [] nodeCmapIds;     
  delete [] elemCmapElemCnts;
  delete [] elemCmapIds;     

  for(unsigned int i=0; i<myElemBlk.size(); i++){
    if(myElemBlk[i]) delete myElemBlk[i];
  }

}

bool
DataMesh::registerNodeSet(int ids, int number_in_set)
{
#ifdef VERBOSE_DEBUG_LOCATION
  _print_entering_location(__AXSIS_FUNCTION_NAMER__);
#endif //VERBOSE_DEBUG_LOCATION

  DMNodeSet ns;
  ns.id = ids;
  ns.numNodes = number_in_set;

  ostringstream fullname;
  string name("Node_Set_");

  fullname << name << ns.id;

  if(ns.numNodes == 0){
    ns.NODE_LIST = -1;
  } else {

    ns.NODE_LIST = myData->registerVariable( IntType, 
  					   fullname.str().c_str(),
  					   NODE,
  					   ns.numNodes ); 
  }

  nodeSets.push_back(ns);

  return true;

}

bool
DataMesh::registerSideSet(int ids, int number_faces_in_set, int number_nodes_in_set)
{
#ifdef VERBOSE_DEBUG_LOCATION
  _print_entering_location(__AXSIS_FUNCTION_NAMER__);
#endif //VERBOSE_DEBUG_LOCATION

  DMSideSet ss;
  ss.id = ids;
  ss.numSides = number_faces_in_set;
  ss.numNodes = number_nodes_in_set;

  if(ss.numSides == 0){
    ss.nodesPerFace = 0;
    ss.FACE_ID_LIST = -1;
    ss.ELEM_ID_LIST = -1;
    ss.FACE_NODE_LIST = -1;
  } else {

    int nodes_per_face = number_nodes_in_set / number_faces_in_set;

    if(nodes_per_face * number_faces_in_set != number_nodes_in_set) {
      throw ParsingException( "Fatal Error: Sidesets may have only one element type...");
    }

    ss.nodesPerFace = nodes_per_face;

    ostringstream fullname;
    string name("FaceID_Side_Set_");
    fullname << name << ss.id;

    ss.FACE_ID_LIST = myData->registerVariable( IntType,
	  				      fullname.str().c_str(),
					      NODE,
					      ss.numSides ); 
    
    name = "Elem_Side_Set_";
    fullname << name << ss.id;
    ss.ELEM_ID_LIST = myData->registerVariable( IntType,
					      fullname.str().c_str(),
					      NODE,
					      ss.numSides ); 

    name = "Face_Node_Side_Set_";
    fullname << name << ss.id;
    ss.FACE_NODE_LIST = myData->registerVariable( IntType, 
                                                  fullname.str().c_str(),
                                                  UNSET,
                                                  ss.numSides*ss.nodesPerFace);
    
  }
  string name("Side_Set_");
  ostringstream fullname;
  fullname << name << ss.id;
  ss.setName = fullname.str();  
  sideSets.push_back(ss);

  return true;
}

void
DataMesh::getCoords(Real** X)
{
  myData->getVariable(XMATCOOR, X[0]);
  myData->getVariable(YMATCOOR, X[1]);
  if( myDimensions == 3 )
    myData->getVariable(ZMATCOOR, X[2]);

  return;
}

Real*
DataMesh::getX0()
{
  Real* pntr;
  myData->getVariable(XMATCOOR0, pntr);
  return pntr;
}

Real*
DataMesh::getY0()
{
  Real* pntr;
  myData->getVariable(YMATCOOR0, pntr);
  return pntr;
}

Real*
DataMesh::getZ0()
{
  Real* pntr;
  if( myDimensions == 3 )
    myData->getVariable(ZMATCOOR0, pntr);
  else
    pntr = NULL;
  return pntr;
}

Real*
DataMesh::getX()
{
  Real* pntr;
  myData->getVariable(XMATCOOR, pntr);
  return pntr;
}

Real*
DataMesh::getY()
{
  Real* pntr;
  myData->getVariable(YMATCOOR, pntr);
  return pntr;
}

Real*
DataMesh::getZ()
{
  Real* pntr;
  if( myDimensions == 3 )
    myData->getVariable(ZMATCOOR, pntr);
  else
    pntr = NULL;
  return pntr;
}

const char*
DataMesh::getTitle()
{
  return (myTitle.c_str());
}

void
DataMesh::setDataContainer(DataContainer* dc)
{
  myData = dc;
};

void
UnsMesh::addElemBlk( Topological::Element* eb )
{
  numElems += eb->getNumElem();
  myElemBlk.push_back(eb);
}

Topological::Element* 
DataMesh::getElemBlk( int number )
{
  return myElemBlk[number];
}

int
UnsMesh::getNumElemBlks()
{
  return (myElemBlk.size());
}

int
DataMesh::getNumElemInBlk(int block)
{
  int number_of_elem_in_block = myElemBlk[block]->getNumElem();
  return number_of_elem_in_block;
}

bool
DataMesh::registerData()
{

#ifdef VERBOSE_DEBUG_LOCATION
  _print_entering_location(__AXSIS_FUNCTION_NAMER__);
#endif //VERBOSE_DEBUG_LOCATION

  DataContainer* dc = myData;


  XMATCOOR       = dc->registerVariable( RealType,
					 "XMATCOOR",
					 NODE,
					 numNodes ); 
  

  YMATCOOR       = dc->registerVariable( RealType,
					 "YMATCOOR",
					 NODE,
					 numNodes ); 
  
  if(myDimensions == 3) {
    ZMATCOOR       = dc->registerVariable( RealType,
  					   "ZMATCOOR",
					   NODE,
					   numNodes );
  }

  XMATCOOR0      = dc->registerVariable(RealType, 
                                        "XMATCOOR0", 
                                        NODE, 
                                        numNodes);

  YMATCOOR0      = dc->registerVariable(RealType, 
                                        "YMATCOOR0",
                                        NODE, 
                                        numNodes);

  if(myDimensions == 3) {
    ZMATCOOR0      = dc->registerVariable( RealType,
  					   "ZMATCOOR0",
					   NODE,
					   numNodes );
  }

  return true;

}

void DataMesh::setNumNodes( int Nnp ) { numNodes = Nnp; }
int DataMesh::getNumNodes() { return numNodes; }

int DataMesh::getNumElems() { return numElems; }


void DataMesh::setDimensions(int dimensions) { myDimensions = dimensions; }
int DataMesh::getDimensions() { return myDimensions; }

void DataMesh::setTitle(const char* title) { myTitle = title; }


/*****************************************************************************/
void DataMesh::Connect(int* node_gid_list, int block_index, int nlid_in_blk)
/*****************************************************************************/
{
  Topological::Element* elem = myElemBlk[block_index];
  int Nnpe = elem->getNnpe();
  int* conn = elem->getNodeConnect() + nlid_in_blk*Nnpe;
  for(int i=0;i<Nnpe;i++) 
    node_gid_list[i] = conn[i];
}

//*********************************************************************
int StrMesh::indexMap(int i, int j, int k, int I, int J, int K)
//*********************************************************************
{
   if( indexOrdering == Cstyle )
     return K*J*i + K*j + k;
   else 
     return i + I*j + I*J*k;
}

//*********************************************************************
bool StrMesh::parseMesh(pugi::xml_node& input)
//*********************************************************************
{

  string ordering = Plato::Parse::getString( input, "index_ordering" );
  if( ordering == "C" ) 
    indexOrdering = Cstyle;
  else if( ordering == "Fortran" ) 
    indexOrdering = FortranStyle;
  else
    throw ParsingException("Unrecognized index ordering.  Specify C or Fortran.");
  
  if( Plato::Parse::numChildren( input, "intervals" ) != 3 )
    throw ParsingException("'intervals' must be length 3");

  pugi::xml_node interval_node = input.child("intervals");
  numGlobalElementsInX = Plato::Parse::getInt(interval_node, "interval");
  interval_node = interval_node.next_sibling("intervals");
  numGlobalElementsInY = Plato::Parse::getInt(interval_node, "interval");
  interval_node = interval_node.next_sibling("intervals");
  numGlobalElementsInZ = Plato::Parse::getInt(interval_node, "interval");

  string nodeLocations = Plato::Parse::getString(input, "node_locations");
  if( nodeLocations == "external" ){
    nodeLocationsExternal = true;
    xBegin = 0.0; xEnd   = 0.0;
    yBegin = 0.0; yEnd   = 0.0;
    zBegin = 0.0; zEnd   = 0.0;
  } else {
    pugi::xml_node cur_node = input.child("xlimits");
    xBegin = Plato::Parse::getDouble(cur_node, "xlimit");
    cur_node = cur_node.next_sibling("xlimits");
    xEnd = Plato::Parse::getDouble(cur_node, "xlimit");
    cur_node = input.child("ylimits");
    yBegin = Plato::Parse::getDouble(cur_node, "ylimit");
    cur_node = cur_node.next_sibling("ylimits");
    yEnd = Plato::Parse::getDouble(cur_node, "ylimit");
    cur_node = input.child("zlimits");
    zBegin = Plato::Parse::getDouble(cur_node, "zlimit");
    cur_node = cur_node.next_sibling("zlimits");
    zEnd = Plato::Parse::getDouble(cur_node, "zlimit");
  }

  globalNel = numGlobalElementsInX * numGlobalElementsInY * numGlobalElementsInZ;

  numGlobalNodesInX = numGlobalElementsInX + 1;
  numGlobalNodesInY = numGlobalElementsInY + 1;
  numGlobalNodesInZ = numGlobalElementsInZ + 1;

  globalNnp = numGlobalNodesInX * numGlobalNodesInY * numGlobalNodesInZ;

  WorldComm.cartCreate();
  WorldComm.getElementDomain( 0, numGlobalElementsInX, iBegin, iEnd );
  WorldComm.getElementDomain( 1, numGlobalElementsInY, jBegin, jEnd );
  WorldComm.getElementDomain( 2, numGlobalElementsInZ, kBegin, kEnd );
  
  numLocalElementsInX = iEnd - iBegin;
  numLocalElementsInY = jEnd - jBegin;
  numLocalElementsInZ = kEnd - kBegin;
  numElems = numLocalElementsInX * numLocalElementsInY * numLocalElementsInZ;
 
  numLocalNodesInX = numLocalElementsInX + 1;
  numLocalNodesInY = numLocalElementsInY + 1;
  numLocalNodesInZ = numLocalElementsInZ + 1;
  numNodes = numLocalNodesInX * numLocalNodesInY * numLocalNodesInZ;

  /****  CREATE SINGLE BLOCK **********************************************/
  // for now, StrMesh is single-block.  
  if( Plato::Parse::numChildren( input, "block" ) != 1 )
    throw ParsingException("Virtual mesh capability is single block. One (and only one) block spec required.");

  pugi::xml_node blockspec = input.child( "block" );
  pugi::xml_node integration = blockspec.child( "integration" );
  Topological::Hex8* eblock = new Topological::Hex8( numElems, integration );

  eblock->setDataContainer( myData );
  eblock->registerData();
  eblock->setBlockId(1);

  myElemBlk.push_back(eblock);
  /************************************************************************/


  registerData();

  // range of node indices
  int nKl = numLocalNodesInZ;
  int nJl = numLocalNodesInY;
  int nIl = numLocalNodesInX;

  int nKg = numGlobalNodesInZ;
  int nJg = numGlobalNodesInY;
  int nIg = numGlobalNodesInX;

  nodeGlobalIds = new int[numNodes];
  nodeOwnership = new int[numNodes];

  int localNodeIndex = 0;
  for( int inode=iBegin; inode<=iEnd; inode++)
    for( int jnode=jBegin; jnode<=jEnd; jnode++)
      for( int knode=kBegin; knode<=kEnd; knode++){
        nodeGlobalIds[localNodeIndex] = indexMap(inode, jnode, knode, nIg, nJg, nKg);
        nodeOwnership[localNodeIndex] = 1;
        localNodeIndex++;
      }

  if( (iEnd+1) != numGlobalNodesInX ){
    int inode = nIl-1;
    for( int jnode=0; jnode<nJl; jnode++)
      for( int knode=0; knode<nKl; knode++){
        localNodeIndex = indexMap(inode, jnode, knode, nIl, nJl, nKl);
        nodeOwnership[localNodeIndex] = 0;
      }
  }
  if( (jEnd+1) != numGlobalNodesInY ){
    int jnode = nJl-1;
    for( int inode=0; inode<nIl; inode++)
      for( int knode=0; knode<nKl; knode++){
        localNodeIndex = indexMap(inode, jnode, knode, nIl, nJl, nKl);
        nodeOwnership[localNodeIndex] = 0;
      }
  }
  if( (kEnd+1) != numGlobalNodesInZ ){
    int knode = nKl-1;
    for( int inode=0; inode<nIl; inode++)
      for( int jnode=0; jnode<nJl; jnode++){
        localNodeIndex = indexMap(inode, jnode, knode, nIl, nJl, nKl);
        nodeOwnership[localNodeIndex] = 0;
      }
  }


  // create a default set of nodesets:
  // 1,2:  -X,X face
  // 3,4:  -Y,Y face
  // 5,6:  -Z,Z face

  // negative X face
  if( (iBegin) == 0 ){
    registerNodeSet(1, nJl*nKl);
    int inode = 0, index = 0;
    int* nodes;
    myData->getVariable(nodeSets[0].NODE_LIST, nodes);
    for( int jnode=0; jnode<nJl; jnode++)
      for( int knode=0; knode<nKl; knode++)
        nodes[index++] = indexMap(inode, jnode, knode, nIl, nJl, nKl);
  } else registerNodeSet(1, 0);

  // positive X face
  if( (iEnd+1) == numGlobalNodesInX ){
    registerNodeSet(2, nJl*nKl);
    int inode = nIl-1, index = 0;
    int* nodes;
    myData->getVariable(nodeSets[1].NODE_LIST, nodes);
    for( int jnode=0; jnode<nJl; jnode++)
      for( int knode=0; knode<nKl; knode++)
        nodes[index++] = indexMap(inode, jnode, knode, nIl, nJl, nKl);
  } else registerNodeSet(2, 0);

  // negative Y face
  if( (jBegin) == 0 ){
    registerNodeSet(3, nIl*nKl);
    int jnode = 0, index = 0;
    int* nodes;
    myData->getVariable(nodeSets[2].NODE_LIST, nodes);
    for( int inode=0; inode<nIl; inode++)
      for( int knode=0; knode<nKl; knode++)
        nodes[index++] = indexMap(inode, jnode, knode, nIl, nJl, nKl);
  } else registerNodeSet(3, 0);

  // positive Y face
  if( (jEnd+1) == numGlobalNodesInY ){
    registerNodeSet(4, nIl*nKl);
    int jnode = nJl-1, index = 0;
    int* nodes;
    myData->getVariable(nodeSets[3].NODE_LIST, nodes);
    for( int inode=0; inode<nIl; inode++)
      for( int knode=0; knode<nKl; knode++)
        nodes[index++] = indexMap(inode, jnode, knode, nIl, nJl, nKl);
  } else registerNodeSet(4, 0);

  // negative Z face
  if( (kBegin) == 0 ){
    registerNodeSet(5, nIl*nJl);
    int knode = 0, index = 0;
    int* nodes;
    myData->getVariable(nodeSets[4].NODE_LIST, nodes);
    for( int inode=0; inode<nIl; inode++)
      for( int jnode=0; jnode<nJl; jnode++)
        nodes[index++] = indexMap(inode, jnode, knode, nIl, nJl, nKl);
  } else registerNodeSet(5, 0);

  // positive Z face
  if( (kEnd+1) == numGlobalNodesInZ ){
    registerNodeSet(6, nIl*nJl);
    int knode = nKl-1, index = 0;
    int* nodes;
    myData->getVariable(nodeSets[5].NODE_LIST, nodes);
    for( int inode=0; inode<nIl; inode++)
      for( int jnode=0; jnode<nJl; jnode++)
        nodes[index++] = indexMap(inode, jnode, knode, nIl, nJl, nKl);
  } else registerNodeSet(6, 0);



  // element limits and range
  int Kg = numGlobalElementsInZ;
  int Jg = numGlobalElementsInY;
  int Ig = numGlobalElementsInX;
  int Il = iEnd - iBegin;
  int Kl = kEnd - kBegin;
  int Jl = jEnd - jBegin;

  // HARDWIRED: single block
  Topological::Element* myblock = myElemBlk[0];
  int localConnectivity[8];

  for( int iel=0; iel<Il; iel++)
    for( int jel=0; jel<Jl; jel++)
      for( int kel=0; kel<Kl; kel++){
        int localElementIndex = indexMap(iel, jel, kel, Il, Jl, Kl);
        int globalElementIndex = indexMap(iel+iBegin, jel+jBegin, kel+kBegin, Ig, Jg, Kg);
        localConnectivity[0] = indexMap(iel,   jel,   kel,   nIl, nJl, nKl);
        localConnectivity[1] = indexMap(iel+1, jel,   kel,   nIl, nJl, nKl);
        localConnectivity[2] = indexMap(iel+1, jel+1, kel,   nIl, nJl, nKl);
        localConnectivity[3] = indexMap(iel,   jel+1, kel,   nIl, nJl, nKl);
        localConnectivity[4] = indexMap(iel,   jel,   kel+1, nIl, nJl, nKl);
        localConnectivity[5] = indexMap(iel+1, jel,   kel+1, nIl, nJl, nKl);
        localConnectivity[6] = indexMap(iel+1, jel+1, kel+1, nIl, nJl, nKl);
        localConnectivity[7] = indexMap(iel,   jel+1, kel+1, nIl, nJl, nKl);
        myblock->connectNodes(localElementIndex, globalElementIndex, localConnectivity);
      }

  double* X = getX();
  double* Y = getY();
  double* Z = getZ();

  if( !nodeLocationsExternal ){
  double rangeX = xEnd-xBegin;
  double rangeY = yEnd-yBegin;
  double rangeZ = zEnd-zBegin;

  // initialize node positions
  localNodeIndex = 0;
  for( int inode=iBegin; inode<=iEnd; inode++)
    for( int jnode=jBegin; jnode<=jEnd; jnode++)
      for( int knode=kBegin; knode<=kEnd; knode++){
        double xval = xBegin + inode*rangeX / Ig;
        double yval = yBegin + jnode*rangeY / Jg;
        double zval = zBegin + knode*rangeZ / Kg;
        X[localNodeIndex] = xval;
        Y[localNodeIndex] = yval;
        Z[localNodeIndex] = zval;
        localNodeIndex++;
      }
  }
  return true;
}

//*********************************************************************
int StrMesh::getNumElemBlks()
//*********************************************************************
{
  return 1;
}


//*********************************************************************
void StrMesh::zeroSet()
//*********************************************************************
{
  myMeshType = STR_DM;
  nodeLocationsExternal = false;
};


//*********************************************************************
int* StrMesh::getElemToNodeConnInBlk(int blk)
//*********************************************************************
{
  return NULL;
}


//*********************************************************************
int StrMesh::getBlockIndex(int blk_id)
//*********************************************************************
{
  return -1;
}


//*********************************************************************
int StrMesh::getBlockId(int blk)
//*********************************************************************
{
  return 1;
}

//*********************************************************************
bool StrMesh::readNodePlot(Real* data, string name)
//*********************************************************************
{
    return false;
}

//*********************************************************************
int StrMesh::getNumElemInBlk(int block)
//*********************************************************************
{
  return 1; // single block structured.
}


//*********************************************************************
const char* StrMesh::getElemTypeInBlk(int blk)
//*********************************************************************
{
  return NULL;
}

//*********************************************************************
bool UnsMesh::readNodePlot(Real* data, string name)
//*********************************************************************
{
    myMeshInput->readNodePlot(data, name);
    return true;
}

//*********************************************************************
bool
UnsMesh::parseMesh(pugi::xml_node& meshspec)
// This routine parses mesh file name and news associated IO object.
//*********************************************************************
{
#ifdef VERBOSE_DEBUG_LOCATION
  _print_entering_location(__AXSIS_FUNCTION_NAMER__);
#endif //VERBOSE_DEBUG_LOCATION

  // create new meshIO object for input
  string meshformat = Plato::Parse::getString(meshspec,"format");

  bool ignore_node_map = Plato::Parse::getBool(meshspec,"ignore_node_map");
  bool ignore_elem_map = Plato::Parse::getBool(meshspec,"ignore_element_map");

  if( meshformat == "exodus") {
    if( WorldComm.GetSize() == 1 ) {
      myMeshInput = new ExodusIO();
    } else {
      myMeshInput = new NemesisIO();
    }

    string filename = Plato::Parse::getString(meshspec,"mesh");
    ostringstream zeros;
    ostringstream buffer;
    if( WorldComm.GetSize() == 1 ) {
      buffer << filename;
      filename = buffer.str();
    } else {
      int my_pid = WorldComm.GetPID();
      int my_sze = WorldComm.GetSize();
      int div = 10;
      int quo = my_sze/div;
      int count = 0;
      while(quo>0) {
        count++;
        div*=10;
        quo = my_sze/div;
      }
      div = 10;
      quo = (my_pid/div);
      while(quo>0) {
        count--;
        div*=10;
        quo = my_pid/div;
      }
      for(int i=0;i<count;i++)
        zeros << '0';
      buffer << "./";
      buffer << filename;
      buffer << "." << my_sze << "." << zeros.str() << my_pid;
      filename = buffer.str();
    }
    myMeshInput->setName(filename.c_str());
    myMeshInput->setIgnoreNodeMap(ignore_node_map);
    myMeshInput->setIgnoreElemMap(ignore_elem_map);
    myMeshInput->setMode(MeshIO::READ);
  } else {
    stringstream msg;
    msg << "Fatal Error: Unknown unstructured mesh format <" << meshformat
        << "> requested...";
    throw ParsingException(msg.str());
  }

  myMeshInput->setData(myData);
  myMeshInput->setMesh(this);
  if(!myMeshInput->openMeshIO())
  {
    stringstream msg;
    msg << "Fatal Error: Couldn't open mesh file " << myMeshInput->getName();
    throw ParsingException(msg.str());
  }
  myMeshInput->readMeshIO();

  // element blocks are created by the meshInput object, but not initialized 
  // with the integration scheme.  So...
  int nblocks = myElemBlk.size();
  pugi::xml_node defaultInt = meshspec.child("integration");
  for(int ib=0; ib<nblocks; ib++){
    pugi::xml_node intg = defaultInt;
    pugi::xml_node blockspec = meshspec.child("block");
    if( Plato::Parse::numChildren( blockspec, "integration" ) != 0 )
      intg = blockspec.child("integration");
    myElemBlk[ib]->setIntegrationMethod( intg );
  }
  

  return true;
}

int  
DataMesh::getNnpeInBlk(int blk)
{
  int nnpe = myElemBlk[blk]->getNnpe();
  return nnpe;
}

int 
UnsMesh::getBlockIndex(int blk_id)
{
  int nblocks = myElemBlk.size();
  for(int iblock=0; iblock<nblocks; iblock++){
    if( myElemBlk[iblock]->getBlockId() == blk_id) return iblock;
  }
    
  pXcout << "Error: block with id " << blk_id << " not found." << endl;
  return -1;
}

int
UnsMesh::getBlockId(int blk)
{
  assert((size_t) blk < myElemBlk.size());
  int id = myElemBlk[blk]->getBlockId();
  return id;
}

int* 
UnsMesh::getElemToNodeConnInBlk(int blk)
{
  int* pnc = myElemBlk[blk]->getNodeConnect();
  return pnc; 
}

const char*
UnsMesh::getElemTypeInBlk(int blk)
{
  assert((size_t)blk < myElemBlk.size());
  return (myElemBlk[blk]->getType());
}

void
UnsMesh::zeroSet()
{
  myMeshType = UNS_DM;
};

#if 0
/******************************************************************************/
bool ParseBlocks(DataMesh* data_mesh, cfg_t* cfg) {
/******************************************************************************/

  cfg_t *meshspec = cfg_getsec(cfg,"mesh");

  int nblockspecs = cfg_size(meshspec,"block");

  int ntopoblocks = data_mesh->getNumElemBlks();

  if(nblockspecs != ntopoblocks){
    throw ParsingException(meshspec, "Fatal Error: # blocks != # block specs ");
  }

  return true;
  
}
#endif



