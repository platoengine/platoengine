#include "types.hpp"
#include "mesh_io.hpp"
#include "data_container.hpp"
#include "data_mesh.hpp"


// Base-class definitions
MeshIO::MeshIO() :
        myType(READ),
        myName("MeshIO"),
        myTitle("Base MeshIO"),
        myFileID(-1),
        myMesh(NULL),
        myData(NULL)
{
}

MeshIO::~MeshIO()
{
}

bool
MeshIO::initMeshIO( DataMesh* dm, 
                    DataContainer* dc,
                    const char* name,
                    MeshIO::Type type)
{ 
  myMesh = dm; myData = dc; myName = name; myType = type;
  return true;
}

