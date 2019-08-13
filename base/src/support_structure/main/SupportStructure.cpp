/*--------------------------------------------------------------------*/
/*    Copyright 2002 - 2008, 2010, 2011 National Technology &         */
/*    Engineering Solutions of Sandia, LLC (NTESS). Under the terms   */
/*    of Contract DE-NA0003525 with NTESS, there is a                 */
/*    non-exclusive license for use of this work by or on behalf      */
/*    of the U.S. Government.  Export of this program may require     */
/*    a license from the United States Government.                    */
/*--------------------------------------------------------------------*/
#include "SupportStructure.hpp"

#include <stk_io/StkMeshIoBroker.hpp>
#include <stk_io/IossBridge.hpp>
#include <stk_mesh/base/CreateFaces.hpp>
#include <stk_mesh/base/CreateEdges.hpp>
#include <stk_mesh/base/MetaData.hpp>
#include <stk_mesh/base/DestroyElements.hpp>
#include <stk_util/parallel/CommSparse.hpp>

#include <Ioss_SubSystem.h>

#define platoPI 3.141592653589793

namespace plato
{

namespace support_structure
{

SupportStructure::~SupportStructure()
{
    if(mSTKMeshIn)
        delete mSTKMeshIn;
    if(mSTKMeshOut)
        delete mSTKMeshOut;
}

bool SupportStructure::createMeshAPIsStandAlone(int argc, char **argv)
{
    // Set some defaults
    mMeshIn = "";
    mMeshOut = "";
    mDesignFieldName = "Topology";
    mSTKMeshIn = NULL;
    mSTKMeshOut = NULL;
    mDesignFieldThresholdValue = 0.5;
    mConcatenateResults = 0;
    mOverhangAngle = 45.0;
    mCellSizeMultiplier = 1.0;
    mReadSpreadFile = 0;
    mTimeStep = 1;
    mAlgorithm=0; // node based
    mBuildPlateNormal[0] = 0.0;
    mBuildPlateNormal[1] = 0.0;
    mBuildPlateNormal[2] = 1.1;
    mOutputFields = "";
    mNeighborSearchRadius=2;

    stk::ParallelMachine *comm = new stk::ParallelMachine(stk::parallel_machine_init(&argc, &argv));
    if(!comm)
    {
        std::cout << "Failed to initialize the parallel machine." << std::endl;
        return false;
    }
    mComm = comm;
    if ( !readCommandLine( argc, argv ) )
        return false;

    if(!initSingleMeshAPIs())
    {
        return false;
    }

    return true;
}

bool SupportStructure::initSingleMeshAPIs()
{

    mSTKMeshIn = new MeshWrapper(mComm);
    mSTKMeshIn->prepare_as_source();

    if(!mSTKMeshIn->read_exodus_mesh(mMeshIn, mDesignFieldName, mOutputFields, mReadSpreadFile, mTimeStep))
        return false;

    mSTKMeshOut = mSTKMeshIn;

    return true;
}


bool SupportStructure::run()
{
    bool return_val = true;
    try
    {
        if(mAlgorithm == 0)
            return_val = runPrivateNodeBased();
        else if(mAlgorithm == 1)
            return_val = runPrivateElementBased();
        else if(mAlgorithm == 2)
            return_val = runPrivateProjectTriangle();
        else if(mAlgorithm == 3)
            return_val = runPrivateNodeBasedMaxDensityAbove();
        else if(mAlgorithm == 4)
            return_val = runPrivateNodeBasedMaxDensityAboveTopDown();
        else
            return_val = runPrivateVoxelBasedInefficientMemory();
    }
    catch (std::exception exc)
    {
        delete mSTKMeshIn;
        mSTKMeshIn = mSTKMeshOut = NULL;
        stk::parallel_machine_finalize();
        return false;
    }

    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
    {
        if ( return_val )
            std::cout << "Exiting Successfully " << std::endl;
        else
            std::cout << "Exiting with Failure " << std::endl;
    }

    delete mSTKMeshIn;
    mSTKMeshIn = mSTKMeshOut = NULL;
    stk::parallel_machine_finalize();

    return return_val;
}

int SupportStructure::getIntersectionInfo(stk::mesh::Entity &element, 
                                          int aFieldType,
                                          Vector3D &aPlaneNormal,
                                          Vector3D &aPlaneOrigin,
                                          std::vector<Vector3D> &aTriPoints)
{
    // find interection points
    stk::mesh::Entity const *elemNodes = mSTKMeshIn->bulk_data()->begin_nodes(element);
    int numNodes = mSTKMeshIn->bulk_data()->num_nodes(element);

    if(numNodes == 8)
        return getIntersectionInfoHex(elemNodes, aFieldType, aPlaneNormal, aPlaneOrigin, aTriPoints);
    else if(numNodes == 4)
        return getIntersectionInfoTet(elemNodes, aFieldType, aPlaneNormal, aPlaneOrigin, aTriPoints);

    return 0;
}

int SupportStructure::getIntersectionInfoTet(stk::mesh::Entity const *aElementNodes,
                                             int aFieldType,
                                             Vector3D &aPlaneNormal,
                                             Vector3D &aPlaneOrigin,
                                             std::vector<Vector3D> &aTriPoints)
{
    int planeNormalSet = 0;
    double valueTol = 1e-12;
    Vector3D materialToVoidDirection;
    Vector3D averageMaterial;
    Vector3D averageVoid;
    double fieldValues[4];
    Vector3D nodeCoords[4];
    int numMaterialNodes = 0;
    int numVoidNodes = 0;
    double v0, v1, v2, v3;
    double lowerBound = .5 - valueTol;
    double upperBound = .5 + valueTol;
    std::vector<Vector3D> nodesUsedInTris;

    for(int i=0; i<3; ++i)
    {
        averageMaterial[i] = 0.0;
        averageVoid[i] = 0.0;
    }
    // Get field values for nodes in this element.
    // Get rough direction from material to void in this element
    for(int i=0; i<4; ++i)
    {
        fieldValues[i] = mSTKMeshIn->getFieldValue(aElementNodes[i], aFieldType);
        mSTKMeshIn->nodeCoordinates(aElementNodes[i], nodeCoords[i].data());
        if((fieldValues[i] + valueTol) >= 0.5)
        {
            numMaterialNodes++;
            averageMaterial += nodeCoords[i];
        }
        else
        {
            numVoidNodes++;
            averageVoid += nodeCoords[i];
        }
    }
    if(numMaterialNodes)
    {
        averageMaterial /= ((double)numMaterialNodes);
    }
    if(numVoidNodes)
    {
        averageVoid /= ((double)numVoidNodes);
    }
    if(numMaterialNodes != 0 && numVoidNodes != 0)
    {
        materialToVoidDirection = averageVoid - averageMaterial;
        materialToVoidDirection.normalize();

        v0 = fieldValues[0]; v1 = fieldValues[1]; v2 = fieldValues[2]; v3 = fieldValues[3];

        // Look for nodes with iso surface going through them.
        if(v0 <= upperBound && v0 >= lowerBound)
        {
            nodesUsedInTris.push_back(nodeCoords[0]);
        }
        if(v1 <= upperBound && v1 >= lowerBound)
        {
            nodesUsedInTris.push_back(nodeCoords[1]);
        }
        if(v2 <= upperBound && v2 >= lowerBound)
        {
            nodesUsedInTris.push_back(nodeCoords[2]);
        }
        if(v3 <= upperBound && v3 >= lowerBound)
        {
            nodesUsedInTris.push_back(nodeCoords[3]);
        }

        // edge 01
        if((v0 > upperBound && v1 < lowerBound) || (v0 < lowerBound && v1 > upperBound))
        {
            double t = (.5 - fieldValues[0]) / (fieldValues[1]-fieldValues[0]);
            nodesUsedInTris.push_back(nodeCoords[0] + t*(nodeCoords[1]-nodeCoords[0]));
        }
        // edge 20
        if((v0 > upperBound && v2 < lowerBound) || (v0 < lowerBound && v2 > upperBound))
        {
            double t = (.5 - fieldValues[0]) / (fieldValues[2]-fieldValues[0]);
            nodesUsedInTris.push_back(nodeCoords[0] + t*(nodeCoords[2]-nodeCoords[0]));
        }
        // edge 30
        if((v0 > upperBound && v3 < lowerBound) || (v0 < lowerBound && v3 > upperBound))
        {
            double t = (.5 - fieldValues[0]) / (fieldValues[3]-fieldValues[0]);
            nodesUsedInTris.push_back(nodeCoords[0] + t*(nodeCoords[3]-nodeCoords[0]));
        }
        // edge 12
        if((v1 > upperBound && v2 < lowerBound) || (v1 < lowerBound && v2 > upperBound))
        {
            double t = (.5 - fieldValues[1]) / (fieldValues[2]-fieldValues[1]);
            nodesUsedInTris.push_back(nodeCoords[1] + t*(nodeCoords[2]-nodeCoords[1]));
        }
        // edge 31
        if((v1 > upperBound && v3 < lowerBound) || (v1 < lowerBound && v3 > upperBound))
        {
            double t = (.5 - fieldValues[1]) / (fieldValues[3]-fieldValues[1]);
            nodesUsedInTris.push_back(nodeCoords[1] + t*(nodeCoords[3]-nodeCoords[1]));
        }
        // edge 32
        if((v2 > upperBound && v3 < lowerBound) || (v2 < lowerBound && v3 > upperBound))
        {
            double t = (.5 - fieldValues[2]) / (fieldValues[3]-fieldValues[2]);
            nodesUsedInTris.push_back(nodeCoords[2] + t*(nodeCoords[3]-nodeCoords[2]));
        }

        double toleranceSquared = 1e-12;

        for(int s=0; s<3; ++s)
            aPlaneOrigin[s] = 0.0;

        double numTrianglePoints = 0.0;
        if(nodesUsedInTris.size() == 3)
        {
            Vector3D n1 = nodesUsedInTris[0];
            Vector3D n2 = nodesUsedInTris[1];
            Vector3D n3 = nodesUsedInTris[2];
            double distanceSquared1 = n1.distanceSquared(n2);
            double distanceSquared2 = n1.distanceSquared(n3);
            double distanceSquared3 = n2.distanceSquared(n3);
            if((distanceSquared1 > toleranceSquared) &&
                    (distanceSquared2 > toleranceSquared) &&
                    (distanceSquared3 > toleranceSquared))
            {
                numTrianglePoints += 3.0;

                aTriPoints.push_back(n1);
                aTriPoints.push_back(n2);
                aTriPoints.push_back(n3);

                // Calculate the plane normal;
                aPlaneOrigin += n1;
                aPlaneOrigin += n2;
                aPlaneOrigin += n3;

                Vector3D triNormal;
                Vector3D vec1 = n2 - n1;
                Vector3D vec2 = n3 - n1;
                Vector3D cross = vec1 * vec2;
                cross.normalize();
                double dot = cross % materialToVoidDirection;
                if(dot < 0.0)
                    cross *= -1.0;
                aPlaneNormal += cross;
                planeNormalSet = 1;
            }
        }
        else if(nodesUsedInTris.size() == 4)
        {
            Vector3D n1 = nodesUsedInTris[0];
            Vector3D n2 = nodesUsedInTris[1];
            Vector3D n3 = nodesUsedInTris[2];
            double distanceSquared1 = n1.distanceSquared(n2);
            double distanceSquared2 = n1.distanceSquared(n3);
            double distanceSquared3 = n2.distanceSquared(n3);
            if((distanceSquared1 > toleranceSquared) &&
                    (distanceSquared2 > toleranceSquared) &&
                    (distanceSquared3 > toleranceSquared))
            {
                numTrianglePoints += 3.0;

                aTriPoints.push_back(n1);
                aTriPoints.push_back(n2);
                aTriPoints.push_back(n3);

                // Calculate the plane normal;
                aPlaneOrigin += n1;
                aPlaneOrigin += n2;
                aPlaneOrigin += n3;

                Vector3D triNormal;
                Vector3D vec1 = n2 - n1;
                Vector3D vec2 = n3 - n1;
                Vector3D cross = vec1 * vec2;
                cross.normalize();
                double dot = cross % materialToVoidDirection;
                if(dot < 0.0)
                    cross *= -1.0;
                aPlaneNormal += cross;
                planeNormalSet = 1;
            }
            Vector3D vec1 = nodesUsedInTris[0] - nodesUsedInTris[3];
            Vector3D vec2 = nodesUsedInTris[1] - nodesUsedInTris[3];
            Vector3D vec3 = nodesUsedInTris[2] - nodesUsedInTris[3];
            vec1.normalize();
            vec2.normalize();
            vec3.normalize();
            double dot1 = vec1 % vec2;
            double dot2 = vec1 % vec3;
            double dot3 = vec2 % vec3;
            int index1 = -1;
            int index2 = -1;
            int index3 = -1;
            if(dot1 < dot2 && dot1 < dot3)
            {
                index1 = 0;
                index2 = 1;
                index3 = 3;
            }
            else if(dot2 < dot1 && dot2 < dot3)
            {
                index1 = 0;
                index2 = 2;
                index3 = 3;
            }
            else if(dot3 < dot1 && dot3 < dot2)
            {
                index1 = 1;
                index2 = 2;
                index3 = 3;
            }
            if(index1 != -1)
            {
                double distanceSquared1 = nodesUsedInTris[index1].distanceSquared(nodesUsedInTris[index2]);
                double distanceSquared2 = nodesUsedInTris[index1].distanceSquared(nodesUsedInTris[index3]);
                double distanceSquared3 = nodesUsedInTris[index2].distanceSquared(nodesUsedInTris[index3]);
                if((distanceSquared1 > toleranceSquared) &&
                        (distanceSquared2 > toleranceSquared) &&
                        (distanceSquared3 > toleranceSquared))
                {
                    numTrianglePoints += 3.0;

                    aTriPoints.push_back(nodesUsedInTris[index1]);
                    aTriPoints.push_back(nodesUsedInTris[index2]);
                    aTriPoints.push_back(nodesUsedInTris[index3]);

                    // Calculate the plane normal;
                    aPlaneOrigin += nodesUsedInTris[index1];
                    aPlaneOrigin += nodesUsedInTris[index2];
                    aPlaneOrigin += nodesUsedInTris[index3];

                    Vector3D triNormal;
                    Vector3D vec1 = nodesUsedInTris[index2] - nodesUsedInTris[index1];
                    Vector3D vec2 = nodesUsedInTris[index3] - nodesUsedInTris[index1];
                    Vector3D cross = vec1 * vec2;
                    cross.normalize();
                    double dot = cross % materialToVoidDirection;
                    if(dot < 0.0)
                        cross *= -1.0;
                    aPlaneNormal += cross;
                    planeNormalSet = 1;
                }
            }
        }

        if(numTrianglePoints > 1.0)
            aPlaneOrigin /= numTrianglePoints;

        if(planeNormalSet == 1)
        {
            aPlaneNormal.normalize();
        }
    }

    return planeNormalSet;
}

int SupportStructure::getIntersectionInfoHex(stk::mesh::Entity const *aElementNodes,
                                             int aFieldType,
                                             Vector3D &aPlaneNormal,
                                             Vector3D &aPlaneOrigin,
                                             std::vector<Vector3D> &aTriPoints)
{
    int planeNormalSet = 0;
    int cubeindex;
    Vector3D intersectionPoints[12];
    double fieldValues[8];
    Vector3D nodeCoords[8];
    Vector3D materialToVoidDirection;
    Vector3D averageMaterial;
    Vector3D averageVoid;
    int numMaterialNodes = 0;
    int numVoidNodes = 0;
    for(int i=0; i<3; ++i)
    {
        averageMaterial[i] = 0.0;
        averageVoid[i] = 0.0;
    }
    // Get field values for nodes in this element.
    // Get rough direction from material to void in this element
    for(int i=0; i<8; ++i)
    {
        fieldValues[i] = mSTKMeshIn->getFieldValue(aElementNodes[i], aFieldType);
        mSTKMeshIn->nodeCoordinates(aElementNodes[i], nodeCoords[i].data());
        if((fieldValues[i] + 1e-6) >= 0.5)
        {
            numMaterialNodes++;
            averageMaterial += nodeCoords[i];
        }
        else
        {
            numVoidNodes++;
            averageVoid += nodeCoords[i];
        }
    }
    if(numMaterialNodes)
    {
        averageMaterial /= ((double)numMaterialNodes);
    }
    if(numVoidNodes)
    {
        averageVoid /= ((double)numVoidNodes);
    }
    if(numMaterialNodes != 0 && numVoidNodes != 0)
    {
        materialToVoidDirection = averageVoid - averageMaterial;
        materialToVoidDirection.normalize();

        int edgeTable[256]={
                            0x0  , 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
                            0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
                            0x190, 0x99 , 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
                            0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
                            0x230, 0x339, 0x33 , 0x13a, 0x636, 0x73f, 0x435, 0x53c,
                            0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
                            0x3a0, 0x2a9, 0x1a3, 0xaa , 0x7a6, 0x6af, 0x5a5, 0x4ac,
                            0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
                            0x460, 0x569, 0x663, 0x76a, 0x66 , 0x16f, 0x265, 0x36c,
                            0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
                            0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff , 0x3f5, 0x2fc,
                            0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
                            0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55 , 0x15c,
                            0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
                            0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc ,
                            0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
                            0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
                            0xcc , 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
                            0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
                            0x15c, 0x55 , 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
                            0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
                            0x2fc, 0x3f5, 0xff , 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
                            0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
                            0x36c, 0x265, 0x16f, 0x66 , 0x76a, 0x663, 0x569, 0x460,
                            0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
                            0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa , 0x1a3, 0x2a9, 0x3a0,
                            0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
                            0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33 , 0x339, 0x230,
                            0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
                            0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99 , 0x190,
                            0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
                            0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0   };
        int triTable[256][16] =
        {{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
         {3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
         {3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
         {3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
         {9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
         {1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
         {9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
         {2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
         {8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
         {9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
         {4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
         {3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
         {1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
         {4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
         {4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
         {9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
         {1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
         {5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
         {2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
         {9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
         {0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
         {2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
         {10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
         {4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
         {5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
         {5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
         {9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
         {0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
         {1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
         {10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
         {8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
         {2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
         {7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
         {9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
         {2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
         {11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
         {9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
         {5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
         {11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
         {11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
         {1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
         {9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
         {5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
         {2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
         {0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
         {5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
         {6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
         {0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
         {3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
         {6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
         {5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
         {1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
         {10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
         {6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
         {1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
         {8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
         {7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
         {3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
         {5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
         {0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
         {9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
         {8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
         {5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
         {0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
         {6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
         {10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
         {10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
         {8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
         {1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
         {3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
         {0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
         {10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
         {0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
         {3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
         {6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
         {9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
         {8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
         {3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
         {6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
         {0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
         {10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
         {10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
         {1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
         {2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
         {7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
         {7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
         {2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
         {1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
         {11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
         {8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
         {0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
         {7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
         {10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
         {2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
         {6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
         {7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
         {2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
         {1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
         {10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
         {10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
         {0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
         {7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
         {6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
         {8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
         {9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
         {6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
         {1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
         {4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
         {10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
         {8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
         {0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
         {1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
         {8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
         {10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
         {4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
         {10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
         {5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
         {11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
         {9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
         {6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
         {7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
         {3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
         {7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
         {9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
         {3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
         {6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
         {9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
         {1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
         {4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
         {7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
         {6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
         {3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
         {0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
         {6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
         {1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
         {0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
         {11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
         {6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
         {5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
         {9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
         {1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
         {1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
         {10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
         {0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
         {5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
         {10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
         {11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
         {0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
         {9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
         {7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
         {2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
         {8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
         {9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
         {9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
         {1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
         {9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
         {9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
         {5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
         {0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
         {10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
         {2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
         {0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
         {0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
         {9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
         {5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
         {3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
         {5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
         {8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
         {0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
         {9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
         {0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
         {1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
         {3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
         {4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
         {9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
         {11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
         {11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
         {2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
         {9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
         {3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
         {1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
         {4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
         {4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
         {0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
         {3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
         {3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
         {0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
         {9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
         {1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}};

        /*
   Determine the index into the edge table which
   tells us which vertices are inside of the surface
         */

        cubeindex = 0;
        double isolevel = 0.5;
        if (fieldValues[0] < isolevel) cubeindex |= 1;
        if (fieldValues[1] < isolevel) cubeindex |= 2;
        if (fieldValues[2] < isolevel) cubeindex |= 4;
        if (fieldValues[3] < isolevel) cubeindex |= 8;
        if (fieldValues[4] < isolevel) cubeindex |= 16;
        if (fieldValues[5] < isolevel) cubeindex |= 32;
        if (fieldValues[6] < isolevel) cubeindex |= 64;
        if (fieldValues[7] < isolevel) cubeindex |= 128;

        /* Cube is entirely in/out of the surface */
        if (edgeTable[cubeindex] == 0)
            return 0;

        /* Find the vertices where the surface intersects the cube */
        if (edgeTable[cubeindex] & 1)
        {
            double t = (.5 - fieldValues[0]) / (fieldValues[1]-fieldValues[0]);
            intersectionPoints[0] = nodeCoords[0] + t*(nodeCoords[1]-nodeCoords[0]);
        }
        if (edgeTable[cubeindex] & 2)
        {
            double t = (.5 - fieldValues[1]) / (fieldValues[2]-fieldValues[1]);
            intersectionPoints[1] = nodeCoords[1] + t*(nodeCoords[2]-nodeCoords[1]);
        }
        if (edgeTable[cubeindex] & 4)
        {
            double t = (.5 - fieldValues[2]) / (fieldValues[3]-fieldValues[2]);
            intersectionPoints[2] = nodeCoords[2] + t*(nodeCoords[3]-nodeCoords[2]);
        }
        if (edgeTable[cubeindex] & 8)
        {
            double t = (.5 - fieldValues[0]) / (fieldValues[3]-fieldValues[0]);
            intersectionPoints[3] = nodeCoords[0] + t*(nodeCoords[3]-nodeCoords[0]);
        }
        if (edgeTable[cubeindex] & 16)
        {
            double t = (.5 - fieldValues[4]) / (fieldValues[5]-fieldValues[4]);
            intersectionPoints[4] = nodeCoords[4] + t*(nodeCoords[5]-nodeCoords[4]);
        }
        if (edgeTable[cubeindex] & 32)
        {
            double t = (.5 - fieldValues[6]) / (fieldValues[5]-fieldValues[6]);
            intersectionPoints[5] = nodeCoords[6] + t*(nodeCoords[5]-nodeCoords[6]);
        }
        if (edgeTable[cubeindex] & 64)
        {
            double t = (.5 - fieldValues[6]) / (fieldValues[7]-fieldValues[6]);
            intersectionPoints[6] = nodeCoords[6] + t*(nodeCoords[7]-nodeCoords[6]);
        }
        if (edgeTable[cubeindex] & 128)
        {
            double t = (.5 - fieldValues[4]) / (fieldValues[7]-fieldValues[4]);
            intersectionPoints[7] = nodeCoords[4] + t*(nodeCoords[7]-nodeCoords[4]);
        }
        if (edgeTable[cubeindex] & 256)
        {
            double t = (.5 - fieldValues[4]) / (fieldValues[0]-fieldValues[4]);
            intersectionPoints[8] = nodeCoords[4] + t*(nodeCoords[0]-nodeCoords[4]);
        }
        if (edgeTable[cubeindex] & 512)
        {
            double t = (.5 - fieldValues[5]) / (fieldValues[1]-fieldValues[5]);
            intersectionPoints[9] = nodeCoords[5] + t*(nodeCoords[1]-nodeCoords[5]);
        }
        if (edgeTable[cubeindex] & 1024)
        {
            double t = (.5 - fieldValues[6]) / (fieldValues[2]-fieldValues[6]);
            intersectionPoints[10] = nodeCoords[6] + t*(nodeCoords[2]-nodeCoords[6]);
        }
        if (edgeTable[cubeindex] & 2048)
        {
            double t = (.5 - fieldValues[7]) / (fieldValues[3]-fieldValues[7]);
            intersectionPoints[11] = nodeCoords[7] + t*(nodeCoords[3]-nodeCoords[7]);
        }

        double toleranceSquared = 1e-12;

        for(int s=0; s<3; ++s)
            aPlaneOrigin[s] = 0.0;

        double numTrianglePoints = 0.0;
        for (int i=0; triTable[cubeindex][i]!=-1; i+=3)
        {
            Vector3D n1 = intersectionPoints[triTable[cubeindex][i  ]];
            Vector3D n2 = intersectionPoints[triTable[cubeindex][i+1]];
            Vector3D n3 = intersectionPoints[triTable[cubeindex][i+2]];
            double distanceSquared1 = n1.distanceSquared(n2);
            double distanceSquared2 = n1.distanceSquared(n3);
            double distanceSquared3 = n2.distanceSquared(n3);
            if((distanceSquared1 > toleranceSquared) &&
                    (distanceSquared2 > toleranceSquared) &&
                    (distanceSquared3 > toleranceSquared))
            {
                numTrianglePoints += 3.0;

                aTriPoints.push_back(n1);
                aTriPoints.push_back(n2);
                aTriPoints.push_back(n3);

                // Calculate the plane normal;
                aPlaneOrigin += n1;
                aPlaneOrigin += n2;
                aPlaneOrigin += n3;

                Vector3D triNormal;
                Vector3D vec1 = n2 - n1;
                Vector3D vec2 = n3 - n1;
                Vector3D cross = vec1 * vec2;
                cross.normalize();
                double dot = cross % materialToVoidDirection;
                if(dot < 0.0)
                    cross *= -1.0;
                aPlaneNormal += cross;
                planeNormalSet = 1;
            }
        }

        if(numTrianglePoints > 1.0)
            aPlaneOrigin /= numTrianglePoints;

        if(planeNormalSet == 1)
        {
            aPlaneNormal.normalize();
        }
    }
    return planeNormalSet;
}

void SupportStructure::projectPointToPlane(Vector3D &point, Vector3D &planeOrigin,
                                           Vector3D &planeNormal, Vector3D &projectedPoint)
{
    Vector3D vec;
    vec = point - planeOrigin;
    double dist = vec % planeNormal;
    projectedPoint = point - dist*planeNormal;
}

bool SupportStructure::pointInTriangle(Vector3D &p,
                                       Vector3D &t1,
                                       Vector3D &t2,
                                       Vector3D &t3)
{
    // Compute vectors
    double tolerance = 1e-12;
    Vector3D v0, v1, v2;
    v0 = t2-t1;
    v1 = t3-t1;
    v2 = p-t1;
    // Compute dot products
    double dot00 = v0 % v0;
    double dot01 = v0 % v1;
    double dot02 = v0 % v2;
    double dot11 = v1 % v1;
    double dot12 = v1 % v2;
    // Compute barycentric coordinates
    double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    double v = (dot00 * dot12 - dot01 * dot02) * invDenom;
    // Check if point is in triangle
    return ((u + tolerance >= 0.0) && (v + tolerance >= 0.0) && ((u + v - tolerance) <= 1.0));
}

bool SupportStructure::runPrivateElementBased()
{
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Generating support material field. " << std::endl;

    stk::mesh::Selector mySelector = mSTKMeshIn->meta_data()->universal_part();
    stk::mesh::BucketVector const &elementBuckets = mSTKMeshIn->bulk_data()->get_buckets(
            stk::topology::ELEMENT_RANK, mySelector );

    if(elementBuckets.size() == 0)
    {
        std::cout << "Failed to find any elements." << std::endl;
        return false;
    }

    // Get the build plate normal and normalize it
    std::string workingString = mBuildPlateNormalString; // make a copy since we will be modifying it
    size_t spacePos = workingString.find(' ');
    int cntr = 0;
    while(spacePos != std::string::npos)
    {
        std::string cur_string = workingString.substr(0,spacePos);
        workingString = workingString.substr(spacePos+1);
        mBuildPlateNormal[cntr] = std::atof(cur_string.c_str());
        cntr++;
        spacePos = workingString.find(' ');
    }
    mBuildPlateNormal[cntr] = std::atof(workingString.c_str());
    mBuildPlateNormal.normalize();

    // Initialize support structure field nodes to (1-density) for
    // nodes that are in the design.
    // Initialize all other support structure nodes to the density
    // value at that node.
    double tolerance = 1e-12;
    std::set<stk::mesh::Entity> staticElemsToCheck, newElemsToCheck;
    std::vector<stk::mesh::Entity> elemsToCheck;
    for ( stk::mesh::BucketVector::const_iterator bucketIter = elementBuckets.begin();
            bucketIter != elementBuckets.end();
            ++bucketIter )
    {
        stk::mesh::Bucket &tmpBucket = **bucketIter;
        size_t numElems = tmpBucket.size();
        for (size_t i=0; i<numElems; ++i)
        {
            bool allNodesInside = true;
            bool allNodesOutside = true;
            stk::mesh::Entity curElem = tmpBucket[i];
            stk::mesh::Entity const *elemNodes = mSTKMeshIn->bulk_data()->begin_nodes(curElem);
            int numNodes = mSTKMeshIn->bulk_data()->num_nodes(curElem);
            for(int j=0; j<numNodes; ++j)
            {
                stk::mesh::Entity curNode = elemNodes[j];
                double designVariableValue = mSTKMeshIn->getMaxNodalIsoFieldVariable(curNode.m_value);
                if((designVariableValue + tolerance) >= mDesignFieldThresholdValue)
                {
                    mSTKMeshIn->setSupportStructureFieldValue(curNode.m_value, (1.0-designVariableValue));
                    allNodesOutside = false;
                }
                else
                {
                    mSTKMeshIn->setSupportStructureFieldValue(curNode.m_value, designVariableValue);
                    allNodesInside = false;
                }
            }
            if(!allNodesInside && !allNodesOutside)
            {
                // This is a vector that we will pop elements off of.
                elemsToCheck.push_back(curElem);
                // This is a set that we will use for searching.
                staticElemsToCheck.insert(curElem);
            }
        }
    }
    bool done = false;
    int iterationCount = 1;
    std::vector<stk::mesh::Entity> elemsToFlip;
    while(!done)
    {
        if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        {
            std::cout << "Iteration " << iterationCount << ":" << std::endl;
        }
        newElemsToCheck.clear();
        while(elemsToCheck.size() > 0)
        {
            stk::mesh::Entity curElem = elemsToCheck.back();
            elemsToCheck.pop_back();
            // Pass #1
            // Find all elements on interface of design domain that need support
            // structure.  Do this by calculating the normal of the interface
            // in that element and compare it to the build direction.  Add all these
            // elements to a list which will later have the support structure values
            // flipped from density to (1-density). For each of these elements find
            // neighbor elements in the direction of the build plate that should
            // be checked on the next pass (maybe start with all elements attached to
            // the nodes being flipped that are more or less in the direction of the
            // build plate but were not in the last list of elements to check).
            // Now flip all node values on the design interface in the list of elements
            // found above and also flip all node values on the interface that were on the
            // design side of the interface.
            if(iterationCount == 1)
            {
                // See if element has interace
                // if interface is pointing "down"
                //     	Set all support value for all nodes in this element to 1-density
                //     	Add all elements (except those already processed--in design, 
                //      on interface but not pointing down) hooked to nodes that are 
                //      "below" the interface to the 
                //     	list of next elements to process.	

                Vector3D interfaceNormal;
                Vector3D interfaceOrigin;
                std::vector<Vector3D> triPoints;
                uint64_t globalElemId = mSTKMeshIn->bulk_data()->identifier(curElem);
                if(globalElemId == 14714)
                {
                    int y=0;
                    ++y;
                }
                if(getIntersectionInfo(curElem, FIELD_DENSITY, interfaceNormal, interfaceOrigin, triPoints))
                {
                    // See if the interface is pointing toward the build plate
                    double dot=interfaceNormal % mBuildPlateNormal;
                    if(dot < -.707)
                    {
                //        elemsToFlip.push_back(curElem);
                        // project all of the interface triangle points to a plane parallel to build plate
                        for(size_t r=0; r<triPoints.size(); ++r)
                            projectPointToPlane(triPoints[r], interfaceOrigin, mBuildPlateNormal, triPoints[r]);

                        stk::mesh::Entity const *elemNodes = mSTKMeshIn->bulk_data()->begin_nodes(curElem);
                        int numNodes = mSTKMeshIn->bulk_data()->num_nodes(curElem);
                        for(int j=0; j<numNodes; ++j)
                        {
                            stk::mesh::Entity curNode = elemNodes[j];
                            double designVariableValue = mSTKMeshIn->getMaxNodalIsoFieldVariable(curNode.m_value);
                            // Set all node values to 1-density to maintain the same interface
                            // shape in this element.
   //                         mSTKMeshIn->setSupportStructureFieldValue(curNode.m_value, (1.0-designVariableValue));
                            // If this node was not in the original domain see if it is 
                            // "below" the interface and whether we should add its attached
                            // elements to the list of ones to process next
                            if((designVariableValue + tolerance) < mDesignFieldThresholdValue)
                            {
                                // see if node is "below" the interface 
                                // project the current node to a plane parallel to build plate
                                Vector3D planeOriginToPoint;
                                Vector3D pointCoords;
                                mSTKMeshIn->nodeCoordinates(curNode, pointCoords.data());
                                Vector3D pointOnPlane;
                                projectPointToPlane(pointCoords, interfaceOrigin, mBuildPlateNormal, pointOnPlane);

                                if(fabs(pointCoords[0]-.1) < 1e-12 &&
                                   fabs(pointCoords[1]+.6) < 1e-12 &&
                                   fabs(pointCoords[2]-.1) < 1e-12)
                                {
                                    int g= 0;
                                    g++;
                                }

                                bool pointIsBelowInterface = false;
                                // see if the current node is in any of the triangles
                                for(size_t w=0; !pointIsBelowInterface && w<triPoints.size(); w+=3)
                                {
                                    if(pointInTriangle(pointOnPlane, triPoints[w], triPoints[w+1], triPoints[w+2]))
                                        pointIsBelowInterface = true;
                                }

                                if(pointIsBelowInterface)
                                {
                                    mSTKMeshIn->setSupportStructureFieldValue(curNode.m_value, (1.0-designVariableValue));
                                    stk::mesh::Entity const *nodeElements = mSTKMeshIn->bulk_data()->begin_elements(curNode);
                                    int numElems = mSTKMeshIn->bulk_data()->num_elements(curNode);
                                    for(int i=0; i<numElems; ++i)
                                    {
                                        stk::mesh::Entity curNeighborElem = nodeElements[i];
                                     //   if(staticElemsToCheck.find(curNeighborElem) == staticElemsToCheck.end())
                                        {
                                            newElemsToCheck.insert(curNeighborElem);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            else //if (0)
            {
                // See if element has interace
                // if interface is pointing "down"
                //     	Set nodes "below" the interface to 1-density
                //     	Add all elements (except those already processed--in design, 
                //      on interface but not pointing down) hooked to nodes that are 
                //      "below" the interface to the 
                //     	list of next elements to process.	
                Vector3D interfaceNormal;
                Vector3D interfaceOrigin;
                std::vector<Vector3D> triPoints;
                uint64_t globalElemId = mSTKMeshIn->bulk_data()->identifier(curElem);
                if(globalElemId == 14684)
                {
                    int y=0;
                    ++y;
                }
                if(getIntersectionInfo(curElem, FIELD_SUPPORT, interfaceNormal, interfaceOrigin, triPoints))
                {
                    // See if the interface is pointing toward the build plate
                    double dot=0.0;
                    for(int i=0; i<3; ++i)
                        dot += interfaceNormal[i]*mBuildPlateNormal[i];
                    if(dot < -.707)
                    {
                        // project all of the interface triangle points to a plane parallel to build plate
                        for(size_t r=0; r<triPoints.size(); ++r)
                            projectPointToPlane(triPoints[r], interfaceOrigin, mBuildPlateNormal, triPoints[r]);

                        stk::mesh::Entity const *elemNodes = mSTKMeshIn->bulk_data()->begin_nodes(curElem);
                        int numNodes = mSTKMeshIn->bulk_data()->num_nodes(curElem);
                        for(int j=0; j<numNodes; ++j)
                        {
                            stk::mesh::Entity curNode = elemNodes[j];
                            double designVariableValue = mSTKMeshIn->getMaxNodalIsoFieldVariable(curNode.m_value);
                            // Set all node values to 1-density to maintain the same interface
                            // shape in this element.
                     //       mSTKMeshIn->setSupportStructureFieldValue(curNode.m_value, (1.0-designVariableValue));
                            // If this node was not in the original domain see if it is
                            // "below" the interface and whether we should add its attached
                            // elements to the list of ones to process next
                            if((designVariableValue + tolerance) < mDesignFieldThresholdValue)
                            {
                                // see if node is "below" the interface
                                // project the current node to a plane parallel to build plate
                                Vector3D planeOriginToPoint;
                                Vector3D pointCoords;
                                mSTKMeshIn->nodeCoordinates(curNode, pointCoords.data());
                                Vector3D pointOnPlane;
                                projectPointToPlane(pointCoords, interfaceOrigin, mBuildPlateNormal, pointOnPlane);

                                if(fabs(pointCoords[0]-.1) < 1e-12 &&
                                   fabs(pointCoords[1]+.6) < 1e-12 &&
                                   fabs(pointCoords[2]-.1) < 1e-12)
                                {
                                    int g= 0;
                                    g++;
                                }


                                bool pointIsBelowInterface = false;
                                // see if the current node is in any of the triangles
                                for(size_t w=0; !pointIsBelowInterface && w<triPoints.size(); w+=3)
                                {
                                    if(pointInTriangle(pointOnPlane, triPoints[w], triPoints[w+1], triPoints[w+2]))
                                        pointIsBelowInterface = true;
                                }

                                if(pointIsBelowInterface)
                                {
                                    mSTKMeshIn->setSupportStructureFieldValue(curNode.m_value, (1.0-designVariableValue));
                                    stk::mesh::Entity const *nodeElements = mSTKMeshIn->bulk_data()->begin_elements(curNode);
                                    int numElems = mSTKMeshIn->bulk_data()->num_elements(curNode);
                                    for(int i=0; i<numElems; ++i)
                                    {
                                        stk::mesh::Entity curNeighborElem = nodeElements[i];
                                     //   if(staticElemsToCheck.find(curNeighborElem) == staticElemsToCheck.end())
                                        {
                                            newElemsToCheck.insert(curNeighborElem);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        if(newElemsToCheck.size() == 0 || iterationCount > 500)
            done = true;
        else
        {
            std::set<stk::mesh::Entity>::iterator iter = newElemsToCheck.begin();
            staticElemsToCheck.clear();
            while(iter != newElemsToCheck.end())
            {
                elemsToCheck.push_back(*iter);
                staticElemsToCheck.insert(*iter);
                iter++;
            }
        }

        iterationCount++;
    }

    mSTKMeshIn->write_exodus_mesh(mMeshOut, mConcatenateResults);

    return true;
}

bool comparePairs(const std::pair<stk::mesh::Entity,double>& lhs,
                  const std::pair<stk::mesh::Entity,double>& rhs)
{
    return lhs.second < rhs.second;
}

bool SupportStructure::runPrivateNodeBasedMaxDensityAbove()
{
    // Get the build direction.
    std::string workingString = mBuildPlateNormalString;
    size_t spacePos = workingString.find(' ');
    int cntr = 0;
    while(spacePos != std::string::npos)
    {
        std::string cur_string = workingString.substr(0,spacePos);
        workingString = workingString.substr(spacePos+1);
        mBuildPlateNormal[cntr] = std::atof(cur_string.c_str());
        cntr++;
        spacePos = workingString.find(' ');
    }
    mBuildPlateNormal[cntr] = std::atof(workingString.c_str());
    mBuildPlateNormal.normalize();

    // Calculate the interface angle for all interface elements and store in a map
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Calculating interface angle for all elements. " << std::endl;
    stk::mesh::Selector myElemSelector = mSTKMeshIn->meta_data()->universal_part();
    stk::mesh::BucketVector const &elementBuckets = mSTKMeshIn->bulk_data()->get_buckets(
            stk::topology::ELEMENT_RANK, myElemSelector );

    if(elementBuckets.size() == 0)
    {
        std::cout << "Failed to find any elements." << std::endl;
        return false;
    }

    double tolerance = 1e-12;
    std::set<stk::mesh::Entity> designInterfaceElements;
    std::map<stk::mesh::Entity, double> interfaceMap;
    for ( stk::mesh::BucketVector::const_iterator bucketIter = elementBuckets.begin();
            bucketIter != elementBuckets.end(); ++bucketIter )
    {
        stk::mesh::Bucket &tmpBucket = **bucketIter;
        size_t numElems = tmpBucket.size();
        for (size_t i=0; i<numElems; ++i)
        {
            bool allNodesInside = true;
            bool allNodesOutside = true;
            stk::mesh::Entity curElem = tmpBucket[i];
            stk::mesh::Entity const *elemNodes = mSTKMeshIn->bulk_data()->begin_nodes(curElem);
            int numNodes = mSTKMeshIn->bulk_data()->num_nodes(curElem);
            for(int j=0; j<numNodes; ++j)
            {
                stk::mesh::Entity curNode = elemNodes[j];
                double designVariableValue = mSTKMeshIn->getMaxNodalIsoFieldVariable(curNode.m_value);
                if((designVariableValue + tolerance) >= mDesignFieldThresholdValue)
                    allNodesOutside = false;
                else
                    allNodesInside = false;
            }
            if(!allNodesInside && !allNodesOutside)
            {
                Vector3D interfaceNormal;
                Vector3D interfaceOrigin;
                std::vector<Vector3D> triPoints;
                if(getIntersectionInfo(curElem, FIELD_DENSITY, interfaceNormal, interfaceOrigin, triPoints))
                {
                    // See if the interface is pointing toward the build plate
                    double dot=interfaceNormal % mBuildPlateNormal;
                    // Just add elements to the map that will actually affect things later on.
                    if(dot < 0.0)
                    {
                        interfaceMap[curElem] = dot;
                    }
                }
            }
        }
    }

    // For all nodes in interface elements calculate an average interface angle
    // from all of the attached interface elements.
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Calculating average interface angle for all nodes. " << std::endl;
    std::map<stk::mesh::Entity, double> nodeInterfaceAngles;
    std::map<stk::mesh::Entity, double>::iterator interfaceMapIter = interfaceMap.begin();
    while(interfaceMapIter != interfaceMap.end())
    {
        stk::mesh::Entity curElem = interfaceMapIter->first;
        stk::mesh::Entity const *elemNodes = mSTKMeshIn->bulk_data()->begin_nodes(curElem);
        int numNodes = mSTKMeshIn->bulk_data()->num_nodes(curElem);
        for(int j=0; j<numNodes; ++j)
        {
            stk::mesh::Entity curNode = elemNodes[j];
            if(nodeInterfaceAngles.count(curNode) == 0)
            {
                stk::mesh::Entity const *nodeElements = mSTKMeshIn->bulk_data()->begin_elements(curNode);
                int numElems = mSTKMeshIn->bulk_data()->num_elements(curNode);
                double averageDot = 0.0;
                double numInterfaceNeighbors = 0.0;
                for(int q=0; q<numElems; ++q)
                {
                    stk::mesh::Entity curNeighborElem = nodeElements[q];
                    std::map<stk::mesh::Entity, double>::iterator curNeighborIter = interfaceMap.find(curNeighborElem);
                    if(curNeighborIter != interfaceMap.end())
                    {
                        numInterfaceNeighbors += 1.0;
                        averageDot += curNeighborIter->second;
                    }
                }
                if(numInterfaceNeighbors > 0.1)
                {
                    averageDot /= numInterfaceNeighbors;
                    nodeInterfaceAngles[curNode] = averageDot;
                }
            }
        }
        interfaceMapIter++;
    }

    // Calculating an average element length.
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Calculating an average edge length. " << std::endl;
    double averageEdgeLength = 0.0;
    double numOverall = 0.0;
    bool tetsExist = false;
    for ( stk::mesh::BucketVector::const_iterator bucketIter = elementBuckets.begin();
            bucketIter != elementBuckets.end(); ++bucketIter )
    {
        stk::mesh::Bucket &tmpBucket = **bucketIter;
        size_t numElems = tmpBucket.size();
        for (size_t i=0; i<numElems; ++i)
        {
            double curAverage = 0.0;
            stk::mesh::Entity curElem = tmpBucket[i];
            stk::mesh::Entity const *elemNodes = mSTKMeshIn->bulk_data()->begin_nodes(curElem);
            int numNodes = mSTKMeshIn->bulk_data()->num_nodes(curElem);
            if(numNodes == 8)
            {
                Vector3D p0, p1, p3, p4;
                mSTKMeshIn->nodeCoordinates(elemNodes[0], p0.data());
                mSTKMeshIn->nodeCoordinates(elemNodes[1], p1.data());
                mSTKMeshIn->nodeCoordinates(elemNodes[3], p3.data());
                mSTKMeshIn->nodeCoordinates(elemNodes[4], p4.data());
                curAverage += p0.distance(p1);
                curAverage += p0.distance(p3);
                curAverage += p0.distance(p4);
                curAverage /= 3.0;
            }
            else if(numNodes == 4)
            {
                Vector3D p0, p1, p2, p3;
                mSTKMeshIn->nodeCoordinates(elemNodes[0], p0.data());
                mSTKMeshIn->nodeCoordinates(elemNodes[1], p1.data());
                mSTKMeshIn->nodeCoordinates(elemNodes[2], p2.data());
                mSTKMeshIn->nodeCoordinates(elemNodes[3], p3.data());
                curAverage += p0.distance(p1);
                curAverage += p0.distance(p2);
                curAverage += p2.distance(p3);
                curAverage /= 3.0;
                tetsExist = true;
            }
            averageEdgeLength += curAverage;
            numOverall += 1.0;
        }
    }
    averageEdgeLength /= numOverall;

    stk::mesh::Selector myNodeSelector = mSTKMeshIn->meta_data()->universal_part();
    stk::mesh::BucketVector const &nodeBuckets = mSTKMeshIn->bulk_data()->get_buckets(
            stk::topology::NODE_RANK, myNodeSelector );

    if(nodeBuckets.size() == 0)
    {
        std::cout << "Failed to find any nodes." << std::endl;
        return false;
    }

    // Do the main process loop. For each node in the mesh search
    // "upward" to determine the driving density node/value and
    // then calculate an appropriate support structure value.
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Calculating support structure value for every node. " << std::endl;
    double coarseSearchRadiusSquared = 1.5*1.5*averageEdgeLength*averageEdgeLength;
    double fineSearchRadiusSquared = .5*.5*averageEdgeLength*averageEdgeLength;
    if(tetsExist)
    {
        coarseSearchRadiusSquared = 3.5*3.5*averageEdgeLength*averageEdgeLength;
        fineSearchRadiusSquared = 1.0*1.0*averageEdgeLength*averageEdgeLength;
    }
    for ( stk::mesh::BucketVector::const_iterator nodeBucketIter = nodeBuckets.begin();
            nodeBucketIter != nodeBuckets.end();
            ++nodeBucketIter )
    {
        stk::mesh::Bucket &tmpBucket = **nodeBucketIter;
        size_t numBucketNodes = tmpBucket.size();
        for (size_t i=0; i<numBucketNodes; ++i)
        {
            bool foundDesignNode = false;
            stk::mesh::Entity curNode = tmpBucket[i];
            double curDensity = mSTKMeshIn->getMaxNodalIsoFieldVariable(curNode.m_value);

            if((curDensity + tolerance) >= mDesignFieldThresholdValue)
            {
                // This node is in the design so calculate the support structure
                // value and go to the next node.
                foundDesignNode = true;
                double curDot = nodeInterfaceAngles[curNode];
                double supportStructureValue = calculateSupportStructureValue(curDensity, curDensity, curDot, foundDesignNode);
                mSTKMeshIn->setSupportStructureFieldValue(curNode.m_value, supportStructureValue);
                continue;
            }

            Vector3D curNodeCoords;
            mSTKMeshIn->nodeCoordinates(curNode, curNodeCoords.data());

            // Start the max density at my value.
            double maxAboveDensity = curDensity;

            // Get the next node "above" this node
            std::set<stk::mesh::Entity> processedNodes;
            processedNodes.insert(curNode);
            std::set<stk::mesh::Entity> newElementsToCheck;
            std::set<stk::mesh::Entity> elemsToCheck;
            stk::mesh::Entity const *nodeElements = mSTKMeshIn->bulk_data()->begin_elements(curNode);
            int numElems = mSTKMeshIn->bulk_data()->num_elements(curNode);
            for(int q=0; q<numElems && !foundDesignNode; ++q)
            {
                elemsToCheck.insert(nodeElements[q]);
            }

            bool done = false;
            while(!done && !foundDesignNode)
            {
                while(elemsToCheck.size() && !foundDesignNode)
                {
                    stk::mesh::Entity curNeighborElem = *(elemsToCheck.begin());
                    elemsToCheck.erase(elemsToCheck.begin());

                    stk::mesh::Entity const *elemNodes = mSTKMeshIn->bulk_data()->begin_nodes(curNeighborElem);
                    int numElemNodes = mSTKMeshIn->bulk_data()->num_nodes(curNeighborElem);
                    for(int j=0; j<numElemNodes && !foundDesignNode; ++j)
                    {
                        stk::mesh::Entity curNeighborNode = elemNodes[j];
                        if(processedNodes.find(curNeighborNode) == processedNodes.end())
                        {
                            processedNodes.insert(curNeighborNode);
                            Vector3D curNeighborCoords;
                            mSTKMeshIn->nodeCoordinates(curNeighborNode, curNeighborCoords.data());
                            Vector3D curVec = curNeighborCoords - curNodeCoords;
                            // Make sure node is in the right direction
                            if(curVec % mBuildPlateNormal > 0.0)
                            {
                                // Now check to see if the node is within the fine search range
                                Vector3D curNeighborOnPlane;
                                projectPointToPlane(curNeighborCoords, curNodeCoords, mBuildPlateNormal, curNeighborOnPlane);
                                if(curNeighborOnPlane.distanceSquared(curNodeCoords) < fineSearchRadiusSquared)
                                {
                                    double curNeighborDensity = mSTKMeshIn->getMaxNodalIsoFieldVariable(curNeighborNode.m_value);
                                    if((curNeighborDensity + tolerance) >= mDesignFieldThresholdValue)
                                    {
                                        foundDesignNode = true;
                                        double curNeighborDot = nodeInterfaceAngles[curNeighborNode];
                                        double supportStructureValue = calculateSupportStructureValue(curDensity,
                                                                                                      curNeighborDensity, curNeighborDot, foundDesignNode);
                                        mSTKMeshIn->setSupportStructureFieldValue(curNode.m_value, supportStructureValue);
                                    }
                                    else
                                    {
                                        if(curNeighborDensity > maxAboveDensity)
                                            maxAboveDensity = curNeighborDensity;
                                    }
                                }
                                else
                                {
                                    if(curNeighborOnPlane.distanceSquared(curNodeCoords) < coarseSearchRadiusSquared)
                                    {
                                        stk::mesh::Entity const *tmpNodeElements = mSTKMeshIn->bulk_data()->begin_elements(curNeighborNode);
                                        int tmpNumElems = mSTKMeshIn->bulk_data()->num_elements(curNeighborNode);
                                        for(int w=0; w<tmpNumElems && !foundDesignNode; ++w)
                                        {
                                            if(tmpNodeElements[w] != curNeighborElem)
                                                newElementsToCheck.insert(tmpNodeElements[w]);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                if(!foundDesignNode)
                {
                    if(newElementsToCheck.size() == 0)
                        done = true;
                    else
                    {
                        elemsToCheck = newElementsToCheck;
                        newElementsToCheck.clear();
                    }
                }
            }
            if(!foundDesignNode)
            {
                double supportStructureValue = calculateSupportStructureValue(curDensity, maxAboveDensity, 0.0, foundDesignNode);
                mSTKMeshIn->setSupportStructureFieldValue(curNode.m_value, supportStructureValue);
            }
        }
    }

    mSTKMeshIn->write_exodus_mesh(mMeshOut, mConcatenateResults);

    return true;
}

bool SupportStructure::getBuildPlateCoordinateSystem(Vector3D &aX, Vector3D &aY, Vector3D &aZ, Vector3D &origin)
{
    stk::mesh::Selector myNodeSelector = mSTKMeshIn->meta_data()->universal_part();
    stk::mesh::BucketVector const &nodeBuckets = mSTKMeshIn->bulk_data()->get_buckets(
            stk::topology::NODE_RANK, myNodeSelector );

    bool haveOrigin = false;
    bool haveAxes = false;

    aZ = mBuildPlateNormal;

    // Loop until we have enough info to create the cooridnate axes
    std::vector<std::pair<stk::mesh::Entity, double> > nodeDotPairs;
    for ( stk::mesh::BucketVector::const_iterator nodeBucketIter = nodeBuckets.begin();
            nodeBucketIter != nodeBuckets.end() && (!haveOrigin || !haveAxes);
            ++nodeBucketIter )
    {
        stk::mesh::Bucket &tmpBucket = **nodeBucketIter;
        size_t numBucketNodes = tmpBucket.size();
        for (size_t i=0; i<numBucketNodes && (!haveOrigin || !haveAxes); ++i)
        {
            stk::mesh::Entity curNode = tmpBucket[i];
            Vector3D curNodeCoords;
            mSTKMeshIn->nodeCoordinates(curNode, curNodeCoords.data());
            if(!haveOrigin)
            {
                origin = curNodeCoords;
                haveOrigin = true;
            }
            else
            {
                Vector3D vec = curNodeCoords-origin;
                // Subtract off the z direction and see if we still have a non-zero vector
                double zComponent = vec % aZ;
                vec = vec - zComponent * aZ;
                if(vec.mag() > 1e-4)
                {
                    aX = vec;
                    aX.normalize();
                    aY = aZ * aX;
                    haveAxes = true;
                }
            }
        }
    }

    return (haveOrigin && haveAxes);
}

void SupportStructure::getNodesSortedInZDirection(Vector3D &aOrigin,
                                                  Vector3D &aAxis,
                                                  std::vector<std::pair<stk::mesh::Entity, double> > &aSortedNodeDistancePairs,
                                                  double &zMin, double &zMax)
{
    stk::mesh::Selector myNodeSelector = mSTKMeshIn->meta_data()->universal_part();
    stk::mesh::BucketVector const &nodeBuckets = mSTKMeshIn->bulk_data()->get_buckets(
            stk::topology::NODE_RANK, myNodeSelector );
    for ( stk::mesh::BucketVector::const_iterator nodeBucketIter = nodeBuckets.begin();
            nodeBucketIter != nodeBuckets.end();
            ++nodeBucketIter )
    {
        stk::mesh::Bucket &tmpBucket = **nodeBucketIter;
        size_t numBucketNodes = tmpBucket.size();
        for (size_t i=0; i<numBucketNodes; ++i)
        {
            stk::mesh::Entity curNode = tmpBucket[i];
            Vector3D curNodeCoords;
            mSTKMeshIn->nodeCoordinates(curNode, curNodeCoords.data());
            Vector3D curVec = curNodeCoords - aOrigin;
            double dot = curVec % aAxis;
            aSortedNodeDistancePairs.push_back(std::make_pair(curNode, dot));
        }
    }
    std::sort(aSortedNodeDistancePairs.begin(), aSortedNodeDistancePairs.end(), comparePairs);
    zMin = aSortedNodeDistancePairs[0].second;
    zMax = aSortedNodeDistancePairs[aSortedNodeDistancePairs.size()-1].second;
}

void SupportStructure::getAverageEdgeLength(double &averageEdgeLength)
{
    averageEdgeLength = 0.0;

    double numOverall = 0.0;
    stk::mesh::Selector myElemSelector = mSTKMeshIn->meta_data()->universal_part();
    stk::mesh::BucketVector const &elementBuckets = mSTKMeshIn->bulk_data()->get_buckets(
            stk::topology::ELEMENT_RANK, myElemSelector );
    for ( stk::mesh::BucketVector::const_iterator bucketIter = elementBuckets.begin();
            bucketIter != elementBuckets.end(); ++bucketIter )
    {
        stk::mesh::Bucket &tmpBucket = **bucketIter;
        size_t numElems = tmpBucket.size();
        for (size_t i=0; i<numElems; ++i)
        {
            double curAverage = 0.0;
            stk::mesh::Entity curElem = tmpBucket[i];
            stk::mesh::Entity const *elemNodes = mSTKMeshIn->bulk_data()->begin_nodes(curElem);
            int numNodes = mSTKMeshIn->bulk_data()->num_nodes(curElem);
            if(numNodes == 8)
            {
                Vector3D p0, p1, p3, p4;
                mSTKMeshIn->nodeCoordinates(elemNodes[0], p0.data());
                mSTKMeshIn->nodeCoordinates(elemNodes[1], p1.data());
                mSTKMeshIn->nodeCoordinates(elemNodes[3], p3.data());
                mSTKMeshIn->nodeCoordinates(elemNodes[4], p4.data());
                curAverage += p0.distance(p1);
                curAverage += p0.distance(p3);
                curAverage += p0.distance(p4);
                curAverage /= 3.0;
            }
            else if(numNodes == 4)
            {
                Vector3D p0, p1, p2, p3;
                mSTKMeshIn->nodeCoordinates(elemNodes[0], p0.data());
                mSTKMeshIn->nodeCoordinates(elemNodes[1], p1.data());
                mSTKMeshIn->nodeCoordinates(elemNodes[2], p2.data());
                mSTKMeshIn->nodeCoordinates(elemNodes[3], p3.data());
                curAverage += p0.distance(p1);
                curAverage += p0.distance(p2);
                curAverage += p2.distance(p3);
                curAverage /= 3.0;
            }
            averageEdgeLength += curAverage;
            numOverall += 1.0;
        }
    }
    averageEdgeLength /= numOverall;
}

void SupportStructure::getGridDimensions(double &aAverageEdgeLength, Vector3D &aOrigin,
                                         Vector3D &aXAxis, Vector3D &aYAxis, int &aNumGridX, int &aNumGridY,
                                         Vector3D &aMinCoords, Vector3D &aMaxCoords,
                                         double &aGridSizeX, double &aGridSizeY)
{
    stk::mesh::Selector myNodeSelector = mSTKMeshIn->meta_data()->universal_part();
    stk::mesh::BucketVector const &nodeBuckets = mSTKMeshIn->bulk_data()->get_buckets(
            stk::topology::NODE_RANK, myNodeSelector );

    double maxX, maxY, minX, minY;
    bool firstNode = true;

    minY = minX = maxX = maxY = 0.0;

    std::vector<std::pair<stk::mesh::Entity, double> > nodeDotPairs;
    for ( stk::mesh::BucketVector::const_iterator nodeBucketIter = nodeBuckets.begin();
            nodeBucketIter != nodeBuckets.end();
            ++nodeBucketIter )
    {
        stk::mesh::Bucket &tmpBucket = **nodeBucketIter;
        size_t numBucketNodes = tmpBucket.size();
        for (size_t i=0; i<numBucketNodes; ++i)
        {
            stk::mesh::Entity curNode = tmpBucket[i];
            Vector3D curNodeCoords;
            mSTKMeshIn->nodeCoordinates(curNode, curNodeCoords.data());
            Vector3D vec = curNodeCoords - aOrigin;
            double dotX = vec % aXAxis;
            double dotY = vec % aYAxis;
            if(firstNode)
            {
                maxX = minX = dotX;
                maxY = minY = dotY;
                firstNode = false;
            }
            else
            {
                if(dotX > maxX)
                    maxX = dotX;
                if(dotX < minX)
                    minX = dotX;
                if(dotY > maxY)
                    maxY = dotY;
                if(dotY < minY)
                    minY = dotY;
            }
        }
    }
    aMinCoords[0] = minX;
    aMinCoords[1] = minY;
    aMinCoords[2] = 0.0;
    aMaxCoords[0] = maxX;
    aMaxCoords[1] = maxY;
    aMaxCoords[2] = 0.0;
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    aNumGridX = (int)((xRange / aAverageEdgeLength) + 1.0);
    aNumGridY = (int)((yRange / aAverageEdgeLength) + 1.0);
    aGridSizeX = xRange / (double)aNumGridX;
    aGridSizeY = yRange / (double)aNumGridY;
}

void SupportStructure::getNodeInterfaceData(std::map<stk::mesh::Entity, double> &aNodeInterfaceAngles)
{
    // Calculate the interface angle for all interface elements and store in a map
    stk::mesh::Selector myElemSelector = mSTKMeshIn->meta_data()->universal_part();
    stk::mesh::BucketVector const &elementBuckets = mSTKMeshIn->bulk_data()->get_buckets(
            stk::topology::ELEMENT_RANK, myElemSelector );

    double tolerance = 1e-12;
    std::set<stk::mesh::Entity> designInterfaceElements;
    std::map<stk::mesh::Entity, Vector3D> interfaceMap;
    for ( stk::mesh::BucketVector::const_iterator bucketIter = elementBuckets.begin();
            bucketIter != elementBuckets.end(); ++bucketIter )
    {
        stk::mesh::Bucket &tmpBucket = **bucketIter;
        size_t numElems = tmpBucket.size();
        for (size_t i=0; i<numElems; ++i)
        {
            bool allNodesInside = true;
            bool allNodesOutside = true;
            stk::mesh::Entity curElem = tmpBucket[i];
            stk::mesh::Entity const *elemNodes = mSTKMeshIn->bulk_data()->begin_nodes(curElem);
            int numNodes = mSTKMeshIn->bulk_data()->num_nodes(curElem);
            for(int j=0; j<numNodes; ++j)
            {
                stk::mesh::Entity curNode = elemNodes[j];
                double designVariableValue = mSTKMeshIn->getMaxNodalIsoFieldVariable(curNode.m_value);
                if((designVariableValue + tolerance) >= mDesignFieldThresholdValue)
                    allNodesOutside = false;
                else
                    allNodesInside = false;
            }
            if(!allNodesInside && !allNodesOutside)
            {
                Vector3D interfaceNormal;
                Vector3D interfaceOrigin;
                std::vector<Vector3D> triPoints;
                if(getIntersectionInfo(curElem, FIELD_DENSITY, interfaceNormal, interfaceOrigin, triPoints))
                {
                    // See if the interface is pointing toward the build plate
                    double dot=interfaceNormal % mBuildPlateNormal;
                    // Just add elements to the map that will actually affect things later on.
                    if(dot < 0.0)
                    {
                        interfaceMap[curElem] = interfaceNormal;
                    }
                }
            }
        }
    }

    // For all nodes in interface elements calculate an average interface angle
    // from all of the attached interface elements.
    std::map<stk::mesh::Entity, Vector3D>::iterator interfaceMapIter = interfaceMap.begin();
    while(interfaceMapIter != interfaceMap.end())
    {
        stk::mesh::Entity curElem = interfaceMapIter->first;
        stk::mesh::Entity const *elemNodes = mSTKMeshIn->bulk_data()->begin_nodes(curElem);
        int numNodes = mSTKMeshIn->bulk_data()->num_nodes(curElem);
        for(int j=0; j<numNodes; ++j)
        {
            stk::mesh::Entity curNode = elemNodes[j];
            if(aNodeInterfaceAngles.count(curNode) == 0)
            {
                Vector3D p0;
                mSTKMeshIn->nodeCoordinates(curNode, p0.data());
                stk::mesh::Entity const *nodeElements = mSTKMeshIn->bulk_data()->begin_elements(curNode);
                int numElems = mSTKMeshIn->bulk_data()->num_elements(curNode);
                double averageDot = 0.0;
                Vector3D averageNormal;
                averageNormal[0] = averageNormal[1] = averageNormal[2] = 0.0;
                double numInterfaceNeighbors = 0.0;
                for(int q=0; q<numElems; ++q)
                {
                    stk::mesh::Entity curNeighborElem = nodeElements[q];
                    std::map<stk::mesh::Entity, Vector3D>::iterator curNeighborIter = interfaceMap.find(curNeighborElem);
                    if(curNeighborIter != interfaceMap.end())
                    {
                        numInterfaceNeighbors += 1.0;
                        averageNormal += curNeighborIter->second;
                    }
                }
                if(numInterfaceNeighbors > 0.1)
                {
                    averageNormal.normalize();
                    averageDot = averageNormal % mBuildPlateNormal;
                    aNodeInterfaceAngles[curNode] = averageDot;
                }
            }
        }
        interfaceMapIter++;
    }
}

void SupportStructure::getNodeXY(stk::mesh::Entity &aNode, Vector3D &aOrigin, Vector3D &aXAxis,
                                 Vector3D &aYAxis, Vector3D &aMinCoords, double &aGridSizeX,
                                 double &aGridSizeY, int &aNumGridX, int &aNumGridY, int &aNodeX,
                                 int &aNodeY)
{
    Vector3D nodeCoords;
    mSTKMeshIn->nodeCoordinates(aNode, nodeCoords.data());
    /*
    if(fabs(nodeCoords[0]+.158569) < .0001 && fabs(nodeCoords[1]+.604976) < .0001 && fabs(nodeCoords[2]+.143273) < .0001)
    {
        int hh=0;
        hh++;
    }
    */
    Vector3D vec = nodeCoords - aOrigin;
    double dotX = vec % aXAxis;
    double dotY = vec % aYAxis;
    double xDistanceFromMin = dotX - aMinCoords[0];
    double yDistanceFromMin = dotY - aMinCoords[1];
    aNodeX = (int)(xDistanceFromMin/aGridSizeX);
    aNodeY = (int)(yDistanceFromMin/aGridSizeY);
    if(aNodeX > (aNumGridX-1))
        aNodeX = aNumGridX-1;
    if(aNodeY > (aNumGridY-1))
        aNodeY = aNumGridY-1;
}

void SupportStructure::getValsForSettingVoxel(stk::mesh::Entity aNode,
                                              int aXIndex, int aYIndex, int aNumGridX, int aNumGridY,
                                              std::vector<std::vector<VoxelData> > &aVoxelData,
                                              std::map<stk::mesh::Entity, double> &aNodeInterfaceAngles,
                                              double &aDensity,
                                              double &aDot,
                                              bool &aHasInterface)
{
    if(aNode.m_value != 0)
    {
        if(aNodeInterfaceAngles.count(aNode))
        {
            aDot = aNodeInterfaceAngles[aNode];
            aHasInterface = true;
        }
        else
            aHasInterface = false;
        aDensity = mSTKMeshIn->getMaxNodalIsoFieldVariable(aNode.m_value);
    }
    else
    {
        int imin=aXIndex-mNeighborSearchRadius;
        if(imin<0)
            imin=0;
        int jmin=aYIndex-mNeighborSearchRadius;
        if(jmin<0)
            jmin=0;
        int imax = aXIndex+mNeighborSearchRadius;
        if(imax > (aNumGridX-1))
            imax = aNumGridX-1;
        int jmax = aYIndex+mNeighborSearchRadius;
        if(jmax > (aNumGridY-1))
            jmax = aNumGridY-1;
        double weightedAverageDensity = 0.0;
        double weightedAverageDot = 0.0;
        double sumDensityWeights = 0.0;
        double sumDotWeights = 0.0;
        int numFound=0;
        aHasInterface = false;
        for(int i=imin; i<=imax; i++)
        {
            for(int j=jmin; j<=jmax; j++)
            {
                if(i!=aXIndex || j!=aYIndex)
                {
                    if(aVoxelData[i][j].setByNode) // was set by a node being there
                    {
                        numFound++;
                        double distance = abs(i-aXIndex) + abs(j-aYIndex);
                        double weight=1.0/distance;
                        weightedAverageDensity += (aVoxelData[i][j].maxDensity * weight);
                        if(aVoxelData[i][j].hasInterface || aVoxelData[i][j].maxIsDesign)
                        {
                            weightedAverageDot += (aVoxelData[i][j].maxDot * weight);
                            sumDotWeights += weight;
                            aHasInterface = true;
                        }
                        sumDensityWeights += weight;
                    }
                }
            }
        }
        if(numFound > .1)
        {
            aDensity = weightedAverageDensity/sumDensityWeights;
            if(aHasInterface)
                aDot = weightedAverageDot/sumDotWeights;
        }
        std::cout << "Number of neighbors found: " << numFound << std::endl;
    }
}

void SupportStructure::setVoxelData(stk::mesh::Entity aNode,
                                    int aXIndex, int aYIndex, int aNumGridX, int aNumGridY,
                                    std::vector<std::vector<std::vector<VoxelData> > > &aVoxelData,
                                    std::map<stk::mesh::Entity, double> &aNodeInterfaceAngles,
                                    int aZLayer,
                                    int aNumZLayers,
                                    int aLayerIndex)
{
    double tolerance = 1e-12;
    double curDensity=0.0;
    double curDot=0.0;
    bool hasInterface = false;
    bool curVoxelHasNode = (aNode.m_value != 0);
    int curLayerIndex = mNeighborSearchRadius;
    if(aLayerIndex != -1)
        curLayerIndex = aLayerIndex;
    int aboveLayerIndex = curLayerIndex-1;
    VoxelData aboveVoxelData = aVoxelData[aboveLayerIndex][aXIndex][aYIndex];
    VoxelData &curVoxelData = aVoxelData[curLayerIndex][aXIndex][aYIndex];

    if(aXIndex == 32 && aYIndex == 29)
    {
        int ff=0;
        ff++;
    }

    if(curVoxelHasNode)
    {
        if(aNodeInterfaceAngles.count(aNode))
        {
            curDot = aNodeInterfaceAngles[aNode];
            hasInterface = true;
        }
        else
            hasInterface = false;
        curDensity = mSTKMeshIn->getMaxNodalIsoFieldVariable(aNode.m_value);
        if(aboveVoxelData.dataExists)
        {
            // Compare against above data to determine what to do.
            if(aboveVoxelData.inDesignRegion)
            {
                if((curDensity + tolerance) >= mDesignFieldThresholdValue)
                {
                    double supportStructureValue = calculateSupportStructureValue(curDensity, curDensity, curDot, false);
                    mSTKMeshIn->setSupportStructureFieldValue(aNode.m_value, supportStructureValue);
                    curVoxelData.maxDensity = curDensity;
                    curVoxelData.maxDot = curDot;
                    curVoxelData.dataExists = true;
                    curVoxelData.hasInterface = false;
                    curVoxelData.inDesignRegion = true;
                    curVoxelData.maxIsDesign = true;
                    curVoxelData.setByNode = true;
                }
                else
                {
                    double supportStructureValue = calculateSupportStructureValue(curDensity, aboveVoxelData.maxDensity, aboveVoxelData.maxDot, true);
                    mSTKMeshIn->setSupportStructureFieldValue(aNode.m_value, supportStructureValue);
                    curVoxelData.maxDensity = aboveVoxelData.maxDensity;
                    curVoxelData.maxDot = aboveVoxelData.maxDot;
                    curVoxelData.dataExists = true;
                    curVoxelData.hasInterface = true;
                    curVoxelData.maxIsDesign = true;
                    curVoxelData.setByNode = true;
                    curVoxelData.inDesignRegion = false;
                }
            }
            else
            {
                if((curDensity + tolerance) >= mDesignFieldThresholdValue)
                {
                    double supportStructureValue = calculateSupportStructureValue(curDensity, curDensity, curDot, false);
                    mSTKMeshIn->setSupportStructureFieldValue(aNode.m_value, supportStructureValue);
                    curVoxelData.maxDensity = curDensity;
                    curVoxelData.maxDot = curDot;
                    curVoxelData.dataExists = true;
                    curVoxelData.hasInterface = false;
                    curVoxelData.maxIsDesign = true;
                    curVoxelData.setByNode = true;
                    curVoxelData.inDesignRegion = true;
                }
                else
                {
                    if(aboveVoxelData.maxIsDesign)
                    {
                        double supportStructureValue = calculateSupportStructureValue(curDensity, aboveVoxelData.maxDensity, aboveVoxelData.maxDot, true);
                        mSTKMeshIn->setSupportStructureFieldValue(aNode.m_value, supportStructureValue);
                        curVoxelData.maxDensity = aboveVoxelData.maxDensity;
                        curVoxelData.maxDot = aboveVoxelData.maxDot;
                        curVoxelData.dataExists = true;
                        curVoxelData.hasInterface = false;
                        curVoxelData.maxIsDesign = true;
                        curVoxelData.setByNode = true;
                        curVoxelData.inDesignRegion = false;
                    }
                    else
                    {
                        if(curDensity > aboveVoxelData.maxDensity)
                            curVoxelData.maxDensity = curDensity;
                        else
                            curVoxelData.maxDensity = aboveVoxelData.maxDensity;

                        double supportStructureValue = calculateSupportStructureValue(curDensity, curVoxelData.maxDensity, curDot, false);
                        mSTKMeshIn->setSupportStructureFieldValue(aNode.m_value, supportStructureValue);
                        curVoxelData.maxDot = curDot;
                        curVoxelData.dataExists = true;
                        curVoxelData.hasInterface = false;
                        curVoxelData.maxIsDesign = false;
                        curVoxelData.setByNode = true;
                        curVoxelData.inDesignRegion = false;
                    }
                }
            }
        }
        else
        {
            // Just set the current voxel data.
            curVoxelData.dataExists = true;
            curVoxelData.hasInterface = hasInterface;
            if((curDensity + tolerance) >= mDesignFieldThresholdValue)
            {
                curVoxelData.inDesignRegion = true;
                curVoxelData.maxIsDesign = true;
            }
            else
            {
                curVoxelData.inDesignRegion = false;
                curVoxelData.maxIsDesign = false;
            }
            curVoxelData.maxDensity = curDensity;
            curVoxelData.maxDot = curDot;
            curVoxelData.setByNode = true;
        }
    }
    else
    {
        // Get values for density, dot, and has interface from neighbors
        int imin=aXIndex-mNeighborSearchRadius;
        if(imin<0)
            imin=0;
        int jmin=aYIndex-mNeighborSearchRadius;
        if(jmin<0)
            jmin=0;
        int kmin=0;
        if(aZLayer < mNeighborSearchRadius)
            kmin = mNeighborSearchRadius-aZLayer;
        int imax = aXIndex+mNeighborSearchRadius;
        if(imax > (aNumGridX-1))
            imax = aNumGridX-1;
        int jmax = aYIndex+mNeighborSearchRadius;
        if(jmax > (aNumGridY-1))
            jmax = aNumGridY-1;
        int kmax=2*mNeighborSearchRadius;
        if(aZLayer + mNeighborSearchRadius > aNumZLayers - 1)
            kmax = aNumZLayers - (aZLayer - 1);
        double weightedAverageDensity = 0.0;
        double weightedAverageHasInterface = 0.0;
        double weightedAverageDot = 0.0;
        double sumDensityWeights = 0.0;
        double sumDotWeights = 0.0;
        int numFound=0;
        for(int k=kmin; k<=kmax; k++)
        {
            for(int i=imin; i<=imax; i++)
            {
                for(int j=jmin; j<=jmax; j++)
                {
                    if(i!=aXIndex || j!=aYIndex || k!=mNeighborSearchRadius)
                    {
                        if(aVoxelData[k][i][j].dataExists && aVoxelData[k][i][j].setByNode) // was set by a node being there
                        {
                            numFound++;
                            double distance = abs(i-aXIndex) + abs(j-aYIndex) + abs(k-mNeighborSearchRadius);
                            double weight=1.0/distance;
                            weightedAverageDensity += (aVoxelData[k][i][j].maxDensity * weight);
                            if(aVoxelData[k][i][j].hasInterface || aVoxelData[k][i][j].maxIsDesign)
                            {
                                weightedAverageDot += (aVoxelData[k][i][j].maxDot * weight);
                                weightedAverageHasInterface += (double)(aVoxelData[k][i][j].hasInterface);
                                sumDotWeights += weight;
                            }
                            sumDensityWeights += weight;
                        }
                    }
                }
            }
        }
        if(numFound > .1)
        {
            curDensity = weightedAverageDensity/sumDensityWeights;
            if(sumDotWeights > .00001 && (weightedAverageHasInterface/sumDotWeights) > 0.5)
                hasInterface = true;
            if(sumDotWeights > .00001)
                curDot = weightedAverageDot/sumDotWeights;
        }
        else
            std::cout << "****** Didn't find any voxel neighbors for interpolating density values! *********" << std::endl;

        // Now set values based on current and above values
        if(aboveVoxelData.dataExists)
        {
            // Compare against above data to determine what to do.
            if(aboveVoxelData.inDesignRegion)
            {
                if((curDensity + tolerance) >= mDesignFieldThresholdValue)
                {
                    curVoxelData.maxDensity = curDensity;
                    curVoxelData.maxDot = curDot;
                    curVoxelData.dataExists = true;
                    curVoxelData.hasInterface = false;
                    curVoxelData.inDesignRegion = true;
                    curVoxelData.maxIsDesign = true;
                    curVoxelData.setByNode = false;
                }
                else
                {
                    curVoxelData.maxDensity = aboveVoxelData.maxDensity;
                    curVoxelData.maxDot = aboveVoxelData.maxDot;
                    curVoxelData.dataExists = true;
                    curVoxelData.hasInterface = true;
                    curVoxelData.maxIsDesign = true;
                    curVoxelData.setByNode = false;
                    curVoxelData.inDesignRegion = false;
                }
            }
            else
            {
                if((curDensity + tolerance) >= mDesignFieldThresholdValue)
                {
                    curVoxelData.maxDensity = curDensity;
                    curVoxelData.maxDot = curDot;
                    curVoxelData.dataExists = true;
                    curVoxelData.hasInterface = false;
                    curVoxelData.maxIsDesign = true;
                    curVoxelData.setByNode = false;
                    curVoxelData.inDesignRegion = true;
                }
                else
                {
                    if(aboveVoxelData.maxIsDesign)
                    {
                        curVoxelData.maxDensity = aboveVoxelData.maxDensity;
                        curVoxelData.maxDot = aboveVoxelData.maxDot;
                        curVoxelData.dataExists = true;
                        curVoxelData.hasInterface = false;
                        curVoxelData.maxIsDesign = true;
                        curVoxelData.setByNode = false;
                        curVoxelData.inDesignRegion = false;
                    }
                    else
                    {
                        if(curDensity > aboveVoxelData.maxDensity)
                            curVoxelData.maxDensity = curDensity;
                        else
                            curVoxelData.maxDensity = aboveVoxelData.maxDensity;

                        curVoxelData.maxDot = curDot;
                        curVoxelData.dataExists = true;
                        curVoxelData.hasInterface = false;
                        curVoxelData.maxIsDesign = false;
                        curVoxelData.setByNode = false;
                        curVoxelData.inDesignRegion = false;
                    }
                }
            }
        }
        else
        {
            // Just set the current voxel data.
            curVoxelData.dataExists = true;
            curVoxelData.hasInterface = hasInterface;
            if((curDensity + tolerance) >= mDesignFieldThresholdValue)
            {
                curVoxelData.inDesignRegion = true;
                curVoxelData.maxIsDesign = true;
            }
            else
            {
                curVoxelData.inDesignRegion = false;
                curVoxelData.maxIsDesign = false;
            }
            curVoxelData.maxDensity = curDensity;
            curVoxelData.maxDot = curDot;
            curVoxelData.setByNode = false;
        }
    }
}

void SupportStructure::setVoxelDataByNeighbor(int aXIndex, int aYIndex, int aNumGridX, int aNumGridY,
                                    std::vector<std::vector<std::vector<VoxelData> > > &aVoxelData,
                                    int aZLayer,
                                    int aNumZLayers)
{
    double tolerance = 1e-12;
    double curDensity=0.0;
    double curDot=0.0;
    bool hasInterface = false;
    int aboveLayerIndex = aZLayer-1;
    VoxelData aboveVoxelData;
    if(aboveLayerIndex >= 0)
        aboveVoxelData = aVoxelData[aboveLayerIndex][aXIndex][aYIndex];
    VoxelData &curVoxelData = aVoxelData[aZLayer][aXIndex][aYIndex];

    if(aXIndex == 32 && aYIndex == 29)
    {
        int ff=0;
        ff++;
    }

    // Get values for density, dot, and has interface from neighbors
    int imin=aXIndex-mNeighborSearchRadius;
    if(imin<0)
        imin=0;
    int jmin=aYIndex-mNeighborSearchRadius;
    if(jmin<0)
        jmin=0;
    int kmin=aZLayer - mNeighborSearchRadius;
    if(kmin < 0)
        kmin = 0;
    int imax = aXIndex+mNeighborSearchRadius;
    if(imax > (aNumGridX-1))
        imax = aNumGridX-1;
    int jmax = aYIndex+mNeighborSearchRadius;
    if(jmax > (aNumGridY-1))
        jmax = aNumGridY-1;
    int kmax=aZLayer + mNeighborSearchRadius;
    if(kmax > (aNumZLayers-1))
        kmax = aNumZLayers-1;
    double weightedAverageDensity = 0.0;
    double weightedAverageHasInterface = 0.0;
    double weightedAverageDot = 0.0;
    double sumDensityWeights = 0.0;
    double sumDotWeights = 0.0;
    int numFound=0;
    for(int k=kmin; k<=kmax; k++)
    {
        for(int i=imin; i<=imax; i++)
        {
            for(int j=jmin; j<=jmax; j++)
            {
                if(i!=aXIndex || j!=aYIndex || k!=aZLayer)
                {
                    if(aVoxelData[k][i][j].setByNode) // was set by a node being there
                    {
                        numFound++;
                        double distance = abs(i-aXIndex) + abs(j-aYIndex) + abs(k-aZLayer);
                        double weight=1.0/distance;
                        weightedAverageDensity += (aVoxelData[k][i][j].maxDensity * weight);
                        if(aVoxelData[k][i][j].hasInterface)
//                            if(aVoxelData[k][i][j].hasInterface || aVoxelData[k][i][j].maxIsDesign)
                        {
                            weightedAverageDot += (aVoxelData[k][i][j].maxDot * weight);
                            weightedAverageHasInterface += (double)(aVoxelData[k][i][j].hasInterface);
                            sumDotWeights += weight;
                        }
                        sumDensityWeights += weight;
                    }
                }
            }
        }
    }
    if(numFound > .1)
    {
        curDensity = weightedAverageDensity/sumDensityWeights;
        if(sumDotWeights > .00001 && (weightedAverageHasInterface/sumDotWeights) > 0.5)
            hasInterface = true;
        if(sumDotWeights > .00001)
            curDot = weightedAverageDot/sumDotWeights;
    }
    else
        std::cout << "****** Didn't find any voxel neighbors for interpolating density values! *********" << std::endl;

    // Now set values based on current and above values
    if(aboveLayerIndex >= 0)
    {
        // Compare against above data to determine what to do.
        if(aboveVoxelData.inDesignRegion)
        {
            if((curDensity + tolerance) >= mDesignFieldThresholdValue)
            {
                curVoxelData.maxDensity = curDensity;
                curVoxelData.maxDot = curDot;
                curVoxelData.dataExists = true;
                curVoxelData.hasInterface = false;
                curVoxelData.inDesignRegion = true;
                curVoxelData.maxIsDesign = true;
                curVoxelData.setByNode = false;
            }
            else
            {
                curVoxelData.maxDensity = aboveVoxelData.maxDensity;
                curVoxelData.maxDot = aboveVoxelData.maxDot;
                curVoxelData.dataExists = true;
                curVoxelData.hasInterface = true;
                curVoxelData.maxIsDesign = true;
                curVoxelData.setByNode = false;
                curVoxelData.inDesignRegion = false;
            }
        }
        else
        {
            if((curDensity + tolerance) >= mDesignFieldThresholdValue)
            {
                curVoxelData.maxDensity = curDensity;
                curVoxelData.maxDot = curDot;
                curVoxelData.dataExists = true;
                curVoxelData.hasInterface = false;
                curVoxelData.maxIsDesign = true;
                curVoxelData.setByNode = false;
                curVoxelData.inDesignRegion = true;
            }
            else
            {
                if(aboveVoxelData.maxIsDesign)
                {
                    curVoxelData.maxDensity = aboveVoxelData.maxDensity;
                    curVoxelData.maxDot = aboveVoxelData.maxDot;
                    curVoxelData.dataExists = true;
                    curVoxelData.hasInterface = false;
                    curVoxelData.maxIsDesign = true;
                    curVoxelData.setByNode = false;
                    curVoxelData.inDesignRegion = false;
                }
                else
                {
                    if(curDensity > aboveVoxelData.maxDensity)
                        curVoxelData.maxDensity = curDensity;
                    else
                        curVoxelData.maxDensity = aboveVoxelData.maxDensity;

                    curVoxelData.maxDot = curDot;
                    curVoxelData.dataExists = true;
                    curVoxelData.hasInterface = false;
                    curVoxelData.maxIsDesign = false;
                    curVoxelData.setByNode = false;
                    curVoxelData.inDesignRegion = false;
                }
            }
        }
    }
    else
    {
        // Just set the current voxel data.
        curVoxelData.dataExists = true;
        curVoxelData.hasInterface = hasInterface;
        if((curDensity + tolerance) >= mDesignFieldThresholdValue)
        {
            curVoxelData.inDesignRegion = true;
            curVoxelData.maxIsDesign = true;
        }
        else
        {
            curVoxelData.inDesignRegion = false;
            curVoxelData.maxIsDesign = false;
        }
        curVoxelData.maxDensity = curDensity;
        curVoxelData.maxDot = curDot;
        curVoxelData.setByNode = false;
    }
}

void SupportStructure::setVoxelDataByNode(int aXIndex, int aYIndex,
                                    std::vector<std::vector<std::vector<VoxelData> > > &aVoxelData,
                                    int aZLayer)
{
    double tolerance = 1e-12;
    int aboveLayerIndex = aZLayer-1;
    VoxelData aboveVoxelData;
    if(aboveLayerIndex >= 0)
        aboveVoxelData = aVoxelData[aboveLayerIndex][aXIndex][aYIndex];
    VoxelData &curVoxelData = aVoxelData[aZLayer][aXIndex][aYIndex];
    double curDensity=curVoxelData.maxDensity;
    double curDot=curVoxelData.maxDot;
    bool hasInterface=curVoxelData.hasInterface;
//    stk::mesh::Entity aNode = curVoxelData.node;

    if(aXIndex == 32 && aYIndex == 29)
    {
        int ff=0;
        ff++;
    }

    if(aboveLayerIndex >= 0)
    {
        // Compare against above data to determine what to do.
        if(aboveVoxelData.inDesignRegion)
        {
            if((curDensity + tolerance) >= mDesignFieldThresholdValue)
            {
                double supportStructureValue = calculateSupportStructureValue(curDensity, curDensity, curDot, false);
                for(size_t j=0; j<curVoxelData.nodes.size(); ++j)
                    mSTKMeshIn->setSupportStructureFieldValue(curVoxelData.nodes[j].m_value, supportStructureValue);
                curVoxelData.maxDensity = curDensity;
                curVoxelData.maxDot = curDot;
                curVoxelData.dataExists = true;
                curVoxelData.hasInterface = false;
                curVoxelData.inDesignRegion = true;
                curVoxelData.maxIsDesign = true;
                curVoxelData.setByNode = true;
            }
            else
            {
                double supportStructureValue = calculateSupportStructureValue(curDensity, aboveVoxelData.maxDensity, aboveVoxelData.maxDot, true);
                for(size_t j=0; j<curVoxelData.nodes.size(); ++j)
                    mSTKMeshIn->setSupportStructureFieldValue(curVoxelData.nodes[j].m_value, supportStructureValue);
                curVoxelData.maxDensity = aboveVoxelData.maxDensity;
                curVoxelData.maxDot = aboveVoxelData.maxDot;
                curVoxelData.dataExists = true;
                curVoxelData.hasInterface = true;
                curVoxelData.maxIsDesign = true;
                curVoxelData.setByNode = true;
                curVoxelData.inDesignRegion = false;
            }
        }
        else
        {
            if((curDensity + tolerance) >= mDesignFieldThresholdValue)
            {
                double supportStructureValue = calculateSupportStructureValue(curDensity, curDensity, curDot, false);
                for(size_t j=0; j<curVoxelData.nodes.size(); ++j)
                    mSTKMeshIn->setSupportStructureFieldValue(curVoxelData.nodes[j].m_value, supportStructureValue);
                curVoxelData.maxDensity = curDensity;
                curVoxelData.maxDot = curDot;
                curVoxelData.dataExists = true;
                curVoxelData.hasInterface = false;
                curVoxelData.maxIsDesign = true;
                curVoxelData.setByNode = true;
                curVoxelData.inDesignRegion = true;
            }
            else
            {
                if(aboveVoxelData.maxIsDesign)
                {
                    double supportStructureValue = calculateSupportStructureValue(curDensity, aboveVoxelData.maxDensity, aboveVoxelData.maxDot, true);
                    for(size_t j=0; j<curVoxelData.nodes.size(); ++j)
                        mSTKMeshIn->setSupportStructureFieldValue(curVoxelData.nodes[j].m_value, supportStructureValue);
                    curVoxelData.maxDensity = aboveVoxelData.maxDensity;
                    curVoxelData.maxDot = aboveVoxelData.maxDot;
                    curVoxelData.dataExists = true;
                    curVoxelData.hasInterface = false;
                    curVoxelData.maxIsDesign = true;
                    curVoxelData.setByNode = true;
                    curVoxelData.inDesignRegion = false;
                }
                else
                {
                    if(curDensity > aboveVoxelData.maxDensity)
                        curVoxelData.maxDensity = curDensity;
                    else
                        curVoxelData.maxDensity = aboveVoxelData.maxDensity;

                    double supportStructureValue = calculateSupportStructureValue(curDensity, curVoxelData.maxDensity, curDot, false);
                    for(size_t j=0; j<curVoxelData.nodes.size(); ++j)
                        mSTKMeshIn->setSupportStructureFieldValue(curVoxelData.nodes[j].m_value, supportStructureValue);
                    curVoxelData.maxDot = curDot;
                    curVoxelData.dataExists = true;
                    curVoxelData.hasInterface = false;
                    curVoxelData.maxIsDesign = false;
                    curVoxelData.setByNode = true;
                    curVoxelData.inDesignRegion = false;
                }
            }
        }
    }
    else
    {
        // Just set the current voxel data.
        curVoxelData.dataExists = true;
        curVoxelData.hasInterface = hasInterface;
        if((curDensity + tolerance) >= mDesignFieldThresholdValue)
        {
            curVoxelData.inDesignRegion = true;
            curVoxelData.maxIsDesign = true;
        }
        else
        {
            curVoxelData.inDesignRegion = false;
            curVoxelData.maxIsDesign = false;
        }
        curVoxelData.maxDensity = curDensity;
        curVoxelData.maxDot = curDot;
        curVoxelData.setByNode = true;
        double supportStructureValue = calculateSupportStructureValue(curDensity, curDensity, curDot, false);
        for(size_t j=0; j<curVoxelData.nodes.size(); ++j)
            mSTKMeshIn->setSupportStructureFieldValue(curVoxelData.nodes[j].m_value, supportStructureValue);
    }
}

void SupportStructure::setVoxelNodeData(stk::mesh::Entity aNode,
                                    int aXIndex, int aYIndex,
                                    std::vector<std::vector<std::vector<VoxelData> > > &aVoxelData,
                                    std::map<stk::mesh::Entity, double> &aNodeInterfaceAngles,
                                    int aZLayer)
{
    double curDensity=0.0;
    double curDot=0.0;
    bool hasInterface = false;
    VoxelData &curVoxelData = aVoxelData[aZLayer][aXIndex][aYIndex];

    if(aXIndex == 32 && aYIndex == 29)
    {
        int ff=0;
        ff++;
    }

    if(aNodeInterfaceAngles.count(aNode))
    {
        curDot = aNodeInterfaceAngles[aNode];
        hasInterface = true;
    }
    else
        hasInterface = false;

    curDensity = mSTKMeshIn->getMaxNodalIsoFieldVariable(aNode.m_value);

    // Just set the current voxel data.
    curVoxelData.hasInterface = hasInterface;
    curVoxelData.maxDensity = curDensity;
    curVoxelData.maxDot = curDot;
    curVoxelData.setByNode = true;
    curVoxelData.nodes.push_back(aNode);
}

void SupportStructure::getValsForSettingVoxel3D(stk::mesh::Entity aNode,
                                              int aXIndex, int aYIndex, int aNumGridX, int aNumGridY,
                                              std::vector<std::vector<std::vector<VoxelData> > > &aVoxelData,
                                              std::map<stk::mesh::Entity, double> &aNodeInterfaceAngles,
                                              double &aDensity,
                                              double &aDot,
                                              bool &aHasInterface,
                                              int aZLayer,
                                              int aNumZLayers)
{
    if(aNode.m_value != 0)
    {
        if(aNodeInterfaceAngles.count(aNode))
        {
            aDot = aNodeInterfaceAngles[aNode];
            aHasInterface = true;
        }
        else
            aHasInterface = false;
        aDensity = mSTKMeshIn->getMaxNodalIsoFieldVariable(aNode.m_value);
    }
    else
    {
        int imin=aXIndex-mNeighborSearchRadius;
        if(imin<0)
            imin=0;
        int jmin=aYIndex-mNeighborSearchRadius;
        if(jmin<0)
            jmin=0;
        int kmin=0;
        if(aZLayer < mNeighborSearchRadius)
            kmin = mNeighborSearchRadius-aZLayer;
        int imax = aXIndex+mNeighborSearchRadius;
        if(imax > (aNumGridX-1))
            imax = aNumGridX-1;
        int jmax = aYIndex+mNeighborSearchRadius;
        if(jmax > (aNumGridY-1))
            jmax = aNumGridY-1;
        int kmax=2*mNeighborSearchRadius;
        if(aZLayer + mNeighborSearchRadius > aNumZLayers - 1)
            kmax = aNumZLayers - (aZLayer - 1);
        double weightedAverageDensity = 0.0;
        double weightedAverageDot = 0.0;
        double sumDensityWeights = 0.0;
        double sumDotWeights = 0.0;
        int numFound=0;
        aHasInterface = false;
        for(int k=kmin; k<=kmax; k++)
        {
            for(int i=imin; i<=imax; i++)
            {
                for(int j=jmin; j<=jmax; j++)
                {
                    if(i!=aXIndex || j!=aYIndex || k!=mNeighborSearchRadius)
                    {
                        if(aVoxelData[k][i][j].setByNode) // was set by a node being there
                        {
                            numFound++;
                            double distance = abs(i-aXIndex) + abs(j-aYIndex) + abs(k-mNeighborSearchRadius);
                            double weight=1.0/distance;
                            weightedAverageDensity += (aVoxelData[k][i][j].maxDensity * weight);
                            if(aVoxelData[k][i][j].hasInterface || aVoxelData[k][i][j].maxIsDesign)
                            {
                                weightedAverageDot += (aVoxelData[k][i][j].maxDot * weight);
                                sumDotWeights += weight;
                                aHasInterface = true;
                            }
                            sumDensityWeights += weight;
                        }
                    }
                }
            }
        }
        if(numFound > .1)
        {
            aDensity = weightedAverageDensity/sumDensityWeights;
            if(aHasInterface)
                aDot = weightedAverageDot/sumDotWeights;
        }
        else
            std::cout << "****** Didn't find any voxel neighbors for interpolating density values! *********" << std::endl;
//        std::cout << "Number of neighbors found: " << numFound << std::endl;
    }
}

void SupportStructure::setVoxelDataAndSupportStructure(stk::mesh::Entity aNode,
                                                       int nodeX, int nodeY,
                                                       std::vector<std::vector<VoxelData> > &aVoxelData,
                                                       double &aDensity,
                                                       double &aDot,
                                                       bool &aHasInterface)
{
    double tolerance = 1e-12;

    aVoxelData[nodeX][nodeY].hasInterface = aHasInterface;
    if(aVoxelData[nodeX][nodeY].maxDensity < 0.0)
    {
        // nothing set yet
        aVoxelData[nodeX][nodeY].maxDensity = aDensity;
        if((aDensity + tolerance) >= mDesignFieldThresholdValue)
        {
            aVoxelData[nodeX][nodeY].inDesignRegion = true;
            aVoxelData[nodeX][nodeY].maxIsDesign = true;
            aVoxelData[nodeX][nodeY].maxDot = aDot;
        }
        if(aNode.m_value != 0)
        {
            double supportStructureValue = calculateSupportStructureValue(aVoxelData[nodeX][nodeY].maxDensity,
                                                                          aVoxelData[nodeX][nodeY].maxDensity,
                                                                          aDot, false);
            mSTKMeshIn->setSupportStructureFieldValue(aNode.m_value, supportStructureValue);
        }
    }
    else
    {
        if(aVoxelData[nodeX][nodeY].inDesignRegion)
        {
            if((aDensity + tolerance) >= mDesignFieldThresholdValue)
            {
                aVoxelData[nodeX][nodeY].maxDensity = aDensity;
                aVoxelData[nodeX][nodeY].maxDot = aDot;
                if(aNode.m_value != 0)
                {
                    double supportStructureValue = calculateSupportStructureValue(aDensity,
                                                                                  aDensity,
                                                                                  aDot, false);
                    mSTKMeshIn->setSupportStructureFieldValue(aNode.m_value, supportStructureValue);
                }
            }
            else
            {
                aVoxelData[nodeX][nodeY].inDesignRegion = false;
                if(aNode.m_value != 0)
                {
                    double supportStructureValue = calculateSupportStructureValue(aDensity,
                                                                                  aVoxelData[nodeX][nodeY].maxDensity,
                                                                                  aVoxelData[nodeX][nodeY].maxDot, true);
                    mSTKMeshIn->setSupportStructureFieldValue(aNode.m_value, supportStructureValue);
                }
            }
        }
        else
        {
            if((aDensity + tolerance) >= mDesignFieldThresholdValue)
            {
                aVoxelData[nodeX][nodeY].inDesignRegion = true;
                aVoxelData[nodeX][nodeY].maxIsDesign = true;
                aVoxelData[nodeX][nodeY].maxDensity = aDensity;
                aVoxelData[nodeX][nodeY].maxDot = aDot;
                if(aNode.m_value != 0)
                {
                    double supportStructureValue = calculateSupportStructureValue(aDensity,
                                                                                  aDensity,
                                                                                  aDot, false);
                    mSTKMeshIn->setSupportStructureFieldValue(aNode.m_value, supportStructureValue);
                }
            }
            else
            {
                if(aVoxelData[nodeX][nodeY].maxIsDesign)
                {

                    if(aNode.m_value != 0)
                    {
                        double supportStructureValue = calculateSupportStructureValue(aDensity,
                                                                                      aVoxelData[nodeX][nodeY].maxDensity,
                                                                                      aVoxelData[nodeX][nodeY].maxDot, true);
                        mSTKMeshIn->setSupportStructureFieldValue(aNode.m_value, supportStructureValue);
                    }
                }
                else
                {
                    if(aDensity > aVoxelData[nodeX][nodeY].maxDensity)
                        aVoxelData[nodeX][nodeY].maxDensity = aDensity;
                    if(aNode.m_value != 0)
                    {
                        double supportStructureValue = calculateSupportStructureValue(aDensity,
                                                                                      aVoxelData[nodeX][nodeY].maxDensity,
                                                                                      aVoxelData[nodeX][nodeY].maxDot, false);
                        mSTKMeshIn->setSupportStructureFieldValue(aNode.m_value, supportStructureValue);
                    }
                }
            }
        }
    }
}

bool SupportStructure::runPrivateVoxelBased()
{
    // Get the build direction.
    std::string workingString = mBuildPlateNormalString;
    size_t spacePos = workingString.find(' ');
    int cntr = 0;
    while(spacePos != std::string::npos)
    {
        std::string cur_string = workingString.substr(0,spacePos);
        workingString = workingString.substr(spacePos+1);
        mBuildPlateNormal[cntr] = std::atof(cur_string.c_str());
        cntr++;
        spacePos = workingString.find(' ');
    }
    mBuildPlateNormal[cntr] = std::atof(workingString.c_str());
    mBuildPlateNormal.normalize();

    // Define a coordinate system with z axis being the
    // build plate normal and x and y defined arbitrarily.
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Calculating coordinate system based on build plate normal. " << std::endl;
    Vector3D xAxis, yAxis, zAxis, origin;
    if(!getBuildPlateCoordinateSystem(xAxis, yAxis, zAxis, origin))
    {
        // error message
        return false;
    }

    // Sort nodes in z direction.
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Sorting nodes by distance in build plate direction. " << std::endl;
    std::vector<std::pair<stk::mesh::Entity, double> > sortedNodeDistancePairs;
    double zMin, zMax;
    getNodesSortedInZDirection(origin, zAxis, sortedNodeDistancePairs, zMin, zMax);

    // Get the average edge size
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Calculating average edge length. " << std::endl;
    double averageEdgeLength;
    getAverageEdgeLength(averageEdgeLength);
    averageEdgeLength *= mCellSizeMultiplier;

    // Setup up voxel layer based on the average mesh size.
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Set up voxel dimensions for traversal. " << std::endl;
    int numGridX, numGridY;
    double gridSizeX, gridSizeY;
    Vector3D minCoords, maxCoords;
    getGridDimensions(averageEdgeLength, origin, xAxis, yAxis, numGridX, numGridY, minCoords, maxCoords,
                      gridSizeX, gridSizeY);
    /*
    std::vector<std::vector<VoxelData> > voxelData(numGridX, std::vector<VoxelData>(numGridY));
    for(int i=0; i<numGridX; ++i)
    {
        for(int j=0; j<numGridY; ++j)
        {
            voxelData[i][j].inDesignRegion = false;
            voxelData[i][j].maxDensity = -1.0;
            voxelData[i][j].maxDot = 0.0;
            voxelData[i][j].maxIsDesign = false;
            voxelData[i][j].setByNode = false;
        }
    }
    */

    // For nodes that are in elements that contain the design/void interface
    // going through them calculate an interface angle to be used to
    // compare against the critical overhang angle.
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Calculating interface angles for all nodes. " << std::endl;
    std::map<stk::mesh::Entity, double> nodeInterfaceAngles;
    getNodeInterfaceData(nodeInterfaceAngles);

    // Bump the z extents a bit to avoid tolerance issues.
    zMax += 1e-4;
    zMin -= 1e-4;
    int numZLayers = (int)(((zMax-zMin)/averageEdgeLength) + 1.0);
    double zStep = (zMax - zMin) / (double)numZLayers;
    double curZ = zMax - zStep;
    std::cout << "Number grid x: " << numGridX << std::endl;
    std::cout << "Number grid y: " << numGridY << std::endl;
    std::cout << "Number grid z: " << numZLayers << std::endl;

    // New approach with 3D vector
    // 3D vector: 1st index is the z layer, 2nd is x, and 3rd is y
    int voxelNumLayers = 2*mNeighborSearchRadius + 1;
    std::vector<std::vector<std::vector<VoxelData> > > voxelData3D(voxelNumLayers,
                         std::vector<std::vector<VoxelData> >(numGridX,
                         std::vector<VoxelData>(numGridY)));
    for(int i=0; i<voxelNumLayers; ++i)
    {
        for(int j=0; j<numGridX; ++j)
        {
            for(int k=0; k<numGridY; ++k)
            {
                voxelData3D[i][j][k].dataExists = false;
            }
        }
    }

    int voxelMax = mNeighborSearchRadius;
    int voxelMemoryOffset = mNeighborSearchRadius;
    for(int zLayer=0; zLayer < numZLayers; ++zLayer)
    {
        std::cout << "Processing Layer: " << zLayer << std::endl;
        if(zLayer == 0)
        {
            curZ = zMax - zStep;
            for(int g=0; g <= voxelMax; ++g)
            {
                for(int i=0; i<numGridX; ++i)
                    for(int j=0; j<numGridY; ++j)
                        voxelData3D[voxelMemoryOffset+g][i][j].setByNode = false;

                bool done = false;
                while(!done)
                {
                    if(sortedNodeDistancePairs.size() > 0)
                    {
                        stk::mesh::Entity curNode = sortedNodeDistancePairs.back().first;
                        double nodeZValue = sortedNodeDistancePairs.back().second;
                        if(nodeZValue > curZ)
                        {
                            sortedNodeDistancePairs.pop_back();
                            int nodeX, nodeY;
                            getNodeXY(curNode, origin, xAxis, yAxis, minCoords, gridSizeX, gridSizeY,
                                      numGridX, numGridY, nodeX, nodeY);
                            setVoxelData(curNode, nodeX, nodeY, numGridX, numGridY, voxelData3D, nodeInterfaceAngles, 0, 0, voxelMemoryOffset+g);
                            /*
                            double curNodeDensity, curNodeDot=0.0;
                            bool hasInterface;
                            getValsForSettingVoxel(curNode, nodeX, nodeY, numGridX, numGridY, voxelData3D[voxelMemoryOffset+g],
                                                   nodeInterfaceAngles,
                                                   curNodeDensity, curNodeDot, hasInterface);
                            setVoxelDataAndSupportStructure(curNode, nodeX, nodeY, voxelData3D[voxelMemoryOffset+g],
                                                            curNodeDensity, curNodeDot, hasInterface);
                            voxelData3D[voxelMemoryOffset+g][nodeX][nodeY].setByNode = true;
                            voxelData3D[voxelMemoryOffset+g][nodeX][nodeY].dataExists = true;
                            */
                        }
                        else
                            done = true;
                    }
                    else
                        done = true;
                }
                curZ -= zStep;
            }
        }
        else
        {
            int curZLayer = zLayer + mNeighborSearchRadius;
            if(curZLayer < numZLayers)
            {
                curZ = zMax - (curZLayer+1)*zStep;

                // Shift voxel memory by 1
                for(int h=0; h<(voxelNumLayers-1); ++h)
                    voxelData3D[h] = voxelData3D[h+1];

                int voxelLayer = voxelNumLayers-1;

                for(int i=0; i<numGridX; ++i)
                    for(int j=0; j<numGridY; ++j)
                        voxelData3D[voxelLayer][i][j].setByNode = false;

                bool done = false;
                while(!done)
                {
                    if(sortedNodeDistancePairs.size() > 0)
                    {
                        stk::mesh::Entity curNode = sortedNodeDistancePairs.back().first;
                        double nodeZValue = sortedNodeDistancePairs.back().second;
                        if(nodeZValue > curZ)
                        {
                            Vector3D p0;
                            mSTKMeshIn->nodeCoordinates(curNode, p0.data());
                            if(fabs(p0[0]+.135930) < .0001 && fabs(p0[1]+.545187) < .0001 && fabs(p0[2]+.013294) < .0001)
                            {
                                int tr=0;
                                tr++;
                            }
                            sortedNodeDistancePairs.pop_back();
                            int nodeX, nodeY;
                            getNodeXY(curNode, origin, xAxis, yAxis, minCoords, gridSizeX, gridSizeY,
                                      numGridX, numGridY, nodeX, nodeY);
                            if(nodeX == 32 && nodeY == 29)
                            {
                                int gh=0;
                                gh++;
                            }
                            setVoxelData(curNode, nodeX, nodeY, numGridX, numGridY, voxelData3D, nodeInterfaceAngles, 0, 0, voxelLayer);
                            /*
                            double curNodeDensity, curNodeDot=0.0;
                            bool hasInterface;
                            getValsForSettingVoxel(curNode, nodeX, nodeY, numGridX, numGridY, voxelData3D[voxelLayer], nodeInterfaceAngles,
                                                   curNodeDensity, curNodeDot, hasInterface);
                            setVoxelDataAndSupportStructure(curNode, nodeX, nodeY, voxelData3D[voxelLayer],
                                                            curNodeDensity, curNodeDot, hasInterface);
                            voxelData3D[voxelLayer][nodeX][nodeY].setByNode = true;
                            voxelData3D[voxelLayer][nodeX][nodeY].dataExists = true;
                            */
                        }
                        else
                            done = true;
                    }
                    else
                        done = true;
                }
            }
        }
        int numNotSet=0;
        int voxelLayer = mNeighborSearchRadius;
        for(int i=0; i<numGridX; ++i)
        {
            for(int j=0; j<numGridY; ++j)
            {
                if(voxelData3D[voxelLayer][i][j].setByNode == false)
                {
                    if(i == 32 && j == 29)
                    {
                        int gh=0;
                        gh++;
                    }
                    stk::mesh::Entity dummyNode;
                    dummyNode.m_value = 0;
                    setVoxelData(dummyNode, i, j, numGridX, numGridY, voxelData3D, nodeInterfaceAngles, zLayer, numZLayers, -1);
                    /*
                    double curDensity;
                    double curDot = 0.0;
                    bool hasInterface;
                    getValsForSettingVoxel3D(dummyNode, i, j, numGridX, numGridY, voxelData3D, nodeInterfaceAngles,
                                           curDensity, curDot, hasInterface, zLayer, numZLayers);
                    setVoxelDataAndSupportStructure(dummyNode, i, j, voxelData3D[voxelLayer],
                                                    curDensity, curDot, hasInterface);
                                                    */
                    numNotSet++;
                }
            }
        }
        std::cout << "Num voxels not set: " << numNotSet << std::endl;
    }

#if 0
    bool firstLayer = true;
    while(curZ > zMin)
    {
        for(int i=0; i<numGridX; ++i)
            for(int j=0; j<numGridY; ++j)
                voxelData[i][j].setByNode = false;

        bool done = false;
        while(!done)
        {
            if(sortedNodeDistancePairs.size() > 0)
            {
                stk::mesh::Entity curNode = sortedNodeDistancePairs.back().first;
                double nodeZValue = sortedNodeDistancePairs.back().second;
                if(nodeZValue > curZ)
                {
                    sortedNodeDistancePairs.pop_back();
                    int nodeX, nodeY;
                    getNodeXY(curNode, origin, xAxis, yAxis, minCoords, gridSizeX, gridSizeY,
                              numGridX, numGridY, nodeX, nodeY);
                    double curNodeDensity, curNodeDot=0.0;
                    bool hasInterface;
                    getValsForSettingVoxel(curNode, nodeX, nodeY, numGridX, numGridY, voxelData, nodeInterfaceAngles,
                                           curNodeDensity, curNodeDot, hasInterface);
                    setVoxelDataAndSupportStructure(curNode, nodeX, nodeY, voxelData,
                                                    curNodeDensity, curNodeDot, hasInterface);
                    voxelData[nodeX][nodeY].setByNode = true;
                }
                else
                    done = true;
            }
            else
                done = true;
        }
        for(int i=0; i<numGridX; ++i)
        {
            for(int j=0; j<numGridY; ++j)
            {
                if(voxelData[i][j].setByNode == false)
                {
                    stk::mesh::Entity dummyNode;
                    dummyNode.m_value = 0;
                    double curDensity;
                    double curDot = 0.0;
                    bool hasInterface;
                    getValsForSettingVoxel(dummyNode, i, j, numGridX, numGridY, voxelData, nodeInterfaceAngles,
                                           curDensity, curDot, hasInterface);
                    setVoxelDataAndSupportStructure(dummyNode, i, j, voxelData,
                                                    curDensity, curDot, hasInterface);
                }
            }
        }
        curZ -= zStep;
        firstLayer = false;
        // Communicate boundary node info if running in parallel
    }
#endif
    mSTKMeshIn->write_exodus_mesh(mMeshOut, mConcatenateResults);
    return true;
}

bool SupportStructure::runPrivateVoxelBasedInefficientMemory()
{
    // Get the build direction.
    std::string workingString = mBuildPlateNormalString;
    size_t spacePos = workingString.find(' ');
    int cntr = 0;
    while(spacePos != std::string::npos)
    {
        std::string cur_string = workingString.substr(0,spacePos);
        workingString = workingString.substr(spacePos+1);
        mBuildPlateNormal[cntr] = std::atof(cur_string.c_str());
        cntr++;
        spacePos = workingString.find(' ');
    }
    mBuildPlateNormal[cntr] = std::atof(workingString.c_str());
    mBuildPlateNormal.normalize();

    // Define a coordinate system with z axis being the
    // build plate normal and x and y defined arbitrarily.
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Calculating coordinate system based on build plate normal. " << std::endl;
    Vector3D xAxis, yAxis, zAxis, origin;
    if(!getBuildPlateCoordinateSystem(xAxis, yAxis, zAxis, origin))
    {
        // error message
        return false;
    }

    // Sort nodes in z direction.
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Sorting nodes by distance in build plate direction. " << std::endl;
    std::vector<std::pair<stk::mesh::Entity, double> > sortedNodeDistancePairs;
    double zMin, zMax;
    getNodesSortedInZDirection(origin, zAxis, sortedNodeDistancePairs, zMin, zMax);

    // Get the average edge size
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Calculating average edge length. " << std::endl;
    double averageEdgeLength;
    getAverageEdgeLength(averageEdgeLength);
    averageEdgeLength *= mCellSizeMultiplier;

    // Setup up voxel layer based on the average mesh size.
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Set up voxel dimensions for traversal. " << std::endl;
    int numGridX, numGridY;
    double gridSizeX, gridSizeY;
    Vector3D minCoords, maxCoords;
    getGridDimensions(averageEdgeLength, origin, xAxis, yAxis, numGridX, numGridY, minCoords, maxCoords,
                      gridSizeX, gridSizeY);

    // For nodes that are in elements that contain the design/void interface
    // going through them calculate an interface angle to be used to
    // compare against the critical overhang angle.
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Calculating interface angles for all nodes. " << std::endl;
    std::map<stk::mesh::Entity, double> nodeInterfaceAngles;
    getNodeInterfaceData(nodeInterfaceAngles);

    // Bump the z extents a bit to avoid tolerance issues.
    zMax += 1e-4;
    zMin -= 1e-4;
    int numZLayers = (int)(((zMax-zMin)/averageEdgeLength) + 1.0);
    double zStep = (zMax - zMin) / (double)numZLayers;
    double curZ = zMax - zStep;
    std::cout << "Number grid x: " << numGridX << std::endl;
    std::cout << "Number grid y: " << numGridY << std::endl;
    std::cout << "Number grid z: " << numZLayers << std::endl;

    // New approach with 3D vector
    // 3D vector: 1st index is the z layer, 2nd is x, and 3rd is y
    std::vector<std::vector<std::vector<VoxelData> > > voxelData3D(numZLayers,
                         std::vector<std::vector<VoxelData> >(numGridX,
                         std::vector<VoxelData>(numGridY)));

    for(int i=0; i<numZLayers; ++i)
    {
        for(int j=0; j<numGridX; ++j)
        {
            for(int k=0; k<numGridY; ++k)
            {
                voxelData3D[i][j][k].dataExists = false;
            }
        }
    }

    // First get all the node data and store it in the voxel data
    std::vector<std::pair<stk::mesh::Entity, double> >::reverse_iterator riter = sortedNodeDistancePairs.rbegin();

    for(int zLayer=0; zLayer < numZLayers; ++zLayer)
    {
        curZ = zMax - (zLayer+1)*zStep;
        bool done = false;
        while(!done)
        {
            if(riter != sortedNodeDistancePairs.rend())
            {
                stk::mesh::Entity curNode = riter->first;
                double nodeZValue = riter->second;
                if(nodeZValue > curZ)
                {
                    riter++;
                    int nodeX, nodeY;
                    getNodeXY(curNode, origin, xAxis, yAxis, minCoords, gridSizeX, gridSizeY,
                              numGridX, numGridY, nodeX, nodeY);
                    setVoxelNodeData(curNode, nodeX, nodeY, voxelData3D, nodeInterfaceAngles, zLayer);
                }
                else
                    done = true;
            }
            else
                done = true;
        }
    }

    for(int zLayer=0; zLayer < numZLayers; ++zLayer)
    {
        std::cout << "Layer: " << zLayer << std::endl;
        int numNotSetByNodes=0;
        for(int i=0; i<numGridX; ++i)
        {
            for(int j=0; j<numGridY; ++j)
            {
                if(i==16 && j==13)
                {
                    int hh=0;
                    ++hh;
                }
                if(voxelData3D[zLayer][i][j].setByNode == true)
                {
                    setVoxelDataByNode(i, j, voxelData3D, zLayer);
                }
                else
                {
                    setVoxelDataByNeighbor(i, j, numGridX, numGridY, voxelData3D, zLayer, numZLayers);
                    numNotSetByNodes++;
                }
            }
        }
        std::cout << "Num voxels without nodes: " << numNotSetByNodes << std::endl;
    }

    mSTKMeshIn->write_exodus_mesh(mMeshOut, mConcatenateResults);
    return true;
}

bool SupportStructure::runPrivateNodeBasedMaxDensityAboveTopDown()
{
    // Get the build direction.
    std::string workingString = mBuildPlateNormalString;
    size_t spacePos = workingString.find(' ');
    int cntr = 0;
    while(spacePos != std::string::npos)
    {
        std::string cur_string = workingString.substr(0,spacePos);
        workingString = workingString.substr(spacePos+1);
        mBuildPlateNormal[cntr] = std::atof(cur_string.c_str());
        cntr++;
        spacePos = workingString.find(' ');
    }
    mBuildPlateNormal[cntr] = std::atof(workingString.c_str());
    mBuildPlateNormal.normalize();

    // Calculate the interface angle for all interface elements and store in a map
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Calculating interface angle for all elements. " << std::endl;
    stk::mesh::Selector myElemSelector = mSTKMeshIn->meta_data()->universal_part();
    stk::mesh::BucketVector const &elementBuckets = mSTKMeshIn->bulk_data()->get_buckets(
            stk::topology::ELEMENT_RANK, myElemSelector );

    if(elementBuckets.size() == 0)
    {
        std::cout << "Failed to find any elements." << std::endl;
        return false;
    }

    double tolerance = 1e-12;
    std::set<stk::mesh::Entity> designInterfaceElements;
    std::map<stk::mesh::Entity, Vector3D> interfaceMap;
    for ( stk::mesh::BucketVector::const_iterator bucketIter = elementBuckets.begin();
            bucketIter != elementBuckets.end(); ++bucketIter )
    {
        stk::mesh::Bucket &tmpBucket = **bucketIter;
        size_t numElems = tmpBucket.size();
        for (size_t i=0; i<numElems; ++i)
        {
            bool allNodesInside = true;
            bool allNodesOutside = true;
            stk::mesh::Entity curElem = tmpBucket[i];
            stk::mesh::Entity const *elemNodes = mSTKMeshIn->bulk_data()->begin_nodes(curElem);
            int numNodes = mSTKMeshIn->bulk_data()->num_nodes(curElem);
            for(int j=0; j<numNodes; ++j)
            {
                stk::mesh::Entity curNode = elemNodes[j];
                double designVariableValue = mSTKMeshIn->getMaxNodalIsoFieldVariable(curNode.m_value);
                if((designVariableValue + tolerance) >= mDesignFieldThresholdValue)
                    allNodesOutside = false;
                else
                    allNodesInside = false;
            }
            if(!allNodesInside && !allNodesOutside)
            {
                Vector3D interfaceNormal;
                Vector3D interfaceOrigin;
                std::vector<Vector3D> triPoints;
                if(getIntersectionInfo(curElem, FIELD_DENSITY, interfaceNormal, interfaceOrigin, triPoints))
                {
                    // See if the interface is pointing toward the build plate
                    double dot=interfaceNormal % mBuildPlateNormal;
                    // Just add elements to the map that will actually affect things later on.
                    if(dot < 0.0)
                    {
                        interfaceMap[curElem] = interfaceNormal;
                    }
                }
            }
        }
    }

    // For all nodes in interface elements calculate an average interface angle
    // from all of the attached interface elements.
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Calculating average interface angle for all nodes. " << std::endl;
    std::map<stk::mesh::Entity, double> nodeInterfaceAngles;
    std::map<stk::mesh::Entity, Vector3D>::iterator interfaceMapIter = interfaceMap.begin();
    while(interfaceMapIter != interfaceMap.end())
    {
        stk::mesh::Entity curElem = interfaceMapIter->first;
        stk::mesh::Entity const *elemNodes = mSTKMeshIn->bulk_data()->begin_nodes(curElem);
        int numNodes = mSTKMeshIn->bulk_data()->num_nodes(curElem);
        for(int j=0; j<numNodes; ++j)
        {
            stk::mesh::Entity curNode = elemNodes[j];
            if(nodeInterfaceAngles.count(curNode) == 0)
            {
                Vector3D p0;
                mSTKMeshIn->nodeCoordinates(curNode, p0.data());
                if(fabs(p0[0]) < 1e-6 &&
                        fabs(p0[1]+.5) < 1e-6 &&
                        fabs(p0[2]) < 1e-6)

                {
                    int p=0;
                    ++p;

                }
                stk::mesh::Entity const *nodeElements = mSTKMeshIn->bulk_data()->begin_elements(curNode);
                int numElems = mSTKMeshIn->bulk_data()->num_elements(curNode);
                double averageDot = 0.0;
                Vector3D averageNormal;
                averageNormal[0] = averageNormal[1] = averageNormal[2] = 0.0;
                double numInterfaceNeighbors = 0.0;
                for(int q=0; q<numElems; ++q)
                {
                    stk::mesh::Entity curNeighborElem = nodeElements[q];
                    std::map<stk::mesh::Entity, Vector3D>::iterator curNeighborIter = interfaceMap.find(curNeighborElem);
                    if(curNeighborIter != interfaceMap.end())
                    {
                        numInterfaceNeighbors += 1.0;
                     //   averageDot += curNeighborIter->second;
                        averageNormal += curNeighborIter->second;
                    }
                }
                if(numInterfaceNeighbors > 0.1)
                {
                    averageNormal.normalize();
                    averageDot = averageNormal % mBuildPlateNormal;
 //                   averageDot /= numInterfaceNeighbors;
                    nodeInterfaceAngles[curNode] = averageDot;
                }
            }
        }
        interfaceMapIter++;
    }

    // Calculating an average element length.
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Calculating an average edge length. " << std::endl;
    double averageEdgeLength = 0.0;
    double numOverall = 0.0;
    bool tetsExist = false;
    for ( stk::mesh::BucketVector::const_iterator bucketIter = elementBuckets.begin();
            bucketIter != elementBuckets.end(); ++bucketIter )
    {
        stk::mesh::Bucket &tmpBucket = **bucketIter;
        size_t numElems = tmpBucket.size();
        for (size_t i=0; i<numElems; ++i)
        {
            double curAverage = 0.0;
            stk::mesh::Entity curElem = tmpBucket[i];
            stk::mesh::Entity const *elemNodes = mSTKMeshIn->bulk_data()->begin_nodes(curElem);
            int numNodes = mSTKMeshIn->bulk_data()->num_nodes(curElem);
            if(numNodes == 8)
            {
                Vector3D p0, p1, p3, p4;
                mSTKMeshIn->nodeCoordinates(elemNodes[0], p0.data());
                mSTKMeshIn->nodeCoordinates(elemNodes[1], p1.data());
                mSTKMeshIn->nodeCoordinates(elemNodes[3], p3.data());
                mSTKMeshIn->nodeCoordinates(elemNodes[4], p4.data());
                curAverage += p0.distance(p1);
                curAverage += p0.distance(p3);
                curAverage += p0.distance(p4);
                curAverage /= 3.0;
            }
            else if(numNodes == 4)
            {
                Vector3D p0, p1, p2, p3;
                mSTKMeshIn->nodeCoordinates(elemNodes[0], p0.data());
                mSTKMeshIn->nodeCoordinates(elemNodes[1], p1.data());
                mSTKMeshIn->nodeCoordinates(elemNodes[2], p2.data());
                mSTKMeshIn->nodeCoordinates(elemNodes[3], p3.data());
                curAverage += p0.distance(p1);
                curAverage += p0.distance(p2);
                curAverage += p2.distance(p3);
                curAverage /= 3.0;
                tetsExist = true;
            }
            averageEdgeLength += curAverage;
            numOverall += 1.0;
        }
    }
    averageEdgeLength /= numOverall;

    stk::mesh::Selector myNodeSelector = mSTKMeshIn->meta_data()->universal_part();
    stk::mesh::BucketVector const &nodeBuckets = mSTKMeshIn->bulk_data()->get_buckets(
            stk::topology::NODE_RANK, myNodeSelector );

    if(nodeBuckets.size() == 0)
    {
        std::cout << "Failed to find any nodes." << std::endl;
        return false;
    }

    // Loop through all the nodes and sort them based on how far they are from
    // the build plate.
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Sorting nodes by distance from the build plate. " << std::endl;
    stk::mesh::Entity refNode = (**(nodeBuckets.begin()))[0];
    Vector3D refNodeCoords;
    mSTKMeshIn->nodeCoordinates(refNode, refNodeCoords.data());

    std::vector<std::pair<stk::mesh::Entity, double> > nodeDotPairs;
    for ( stk::mesh::BucketVector::const_iterator nodeBucketIter = nodeBuckets.begin();
            nodeBucketIter != nodeBuckets.end();
            ++nodeBucketIter )
    {
        stk::mesh::Bucket &tmpBucket = **nodeBucketIter;
        size_t numBucketNodes = tmpBucket.size();
        for (size_t i=0; i<numBucketNodes; ++i)
        {
            stk::mesh::Entity curNode = tmpBucket[i];
            Vector3D curNodeCoords;
            mSTKMeshIn->nodeCoordinates(curNode, curNodeCoords.data());
            Vector3D curVec = curNodeCoords - refNodeCoords;
            double dot = curVec % mBuildPlateNormal;
            nodeDotPairs.push_back(std::make_pair(curNode, dot));
        }
    }
    std::sort(nodeDotPairs.begin(), nodeDotPairs.end(), comparePairs);

    // Do the main process loop. For each node in the mesh search
    // "upward" to determine the driving density node/value and
    // then calculate an appropriate support structure value.
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Calculating support structure value for every node. " << std::endl;
    double coarseSearchRadiusSquared = 1.5*1.5*averageEdgeLength*averageEdgeLength;
    double fineSearchRadiusSquared = .5*.5*averageEdgeLength*averageEdgeLength;
    if(tetsExist)
    {
        coarseSearchRadiusSquared = 5*5*averageEdgeLength*averageEdgeLength;
        fineSearchRadiusSquared = 0.5*0.5*averageEdgeLength*averageEdgeLength;
    }
    std::set<stk::mesh::Entity> processedNodes;
    for(int e=nodeDotPairs.size()-1; e>-1; --e)
    {
        stk::mesh::Entity curNode = nodeDotPairs[e].first;

        if(processedNodes.find(curNode) != processedNodes.end())
            continue;
        processedNodes.insert(curNode);
        double maxDensity = mSTKMeshIn->getMaxNodalIsoFieldVariable(curNode.m_value);
        double maxDot = 0.0;
        bool inDesignRegion = false;
        bool maxIsDesign = false;
        // If the current node is in the design...
        if((maxDensity + tolerance) >= mDesignFieldThresholdValue)
        {
            inDesignRegion = true;
            maxIsDesign = true;
            maxDot = nodeInterfaceAngles[curNode];
        }
        // Calculate the support structure field value for this node and set it.
        double supportStructureValue= calculateSupportStructureValue(maxDensity, maxDensity, maxDot, false);
        mSTKMeshIn->setSupportStructureFieldValue(curNode.m_value, supportStructureValue);

        Vector3D curNodeCoords;
        mSTKMeshIn->nodeCoordinates(curNode, curNodeCoords.data());
        Vector3D lastNodeCoords = curNodeCoords;

        // Get the next node "below" this node
        std::set<stk::mesh::Entity> visitedNodes;
        visitedNodes.insert(curNode);
        std::set<stk::mesh::Entity> newElementsToCheck;
        std::set<stk::mesh::Entity> elemsToCheck;
        stk::mesh::Entity const *nodeElements = mSTKMeshIn->bulk_data()->begin_elements(curNode);
        int numElems = mSTKMeshIn->bulk_data()->num_elements(curNode);
        for(int q=0; q<numElems; ++q)
            elemsToCheck.insert(nodeElements[q]);

        bool done = false;
        while(!done)
        {
            bool foundBelowNode = false;
            while(elemsToCheck.size() && !foundBelowNode)
            {
                stk::mesh::Entity curNeighborElem = *(elemsToCheck.begin());
                elemsToCheck.erase(elemsToCheck.begin());

                stk::mesh::Entity const *elemNodes = mSTKMeshIn->bulk_data()->begin_nodes(curNeighborElem);
                int numElemNodes = mSTKMeshIn->bulk_data()->num_nodes(curNeighborElem);
                for(int j=0; j<numElemNodes && !foundBelowNode; ++j)
                {
                    stk::mesh::Entity curNeighborNode = elemNodes[j];
                    if(visitedNodes.find(curNeighborNode) == visitedNodes.end())
                    {
                        visitedNodes.insert(curNeighborNode);
                        Vector3D curNeighborCoords;
                        mSTKMeshIn->nodeCoordinates(curNeighborNode, curNeighborCoords.data());
                        Vector3D curVec = curNeighborCoords - lastNodeCoords;
                        // Make sure node is in the right direction
                        if(curVec % mBuildPlateNormal < 0.0)
                        {
                            // Now check to see if the node is within the fine search range
                            Vector3D curNeighborOnPlane;
                            projectPointToPlane(curNeighborCoords, curNodeCoords, mBuildPlateNormal, curNeighborOnPlane);
                            if(curNeighborOnPlane.distanceSquared(curNodeCoords) < fineSearchRadiusSquared)
                            {
                                if(fabs(curNeighborCoords[0]) < 1e-6 &&
                                        fabs(curNeighborCoords[1]+.5) < 1e-6 &&
                                        fabs(curNeighborCoords[2]) < 1e-6)

                                {
                                    int p=0;
                                    ++p;

                                }
                                if(fabs(curNeighborCoords[0]+0.161868) < 1e-6 &&
                                        fabs(curNeighborCoords[1]+0.508902) < 1e-6 &&
                                        fabs(curNeighborCoords[2]+0.110684) < 1e-6)

                                {
                                    int p=0;
                                    ++p;

                                }
                                // We found a valid node "below the current node".
                                processedNodes.insert(curNeighborNode);
                                foundBelowNode = true;
                                lastNodeCoords = curNeighborCoords;
                                double curNeighborDensity = mSTKMeshIn->getMaxNodalIsoFieldVariable(curNeighborNode.m_value);
                                if(inDesignRegion)
                                {
                                    // If the current node is in the design
                                    if((curNeighborDensity + tolerance) >= mDesignFieldThresholdValue)
                                    {
                                        maxDensity = curNeighborDensity;
                                        maxDot = nodeInterfaceAngles[curNeighborNode];
                                        double supportStructureValue = calculateSupportStructureValue(curNeighborDensity,
                                                                                                      curNeighborDensity, maxDot, false);
                                        mSTKMeshIn->setSupportStructureFieldValue(curNeighborNode.m_value, supportStructureValue);
                                    }
                                    else
                                    {
                                        inDesignRegion = false;
                                        double supportStructureValue = calculateSupportStructureValue(curNeighborDensity,
                                                                                                      maxDensity, maxDot, true);
                                        mSTKMeshIn->setSupportStructureFieldValue(curNeighborNode.m_value, supportStructureValue);
                                    }
                                }
                                else
                                {
                                    // If the current node is in the design
                                    if((curNeighborDensity + tolerance) >= mDesignFieldThresholdValue)
                                    {
                                        inDesignRegion = true;
                                        maxIsDesign = true;
                                        maxDensity = curNeighborDensity;
                                        maxDot = nodeInterfaceAngles[curNeighborNode];
                                        double supportStructureValue = calculateSupportStructureValue(curNeighborDensity,
                                                                                                      curNeighborDensity, maxDot, false);
                                        mSTKMeshIn->setSupportStructureFieldValue(curNeighborNode.m_value, supportStructureValue);
                                    }
                                    else
                                    {
                                        if(maxIsDesign)
                                        {
                                            double supportStructureValue = calculateSupportStructureValue(curNeighborDensity,
                                                                                                          maxDensity, maxDot, true);
                                            mSTKMeshIn->setSupportStructureFieldValue(curNeighborNode.m_value, supportStructureValue);
                                        }
                                        else
                                        {
                                            if(curNeighborDensity > maxDensity)
                                                maxDensity = curNeighborDensity;
                                            double supportStructureValue = calculateSupportStructureValue(curNeighborDensity,
                                                                                                          maxDensity, maxDot, false);
                                            mSTKMeshIn->setSupportStructureFieldValue(curNeighborNode.m_value, supportStructureValue);
                                        }
                                    }
                                }
                                // Find next elements to check
                                stk::mesh::Entity const *tmpNodeElements = mSTKMeshIn->bulk_data()->begin_elements(curNeighborNode);
                                int tmpNumElems = mSTKMeshIn->bulk_data()->num_elements(curNeighborNode);
                                for(int w=0; w<tmpNumElems; ++w)
                                {
                                    if(tmpNodeElements[w] != curNeighborElem)
                                        newElementsToCheck.insert(tmpNodeElements[w]);
                                }
                            }
                            else
                            {
                                if(curNeighborOnPlane.distanceSquared(curNodeCoords) < coarseSearchRadiusSquared)
                                {
                                    stk::mesh::Entity const *tmpNodeElements = mSTKMeshIn->bulk_data()->begin_elements(curNeighborNode);
                                    int tmpNumElems = mSTKMeshIn->bulk_data()->num_elements(curNeighborNode);
                                    for(int w=0; w<tmpNumElems; ++w)
                                    {
                                        if(tmpNodeElements[w] != curNeighborElem)
                                            newElementsToCheck.insert(tmpNodeElements[w]);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if(newElementsToCheck.size() == 0)
                done = true;
            else
            {
                elemsToCheck = newElementsToCheck;
                newElementsToCheck.clear();
            }
        }
    }

    mSTKMeshIn->write_exodus_mesh(mMeshOut, mConcatenateResults);

    return true;
}

double SupportStructure::calculateSupportStructureValue(double &aMyDensity, double &aAboveDensity, double aNodeDot, bool aFoundDesignNode)
{
    double aCriticalDot = -cos(mOverhangAngle*platoPI/180.0);
    double angleCriteria = 1.0;

    // If in design set f(theta) to 1.0
    // If there is design above you with x>.5 defined evaluate f(theta) using theta from material interface above
    // If no design above set f(theta) to 1.0

//    double h=8.0;
    double h=10.0;
    if(!aFoundDesignNode)
        angleCriteria = 1.0;
    else
    {
        // I used dot values here rather than angles.
//        angleCriteria = 1.0 - log(1.0 + exp(1.0 - (aNodeDot/aCriticalDot)));
        angleCriteria = 1 + 0.5 * (tanh(-h*(aNodeDot-aCriticalDot))+tanh(h*(-1.0-aCriticalDot)));
//        angleCriteria = 2.0 + 1.0 * (tanh(-h*(aNodeDot-aCriticalDot))+tanh(h*(-1.0-aCriticalDot)));
    }


    double k=50.0;
    double valueBasedOnDensities = -log( exp(-k*aAboveDensity) + exp(k*(aMyDensity-1.0)) + exp(-k*angleCriteria) )/k;
    return valueBasedOnDensities;
//    return angleCriteria * valueBasedOnDensities;
//    return sqrt(angleCriteria * valueBasedOnDensities);


    //    return angleCriteria * aAboveDensity * (1.0 - aMyDensity);
//    return angleCriteria * sqrt(aAboveDensity*(1.0 - aMyDensity)) * (1.0 - aMyDensity) + (1.0 - aMyDensity) * aMyDensity;
}

bool SupportStructure::runPrivateProjectTriangle()
{
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Generating support material field. " << std::endl;

    stk::mesh::Selector mySelector = mSTKMeshIn->meta_data()->universal_part();
    stk::mesh::BucketVector const &elementBuckets = mSTKMeshIn->bulk_data()->get_buckets(
            stk::topology::ELEMENT_RANK, mySelector );

    if(elementBuckets.size() == 0)
    {
        std::cout << "Failed to find any elements." << std::endl;
        return false;
    }

    // Get the build plate normal and normalize it
    std::string workingString = mBuildPlateNormalString; // make a copy since we will be modifying it
    size_t spacePos = workingString.find(' ');
    int cntr = 0;
    while(spacePos != std::string::npos)
    {
        std::string cur_string = workingString.substr(0,spacePos);
        workingString = workingString.substr(spacePos+1);
        mBuildPlateNormal[cntr] = std::atof(cur_string.c_str());
        cntr++;
        spacePos = workingString.find(' ');
    }
    mBuildPlateNormal[cntr] = std::atof(workingString.c_str());
    mBuildPlateNormal.normalize();

    // Initialize support structure field nodes to (1-density) for
    // nodes that are in the design.
    // Initialize all other support structure nodes to the density
    // value at that node.
    double tolerance = 1e-12;
    std::set<stk::mesh::Entity> designInterfaceElements;
    for ( stk::mesh::BucketVector::const_iterator bucketIter = elementBuckets.begin();
            bucketIter != elementBuckets.end(); ++bucketIter )
    {
        stk::mesh::Bucket &tmpBucket = **bucketIter;
        size_t numElems = tmpBucket.size();
        for (size_t i=0; i<numElems; ++i)
        {
            bool allNodesInside = true;
            bool allNodesOutside = true;
            stk::mesh::Entity curElem = tmpBucket[i];
            stk::mesh::Entity const *elemNodes = mSTKMeshIn->bulk_data()->begin_nodes(curElem);
            int numNodes = mSTKMeshIn->bulk_data()->num_nodes(curElem);
            for(int j=0; j<numNodes; ++j)
            {
                stk::mesh::Entity curNode = elemNodes[j];
                double designVariableValue = mSTKMeshIn->getMaxNodalIsoFieldVariable(curNode.m_value);
                if((designVariableValue + tolerance) >= mDesignFieldThresholdValue)
                {
                    mSTKMeshIn->setSupportStructureFieldValue(curNode.m_value, (1.0-designVariableValue));
                    allNodesOutside = false;
                }
                else
                {
                    mSTKMeshIn->setSupportStructureFieldValue(curNode.m_value, designVariableValue);
                    allNodesInside = false;
                }
            }
            if(!allNodesInside && !allNodesOutside)
            {
                designInterfaceElements.insert(curElem);
            }
        }
    }
    double angleCosine = cos(mOverhangAngle*platoPI/180.0);
    std::set<stk::mesh::Entity>::iterator interfaceIter = designInterfaceElements.begin();
    while(interfaceIter != designInterfaceElements.end())
    {
        stk::mesh::Entity curElem = *interfaceIter;
        Vector3D interfaceNormal;
        Vector3D interfaceOrigin;
        std::vector<Vector3D> triPoints;
        if(getIntersectionInfo(curElem, FIELD_DENSITY, interfaceNormal, interfaceOrigin, triPoints))
        {
            // See if the interface is pointing toward the build plate
            double dot=interfaceNormal % mBuildPlateNormal;
            if(dot < -angleCosine)
            {
                // Get an edge length value to use in searching
                stk::mesh::Entity const *elemNodes = mSTKMeshIn->bulk_data()->begin_nodes(curElem);
                int numNodes = mSTKMeshIn->bulk_data()->num_nodes(curElem);
                Vector3D p1, p2;
                mSTKMeshIn->nodeCoordinates(elemNodes[0], p1.data());
                mSTKMeshIn->nodeCoordinates(elemNodes[1], p2.data());
//                double edgeLength = .75;
                double edgeLength = p1.distance(p2);

                // project all of the interface triangle points to a plane parallel to build plate
                for(size_t r=0; r<triPoints.size(); ++r)
                    projectPointToPlane(triPoints[r], interfaceOrigin, mBuildPlateNormal, triPoints[r]);

                std::set<stk::mesh::Entity> processedNodes;
                std::set<stk::mesh::Entity> elemsToCheck;
                for(int j=0; j<numNodes; ++j)
                {
                    stk::mesh::Entity curNode = elemNodes[j];
                    double designVariableValue = mSTKMeshIn->getMaxNodalIsoFieldVariable(curNode.m_value);
                    // If this node is not in the design...
                    if((designVariableValue + tolerance) < mDesignFieldThresholdValue)
                    {
                        // see if node is "below" the interface
                        // project the current node to a plane parallel to build plate

                        Vector3D planeOriginToPoint;
                        Vector3D pointCoords;
                        mSTKMeshIn->nodeCoordinates(curNode, pointCoords.data());

                        // First check if the node is on the right side of the interface
                        Vector3D interfaceToNode = pointCoords - interfaceOrigin;
                        if(interfaceToNode % mBuildPlateNormal < 0.0)
                        {
                            Vector3D pointOnPlane;
                            projectPointToPlane(pointCoords, interfaceOrigin, mBuildPlateNormal, pointOnPlane);
                            bool pointIsBelowInterface = false;
                            // see if the current node is in any of the triangles
                            for(size_t w=0; !pointIsBelowInterface && w<triPoints.size(); w+=3)
                            {
                                if(pointInTriangle(pointOnPlane, triPoints[w], triPoints[w+1], triPoints[w+2]))
                                    pointIsBelowInterface = true;
                            }
/*
                            if(pointIsBelowInterface)
                            {
                                mSTKMeshIn->setSupportStructureFieldValue(curNode.m_value, (1.0-designVariableValue));
                                stk::mesh::Entity const *nodeElements = mSTKMeshIn->bulk_data()->begin_elements(curNode);
                                int numElems = mSTKMeshIn->bulk_data()->num_elements(curNode);
                                for(int q=0; q<numElems; ++q)
                                {
                                    stk::mesh::Entity curNeighborElem = nodeElements[q];
                                    if(curNeighborElem != curElem)
                                    {
                                        uint64_t globalElemId2 = mSTKMeshIn->bulk_data()->identifier(curNeighborElem);
                                        elemsToCheck.insert(curNeighborElem);
                                    }
                                }
                            }
                            */
                            if(pointIsBelowInterface)
                            {
                                mSTKMeshIn->setSupportStructureFieldValue(curNode.m_value, (1.0-designVariableValue));
                            }
                            stk::mesh::Entity const *nodeElements = mSTKMeshIn->bulk_data()->begin_elements(curNode);
                            int numElems = mSTKMeshIn->bulk_data()->num_elements(curNode);
                            for(int q=0; q<numElems; ++q)
                            {
                                stk::mesh::Entity curNeighborElem = nodeElements[q];
                                if(curNeighborElem != curElem)
                                {
                                    elemsToCheck.insert(curNeighborElem);
                                }
                            }
                        }
                        processedNodes.insert(curNode);
                    }
                }
                // Now we will project this triangle all the way to the build plate or to design
                // material.
                std::set<stk::mesh::Entity> processedElements;
                double radius = 2*2*edgeLength*edgeLength;
                std::set<stk::mesh::Entity> newElemsToCheck;
                bool done = false;
                while(!done)
                {
                    // Add the current list of elems to check to the 'processed' list
                    // so we don't add them again below.
                    std::set<stk::mesh::Entity>::iterator it = elemsToCheck.begin();
                    while(it != elemsToCheck.end())
                    {
                        processedElements.insert(*it);
                        it++;
                    }

                    while(elemsToCheck.size())
                    {
                        stk::mesh::Entity curElem = *(elemsToCheck.begin());
                        elemsToCheck.erase(elemsToCheck.begin());

                        stk::mesh::Entity const *elemNodes = mSTKMeshIn->bulk_data()->begin_nodes(curElem);
                        int numNodes = mSTKMeshIn->bulk_data()->num_nodes(curElem);
                        for(int j=0; j<numNodes; ++j)
                        {
                            stk::mesh::Entity curNode = elemNodes[j];

                            if(processedNodes.find(curNode) != processedNodes.end())
                                continue;

                            processedNodes.insert(curNode);

                            Vector3D pointCoords;
                            mSTKMeshIn->nodeCoordinates(curNode, pointCoords.data());
                            Vector3D pointOnPlane;
                            projectPointToPlane(pointCoords, interfaceOrigin, mBuildPlateNormal, pointOnPlane);
                            // Check if this node is within the radius of our cylinder we are using to search
                            // for nodes in the "downward" direction.
                            if(pointOnPlane.distanceSquared(interfaceOrigin) < radius)
                            {
                                double designVariableValue = mSTKMeshIn->getMaxNodalIsoFieldVariable(curNode.m_value);
                                // If this node is not in the design...
                                if((designVariableValue + tolerance) < mDesignFieldThresholdValue)
                                {
                                    // First check if the node is on the right side of the interface
                                    Vector3D interfaceToNode = pointCoords - interfaceOrigin;
                                    if(interfaceToNode % mBuildPlateNormal < 0.0)
                                    {
                                        bool pointIsBelowInterface = false;
                                        // see if the current node is in any of the triangles
                                        for(size_t w=0; !pointIsBelowInterface && w<triPoints.size(); w+=3)
                                        {
                                            if(pointInTriangle(pointOnPlane, triPoints[w], triPoints[w+1], triPoints[w+2]))
                                                pointIsBelowInterface = true;
                                        }

                                        if(pointIsBelowInterface)
                                        {
                                            mSTKMeshIn->setSupportStructureFieldValue(curNode.m_value, (1.0-designVariableValue));
                                        }

                                        stk::mesh::Entity const *nodeElements = mSTKMeshIn->bulk_data()->begin_elements(curNode);
                                        int numElems = mSTKMeshIn->bulk_data()->num_elements(curNode);
                                        for(int q=0; q<numElems; ++q)
                                        {
                                            stk::mesh::Entity curNeighborElem = nodeElements[q];
                                            if(processedElements.find(curNeighborElem) == processedElements.end())
                                                newElemsToCheck.insert(curNeighborElem);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    if(newElemsToCheck.size() == 0)
                        done = true;
                    else
                    {
                        elemsToCheck = newElemsToCheck;
                        newElemsToCheck.clear();
                    }
                }
            }
        }
        interfaceIter++;
    }

    mSTKMeshIn->write_exodus_mesh(mMeshOut, mConcatenateResults);

    return true;
}

bool SupportStructure::runPrivateNodeBased()
{
    bool return_val = true;
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Generating support material field. " << std::endl;

    stk::mesh::Selector myselector = mSTKMeshIn->meta_data()->universal_part();
    stk::mesh::BucketVector const &node_buckets = mSTKMeshIn->bulk_data()->get_buckets(
            stk::topology::NODE_RANK, myselector );

    if(node_buckets.size() == 0)
    {
        std::cout << "Failed to find any nodes." << std::endl;
        return false;
    }

    // Get initial set of nodes to check
    std::vector<uint64_t> myNodesToCheck;
    for ( stk::mesh::BucketVector::const_iterator bucket_iter = node_buckets.begin();
            bucket_iter != node_buckets.end();
            ++bucket_iter )
    {
        stk::mesh::Bucket &tmp_bucket = **bucket_iter;
        size_t num_nodes = tmp_bucket.size();
        for (size_t i=0; i<num_nodes; ++i)
        {
            stk::mesh::Entity cur_node = tmp_bucket[i];
            uint64_t localId = cur_node.m_value;
            double val = mSTKMeshIn->getMaxNodalIsoFieldVariable(localId);
            if(val >= mDesignFieldThresholdValue)
            {
                myNodesToCheck.push_back(localId);
            }
            // Initialize all support structure values to 0.0
            mSTKMeshIn->setSupportStructureFieldValue(localId, 0.0);
        }
    }

    // Get the build plate normal
    std::string workingString = mBuildPlateNormalString; // make a copy since we will be modifying it
    size_t spacePos = workingString.find(' ');
    int cntr = 0;
    while(spacePos != std::string::npos)
    {
        std::string cur_string = workingString.substr(0,spacePos);
        workingString = workingString.substr(spacePos+1);
        mBuildPlateNormal[cntr] = std::atof(cur_string.c_str());
        cntr++;
        spacePos = workingString.find(' ');
    }
    mBuildPlateNormal[cntr] = std::atof(workingString.c_str());
    // Normalize the build plate normal in case it isn't already
    double mag = sqrt(mBuildPlateNormal[0]*mBuildPlateNormal[0] +
                      mBuildPlateNormal[1]*mBuildPlateNormal[1] +
                      mBuildPlateNormal[2]*mBuildPlateNormal[2]);
    mBuildPlateNormal[0] /= mag;
    mBuildPlateNormal[1] /= mag;
    mBuildPlateNormal[2] /= mag;


    std::vector<proc_node_map> procs;
    calculateSharingMaps(procs, mSTKMeshIn);

    std::set<uint64_t> mySupportMaterialNodes;
    std::set<uint64_t> myNewSupportMaterialNodes;
    std::vector<uint64_t> myNewNodesToCheck;
    int numProcs = mSTKMeshIn->bulk_data()->parallel_size();
    int myRank = mSTKMeshIn->bulk_data()->parallel_rank();
    bool done = false;
    size_t iterationCount = 1;
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
    {
        std::cout << "Starting to search for support structure nodes." << std::endl;
    }
    while(!done)
    {
        if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        {
            std::cout << "Iteration " << iterationCount << ":" << std::endl;
        }
        myNewNodesToCheck.clear();
        myNewSupportMaterialNodes.clear();
        while(myNodesToCheck.size() > 0)
        {
            uint64_t curNode = myNodesToCheck.back();
            myNodesToCheck.pop_back();
            uint64_t newSupportNodeLocalId;
            stk::mesh::Entity curNodeEntity = mSTKMeshIn->getStkEntity(curNode);
            if(nodeNeedsSupport(curNodeEntity, newSupportNodeLocalId, mySupportMaterialNodes))
            {
                myNewNodesToCheck.push_back(newSupportNodeLocalId);
                mySupportMaterialNodes.insert(newSupportNodeLocalId);
                mSTKMeshIn->setSupportStructureFieldValue(newSupportNodeLocalId, 1.0);
                stk::mesh::Entity newSupportNodeEntity = mSTKMeshIn->getStkEntity(newSupportNodeLocalId);
                uint64_t globalNodeId = mSTKMeshIn->bulk_data()->identifier(newSupportNodeEntity);
                myNewSupportMaterialNodes.insert(globalNodeId);
            }
        }

        // Get the counts of new support structure nodes from each processsor.
        int globalNewSupportNodeCounts[numProcs];
        int localNewSupportNodeCounts[numProcs];
        for(int i=0; i<numProcs; ++i)
        {
            globalNewSupportNodeCounts[i] = 0;
            localNewSupportNodeCounts[i] = 0;
        }
        localNewSupportNodeCounts[myRank] = myNewSupportMaterialNodes.size();

        MPI_Allreduce(localNewSupportNodeCounts, globalNewSupportNodeCounts, numProcs,
                      MPI_INT, MPI_SUM, mSTKMeshIn->bulk_data()->parallel());

        int d;
        bool newSupportNodesWereFound = false;
        for(d=0; d<numProcs; ++d)
        {
            if(globalNewSupportNodeCounts[d] > 0)
            {
                newSupportNodesWereFound = true;
                break;
            }
        }

        bool printData = true;
        if(printData)
        {
            // Now actually get the ids of the new support structure nodes from each processor
            // First allocate/initialize memory for ids from all the processors
            int* localNewSupportNodeIds[numProcs];
            int* globalNewSupportNodeIds[numProcs];
            for(d=0; d<numProcs; ++d)
            {
                if(globalNewSupportNodeCounts[d] > 0)
                {
                    localNewSupportNodeIds[d] = new int [globalNewSupportNodeCounts[d]];
                    globalNewSupportNodeIds[d] = new int [globalNewSupportNodeCounts[d]];
                    for(int e=0; e<globalNewSupportNodeCounts[d]; e++)
                    {
                        localNewSupportNodeIds[d][e] = 0;
                        globalNewSupportNodeIds[d][e] = 0;
                    }
                }
                else
                {
                    localNewSupportNodeIds[d] = NULL;
                    globalNewSupportNodeIds[d] = NULL;
                }
            }
            // Initialize the node id values from my processor
            std::set<uint64_t>::iterator idIterator = myNewSupportMaterialNodes.begin();
            d=0;
            while(idIterator != myNewSupportMaterialNodes.end())
            {
                localNewSupportNodeIds[myRank][d] = *idIterator;
                idIterator++;
                d++;
            }
            // Now communicate all the data
            for(d=0; d<numProcs; ++d)
            {
                if(globalNewSupportNodeCounts[d] > 0)
                {
                    // Only doing MPI_Reduce so proc 0 has all the data.
                    MPI_Reduce(localNewSupportNodeIds[d], globalNewSupportNodeIds[d], globalNewSupportNodeCounts[d],
                               MPI_INT, MPI_SUM, 0, mSTKMeshIn->bulk_data()->parallel());
                }
            }
            if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
            {
                std::set<int> uniqueNodeIdList;
                for(d=0; d<numProcs; ++d)
                {
                    for(int f=0; f<globalNewSupportNodeCounts[d]; f++)
                    {
                        uniqueNodeIdList.insert(globalNewSupportNodeIds[d][f]);
                    }
                }
                std::cout << "Total number of new support nodes on iteration " << iterationCount << ": " << uniqueNodeIdList.size() << std::endl;
                std::set<int>::iterator nodeIter = uniqueNodeIdList.begin();
                std::cout << "Node ids: ";
                while(nodeIter != uniqueNodeIdList.end())
                {
                    std::cout << *nodeIter << " ";
                    nodeIter++;
                }
                std::cout << std::endl;
            }

            // Now free all the data we dynamically allocated
            for(d=0; d<numProcs; ++d)
            {
                if(globalNewSupportNodeCounts[d] > 0)
                {
                    delete [] localNewSupportNodeIds[d];
                    delete [] globalNewSupportNodeIds[d];
                }
            }
        }

        if(!newSupportNodesWereFound)
        {
            done = true;
        }
        else
        {
            myNodesToCheck = myNewNodesToCheck;
            communicateBoundaryInfo(procs, myNewSupportMaterialNodes, mySupportMaterialNodes, myNodesToCheck);
        }

        iterationCount++;
    }
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
    {
        std::cout << "Total number of support " << std::endl;
    }

    mSTKMeshIn->write_exodus_mesh(mMeshOut, mConcatenateResults);

    return return_val;
}

void SupportStructure::communicateBoundaryInfo(std::vector<proc_node_map> &procs,
                                               std::set<uint64_t> &aNewSupportNodes,
                                               std::set<uint64_t> &aAllSupportNodes,
                                               std::vector<uint64_t> &aNodesToCheck)
{
    if(procs.size() > 0)
    {
        stk::CommSparse comm_spec(mSTKMeshIn->bulk_data()->parallel());
        for(int phase=0; phase<2; ++phase)
        {
            for(size_t i=0; i<procs.size(); ++i)
            {
                int other_proc = procs[i].processor;
                // pack the number of nodes
                comm_spec.send_buffer(other_proc).pack<size_t>(procs[i].nodes.size());
                for(size_t j=0; j<procs[i].nodes.size(); ++j)
                {
                    // pack the global node id and then the value for this node
                    comm_spec.send_buffer(other_proc).pack<uint64_t>(procs[i].nodes[j]);
                    int isSupportNode=0;
                    if(aNewSupportNodes.find(procs[i].nodes[j]) != aNewSupportNodes.end())
                        isSupportNode=1;
                    comm_spec.send_buffer(other_proc).pack<int>(isSupportNode);
                }
            }
            if ( phase == 0 )
            {
                comm_spec.allocate_buffers();
            }
            else
            {
                comm_spec.communicate();
            }
        }

        for(size_t i=0; i<procs.size(); ++i)
        {
            int other_proc = procs[i].processor;
            // get the number of nodes from the sending proc
            size_t num_nodes;
            comm_spec.recv_buffer(other_proc).unpack<size_t>(num_nodes);
            for(size_t j=0; j<num_nodes; ++j)
            {
                uint64_t globalNodeId;
                int node_val;
                comm_spec.recv_buffer(other_proc).unpack<uint64_t>(globalNodeId);
                comm_spec.recv_buffer(other_proc).unpack<int>(node_val);
                stk::mesh::EntityId entityId = globalNodeId;
                stk::mesh::Entity nodeEntity = mSTKMeshIn->bulk_data()->get_entity(stk::topology::NODE_RANK, entityId);
                uint64_t localNodeId = nodeEntity.m_value;
                if(node_val > 0 && aAllSupportNodes.find(localNodeId) == aAllSupportNodes.end())
                {
                    mSTKMeshIn->setSupportStructureFieldValue(localNodeId, 1.0);
                    aAllSupportNodes.insert(localNodeId);
                    aNodesToCheck.push_back(localNodeId);
                }
            }
        }
    }
}

bool SupportStructure::elementHasInterfaceDensity(stk::mesh::Entity &aCurElement,
                                                  double aInterfaceNormal[3])
{
    double tolerance = 1e-6;
    stk::mesh::Entity const *ElemNodes = mSTKMeshIn->bulk_data()->begin_nodes(aCurElement);
    int numNodes = mSTKMeshIn->bulk_data()->num_nodes(aCurElement);
    double materialAverageCoords[3];
    double voidAverageCoords[3];
    for(int j=0; j<3; ++j)
    {
        materialAverageCoords[j] = 0.0;
        voidAverageCoords[j] = 0.0;
    }
    int numMaterialNodes = 0;
    int numVoidNodes = 0;
    for(int j=0; j<numNodes; ++j)
    {
        stk::mesh::Entity curNode = ElemNodes[j];
        double curCoords[3];
        mSTKMeshIn->nodeCoordinates(curNode, curCoords);
        double designVariableValue = mSTKMeshIn->getMaxNodalIsoFieldVariable(curNode.m_value);
        if((designVariableValue + tolerance) >= mDesignFieldThresholdValue)
        {
            numMaterialNodes++;
            for(int k=0; k<3; ++k)
                materialAverageCoords[k] += curCoords[k];
        }
        else
        {
            numVoidNodes++;
            for(int k=0; k<3; ++k)
                voidAverageCoords[k] += curCoords[k];
        }
    }
    if(numMaterialNodes > 0 && numVoidNodes > 0)
    {
        double mag = 0.0;
        for(int k=0; k<3; ++k)
        {
            materialAverageCoords[k] /= ((double)numMaterialNodes);
            voidAverageCoords[k] /= ((double)numVoidNodes);
            aInterfaceNormal[k] = voidAverageCoords[k] - materialAverageCoords[k];
            mag += (aInterfaceNormal[k]*aInterfaceNormal[k]);
        }
        mag = sqrt(mag);
        for(int k=0; k<3; ++k)
            aInterfaceNormal[k] /= mag;

        return true;
    }
    return false;
}

bool SupportStructure::elementHasInterfaceSupport(stk::mesh::Entity &aCurElement,
                                                  double aInterfaceNormal[3])
{
    double tolerance = 1e-6;
    stk::mesh::Entity const *ElemNodes = mSTKMeshIn->bulk_data()->begin_nodes(aCurElement);
    int numNodes = mSTKMeshIn->bulk_data()->num_nodes(aCurElement);
    double materialAverageCoords[3];
    double voidAverageCoords[3];
    for(int j=0; j<3; ++j)
    {
        materialAverageCoords[j] = 0.0;
        voidAverageCoords[j] = 0.0;
    }
    int numMaterialNodes = 0;
    int numVoidNodes = 0;
    for(int j=0; j<numNodes; ++j)
    {
        stk::mesh::Entity curNode = ElemNodes[j];
        double curCoords[3];
        mSTKMeshIn->nodeCoordinates(curNode, curCoords);
        double designVariableValue = mSTKMeshIn->getSupportStructureFieldValue(curNode.m_value);
        if((designVariableValue + tolerance) >= mDesignFieldThresholdValue)
        {
            numMaterialNodes++;
            for(int k=0; k<3; ++k)
                materialAverageCoords[k] += curCoords[k];
        }
        else
        {
            numVoidNodes++;
            for(int k=0; k<3; ++k)
                voidAverageCoords[k] += curCoords[k];
        }
    }
    if(numMaterialNodes > 0 && numVoidNodes > 0)
    {
        double mag = 0.0;
        for(int k=0; k<3; ++k)
        {
            materialAverageCoords[k] /= ((double)numMaterialNodes);
            voidAverageCoords[k] /= ((double)numVoidNodes);
            aInterfaceNormal[k] = voidAverageCoords[k] - materialAverageCoords[k];
            mag += (aInterfaceNormal[k]*aInterfaceNormal[k]);
        }
        mag = sqrt(mag);
        for(int k=0; k<3; ++k)
            aInterfaceNormal[k] /= mag;

        return true;
    }
    return false;
}

bool SupportStructure::nodeNeedsSupport(stk::mesh::Entity &aCurNode, uint64_t &aNewSupportNodeLocalId,
                                        std::set<uint64_t> &aExistingSupportMaterialNodes)
{
    double tolerance = 1e-6;
    double inCoords[3];
    double bestDot = -1.0;
    bool needsSupport = true;
    stk::mesh::Entity bestNode;
    uint64_t bestNodeLocalId = 0;
    mSTKMeshIn->nodeCoordinates(aCurNode, inCoords);
    stk::mesh::Entity const *nodeElements = mSTKMeshIn->bulk_data()->begin_elements(aCurNode);
    int numElems = mSTKMeshIn->bulk_data()->num_elements(aCurNode);
    for(int i=0; i<numElems && needsSupport; ++i)
    {
        stk::mesh::Entity curElem = nodeElements[i];
        stk::mesh::Entity const *ElemNodes = mSTKMeshIn->bulk_data()->begin_nodes(curElem);
        int numNodes = mSTKMeshIn->bulk_data()->num_nodes(curElem);
        for(int j=0; j<numNodes && needsSupport; ++j)
        {
            stk::mesh::Entity curNode = ElemNodes[j];
            if(curNode != aCurNode)
            {
                double curCoords[3];
                mSTKMeshIn->nodeCoordinates(curNode, curCoords);
                double curVec[3];
                for(int h=0; h<3; ++h)
                    curVec[h] = inCoords[h]-curCoords[h];
                double mag = 0.0;
                for(int h=0; h<3; ++h)
                    mag += curVec[h]*curVec[h];
                mag = sqrt(mag);
                for(int h=0; h<3; ++h)
                    curVec[h] /= mag;
                double dot = 0.0;
                for(int h=0; h<3; ++h)
                    dot += curVec[h]*mBuildPlateNormal[h];
                if(dot > .707)
                {
                    if(dot > bestDot)
                    {
                        bestDot = dot;
                        bestNode = curNode;
                        bestNodeLocalId = bestNode.m_value;
                    }
                    if(aExistingSupportMaterialNodes.find(curNode.m_value) != aExistingSupportMaterialNodes.end())
                    {
                        needsSupport = false;
                    }
                    else
                    {
                        double designVariableValue = mSTKMeshIn->getMaxNodalIsoFieldVariable(curNode.m_value);
                        if((designVariableValue + tolerance) >= mDesignFieldThresholdValue)
                        {
                            needsSupport = false;
                        }
                    }
                }
            }
        }
    }
    if(needsSupport && bestDot > 0.0)
    {
        aNewSupportNodeLocalId = bestNodeLocalId;
        return true;
    }
    return false;
}

void SupportStructure::calculateSharingMaps(std::vector<proc_node_map> &aProcs,
                                            MeshWrapper *aMesh)
{
    // Find which owned elems are on the boundary
    std::set<uint64_t> sharedBoundaryNodes;
    aMesh->get_shared_boundary_nodes(sharedBoundaryNodes);
    std::set<uint64_t>::iterator it = sharedBoundaryNodes.begin();
    while(it != sharedBoundaryNodes.end())
    {
        uint64_t curBoundaryNode = *it;
        stk::mesh::EntityId node_id = curBoundaryNode;
        stk::mesh::EntityKey key(stk::topology::NODE_RANK, node_id);
        std::vector<int> sharingProcs;
        aMesh->bulk_data()->comm_shared_procs(key, sharingProcs);
        for(size_t i=0; i<sharingProcs.size(); ++i)
        {
            int other_proc_id = sharingProcs[i];
            // if this is not me...
            if(other_proc_id != aMesh->bulk_data()->parallel_rank())
            {
                size_t j = 0;
                for(; j<aProcs.size(); ++j)
                {
                    if(aProcs[j].processor == other_proc_id)
                    {
                        aProcs[j].nodes.push_back(curBoundaryNode);
                        break;
                    }
                }
                // if we don't have an entry for this proc yet...
                if(j == aProcs.size())
                {
                    proc_node_map new_proc;
                    new_proc.processor = other_proc_id;
                    new_proc.nodes.push_back(curBoundaryNode);
                    aProcs.push_back(new_proc);
                }
            }
        }
        ++it;
    }
}

bool SupportStructure::readCommandLine( int argc, char *argv[])
{
    Teuchos::CommandLineProcessor clp;
    clp.setDocString("create_support_structure options");

    clp.setOption("input_mesh",  &mMeshIn, "in mesh file (ExodusII).", true );
    clp.setOption("output_mesh",  &mMeshOut, "out mesh file (ExodusII).", false );
    clp.setOption("design_field_name",  &mDesignFieldName, "field defining level set data.", false );
    clp.setOption("design_field_value",  &mDesignFieldThresholdValue, "specify the design variable threshold value.", false );
    clp.setOption("overhang_angle",  &mOverhangAngle, "specify the allowable overhang angle. (default=45)", false );
    clp.setOption("cell_size_multiplier",  &mCellSizeMultiplier, "specify the multiplier to be used on the average mesh edge length to get a cell/voxel size. (default=1.0)", false );
    clp.setOption("concatenate_results",  &mConcatenateResults, "specify whether to concatenate resulting mesh files.", false );
    clp.setOption("read_spread_file",  &mReadSpreadFile, "specify whether input is already decomposed.", false );
    clp.setOption("time_step",  &mTimeStep, "specify the time step to be read from the file.", false );
    clp.setOption("output_fields",  &mOutputFields, "specify the fields (commma separated, no spaces) to output in the output mesh.", false );
    clp.setOption("build_plate_normal",  &mBuildPlateNormalString, "specify the normal of the build plate (values separated by spaces)", false );
    clp.setOption("algorithm",  &mAlgorithm, "specify the algorithm to use (0=nodal based (default), 1=element based, 2=project triangles)", false );
    clp.setOption("neighbor_search_radius",  &mNeighborSearchRadius, "specify the number of layers to expand search for voxel neighbors (default=2)", false );

    Teuchos::CommandLineProcessor::EParseCommandLineReturn parseReturn =
            Teuchos::CommandLineProcessor::PARSE_SUCCESSFUL;
    try
    {
        parseReturn = clp.parse( argc, argv );
    }
    catch (std::exception exc)
    {
        std::cout << "Failed to parse the command line arguments." << std::endl;
        return false;
    }

    if ( parseReturn == Teuchos::CommandLineProcessor::PARSE_SUCCESSFUL )
        return true;

    std::cout << "Failed to parse the command line arguments." << std::endl;
    return false;
}

// Given three colinear points p, q, r, the function checks if 
// point q lies on line segment 'pr' 
bool SupportStructure::onSegment(Point2D &p, Point2D &q, Point2D &r) 
{ 
    if (q.coords[0] <= std::max(p.coords[0], r.coords[0]) && q.coords[0] >= std::min(p.coords[0], r.coords[0]) &&
            q.coords[1] <= std::max(p.coords[1], r.coords[1]) && q.coords[1] >= std::min(p.coords[1], r.coords[1]))
        return true; 
    return false; 
} 

// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are colinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
int SupportStructure::orientation(Point2D &p, Point2D &q, Point2D &r) 
{ 
    int val = (q.coords[1] - p.coords[1]) * (r.coords[0] - q.coords[0]) -
            (q.coords[0] - p.coords[0]) * (r.coords[1] - q.coords[1]);

    if (val == 0) return 0;  // colinear 
    return (val > 0)? 1: 2; // clock or counterclock wise 
} 

// The function that returns true if line segment 'p1q1' 
// and 'p2q2' intersect. 
bool SupportStructure::doIntersect(Point2D &p1, Point2D &q1, Point2D &p2, Point2D &q2) 
{ 
    // Find the four orientations needed for general and 
    // special cases 
    int o1 = orientation(p1, q1, p2); 
    int o2 = orientation(p1, q1, q2); 
    int o3 = orientation(p2, q2, p1); 
    int o4 = orientation(p2, q2, q1); 

    // General case 
    if (o1 != o2 && o3 != o4) 
        return true; 

    // Special Cases 
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1)) return true; 

    // p1, q1 and p2 are colinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1)) return true; 

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2)) return true; 

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true; 

    return false; // Doesn't fall in any of the above cases 
} 

// Returns true if the point p lies inside the polygon[] with n vertices 
bool SupportStructure::isInside(Point2D polygon[], int n, Point2D &p) 
{ 
    // There must be at least 3 vertices in polygon[] 
    if (n < 3)  return false; 

    // Create a point for line segment from p to infinite 
    Point2D extreme = {10000, p.coords[1]};

    // Count intersections of the above line with sides of polygon 
    int count = 0, i = 0; 
    do
    { 
        int next = (i+1)%n; 

        // Check if the line segment from 'p' to 'extreme' intersects 
        // with the line segment from 'polygon[i]' to 'polygon[next]' 
        if (doIntersect(polygon[i], polygon[next], p, extreme)) 
        { 
            // If the point 'p' is colinear with line segment 'i-next', 
            // then check if it lies on segment. If it lies, return true, 
            // otherwise false 
            if (orientation(polygon[i], p, polygon[next]) == 0) 
                return onSegment(polygon[i], p, polygon[next]);

            count++; 
        } 
        i = next; 
    } while (i != 0); 

    // Return true if count is odd, false otherwise 
    return count&1;  // Same as (count%2 == 1) 
} 


Vector3D SupportStructure::findCentroid(std::vector<Vector3D> &points)
{
    Vector3D center;
    for(int i=0; i<3; ++i)
        center[i] = 0.0;
    for (Vector3D p : points)
    {
        for(int i=0; i<3; ++i)
            center[i] += p[i];
    }
    for(int i=0; i<3; ++i)
        center[i] /= (double)(points.size());
    return center;
}

void SupportStructure::sortVerticies(std::vector<Point2D> &points) 
{
    std::cout << "sortVertices not implemented" << std::endl;
    exit(1);
    /*
    // get centroid
    Point center = findCentroid(points);
    Collections.sort(points, (a, b) -> {
        double a1 = (Math.toDegrees(Math.atan2(a.x - center.x, a.y - center.y)) + 360) % 360;
        double a2 = (Math.toDegrees(Math.atan2(b.x - center.x, b.y - center.y)) + 360) % 360;
        return (int) (a1 - a2);
    });
    return points;
     */
}

Vector3D::Vector3D(double &x, double &y, double &z)
{
    coords[0] = x;
    coords[1] = y;
    coords[2] = z;
}

Vector3D::Vector3D()
{
    coords[0] = 0.0;
    coords[1] = 0.0;
    coords[2] = 0.0;
}

double Vector3D::dot(Vector3D &otherVec)
{
    return coords[0]*otherVec.coords[0] + coords[1]*otherVec.coords[1] + coords[2]*otherVec.coords[2];
}

void Vector3D::normalize()
{
    double magnitude = mag();
    if(magnitude > 1e-12)
    {
        for(int s=0; s<3; ++s)
            coords[s] /= magnitude;
    }
}

double Vector3D::mag()
{
    double mag = 0.0;
    for(int s=0; s<3; ++s)
        mag += coords[s]*coords[s];
    return sqrt(mag);
}

void Vector3D::operator+=(Vector3D &v)
{
    for(int i=0; i<3; ++i)
        coords[i] += v.coords[i];
}

void Vector3D::operator/=(double val)
{
    for(int i=0; i<3; ++i)
        coords[i] /= val;
}

void Vector3D::operator*=(double val)
{
    for(int i=0; i<3; ++i)
        coords[i] *= val;
}

double Vector3D::distanceSquared(const Vector3D &otherVector)
{
    double ret = 0.0;
    for(int i=0; i<3; ++i)
        ret += ((coords[i]-otherVector.coords[i])*(coords[i]-otherVector.coords[i]));
    return ret;
}

double Vector3D::distance(const Vector3D &otherVector)
{
    double ret = 0.0;
    for(int i=0; i<3; ++i)
        ret += ((coords[i]-otherVector.coords[i])*(coords[i]-otherVector.coords[i]));
    return sqrt(ret);
}


#if 0

// Driver program to test above functions 
int main() 
{ 
    Point polygon1[] = {{0, 0}, {10, 0}, {10, 10}, {0, 10}}; 
    int n = sizeof(polygon1)/sizeof(polygon1[0]); 
    Point p = {20, 20}; 
    isInside(polygon1, n, p)? cout << "Yes \n": cout << "No \n"; 

    p = {5, 5}; 
    isInside(polygon1, n, p)? cout << "Yes \n": cout << "No \n"; 

    Point polygon2[] = {{0, 0}, {5, 5}, {5, 0}}; 
    p = {3, 3}; 
    n = sizeof(polygon2)/sizeof(polygon2[0]); 
    isInside(polygon2, n, p)? cout << "Yes \n": cout << "No \n"; 

    p = {5, 1}; 
    isInside(polygon2, n, p)? cout << "Yes \n": cout << "No \n"; 

    p = {8, 1}; 
    isInside(polygon2, n, p)? cout << "Yes \n": cout << "No \n"; 

    Point polygon3[] =  {{0, 0}, {10, 0}, {10, 10}, {0, 10}}; 
    p = {-1,10}; 
    n = sizeof(polygon3)/sizeof(polygon3[0]); 
    isInside(polygon3, n, p)? cout << "Yes \n": cout << "No \n"; 

    return 0; 
}
#endif

} //namespace support_structure

} //namespace plato




