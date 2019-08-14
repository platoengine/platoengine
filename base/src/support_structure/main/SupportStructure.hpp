/*--------------------------------------------------------------------*/
/*    Copyright 2002 - 2008, 2010, 2011 National Technology &         */
/*    Engineering Solutions of Sandia, LLC (NTESS). Under the terms   */
/*    of Contract DE-NA0003525 with NTESS, there is a                 */
/*    non-exclusive license for use of this work by or on behalf      */
/*    of the U.S. Government.  Export of this program may require     */
/*    a license from the United States Government.                    */
/*--------------------------------------------------------------------*/
#ifndef support_structure_SupportStructure_hpp
#define support_structure_SupportStructure_hpp

#include <stdexcept>
#include <sstream>
#include <vector>
#include <iostream>

#include <stk_mesh/base/Field.hpp>
#include <stk_mesh/base/CoordinateSystems.hpp>

#include "Teuchos_CommandLineProcessor.hpp"
#include "MeshWrapper.hpp"

namespace stk
{
namespace io
{
class StkMeshIoBroker;
}
}

namespace plato
{

namespace support_structure
{

class Vector3D
{
private:
    double coords[3];
public:
    Vector3D(double &x, double &y, double &z);
    Vector3D();
    ~Vector3D(){}
    double dot(Vector3D &otherVec);
    void normalize();
    double mag();
    double x() { return coords[0]; }
    double y() { return coords[1]; };
    double z() { return coords[2]; };
    double& operator[](int index) { return coords[index]; }
    void operator+=(Vector3D &v);
    void operator/=(double val);
    void operator*=(double val);
    const double& operator[](int index) const { return coords[index]; }
    double* data () { return coords; }
    double distanceSquared(const Vector3D &otherVector);
    double distance(const Vector3D &otherVector);
    friend Vector3D operator*(const Vector3D &v1, const Vector3D &v2);
    friend double operator%(const Vector3D &v1, const Vector3D &v2);
    friend Vector3D operator-(const Vector3D &v1, const Vector3D &v2);
    friend Vector3D operator+(const Vector3D &v1, const Vector3D &v2);
    friend Vector3D operator*(const Vector3D &v1, const double sclr);
    friend Vector3D operator*(const double sclr, const Vector3D &v1);
};

inline Vector3D operator*(const Vector3D &v, const double sclr)
{
    Vector3D ret;
    for(int i=0; i<3; ++i)
        ret.coords[i] = sclr*v.coords[i];
    return ret;
}

inline Vector3D operator*(const double sclr, const Vector3D &v)
{
    Vector3D ret;
    for(int i=0; i<3; ++i)
        ret.coords[i] = sclr*v.coords[i];
    return ret;
}

inline Vector3D operator-(const Vector3D &v1, const Vector3D &v2)
{
    Vector3D ret;
    for(int i=0; i<3; ++i)
        ret.coords[i] = v1.coords[i] - v2.coords[i];
    return ret;
}

// cross product
inline Vector3D operator*(const Vector3D &v1, const Vector3D &v2)
{
    Vector3D ret;
    ret[0] = v1[1]*v2[2] - v1[2]*v2[1];
    ret[1] = v1[2]*v2[0] - v1[0]*v2[2];
    ret[2] = v1[0]*v2[1] - v1[1]*v2[0];
    return ret;
}
// dot product
inline double operator%(const Vector3D &v1, const Vector3D &v2)
{
    double ret = 0.0;
    for(int i=0; i<3; ++i)
        ret += v1.coords[i]*v2.coords[i];
    return ret;
}

inline Vector3D operator+(const Vector3D &v1, const Vector3D &v2)
{
    Vector3D ret;
    for(int i=0; i<3; ++i)
        ret.coords[i] = v1.coords[i] + v2.coords[i];
    return ret;
}

struct Point2D
{
    double coords[2];
};

struct proc_node_map
{
    int processor;
    std::vector<uint64_t> nodes;
};

struct VoxelData
{
    double maxDensity;
    double maxDot;
    bool inDesignRegion;
    bool maxIsDesign;
    bool setByNode;
    bool hasInterface;
    bool dataExists;
    std::vector<stk::mesh::Entity> nodes;
};

class SupportStructure
{
public:
    SupportStructure(){}
    ~SupportStructure();
    bool createMeshAPIsStandAlone(int argc, char **argv);
    bool run();

private:

    void setVoxelDataByNeighbor(int aXIndex, int aYIndex, int aNumGridX, int aNumGridY,
                                std::vector<std::vector<std::vector<VoxelData> > > &aVoxelData,
                                int aZLayer,
                                int aNumZLayers);
    void setVoxelDataByNode(int aXIndex, int aYIndex,
                            std::vector<std::vector<std::vector<VoxelData> > > &aVoxelData,
                            int aZLayer);
    void setVoxelNodeData(stk::mesh::Entity aNode,
                          int aXIndex, int aYIndex,
                          std::vector<std::vector<std::vector<VoxelData> > > &aVoxelData,
                          std::map<stk::mesh::Entity, double> &aNodeInterfaceAngles,
                          int aZLayer);
    void setVoxelData(stk::mesh::Entity aNode,
                      int aXIndex, int aYIndex, int aNumGridX, int aNumGridY,
                      std::vector<std::vector<std::vector<VoxelData> > > &aVoxelData,
                      std::map<stk::mesh::Entity, double> &aNodeInterfaceAngles,
                      int aZLayer,
                      int aNumZLayers,
                      int aLayerIndex);

    void setVoxelDataAndSupportStructure(stk::mesh::Entity aNode,
                                         int nodeX, int nodeY,
                                         std::vector<std::vector<VoxelData> > &aVoxelData,
                                         double &aDensity,
                                         double &aDot,
                                         bool &aHasInterface);
    void getValsForSettingVoxel(stk::mesh::Entity aNode,
                                int i, int j, int numGridX, int numGridY,
                                std::vector<std::vector<VoxelData> > &aVoxelData,
                                std::map<stk::mesh::Entity, double> &aNodeInterfaceAngles,
                                double &aDensity,
                                double &aDot,
                                bool &aHasInterface);
    void getValsForSettingVoxel3D(stk::mesh::Entity aNode,
                                int i, int j, int numGridX, int numGridY,
                                std::vector<std::vector<std::vector<VoxelData> > > &aVoxelData,
                                std::map<stk::mesh::Entity, double> &aNodeInterfaceAngles,
                                double &aDensity,
                                double &aDot,
                                bool &aHasInterface,
                                int aZLayer,
                                int aNumZLayers);
    void getNodeXY(stk::mesh::Entity &aNode, Vector3D &aOrigin, Vector3D &aXAxis,
                   Vector3D &aYAxis, Vector3D &aMinCoords, double &aGridSizeX,
                   double &aGridSizeY, int &aNumGridX, int &aNumGridY, int &aNodeX,
                   int &aNodeY);
    void getNodeInterfaceData(std::map<stk::mesh::Entity, double> &aNodeInterfaceAngles);
    void getGridDimensions(double &aAverageEdgeLength, Vector3D &aOrigin,
                           Vector3D &aXAxis, Vector3D &aYAxis, int &aNumGridX, int &aNumGridY,
                           Vector3D &aMinCoords, Vector3D &aMaxCoords,
                           double &aGridSizeX, double &aGridSizeY);
    void getAverageEdgeLength(double &averageEdgeLength);
    void getNodesSortedInZDirection(Vector3D &aOrigin,
                                    Vector3D &aAxis,
                                    std::vector<std::pair<stk::mesh::Entity, double> > &aSortedNodeDistancePairs,
                                    double &zMin, double &zMax);
    bool getBuildPlateCoordinateSystem(Vector3D &aX, Vector3D &aY, Vector3D &aZ, Vector3D &origin);
    bool runPrivateVoxelBased();
    bool runPrivateVoxelBasedInefficientMemory();
    bool runPrivateNodeBased();
    bool runPrivateElementBased();
    bool runPrivateProjectTriangle();
    bool runPrivateNodeBasedMaxDensityAbove();
    bool runPrivateNodeBasedMaxDensityAboveTopDown();
    double calculateSupportStructureValue(double &aMyDensity, double &aAboveDensity, double aNodeDot, bool aFoundDesignNode);
    bool readCommandLine(int argc, char *argv[]);
    bool initSingleMeshAPIs();
    void calculateSharingMaps(std::vector<proc_node_map> &aProcs,
                              MeshWrapper *aMesh);
    bool nodeNeedsSupport(stk::mesh::Entity &aCurNode, uint64_t &aNewSupportNodeLocalId,
                          std::set<uint64_t> &aExistingSupportMaterialNodes);
    void communicateBoundaryInfo(std::vector<proc_node_map> &procs,
                                 std::set<uint64_t> &aSupportNodes,
                                 std::set<uint64_t> &aAllSupportNodes,
                                 std::vector<uint64_t> &aNodesToCheck);
    bool elementHasInterfaceDensity(stk::mesh::Entity &aCurElement,
                             double aInterfaceNormal[3]);
    bool elementHasInterfaceSupport(stk::mesh::Entity &aCurElement,
                             double aInterfaceNormal[3]);
    Vector3D findCentroid(std::vector<Vector3D> &points);
    void sortVerticies(std::vector<Point2D> &points);
    bool isInside(Point2D polygon[], int n, Point2D &p);
    bool doIntersect(Point2D &p1, Point2D &q1, Point2D &p2, Point2D &q2);
    int orientation(Point2D &p, Point2D &q, Point2D &r);
    bool onSegment(Point2D &p, Point2D &q, Point2D &r);
    void projectPointToPlane(Vector3D &point, Vector3D &planeOrigin,
                             Vector3D &planeNormal, Vector3D &projectedPoint);
    int getIntersectionInfoTet(stk::mesh::Entity const *aElementNodes,
                                              int aFieldType,
                                              Vector3D &aPlaneNormal,
                                              Vector3D &aPlaneOrigin,
                                            std::vector<Vector3D> &aTriPoints);
    int getIntersectionInfoHex(stk::mesh::Entity const *aElementNodes,
                                              int aFieldType,
                                              Vector3D &aPlaneNormal,
                                              Vector3D &aPlaneOrigin,
                                            std::vector<Vector3D> &aTriPoints);
    int getIntersectionInfo(stk::mesh::Entity &element,
                                            int aFieldType,
                                            Vector3D &aPlaneNormal,
                                            Vector3D &aPlaneOrigin,
                                            std::vector<Vector3D> &aTriPoints);
    bool pointInTriangle(Vector3D &p, Vector3D &t1, Vector3D &t2, Vector3D &t3);







    stk::ParallelMachine *mComm;
    std::string mMeshIn;
    std::string mMeshOut;
    std::string mDesignFieldName;
    std::string mOutputFields;
    std::string mBuildPlateNormalString;
    double mDesignFieldThresholdValue;
    double mOverhangAngle;
    double mCellSizeMultiplier;
    Vector3D mBuildPlateNormal;
    int mReadSpreadFile;
    int mConcatenateResults;
    int mTimeStep;
    int mAlgorithm;
    int mNeighborSearchRadius;
    MeshWrapper *mSTKMeshIn;
    MeshWrapper *mSTKMeshOut;
};


}//namespace support_structure
}//namespace plato

#endif
