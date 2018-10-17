#include <iostream>

#include "Cogent_WriteUtils.hpp"

//******************************************************************************//
void Cogent::Writer::close()
//******************************************************************************//
{
  std::ofstream outfile;
  std::stringstream ss;
  ss << m_fileNameBase << ".case";
  outfile.open(ss.str());

  outfile << "FORMAT" << std::endl;
  outfile << "type: ensight gold" << std::endl;
  outfile << std::endl;
  outfile << "GEOMETRY" << std::endl;
  outfile << "model: " << m_fileNameBase << ".geo*" << std::endl;
  outfile << std::endl;
  outfile << "TIME" << std::endl;
  outfile << "time set: 1" << std::endl;
  outfile << "number of steps: " << m_timeIndices.size() << std::endl;
  outfile << "filename start number: 1" << std::endl;
  outfile << "filename increment: 1" << std::endl;
  outfile << "time values:" << std::endl;
  outfile.precision(5);
  outfile << std::scientific;
  for(unsigned int i=0; i<m_timeIndices.size(); i++){
    outfile << std::setw(12) << (float)(m_timeIndices[i]) << std::endl;
  }
  outfile.close();
}


//******************************************************************************//
void Cogent::writeSurfaceTris(const std::vector<Simplex<RealType,RealType> >& inTets)
//******************************************************************************//
{
  std::vector<Simplex<RealType,RealType>> outTris;
  getSurfaceTris(inTets, outTris);
  
  writeTris(outTris);
}

//******************************************************************************//
void Cogent::getSurfaceTris(
   const std::vector<Simplex<RealType,RealType> >& inTets, 
   std::vector<Simplex<RealType,RealType> >& outTris)
//******************************************************************************//
{
  // loop through each tet and collect tris that have boundaryLS's.
  //
  std::vector<Simplex<RealType,RealType>> boundaryTris;
  for( const auto& tet : inTets ){
    for(int iFace=0; iFace<Cogent::nTetPts; iFace++){
      if(tet.boundaryLS[iFace] != -1){
        Simplex<RealType,RealType> boundaryTri(Cogent::nTriPts);
        for(uint j=0; j<Cogent::nTriPts; j++){
          boundaryTri.points[j] = tet.points[(iFace+j)%Cogent::nTetPts];
        }
        boundaryTris.push_back(boundaryTri);
      }
    }
  }

  // loop through boundary tris and remove duplicates
  // 
  int nTris = boundaryTris.size();
  for(int iTri=0; iTri<nTris; iTri++){
    bool isUnique = true;
    for(int jTri=iTri+1; jTri<nTris; jTri++){
      if(isEqual(boundaryTris[iTri],boundaryTris[jTri])){
        isUnique = false;
        break;
      }
    }
    if(isUnique) outTris.push_back(boundaryTris[iTri]);
  }
}

//******************************************************************************//
bool Cogent::isEqual(
   const Simplex<RealType,RealType>& t0,
   const Simplex<RealType,RealType>& t1)
//******************************************************************************//
{
  unsigned int npts = t0.points.size();
  if(t1.points.size() != npts) return false;

  RealType c0x=0.0, c0y=0.0, c0z=0.0;
  RealType c1x=0.0, c1y=0.0, c1z=0.0;
  for(unsigned int ip=0; ip<npts; ip++){
    RealType x0 = t0.points[ip](0), x1 = t1.points[ip](0);
    RealType y0 = t0.points[ip](1), y1 = t1.points[ip](1);
    RealType z0 = t0.points[ip](2), z1 = t1.points[ip](2);
    c0x += x0; c0y += y0; c0z += z0;
    c1x += x1; c1y += y1; c1z += z1;
  }
  RealType dx = fabs(c0x-c1x);
  RealType dy = fabs(c0y-c1y);
  RealType dz = fabs(c0z-c1z);
  if(dx*dx+dy*dy+dz*dz > 1e-10) return false;
  std::cout << "Found a duplicate" << std::endl;
  return true;
}

