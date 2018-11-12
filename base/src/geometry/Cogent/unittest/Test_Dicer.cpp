#include <gtest/gtest.h>

#include <Intrepid2_HGRAD_HEX_Cn_FEM.hpp>
#include "core/Cogent_Dicer.hpp"
#include "core/Cogent_Integrator.hpp"
#include "core/Cogent_Types.hpp"
#include "core/Cogent_WriteUtils.hpp"
#include "core/Cogent_ModelFactory.hpp"

#include <iostream>
#include <fstream>
#include <stdlib.h>

#include <Teuchos_ParameterList.hpp>
#include <Teuchos_XMLParameterListHelpers.hpp>

using Cogent::RealType;
using Cogent::DFadType;

static RealType tolerance = 1e-6;
static const int nPts = 4;
static const int nDim = 3;

static bool verbose = false;


void fromDFadType(
       std::vector<Cogent::Simplex<DFadType,DFadType>>& fromSimplexes,
       std::vector<Cogent::Simplex<RealType,RealType>>& toSimplexes);

bool diffDerivs(
       const minitensor::Vector<DFadType,nDim>& p1,
       const minitensor::Vector<DFadType,nDim>& p2,
       int nDerivs, RealType tol);

int  getNumDiffDerivs(
       const std::vector<Cogent::Simplex<DFadType,DFadType>>& allSimplexes,
       int nDerivs, RealType tol, bool verbose=true);

void test(Cogent::FContainer<RealType>& topoVals,
          Teuchos::RCP<Teuchos::ParameterList>& geomSpec, double fittedVolume);


TEST(DicerTest, OneTet_OneLS)
{
  // define levelsets:
  //    1. x=0 and positive for x<0.
  //
  int numLevelsets = 1;
  int numNodes = 8;
  Cogent::FContainer<RealType> topoVals("topoVals",numNodes,numLevelsets);
  topoVals(0,0) = -1.0;   topoVals(1,0) =  1.0;   topoVals(2,0) =  1.0;   topoVals(3,0) = -1.0;
  topoVals(4,0) = -1.0;   topoVals(5,0) =  1.0;   topoVals(6,0) =  1.0;   topoVals(7,0) = -1.0;

  // create a model that defines where material is based on defined levelsets
  //
  Teuchos::RCP<Teuchos::ParameterList> geomSpec = 
    Teuchos::getParametersFromXmlString(
    "<ParameterList name='Geometry Construction'>                                        \n"
    "  <Parameter name='Model Type' type='string' value='Non-parameterized'/>            \n"
    "  <Parameter name='Geometry Type' type='string' value='Body'/>                      \n"
    "  <Parameter name='Number of Subdomains' type='int' value='1'/>                     \n"
    "  <Parameter name='Level Set Names' type='Array(string)' value='{LS0}'/>            \n"
    "  <Parameter name='Interface Value' type='double' value='0.0'/>                     \n"
    "  <Parameter name='Maximum Error' type='double' value='1.0e-3'/>                    \n"
    "  <Parameter name='Maximum Refinements' type='int' value='0'/>                      \n"
    "  <ParameterList name='Subdomain 0'>                                                \n"
    "    <Parameter name='Operation' type='string' value='Add'/>                         \n"
    "    <Parameter name='Level Set Indices' type='Array(int)' value='{0}'/>             \n"
    "  </ParameterList>                                                                  \n"
    "</ParameterList>                                                                    \n"
    );

  test(topoVals, geomSpec, 1.0/6.0);

}

TEST(DicerTest, OneTet_TwoLS)
{
  // define levelsets:
  //    1. x=0.1/2.1 and positive for x<0.1/2.1.
  //    2. y=0 and positive for y>0.
  //
  int numLevelsets = 2;
  int numNodes = 8;
  Cogent::FContainer<RealType> topoVals("topoVals",numNodes,numLevelsets);
  topoVals(0,0) =  1.1;   topoVals(1,0) = -1.0;   topoVals(2,0) = -1.0;   topoVals(3,0) =  1.1;
  topoVals(4,0) =  1.1;   topoVals(5,0) = -1.0;   topoVals(6,0) = -1.0;   topoVals(7,0) =  1.1;
  topoVals(0,1) = -1.0;   topoVals(1,1) = -1.0;   topoVals(2,1) =  1.0;   topoVals(3,1) =  1.0;
  topoVals(4,1) = -1.0;   topoVals(5,1) = -1.0;   topoVals(6,1) =  1.0;   topoVals(7,1) =  1.0;

  // create a model that defines where material is based on defined levelsets
  //
  Teuchos::RCP<Teuchos::ParameterList> geomSpec = 
    Teuchos::getParametersFromXmlString(
    "<ParameterList name='Geometry Construction'>                                        \n"
    "  <Parameter name='Model Type' type='string' value='Non-parameterized'/>            \n"
    "  <Parameter name='Geometry Type' type='string' value='Body'/>                      \n"
    "  <Parameter name='Number of Subdomains' type='int' value='1'/>                     \n"
    "  <Parameter name='Level Set Names' type='Array(string)' value='{LS0,LS1}'/>        \n"
    "  <Parameter name='Interface Value' type='double' value='0.0'/>                     \n"
    "  <Parameter name='Maximum Error' type='double' value='1.0e-3'/>                    \n"
    "  <Parameter name='Maximum Refinements' type='int' value='0'/>                      \n"
    "  <ParameterList name='Subdomain 0'>                                                \n"
    "    <Parameter name='Operation' type='string' value='Add'/>                         \n"
    "    <Parameter name='Level Set Indices' type='Array(int)' value='{0,1}'/>           \n"
    "  </ParameterList>                                                                  \n"
    "</ParameterList>                                                                    \n"
    );

  test(topoVals, geomSpec, 1.0/6.0);
}

TEST(DicerTest, OneTet_ThreeLS)
{
  // define levelsets:
  //    1. x=0.1/2.1 and positive for x<0.1/2.1.
  //    1. y=0.1/2.1 and positive for y<0.1/2.1.
  //    2. z=0 and positive for z>0.
  //
  int numLevelsets = 3;
  int numNodes = 8;
  Cogent::FContainer<RealType> topoVals("topoVals",numNodes,numLevelsets);
  topoVals(0,0) =  1.1;   topoVals(1,0) = -1.0;   topoVals(2,0) = -1.0;   topoVals(3,0) =  1.1;
  topoVals(4,0) =  1.1;   topoVals(5,0) = -1.0;   topoVals(6,0) = -1.0;   topoVals(7,0) =  1.1;
  topoVals(0,1) =  1.1;   topoVals(1,1) =  1.1;   topoVals(2,1) = -1.0;   topoVals(3,1) = -1.0;
  topoVals(4,1) =  1.1;   topoVals(5,1) =  1.1;   topoVals(6,1) = -1.0;   topoVals(7,1) = -1.0;
  topoVals(0,2) = -1.0;   topoVals(1,2) = -1.0;   topoVals(2,2) = -1.0;   topoVals(3,2) = -1.0;
  topoVals(4,2) =  1.0;   topoVals(5,2) =  1.0;   topoVals(6,2) =  1.0;   topoVals(7,2) =  1.0;

  // create a model that defines where material is based on defined levelsets
  //
  Teuchos::RCP<Teuchos::ParameterList> geomSpec = 
    Teuchos::getParametersFromXmlString(
    "<ParameterList name='Geometry Construction'>                                        \n"
    "  <Parameter name='Model Type' type='string' value='Non-parameterized'/>            \n"
    "  <Parameter name='Geometry Type' type='string' value='Body'/>                      \n"
    "  <Parameter name='Number of Subdomains' type='int' value='1'/>                     \n"
    "  <Parameter name='Level Set Names' type='Array(string)' value='{LS0,LS1,LS2}'/>    \n"
    "  <Parameter name='Interface Value' type='double' value='0.0'/>                     \n"
    "  <Parameter name='Maximum Error' type='double' value='1.0e-3'/>                    \n"
    "  <Parameter name='Maximum Refinements' type='int' value='0'/>                      \n"
    "  <ParameterList name='Subdomain 0'>                                                \n"
    "    <Parameter name='Operation' type='string' value='Add'/>                         \n"
    "    <Parameter name='Level Set Indices' type='Array(int)' value='{0,1,2}'/>         \n"
    "  </ParameterList>                                                                  \n"
    "</ParameterList>                                                                    \n"
    );

  test(topoVals, geomSpec, 1.0/6.0);

}



bool diffDerivs(
  const minitensor::Vector<DFadType,3>& p1,
  const minitensor::Vector<DFadType,3>& p2,
  int nDerivs,
  RealType tol)
{
  // are points coincident?
  //
  bool areCoincident = false;
  RealType dx = p1(0).val()-p2(0).val();
  RealType dy = p1(1).val()-p2(1).val();
  RealType dz = p1(2).val()-p2(2).val();
  RealType d2 = dx*dx+dy*dy+dz*dz;
  if(d2 - tol*tol < 0.0){ 
    areCoincident = true;
  } 

  // if coincident, are derivatives the same?
  //
  if(areCoincident){
    for(int i=0; i<nDerivs; i++){
      if( fabs(p1(0).dx(i) - p2(0).dx(i)) > tol ||
          fabs(p1(1).dx(i) - p2(1).dx(i)) > tol ||
          fabs(p1(2).dx(i) - p2(2).dx(i)) > tol ){
        return true;
      }
    }
  }
  return false;
}

int  getNumDiffDerivs(
  const std::vector<Cogent::Simplex<DFadType,DFadType>>& allSimplexes,
  int nDerivs,
  RealType tol, 
  bool verbose)
{
  int numDiffDerivs = 0;
  for(unsigned int iSmp=0; iSmp<allSimplexes.size(); iSmp++){
    for(unsigned int jSmp=iSmp+1; jSmp<allSimplexes.size(); jSmp++){
      for(unsigned int iPt=0; iPt<nPts; iPt++){
        for(unsigned int jPt=0; jPt<nPts; jPt++){
          if( diffDerivs(allSimplexes[iSmp].points[iPt],
                         allSimplexes[jSmp].points[jPt], nDerivs, tol) ){
            numDiffDerivs += 1;
            if(verbose){
              std::cout << "Comparing t: " << iSmp << ", p: " << iPt << ", g: " << iSmp*nPts+iPt << " -- " <<
                                     "t: " << jSmp << ", p: " << jPt << ", g: " << jSmp*nPts+jPt << std::endl;
              std::cout << "Points are same, but derivatives are different: " << std::endl;
              std::cout << "Point 1, x: " << allSimplexes[iSmp].points[iPt](0) << std::endl;
              std::cout << "Point 2, x: " << allSimplexes[jSmp].points[jPt](0) << std::endl;
              std::cout << "Point 1, y: " << allSimplexes[iSmp].points[iPt](1) << std::endl;
              std::cout << "Point 2, y: " << allSimplexes[jSmp].points[jPt](1) << std::endl;
              std::cout << "Point 1, z: " << allSimplexes[iSmp].points[iPt](2) << std::endl;
              std::cout << "Point 2, z: " << allSimplexes[jSmp].points[jPt](2) << std::endl;
              
            }
          }
        }
      }
    }
  }
  return numDiffDerivs;
}

void 
fromDFadType(std::vector<Cogent::Simplex<DFadType,DFadType>>& fromSimplexes,
             std::vector<Cogent::Simplex<RealType,RealType>>& toSimplexes)
{
  for( Cogent::Simplex<DFadType,DFadType>& simplex : fromSimplexes ){
    Cogent::Simplex<RealType,RealType> ds(nPts);
    for(int i=0; i<nPts; i++){
      for(int j=0; j<nDim; j++){
        ds.points[i](j) = simplex.points[i](j).val();
      }
    }
    toSimplexes.push_back(ds);
  }
}

void test(Cogent::FContainer<RealType>& topoVals,
          Teuchos::RCP<Teuchos::ParameterList>& geomSpec, double fittedVolume)
{
  // define HEX8 element topology and basis
  //
  const CellTopologyData& celldata = *shards::getCellTopologyData< shards::Hexahedron<8> >();
  Teuchos::RCP<shards::CellTopology> celltype = Teuchos::rcp(new shards::CellTopology( &celldata ) );

  Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType> >
    intrepidBasis = Teuchos::rcp(new Intrepid2::Basis_HGRAD_HEX_C1_FEM<Kokkos::Serial, RealType, RealType >() );
  
  Teuchos::RCP<Cogent::Dicer> dicer = Teuchos::rcp(new Cogent::Dicer(celltype, intrepidBasis, /*interfaceValue=*/0.0));

  Cogent::FContainer<DFadType> Tfad;
  Cogent::toDFadType(topoVals, Tfad);

  // define a single starting tet
  //
  std::vector<Cogent::Simplex<DFadType,DFadType>> implicitSimplex(1);
  implicitSimplex[0] = Cogent::Simplex<DFadType,DFadType>(nPts);
  auto& simplex = implicitSimplex[0];
  simplex.points[0](0) =-1.0; simplex.points[0](1) =-1.0; simplex.points[0](2) =-1.0;
  simplex.points[1](0) = 1.0; simplex.points[1](1) =-1.0; simplex.points[1](2) =-1.0;
  simplex.points[2](0) =-1.0; simplex.points[2](1) = 1.0; simplex.points[2](2) =-1.0;
  simplex.points[3](0) =-1.0; simplex.points[3](1) =-1.0; simplex.points[3](2) = 1.0;

  // cut the single starting tet with the levelsurface (x=0) into a set of conformal tets
  //
  std::vector<Cogent::Simplex<DFadType,DFadType>> allSimplexes;
  dicer->Cut(Tfad, implicitSimplex, allSimplexes);
  
  // write the conformal set
  //
  if(verbose){
    std::cout << "Conformal set for one levelset." << std::endl;
    std::vector<Cogent::Simplex<RealType,RealType>> writeSimplexes;
    fromDFadType(allSimplexes, writeSimplexes);
    Cogent::writeTets(writeSimplexes);
  }

  // compute the volume of the initial tet
  //
  DFadType initVol = dicer->Volume(implicitSimplex[0]);

  // compute the volume of the conformal set of tets
  //
  DFadType newMeasure = 0.0;
  for( auto& spx : allSimplexes ){
     newMeasure += dicer->Volume(spx);
  }

  // The initial (non-conformal) tet should have the same volume as the 
  // conformal tets combined
  //
  std::cout << " Verifying total volume" << std::endl;
  EXPECT_NEAR(initVol.val(), newMeasure.val(), tolerance);

  // The full set of conformal tets should sum to the volume of the
  // initial tet, so the derivative of the volume wrt each levelset value
  // should be zero.
  //
  std::cout << " Verifying total volume derivatives" << std::endl;
  EXPECT_NEAR(newMeasure.dx(0), 0, tolerance);
  EXPECT_NEAR(newMeasure.dx(1), 0, tolerance);
  EXPECT_NEAR(newMeasure.dx(2), 0, tolerance);
  EXPECT_NEAR(newMeasure.dx(3), 0, tolerance);
  EXPECT_NEAR(newMeasure.dx(4), 0, tolerance);
  EXPECT_NEAR(newMeasure.dx(5), 0, tolerance);
  EXPECT_NEAR(newMeasure.dx(6), 0, tolerance);
  EXPECT_NEAR(newMeasure.dx(7), 0, tolerance);

  Cogent::ModelFactory modelFactory;
  Teuchos::RCP<Cogent::Model>
    model = modelFactory.create(*geomSpec);
  
  // create the set of tets ('material set') that are on the positive side of the level surface
  //
  decltype(allSimplexes) fittedSimplexes;
  for( auto& spx : allSimplexes ){
    if(model->isMaterial(spx.sense))
      fittedSimplexes.push_back(spx);
  }

  // write the material set
  //
  if(verbose){
    std::cout << "Material set for one levelset." << std::endl;
    std::vector<Cogent::Simplex<RealType,RealType>> writeSimplexes;
    fromDFadType(allSimplexes, writeSimplexes);
    Cogent::writeTets(writeSimplexes);
  }

  // compute the volume of the material set of tets
  //
  DFadType fittedMeasure = 0.0;
  for( auto& spx : fittedSimplexes ){
     fittedMeasure += dicer->Volume(spx);
  }

  // volume of the material tets
  //
  std::cout << " Verifying material volume" << std::endl;
  EXPECT_NEAR(fittedMeasure.val(), 1.0/6.0, tolerance);

  // use finite differencing to compute the measure and derivatives
  //
  std::vector<Cogent::Simplex<RealType,RealType>> realImplicitSimplex(1);
  realImplicitSimplex[0] = Cogent::Simplex<RealType,RealType>(nPts);
  auto& realSimplex = realImplicitSimplex[0];
  realSimplex.points[0](0) =-1.0; realSimplex.points[0](1) =-1.0; realSimplex.points[0](2) =-1.0;
  realSimplex.points[1](0) = 1.0; realSimplex.points[1](1) =-1.0; realSimplex.points[1](2) =-1.0;
  realSimplex.points[2](0) =-1.0; realSimplex.points[2](1) = 1.0; realSimplex.points[2](2) =-1.0;
  realSimplex.points[3](0) =-1.0; realSimplex.points[3](1) =-1.0; realSimplex.points[3](2) = 1.0;

  std::vector<Cogent::Simplex<RealType,RealType>> realAllSimplexes, realFittedSimplexes;

  RealType epsilon = 1e-5;
  int numNodes = topoVals.dimension(0);
  int numLevelsets = topoVals.dimension(1);
  int numDerivs = numNodes*numLevelsets;
  std::vector<RealType> dmdtopo(numDerivs);
  for(int j=0; j<numNodes; j++){
   for(int k=0; k<numLevelsets; k++){

    topoVals(j,k) += epsilon;

    realAllSimplexes.clear();
    dicer->Cut(topoVals, realImplicitSimplex, realAllSimplexes);

    topoVals(j,k) -= epsilon;

    realFittedSimplexes.clear();
    for( auto& spx : realAllSimplexes ){
      if(model->isMaterial(spx.sense))
        realFittedSimplexes.push_back(spx);
    }

    RealType realFittedMeasure = 0.0;
    for( auto& spx : realFittedSimplexes ){
       realFittedMeasure += dicer->Volume(spx);
    }
    dmdtopo[j*numLevelsets+k] = (realFittedMeasure - fittedMeasure.val())/epsilon;
   }
  }
  if(verbose){
    std::cout << "   AD Measure: ";
    for(int i=0; i<numDerivs; i++){
      std::cout << std::setw(12) << fittedMeasure.dx(i) << " ";
    }
    std::cout << std::endl;
    std::cout << "   FD Measure: ";
    for(int i=0; i<numDerivs; i++){
      std::cout << std::setw(12) << dmdtopo[i] << " ";
    }
    std::cout << std::endl;
  }

  // Compare the finite differenced and automatically differentiated results
  //
  std::cout << " Verifying material volume derivatives" << std::endl;
  for(int i=0; i<numDerivs; i++)
    EXPECT_NEAR(fittedMeasure.dx(i), dmdtopo[i], epsilon);

  int numDiffDerivs = getNumDiffDerivs(allSimplexes,numDerivs, 1e-7);
  
  std::cout << " Verifying consitency of derivatives" << std::endl;
  EXPECT_EQ(numDiffDerivs, 0);

}
