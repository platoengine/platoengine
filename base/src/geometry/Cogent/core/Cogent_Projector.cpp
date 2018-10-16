#include "Cogent_Projector.hpp"

#include <Teuchos_SerialDenseSolver.hpp>

//******************************************************************************//
Cogent::Projector::Projector(
  Teuchos::RCP<shards::CellTopology> celltype,
  const Teuchos::ParameterList& geomSpec)
//******************************************************************************//
{
  int projectionOrder = -1;
  if(geomSpec.isType<int>("Projection Order")){
    projectionOrder = geomSpec.get<int>("Projection Order");
  }

  int nDims = celltype->getDimension();


// JR TODO:  The gauss point locations must be in the same order as the host code, so
// hard-coding the locations and weights below is code specific.  

  // *** hexahedral element ***/
  if( celltype->getBaseName() == shards::getCellTopologyData< shards::Hexahedron<8> >()->name ){


   if(projectionOrder >= 0){

     std::vector<RealType> g,w;
   
     switch ( projectionOrder ) {

       case 0 :
         g.resize(1);
         w.resize(1);
         g[0] = 0.0;
         w[0] = 2.0;
         break;

       case 1 :
         g.resize(2);
         w.resize(2);
         g[0] =  sqrt(1.0/3.0);
         g[1] = -sqrt(1.0/3.0);
         w[0] = 1.0;
         w[1] = 1.0;
         break;

       case 2 :
         g.resize(3);
         w.resize(3);
         g[0] =  sqrt(3.0/5.0);
         g[1] = 0.0;
         g[2] = -sqrt(3.0/5.0);
         w[0] = 5.0/9.0;
         w[1] = 8.0/9.0;
         w[2] = 5.0/9.0;
         break;

       case 3 :
         g.resize(4);
         w.resize(4);
         g[0] =  sqrt(3.0/7.0+2.0/7.0*sqrt(6.0/5.0));
         g[1] =  sqrt(3.0/7.0-2.0/7.0*sqrt(6.0/5.0));
         g[2] = -sqrt(3.0/7.0-2.0/7.0*sqrt(6.0/5.0));
         g[3] = -sqrt(3.0/7.0+2.0/7.0*sqrt(6.0/5.0));
         w[0] = (18.0-sqrt(30.0))/36.0;
         w[1] = (18.0+sqrt(30.0))/36.0;
         w[2] = (18.0+sqrt(30.0))/36.0;
         w[3] = (18.0-sqrt(30.0))/36.0;
         break;

       case 4 :
         g.resize(5);
         w.resize(5);
         g[0] =  sqrt(5.0+2.0*sqrt(10.0/7.0))/3.0;
         g[1] =  sqrt(5.0-2.0*sqrt(10.0/7.0))/3.0;
         g[2] =  0.0;
         g[3] = -sqrt(5.0-2.0*sqrt(10.0/7.0))/3.0;
         g[4] = -sqrt(5.0+2.0*sqrt(10.0/7.0))/3.0;
         w[0] = (322.0-13.0*sqrt(70.0))/900.0;
         w[1] = (322.0+13.0*sqrt(70.0))/900.0;
         w[2] = 128.0/225.0;
         w[3] = (322.0+13.0*sqrt(70.0))/900.0;
         w[4] = (322.0-13.0*sqrt(70.0))/900.0;
         break;

       default :
         TEUCHOS_TEST_FOR_EXCEPTION(true, std::runtime_error, 
            std::endl << "Cogent_Integrator: Projection order must be less than 6." << std::endl);
         break;
     }

     int stride = g.size();
     m_systemSize = pow(stride,nDims);
     m_standardPoints = Cogent::FContainer<RealType>("m_standardPoints",m_systemSize,nDims);
     m_standardWeights = Cogent::FContainer<RealType>("m_standardWeights",m_systemSize);
     m_monomial = Cogent::FContainer<uint>("m_monomial",m_systemSize,nDims);
     for(int i=0; i<=projectionOrder; i++)
       for(int j=0; j<=projectionOrder; j++)
         for(int k=0; k<=projectionOrder; k++){
           m_monomial(k*stride*stride+j*stride+i,0) = i;
           m_monomial(k*stride*stride+j*stride+i,1) = j;
           m_monomial(k*stride*stride+j*stride+i,2) = k;
           m_standardPoints(k*stride*stride+j*stride+i,0) = g[i];
           m_standardPoints(k*stride*stride+j*stride+i,1) = g[j];
           m_standardPoints(k*stride*stride+j*stride+i,2) = g[k];
           m_standardWeights(k*stride*stride+j*stride+i) = w[i]*w[j]*w[k];
         }

     m_A.shape(m_systemSize,m_systemSize);
     for(int I=0; I<m_systemSize; I++){
       for(int J=0; J<m_systemSize; J++)
         m_A(I, J) = mono<RealType>(m_standardPoints(J,0),m_standardPoints(J,1),m_standardPoints(J,2),
                        m_monomial(I,0),m_monomial(I,1),m_monomial(I,2));
     }
     Teuchos::SerialDenseSolver<int, RealType> localSolver;
     localSolver.setMatrix( Teuchos::rcp(&m_A, false) );
     localSolver.invert();

    } else {
      m_systemSize = 0;
    }
  } else {
    TEUCHOS_TEST_FOR_EXCEPTION(true, std::runtime_error, 
      std::endl << "Cogent_Integrator: Projection for " << 
      celltype->getBaseName() << " cell type not supported." << std::endl );
  }
}
