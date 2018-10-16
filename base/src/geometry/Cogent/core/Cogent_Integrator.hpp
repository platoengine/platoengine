#ifndef Integrator_HPP
#define Integrator_HPP

#include <Shards_CellTopology.hpp>
#include <Intrepid2_Basis.hpp>
#include <Intrepid2_Cubature.hpp>

#include "Cogent_Dicer.hpp"
#include "Cogent_Model.hpp"

namespace Cogent {

#ifdef ENABLE_BOOST_TIMER
#include "Cogent_Timer.hpp"
#endif

void toDFadType(const FContainer<RealType>& from, FContainer<DFadType>& to);

class Integrator
{
 public:
  Integrator(
    Teuchos::RCP<shards::CellTopology> celltype,
    Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType, RealType > > basis,
    const Teuchos::ParameterList& geomSpecs);
  virtual ~Integrator(){};

  virtual void getMeasure(
                  RealType& measure, 
                  const FContainer<RealType>& coordCon, 
                  const FContainer<RealType>& geomData);

  virtual void getMeasure(
                  RealType& measure, 
                  FContainer<RealType>& dMdgeom,
                  const FContainer<RealType>& coordCon, 
                  const FContainer<RealType>& geomData);

  // geometry data not provided
  virtual void getCubature( 
                  FContainer<RealType>& weights,
                  FContainer<RealType>& points,
                  const FContainer<RealType>& coordCon)=0;

  virtual void getCubatureWeights( 
                  FContainer<RealType>& weights,
                  const FContainer<RealType>& coordCon){}

  virtual void getCubature( 
                  FContainer<RealType>& weights,
                  FContainer<RealType>& dwdgeom,
                  FContainer<RealType>& points,
                  const FContainer<RealType>& coordCon)=0;

  virtual void getCubatureWeights( 
                  FContainer<RealType>& weights,
                  FContainer<RealType>& dwdgeom,
                  const FContainer<RealType>& coordCon){}


  // geometry data provided
  virtual void getCubature( 
                  FContainer<RealType>& weights,
                  FContainer<RealType>& points,
                  const FContainer<RealType>& geomData, 
                  const FContainer<RealType>& coordCon)=0;

  virtual void getCubatureWeights( 
                  FContainer<RealType>& weights,
                  const FContainer<RealType>& geomData, 
                  const FContainer<RealType>& coordCon){}

  virtual void getCubature( 
                  FContainer<RealType>& weights,
                  FContainer<RealType>& dwdgeom,
                  FContainer<RealType>& points,
                  const FContainer<RealType>& geomData, 
                  const FContainer<RealType>& coordCon)=0;

  virtual void getCubatureWeights( 
                  FContainer<RealType>& weights,
                  FContainer<RealType>& dwdgeom,
                  const FContainer<RealType>& geomData, 
                  const FContainer<RealType>& coordCon){}

  virtual void getBodySimplexes(
          const FContainer<RealType>& geomData, 
          const FContainer<RealType>& coordCon,
          std::vector<Simplex<RealType,RealType>>& tets){}
  
  virtual void getBoundarySimplexes(
          const FContainer<RealType>& geomData, 
          const FContainer<RealType>& coordCon,
          std::vector<Simplex<RealType,RealType>>& tets){}
  

  virtual void
  getStandardPoints(FContainer<RealType>& refPoints){}

  const Teuchos::Array<std::string>& 
  getFieldNames(){return m_model->getFieldNames();}

  const Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType, RealType > >&
  getBasis(){return m_baseElementBasis;}

  bool isParameterized(){return m_model->isParameterized();}


 protected:

  template<typename V, typename P>
  void Refine( std::vector<Simplex<V,P> >& inpolys,
               std::vector<Simplex<V,P> >& outpolys);
  
  template <typename V, typename P>
  V getBodySimplexes(std::vector<Simplex<V,P> >& simplex);
  
  template <typename V, typename P>
  void getBoundarySimplexes( 
          std::vector<Simplex<V,P> >& inSimplex,
          std::vector<Simplex<V,P> >& outSimplex, int iTopo);

  template <typename V, typename P>
  V Measure(Simplex<V,P>& simplex);

  template <typename V, typename P>
  V BoundaryMeasure(Simplex<V,P>& simplex);

  RealType getUniformScaling(const FContainer<RealType>& coordCon);


  Teuchos::RCP<shards::CellTopology> cellTopology;
  Teuchos::RCP<Intrepid2::Cubature<Kokkos::Serial> > m_baseElementCubature;
  Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType, RealType > >    m_baseElementBasis;

  FContainer<RealType> m_centerPoint;
  FContainer<RealType> m_centerWeight;
  
  FContainer<RealType> m_bodySimplexWeights;
  FContainer<RealType> m_bodySimplexPoints;
  
  FContainer<RealType> m_boundarySimplexWeights;
  FContainer<RealType> m_boundarySimplexPoints;
  
  std::vector< std::vector<Simplex<RealType,RealType> > > m_refinement;
  std::vector< std::vector<Simplex<DFadType,DFadType> > > m_DFadRefinement;
  uint m_maxLevels;
  RealType m_maxError;
  RealType m_interfaceValue;
 // Teuchos::Array<std::string> m_fieldNames;
  int m_Verbosity, m_ErrorChecking;
     
  Teuchos::RCP<Dicer> m_dicer;
  Teuchos::RCP<Model> m_model;

  Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType, RealType > > m_simplexBodyBasis;
  Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType, RealType > > m_simplexBoundaryBasis;

  //std::vector<SubDomain> m_subDomains;

  uint m_numDims;

};

}

#include "Cogent_Integrator_Def.hpp"
#endif
