#ifndef _COGENT_BODYINTEGRATOR_PROJECTED_H
#define _COGENT_BODYINTEGRATOR_PROJECTED_H

#include "Cogent_BodyIntegrator.hpp"
#include "Cogent_Projector.hpp"
#include "Cogent_WriteUtils.hpp"

namespace Cogent {

class BodyIntegrator_Projected : public BodyIntegrator, public Projector
{
  public:
    BodyIntegrator_Projected(
      Teuchos::RCP<shards::CellTopology> celltype,
      Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType, RealType > > basis,
      const Teuchos::ParameterList& geomSpecs) :
       BodyIntegrator(celltype, basis, geomSpecs), 
       Projector(celltype, geomSpecs){}

  // no geom data
  void getCubature( 
          FContainer<RealType>& weights,
          FContainer<RealType>& points,
          const FContainer<RealType>& coordCon);
  
  void getCubatureWeights( 
          FContainer<RealType>& weights,
          const FContainer<RealType>& coordCon);
  
  void getCubature( 
          FContainer<RealType>& weights,
          FContainer<RealType>& dwdgeom,
          FContainer<RealType>& points,
          const FContainer<RealType>& coordCon);

  void getCubatureWeights( 
          FContainer<RealType>& weights,
          FContainer<RealType>& dwdgeom,
          const FContainer<RealType>& coordCon);


  // geom data
  void getCubature( 
          FContainer<RealType>& weights,
          FContainer<RealType>& points,
          const FContainer<RealType>& geomData, 
          const FContainer<RealType>& coordCon);
  
  void getCubatureWeights( 
          FContainer<RealType>& weights,
          const FContainer<RealType>& geomData, 
          const FContainer<RealType>& coordCon);
  
  void getCubature( 
          FContainer<RealType>& weights,
          FContainer<RealType>& dwdgeom,
          FContainer<RealType>& points,
          const FContainer<RealType>& geomData, 
          const FContainer<RealType>& coordCon);

  void getCubatureWeights( 
          FContainer<RealType>& weights,
          FContainer<RealType>& dwdgeom,
          const FContainer<RealType>& geomData, 
          const FContainer<RealType>& coordCon);

  void
  getStandardPoints(FContainer<RealType>& refPoints);

  private:
    void getQPointWeights(
            std::vector<Simplex<RealType,RealType> >& explicitSimplexes,
            FContainer<RealType>& weights);

    void getQPointWeights(
            std::vector<Simplex<DFadType,DFadType> >& explicitSimplexes,
            const FContainer<RealType>& coorCon,
            FContainer<RealType>& weights, 
            FContainer<RealType>& dwdgeom);

};

} /** end namespace Cogent */

#endif
