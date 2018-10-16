#ifndef _COGENT_BOUNDARYINTEGRATOR_PROJECTED_H
#define _COGENT_BOUNDARYINTEGRATOR_PROJECTED_H

#include "Cogent_BoundaryIntegrator.hpp"
#include "Cogent_Projector.hpp"

namespace Cogent {

class BoundaryIntegrator_Projected : public BoundaryIntegrator, public Projector
{
  public:
    BoundaryIntegrator_Projected(
      Teuchos::RCP<shards::CellTopology> celltype,
      Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType, RealType > > basis,
      const Teuchos::ParameterList& geomSpecs) : 
       BoundaryIntegrator(celltype, basis, geomSpecs), 
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
            const FContainer<RealType>& coordCon,
            std::vector<Simplex<RealType,RealType> >& explicitSimplexes,
            FContainer<RealType>& weights);

    void getQPointWeights(
            const FContainer<RealType>& coordCon,
            std::vector<Simplex<DFadType,DFadType> >& explicitSimplexes,
            FContainer<RealType>& weights);

    void getQPointWeights(
            const FContainer<RealType>& coordCon,
            std::vector<Simplex<DFadType,DFadType> >& explicitSimplexes,
            FContainer<RealType>& weights, 
            FContainer<RealType>& dwdgeom);

};

} /** end namespace Cogent */

#endif
