#ifndef _COGENT_BODYINTEGRATOR_DIRECT_H
#define _COGENT_BODYINTEGRATOR_DIRECT_H

#include "Cogent_BodyIntegrator.hpp"

namespace Cogent {

class BodyIntegrator_Direct : public BodyIntegrator 
{
  public:
    BodyIntegrator_Direct(
      Teuchos::RCP<shards::CellTopology> celltype,
      Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType, RealType > > basis,
      const Teuchos::ParameterList& geomSpecs) :
       BodyIntegrator(celltype, basis, geomSpecs){}

  // no geom data
  void getCubature( 
          FContainer<RealType>& weights,
          FContainer<RealType>& points,
          const FContainer<RealType>& coordCon);

  void getCubature( 
          FContainer<RealType>& weights,
          FContainer<RealType>& dwdgeom,
          FContainer<RealType>& points,
          const FContainer<RealType>& coordCon);
 
  // geom data
  void getCubature( 
          FContainer<RealType>& weights,
          FContainer<RealType>& points,
          const FContainer<RealType>& geomData, 
          const FContainer<RealType>& coordCon);

  void getCubature( 
          FContainer<RealType>& weights,
          FContainer<RealType>& dwdgeom,
          FContainer<RealType>& points,
          const FContainer<RealType>& geomData, 
          const FContainer<RealType>& coordCon);

  void getBodySimplexes(
          const FContainer<RealType>& geomData, 
          const FContainer<RealType>& coordCon,
          std::vector<Simplex<RealType,RealType>>& tets);
  
  void getBoundarySimplexes(
          const FContainer<RealType>& geomData, 
          const FContainer<RealType>& coordCon,
          std::vector<Simplex<RealType,RealType>>& tets);
  
  private:
    void getQPoints(
            std::vector<Simplex<RealType,RealType> >& explicitSimplexes,
            FContainer<RealType>& weights, 
            FContainer<RealType>& points);

    void getQPoints(
            std::vector<Simplex<DFadType,DFadType> >& explicitSimplexes,
            FContainer<RealType>& weights, 
            FContainer<RealType>& dwdgeom, 
            FContainer<RealType>& points);

    void getQPoints(
            Simplex<DFadType,DFadType>& simplex, int offset,
            FContainer<RealType>& weights,
            FContainer<RealType>& dwdgeom,
            FContainer<RealType>& points);


};

} /** end namespace Cogent */
#endif
