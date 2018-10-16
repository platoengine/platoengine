#ifndef _COGENT_BOUNDARYINTEGRATOR_DIRECT_H
#define _COGENT_BOUNDARYINTEGRATOR_DIRECT_H

#include "Cogent_BoundaryIntegrator_Direct.hpp"
#include "Cogent_BoundaryIntegrator.hpp"

namespace Cogent {

class BoundaryIntegrator_Direct : public BoundaryIntegrator 
{
  public:
    BoundaryIntegrator_Direct(
      Teuchos::RCP<shards::CellTopology> celltype,
      Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType, RealType > > basis,
      const Teuchos::ParameterList& geomSpecs) : BoundaryIntegrator(celltype, basis, geomSpecs){}

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
  
  private:
    void getQPoints(
            std::vector<Simplex<RealType,RealType> >& explicitSimplexes,
            FContainer<RealType>& weights, 
            FContainer<RealType>& points,
            const FContainer<RealType>& coordCon);

    void getQPoints(
            std::vector<Simplex<DFadType,DFadType> >& explicitSimplexes,
            FContainer<RealType>& weights, 
            FContainer<RealType>& dwdgeom, 
            FContainer<RealType>& points,
            const FContainer<RealType>& coordCon);

};

} /** end namespace Cogent */

#endif
