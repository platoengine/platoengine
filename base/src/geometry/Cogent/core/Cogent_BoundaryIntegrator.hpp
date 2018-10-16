#ifndef _COGENT_BOUNDARYINTEGRATOR_H
#define _COGENT_BOUNDARYINTEGRATOR_H

#include "Cogent_Integrator.hpp"

namespace Cogent {

class BoundaryIntegrator : public Integrator 
{
  public:
    BoundaryIntegrator(
      Teuchos::RCP<shards::CellTopology> celltype,
      Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType, RealType > > basis,
      const Teuchos::ParameterList& geomSpecs);

    void getMeasure(
            RealType& measure, 
            const FContainer<RealType>& coordCon, 
            const FContainer<RealType>& geomData);

    void getMeasure(
            RealType& measure, 
            FContainer<RealType>& dMdgeom,
            const FContainer<RealType>& coordCon, 
            const FContainer<RealType>& geomData);

  protected:

    template <typename V, typename P>
    void getQPoints(
            Simplex<V,P>& simplex, int offset,
            FContainer<P>& weights,
            FContainer<P>& points,
            const FContainer<RealType>& coordCon);

    void getQPoints(
            Simplex<DFadType,DFadType>& simplex, int offset,
            FContainer<RealType>& weights,
            FContainer<RealType>& dwdgeom,
            FContainer<RealType>& points,
            const FContainer<RealType>& coordCon);

    uint m_surfaceIndex;

};

} /** end namespace Cogent */

#include "Cogent_BoundaryIntegrator_Def.hpp"
#endif
