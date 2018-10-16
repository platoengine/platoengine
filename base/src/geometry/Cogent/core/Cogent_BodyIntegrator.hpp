#ifndef _COGENT_BODYINTEGRATOR_H
#define _COGENT_BODYINTEGRATOR_H

#include "Cogent_Integrator.hpp"

namespace Cogent {

class BodyIntegrator : public Integrator 
{
  public:
    BodyIntegrator(
      Teuchos::RCP<shards::CellTopology> celltype,
      Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType, RealType > > basis,
      const Teuchos::ParameterList& geomSpecs) : Integrator(celltype,basis,geomSpecs){}

  protected:
    template <typename V, typename P>
    void getQPoints(
                    Simplex<V,P>& simplex, int offset,
                    FContainer<P>& weights,
                    FContainer<P>& points);


};

} /** end namespace Cogent */

#include "Cogent_BodyIntegrator_Def.hpp"

#endif
