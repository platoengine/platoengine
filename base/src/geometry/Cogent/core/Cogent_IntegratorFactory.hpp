#ifndef _COGENT_INTEGRATORFACTORY_H
#define _COGENT_INTEGRATORFACTORY_H

#include <Teuchos_RCP.hpp>

#include "Cogent_Integrator.hpp"

namespace Cogent {

class IntegratorFactory {
  public:
    Teuchos::RCP<Cogent::Integrator> create(
      Teuchos::RCP<shards::CellTopology> celltype,
      Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType, RealType > > basis,
      const Teuchos::ParameterList& geomSpecs);
};

} /** end namespace Cogent */

#endif
