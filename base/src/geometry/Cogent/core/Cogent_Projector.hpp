#ifndef _COGENT_PROJECTOR_H
#define _COGENT_PROJECTOR_H

#include <Shards_CellTopology.hpp>
#include <Teuchos_RCP.hpp>
#include <Teuchos_ParameterList.hpp>
#include <Teuchos_SerialDenseMatrix.hpp>

#include "Cogent_Types.hpp"
#include "Cogent_Utilities.hpp"

namespace Cogent {

class Projector
{
  public:
    Projector(
      Teuchos::RCP<shards::CellTopology> celltype,
      const Teuchos::ParameterList& geomSpecs);
  protected:
    Teuchos::SerialDenseMatrix<int,RealType> m_A;
    int m_systemSize;
    FContainer<RealType> m_standardPoints;
    FContainer<RealType> m_standardWeights;
    FContainer<uint> m_monomial;
};

} /** end namespace Cogent */

#endif
