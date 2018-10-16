#include "Cogent_IntegratorFactory.hpp"

#include "Cogent_BodyIntegrator_Direct.hpp"
#include "Cogent_BodyIntegrator_Projected.hpp"
#include "Cogent_BoundaryIntegrator_Direct.hpp"
#include "Cogent_BoundaryIntegrator_Projected.hpp"

//******************************************************************************//
Teuchos::RCP<Cogent::Integrator>
Cogent::IntegratorFactory::create(
   Teuchos::RCP<shards::CellTopology> _celltype,
   Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType, RealType> > _basis,
   const Teuchos::ParameterList& geomSpec)
//******************************************************************************//
{
  std::string blockType;
  if(geomSpec.isType<std::string>("Geometry Type"))
    blockType = geomSpec.get<std::string>("Geometry Type");
  else
    blockType = "Body";

  std::string integrationType;
  if(geomSpec.isType<int>("Projection Order")){
    integrationType = "Projected";
  }
  else {
    integrationType = "Direct";
  }

  if( blockType == "Body"){

    if( integrationType == "Projected" )
      return Teuchos::rcp(new Cogent::BodyIntegrator_Projected(_celltype, _basis, geomSpec) );
    else
    if( integrationType == "Direct" )
      return Teuchos::rcp(new Cogent::BodyIntegrator_Direct(_celltype, _basis, geomSpec) );
  } else
  if( blockType == "Boundary"){
    if( integrationType == "Projected" )
      return Teuchos::rcp(new Cogent::BoundaryIntegrator_Projected(_celltype, _basis, geomSpec) );
    else
    if( integrationType == "Direct" )
      return Teuchos::rcp(new Cogent::BoundaryIntegrator_Direct(_celltype, _basis, geomSpec) );
  }
  TEUCHOS_TEST_FOR_EXCEPTION(true, std::runtime_error, 
    std::endl << "Cogent_Integrator: Integration type (" << blockType << ") not found." << std::endl);

}
