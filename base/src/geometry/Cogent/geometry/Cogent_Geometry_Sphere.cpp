/*
 * Cogent_Geometry_Sphere.cpp
 *
 *  Created on: July 31, 2017
 *      Author: jrobbin
 */

#include "Cogent_Geometry_Sphere.hpp"
#include "Cogent_Geometry_SphereSurface.hpp"

namespace Cogent {
namespace Geometry {

/******************************************************************************/
Sphere::
Sphere(const Teuchos::ParameterList& params,
         const std::vector<std::string>& shapeParameterNames,
         Teuchos::RCP<Cogent::CoordinateSystem> coordinateSystem) :
  Primitive(params, shapeParameterNames, coordinateSystem,
  /*numDim*/3, /*numLevelsets*/1, /*numLocalParams*/1)
/******************************************************************************/
{
  m_boundaryNames.resize(c_numLevelsets);

  std::string strRadius = params.get<std::string>("Radius");
  m_localParameters.push_back(Teuchos::rcp(new Cogent::ParameterFunction(shapeParameterNames, strRadius)));

  //                        0   
  // localParameters are {radius}

  // sphere surface
  m_boundaryNames[0] = "Sphere Surface";
  m_boundaries.push_back(Teuchos::rcp(new SphereSurface(m_localParameters, Parameter::Radius)));
  
  // fill aliases
  for( auto& alias : m_boundaryAliases ){
    bool foundBoundary = false;
    for(int iBoundary=0; iBoundary<c_numLevelsets; iBoundary++){
      if( alias.boundaryName == m_boundaryNames[iBoundary]){
        alias.localIndex = iBoundary;
        foundBoundary = true;
        break;
      }
    }
    TEUCHOS_TEST_FOR_EXCEPTION(!foundBoundary, std::runtime_error,
      std::endl << "Boundary with name '" << alias.boundaryName << "' not found" << std::endl);
  }
}

Sphere::~Sphere() { }

} /* namespace Geometry */
} /* namespace Cogent */
