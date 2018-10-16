/*
 * Cogent_Geometry_Brick.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: jrobbin
 */

#include "Cogent_Geometry_Brick.hpp"
#include "Cogent_Geometry_Plane.hpp"
#include "core/Cogent_ParameterFunction.hpp"

namespace Cogent {
namespace Geometry {

/******************************************************************************/
Brick::
Brick(const Teuchos::ParameterList& params, 
      const std::vector<std::string>& shapeParameterNames,
      Teuchos::RCP<Cogent::CoordinateSystem> coordinateSystem) :
  Primitive(params, shapeParameterNames, coordinateSystem,
  /*numDim*/3, /*numLevelsets*/6, /*numLocalParams*/3)
/******************************************************************************/
{
  m_boundaryNames.resize(c_numLevelsets);
  
  std::string strXdim = params.get<std::string>("X Dimension");
  m_localParameters.push_back(Teuchos::rcp(new Cogent::ParameterFunction(shapeParameterNames, strXdim)));

  std::string strYdim = params.get<std::string>("Y Dimension");
  m_localParameters.push_back(Teuchos::rcp(new Cogent::ParameterFunction(shapeParameterNames, strYdim)));

  std::string strZdim = params.get<std::string>("Z Dimension");
  m_localParameters.push_back(Teuchos::rcp(new Cogent::ParameterFunction(shapeParameterNames, strZdim)));

  //                        0      1      2
  // localParameters are {dim_x, dim_y, dim_z}

  // negative X face
  m_boundaryNames[0] = "Negative X";
  m_boundaries.push_back(Teuchos::rcp(new Plane(m_localParameters, Parameter::Xdim, 
                                                -1.0/2.0, Axis::X, Sense::Positive)));
  // positive X face
  m_boundaryNames[1] = "Positive X";
  m_boundaries.push_back(Teuchos::rcp(new Plane(m_localParameters, Parameter::Xdim, 
                                                 1.0/2.0, Axis::X, Sense::Negative)));
  // negative Y face
  m_boundaryNames[2] = "Negative Y";
  m_boundaries.push_back(Teuchos::rcp(new Plane(m_localParameters, Parameter::Ydim, 
                                                -1.0/2.0, Axis::Y, Sense::Positive)));
  // positive Y face
  m_boundaryNames[3] = "Positive Y";
  m_boundaries.push_back(Teuchos::rcp(new Plane(m_localParameters, Parameter::Ydim, 
                                                 1.0/2.0, Axis::Y, Sense::Negative)));
  // negative Z face
  m_boundaryNames[4] = "Negative Z";
  m_boundaries.push_back(Teuchos::rcp(new Plane(m_localParameters, Parameter::Zdim, 
                                                -1.0/2.0, Axis::Z, Sense::Positive)));
  // positive Z face
  m_boundaryNames[5] = "Positive Z";
  m_boundaries.push_back(Teuchos::rcp(new Plane(m_localParameters, Parameter::Zdim, 
                                                 1.0/2.0, Axis::Z, Sense::Negative)));

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

Brick::~Brick() { }

} /* namespace Geometry */
} /* namespace Cogent */
