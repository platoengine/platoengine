/*
 * Cogent_Geometry_Torus.cpp
 *
 *  Created on: Aug 1, 2017
 *      Author: jrobbin
 */

#include "Cogent_Geometry_Torus.hpp"
#include "Cogent_Geometry_TorusSurface.hpp"

namespace Cogent {
namespace Geometry {

/******************************************************************************/
Torus::
Torus(const Teuchos::ParameterList& params,
         const std::vector<std::string>& shapeParameterNames,
         Teuchos::RCP<Cogent::CoordinateSystem> coordinateSystem) :
  Primitive(params, shapeParameterNames, coordinateSystem,
  /*numDim*/3, /*numLevelsets*/1, /*numLocalParams*/2)
/******************************************************************************/
{
  m_boundaryNames.resize(c_numLevelsets);

  std::string strMajorRadius = params.get<std::string>("Major Radius");
  m_localParameters.push_back(Teuchos::rcp(new Cogent::ParameterFunction(shapeParameterNames, strMajorRadius)));

  std::string strMinorRadius = params.get<std::string>("Minor Radius");
  m_localParameters.push_back(Teuchos::rcp(new Cogent::ParameterFunction(shapeParameterNames, strMinorRadius)));

  //                        0            1
  // localParameters are {MajorRadius, MinorRadius}


  std::string axisString = params.get<std::string>("Axis");
  if(axisString == "X" || axisString == "x"){ m_axis = Axis::X;}
   else
  if(axisString == "Y" || axisString == "y"){ m_axis = Axis::Y;}
  else
  if(axisString == "Z" || axisString == "z"){ m_axis = Axis::Z;}
  else
    TEUCHOS_TEST_FOR_EXCEPTION(true, std::runtime_error, std::endl <<
      "Cogent::Geometry::Torus: specify X, Y, or Z" << std::endl);


  // sphere surface
  m_boundaryNames[0] = "Torus Surface";
  m_boundaries.push_back(Teuchos::rcp(new TorusSurface(m_localParameters, 
                         Parameter::MajorRadius, Parameter::MinorRadius, m_axis)));
  
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

Torus::~Torus() { }

} /* namespace Geometry */
} /* namespace Cogent */
