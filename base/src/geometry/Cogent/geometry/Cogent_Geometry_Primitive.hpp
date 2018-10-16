/*
 * Cogent_Geometry_Primitive.hpp
 *
 *  Created on: Nov 23, 2016
 *      Author: jrobbin
 */

#ifndef GEOMETRY_COGENT_GEOMETRY_PRIMITIVE_HPP_
#define GEOMETRY_COGENT_GEOMETRY_PRIMITIVE_HPP_

#include <Teuchos_ParameterList.hpp>

#include "core/Cogent_Types.hpp"
#include "core/Cogent_ParameterFunction.hpp"
#include "core/Cogent_CoordinateSystem.hpp"
#include "Cogent_Geometry_Types.hpp"
#include "Cogent_Geometry_PrimitiveSurface.hpp"

namespace Cogent {
namespace Geometry {

class Primitive {
public:
  Primitive(const Teuchos::ParameterList& params, 
            const std::vector<std::string>& names, 
            Teuchos::RCP<Cogent::CoordinateSystem> coordinateSystem,
            int, int, int);
  virtual ~Primitive();
  virtual void update(const std::vector<RealType>& P);
  void compress(const std::vector<Teuchos::RCP<Cogent::Geometry::Primitive>>, int, int);


  /**
   * Determine if a given element intersects the Primitive.
   *
   * @param coordCon   Coordinates {FContainer(I,i), I->NodeIndex, i->dimIndex} of nodes that compose 
   *                   the potentially intersecting element.
   * @param inside     Boolean stating if the given element intersects the Primitive.
   * @param lsIndices  Indices of the Primitive's m_boundaries that intersect the given element
   */
  void intersected(const FContainer<RealType>& coordCon, bool& inside, std::vector<int>& lsIndices) const;

  void compute(const FContainer<RealType>& coordCon, 
               FContainer<RealType> topoVals, 
               int localLSIndex,
               int offset=0) const;

  void computeDeriv(const FContainer<RealType>& coordCon, 
                    Cogent::LocalMatrix<RealType>& dTdG, 
                    int localLSIndex) const;

  bool nodeIsInside(const FContainer<RealType>& coordCon, int nodeIndex) const;
  
  int getNumLevelsets() const { return c_numLevelsets; }

  const std::vector<Teuchos::RCP<Cogent::Geometry::PrimitiveSurface>>&
  getBoundaries() const {return m_boundaries;}

  const std::vector<Cogent::Geometry::BoundaryAlias>&
  getBoundaryAliases() const {return m_boundaryAliases;}

  const Teuchos::RCP<Cogent::CoordinateSystem>&
  getCoordinateSystem() const {return m_coordinateSystem;}

protected:

  void compute(std::vector<RealType> X, std::vector<RealType>& vals) const;
  RealType compute(std::vector<RealType> X, int localLSIndex) const;

  void computeDeriv(const RealType* X, RealType* vals, int levelsetIndex) const;

  std::vector<std::string> m_parameterNames;

  Teuchos::RCP<Cogent::CoordinateSystem> m_coordinateSystem;

  const int c_numDim;
  const int c_numLevelsets;
  const int c_numGlobalParameters;
  const int c_numLocalParameters;

  std::vector<Teuchos::RCP<Cogent::ParameterFunction>> m_localParameters;

  std::vector<Teuchos::RCP<Cogent::Geometry::PrimitiveSurface>> m_boundaries;

  //Cogent::LocalMatrix<RealType> m_dLdG;

  std::vector<Cogent::Geometry::BoundaryAlias> m_boundaryAliases;
  std::vector<std::string> m_boundaryNames;

};

} /* namespace Geometry */
} /* namespace Cogent */

#endif /* GEOMETRY_COGENT_GEOMETRY_PRIMITIVE_HPP_ */
