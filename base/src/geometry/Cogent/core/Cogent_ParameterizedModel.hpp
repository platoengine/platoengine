/*
 * Cogent_ParameterizedModel.hpp
 *
 *  Created on: Dec 6, 2016
 *      Author: jrobbin
 */

#ifndef COGENT_PARAMETERIZEDMODEL_HPP_
#define COGENT_PARAMETERIZEDMODEL_HPP_


#include "Cogent_Model.hpp"
#include "geometry/Cogent_Geometry_Primitive.hpp"

namespace Cogent {

class ParameterizedModel: public Model {
public:
	ParameterizedModel(const Teuchos::ParameterList& geomSpec);
	virtual ~ParameterizedModel();

  bool isMaterial(const std::vector<Sense>& sense);
  bool isEmptyElement();
  bool isPartialElement();


  /**
   *   Compute signed distance values for levelsets that are both active and unique
   *   (i.e., in m_LevelsetMap).
   *   @param topoData Signed distance values, 
   *          \f$\eta(I,j)\f$, \f$I=0,...,nNodes-1\f&, \f$j=0,...,nActiveLevelsets-1\f$.
   *
   *   Idempotent = true.
   */

  void getSignedDistance(FContainer<RealType>& topoData);


  /**
   *
   *
   */

  void mapToGeometryData(const FContainer<RealType>& dMdT, 
                               FContainer<RealType>& dMdG);


  /**
   *   Update CSG model with current global parameters.
   *   @param geomData Global parameter array, \f$\mathbf{G}\f$
   *
   *   Idempotent = true.
   */
  void setModelData(const Cogent::FContainer<RealType>& geomData);


  /**
   *   Pass the nodal coordninates of the element into the ParameterizedModel.
   *   @param coordCon Nodal coordinate container, 
   *                   \f$X(I,i)\f$, \f$I=0,...,nNodes-1\f$, \f$i=0,...,nSpaceDims-1\f$.
   *
   *   Idempotent = true.
   */
  void setElementData(const Cogent::FContainer<RealType>& coordCon);


  bool isParameterized(){return true;}

private:

  enum struct Operation { Add, Subtract, NotSet };
  struct SubDomain {
    Teuchos::RCP<Cogent::Geometry::Primitive> m_primitive;
    Operation m_operation;
  };


  /**
   *  Create the m_SubdomainMap and m_LevelsetMap data members.
   *  This function modifies member data (m_SubdomainMap, m_LevelsetMap,
   *  m_surfaceIndex, m_surfaceIsPresent).
   *
   *  Idempotent = true.
   */
  void createMaps();


  /**
   * m_SubdomainMap[I] = (vector<activeLevelset>,operation) pair for 
   * activeDomain I.  vector<activeLevelset> is a list of indices into the 
   * vector of activeLevelsets.  operation is the operation of the activeDomain,
   * i.e., add or subtract.  
   */
  std::vector<std::pair<std::vector<int>,Operation> > m_SubdomainMap;


  /**
   *  m_LevelsetMap[I] = vector of (domainIndex,boundaryIndex) pairs matching
   *  activeLevelset I.  It's possible to have levelsets in a CSG construction 
   *  that are identical.  If these duplicates are used for intersecting an 
   *  element, repeated cuts are made which is wasteful and error prone.  
   *  activeLevelset I is guaranteed to be unique.  The list of (domainIndex,
   *  boundaryIndex) pairs are the domain/boundaries that are equivalent.
   */
  std::vector<std::vector<std::pair<int,int> > > m_LevelsetMap;
  
  std::vector<std::string> m_parameterNames;

  Teuchos::Array<SubDomain> m_subDomains;
  int m_numLevelsets=0;
  std::pair<int,int> m_surfacePair;

};

} /* namespace Cogent */

#endif /* COGENT_PARAMETERIZEDMODEL_HPP_ */
