/*
 * Cogent_Model.hpp
 *
 *  Created on: Dec 5, 2016
 *      Author: jrobbin
 */

#ifndef COGENT_MODEL_HPP_
#define COGENT_MODEL_HPP_

#include <Teuchos_ParameterList.hpp>

#include "Cogent_Types.hpp"

namespace Cogent {

class Model {
public:
  Model(const Teuchos::ParameterList& modelSpec);
  virtual ~Model();

  virtual bool isMaterial(const std::vector<Sense>& sense)=0;
  virtual bool isEmptyElement()=0;
  virtual bool isPartialElement()=0;
  virtual bool isSurfaceElement(){return m_surfaceIsPresent;}
  virtual void getSignedDistance(FContainer<RealType>& topoData)=0;
  virtual void mapToGeometryData(const FContainer<RealType>& dMdT,
                                       FContainer<RealType>& dMdG)=0;

  virtual void setModelData(const Cogent::FContainer<RealType>& geomData) {m_geomData = geomData;}
  virtual void setElementData(const Cogent::FContainer<RealType>& coordCon) {m_coordCon = coordCon;}

  virtual bool isParameterized(){return false;}

  virtual int getSurfaceIndex();
 
  const Teuchos::Array<std::string>& getFieldNames() const {return m_fieldNames;}

protected:

  Teuchos::Array<std::string> m_fieldNames;
  FContainer<RealType> m_geomData;
  FContainer<RealType> m_coordCon;
  RealType m_interfaceValue;
  bool m_surfaceIsPresent;
  int m_surfaceIndex;

};

} /* namespace Cogent */

#endif /* COGENT_MODEL_HPP_ */
